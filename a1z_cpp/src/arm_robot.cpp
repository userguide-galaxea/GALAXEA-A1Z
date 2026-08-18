#include "a1z/arm_robot.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>

namespace a1z {

ArmRobot::ArmRobot(std::shared_ptr<MixedMotorChain> motor_chain,
                   std::shared_ptr<GravityModel> gravity_model,
                   std::shared_ptr<Gripper> gripper,
                   int control_freq_hz,
                   double min_freq_hz)
    : motor_chain_(std::move(motor_chain))
    , gravity_model_(std::move(gravity_model))
    , gripper_(std::move(gripper))
    , control_freq_hz_(control_freq_hz)
    , min_freq_hz_(min_freq_hz) {
    control_period_s_ = 1.0 / control_freq_hz_;
    last_feedback_time_ = std::chrono::steady_clock::now();
}

ArmRobot::~ArmRobot() {
    if (running_) {
        stop();
    }
}

void ArmRobot::start(const JointVector* initial_kp, const JointVector* initial_kd) {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);

    if (running_) {
        throw std::runtime_error("ArmRobot already running");
    }

    // Reset state
    stop_requested_ = false;
    estop_latch_ = false;
    commands_blocked_ = true;

    // Enable motors
    std::cout << "[ArmRobot] Enabling motors..." << std::endl;
    motor_chain_->enable_all();
    if (gripper_) {
        gripper_->enable();
        gripper_->home();
        // Route gripper CAN feedback through drain_and_update so its
        // last_feedback stays fresh (matches the Python SDK).
        motor_chain_->register_external_motor(gripper_->motor());
    }

    // MotorA does not return feedback on enable alone — it needs at least
    // one MIT command first. Send zero-position-gain probe frames so the
    // motors answer without applying a position correction.
    std::cout << "[ArmRobot] Waiting for feedback..." << std::endl;
    JointVector zero = {};
    JointVector probe_kd;
    probe_kd.fill(0.05);

    bool feedback_ok = false;
    for (int attempt = 0; attempt < 3 && !feedback_ok; ++attempt) {
        motor_chain_->send_commands(zero, zero, zero, probe_kd, zero);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        auto start_time = std::chrono::steady_clock::now();
        while (std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count() < 0.5) {
            read_state();
            if (motor_chain_->all_feedback_fresh(0.2)) {
                feedback_ok = true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        if (!feedback_ok) {
            std::cout << "[ArmRobot] Startup probe incomplete, retrying ("
                      << (attempt + 2) << "/3)..." << std::endl;
        }
    }

    if (!feedback_ok) {
        if (!motor_chain_->disable_all()) {
            std::cerr << "[ArmRobot] ERROR: failed to disable all motors after "
                         "startup failure — check motor state manually!" << std::endl;
        }
        throw std::runtime_error("Failed to receive feedback from all joints");
    }

    // Initialize command to current position
    {
        std::lock_guard<std::mutex> cmd_lock(command_mutex_);
        std::lock_guard<std::mutex> state_lock(state_mutex_);
        command_.pos = state_.pos;
        command_.vel.fill(0.0);
        command_.acc.fill(0.0);
        command_.torque_ff.fill(0.0);

        if (initial_kp) {
            command_.kp = *initial_kp;
        } else if (!zero_gravity_mode_) {
            command_.kp = default_kp_;
        } else {
            command_.kp.fill(0.0);
        }

        if (initial_kd) {
            command_.kd = *initial_kd;
        } else if (!zero_gravity_mode_) {
            command_.kd = default_kd_;
        } else {
            for (size_t i = 0; i < 6; ++i) {
                command_.kd[i] = default_kd_[i] * 0.5;
            }
        }
    }

    // Send initial hold command
    {
        std::lock_guard<std::mutex> state_lock(state_mutex_);
        JointVector zero = {};
        motor_chain_->send_commands(state_.pos, zero, command_.kp, command_.kd, zero);
    }

    // Start control thread
    running_ = true;
    control_state_ = ControlState::RUNNING;
    control_thread_ = std::thread(&ArmRobot::control_loop, this);

    // Wait for thread to start
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (!control_thread_.joinable()) {
        running_ = false;
        throw std::runtime_error("Failed to start control thread");
    }

    commands_blocked_ = false;
    std::cout << "[ArmRobot] Control loop started at " << control_freq_hz_ << " Hz" << std::endl;
}

void ArmRobot::stop() {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);

    if (!running_) {
        return;
    }

    std::cout << "[ArmRobot] Stopping control loop..." << std::endl;
    commands_blocked_ = true;
    stop_requested_ = true;

    if (control_thread_.joinable()) {
        control_thread_.join();
    }

    running_ = false;
    control_state_ = ControlState::STOPPED;
    std::cout << "[ArmRobot] Control loop stopped" << std::endl;
}

ControlState ArmRobot::control_state() const {
    return control_state_;
}

bool ArmRobot::command_joint_state(const JointCommand& cmd) {
    if (commands_blocked_) {
        std::cerr << "[ArmRobot] Command rejected: robot is blocked" << std::endl;
        return false;
    }

    // Validate position with circle-unwrap (S¹ topology)
    JointVector current_pos;
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        current_pos = command_.pos;
    }
    auto validated_pos = clip_joint_pos(cmd.pos, &current_pos);
    if (!validated_pos) {
        std::cerr << "[ArmRobot] Command rejected: position out of limits" << std::endl;
        return false;
    }

    // Validate velocity
    for (size_t i = 0; i < 6; ++i) {
        if (std::abs(cmd.vel[i]) > vel_limit_[i]) {
            std::cerr << "[ArmRobot] Command rejected: velocity limit exceeded on joint "
                      << i + 1 << std::endl;
            return false;
        }
    }

    // Validate gains
    for (size_t i = 0; i < 6; ++i) {
        if (cmd.kp[i] < 0 || cmd.kp[i] > 200.0) {
            std::cerr << "[ArmRobot] Command rejected: kp out of range on joint "
                      << i + 1 << std::endl;
            return false;
        }
        if (cmd.kd[i] < 0 || cmd.kd[i] > 5.0) {
            std::cerr << "[ArmRobot] Command rejected: kd out of range on joint "
                      << i + 1 << std::endl;
            return false;
        }
    }

    // Apply command
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        command_.pos = *validated_pos;
        command_.vel = cmd.vel;
        command_.acc = cmd.acc;
        command_.kp = cmd.kp;
        command_.kd = cmd.kd;
        command_.torque_ff = cmd.torque_ff;
    }

    return true;
}

bool ArmRobot::command_joint_pos(const JointVector& pos) {
    JointCommand cmd;
    cmd.pos = pos;
    cmd.vel.fill(0.0);
    cmd.acc.fill(0.0);
    cmd.kp = default_kp_;
    cmd.kd = default_kd_;
    cmd.torque_ff.fill(0.0);
    return command_joint_state(cmd);
}

JointState ArmRobot::get_joint_state() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return state_;
}

void ArmRobot::estop() {
    estop_latch_ = true;
    commands_blocked_ = true;
    std::cout << "[ArmRobot] Emergency stop engaged" << std::endl;
}

void ArmRobot::release() {
    estop_latch_ = false;
    commands_blocked_ = false;
    std::cout << "[ArmRobot] Emergency stop released" << std::endl;
}

void ArmRobot::set_gravity_comp_factor(double factor) {
    gravity_comp_factor_ = std::clamp(factor, 0.0, 1.0);
}

void ArmRobot::set_joint_sign(const JointVector& sign) {
    joint_sign_ = sign;
}

void ArmRobot::set_joint_limits(const std::array<std::pair<double, double>, 6>& limits) {
    joint_limits_ = limits;
}

void ArmRobot::set_gravity_mode(bool enabled) {
    {
        std::lock_guard<std::mutex> cmd_lock(command_mutex_);
        if (enabled) {
            command_.kp.fill(0.0);
            for (size_t i = 0; i < 6; ++i) {
                command_.kd[i] = default_kd_[i] * 0.5;
            }
        } else {
            command_.kp = default_kp_;
            command_.kd = default_kd_;
        }
    }
    zero_gravity_mode_ = enabled;
}

void ArmRobot::set_recording(bool enable) {
    std::lock_guard<std::mutex> lock(record_mutex_);
    recording_ = enable;
    if (enable) {
        record_buffer_.clear();
        last_record_time_ = std::chrono::steady_clock::now();
    }
}

std::vector<std::pair<double, JointVector>> ArmRobot::get_recording() const {
    std::lock_guard<std::mutex> lock(record_mutex_);
    return record_buffer_;
}

void ArmRobot::clear_recording() {
    std::lock_guard<std::mutex> lock(record_mutex_);
    record_buffer_.clear();
}

void ArmRobot::control_loop() {
    const double FREQ_CHECK_INTERVAL = 2.0;  // Check frequency every 2s
    const int MAX_SLOW_PERIODS = 3;          // Emergency stop after 3 consecutive slow periods

    auto last_check_time = std::chrono::steady_clock::now();
    int iteration_count = 0;
    int consecutive_slow = 0;

    while (!stop_requested_) {
        auto loop_start = std::chrono::steady_clock::now();

        try {
            update();
        } catch (const std::exception& e) {
            std::cerr << "[ArmRobot] Control loop error: " << e.what() << std::endl;
            std::cerr << "[ArmRobot] Emergency stop!" << std::endl;
            send_zero_torque_and_disable();
            running_ = false;
            control_state_ = ControlState::HARD_DISABLED;
            return;
        }

        iteration_count++;
        auto now = std::chrono::steady_clock::now();

        // Frequency monitoring
        double elapsed_since_check = std::chrono::duration<double>(now - last_check_time).count();
        if (elapsed_since_check >= FREQ_CHECK_INTERVAL) {
            double freq = iteration_count / elapsed_since_check;
            std::cout << "[ArmRobot] Control loop frequency: " << freq << " Hz" << std::endl;

            if (freq < min_freq_hz_) {
                consecutive_slow++;
                std::cerr << "[ArmRobot] Control loop too slow: " << freq << " Hz < "
                          << min_freq_hz_ << " Hz (" << consecutive_slow << "/"
                          << MAX_SLOW_PERIODS << ")" << std::endl;

                if (consecutive_slow >= MAX_SLOW_PERIODS) {
                    std::cerr << "[ArmRobot] Frequency below " << min_freq_hz_
                              << " Hz for " << consecutive_slow * FREQ_CHECK_INTERVAL
                              << "s — emergency stop!" << std::endl;
                    send_zero_torque_and_disable();
                    running_ = false;
                    control_state_ = ControlState::HARD_DISABLED;
                    return;
                }
            } else {
                consecutive_slow = 0;
            }

            last_check_time = now;
            iteration_count = 0;
        }

        // Sleep to maintain control frequency
        double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - loop_start).count();
        double sleep_time = control_period_s_ - elapsed;
        if (sleep_time > 0) {
            std::this_thread::sleep_for(std::chrono::duration<double>(sleep_time));
        }
    }

    // Cleanup: send zero torque then disable
    send_zero_torque_and_disable();
    running_ = false;
}

void ArmRobot::update() {
    // 1) Read current joint state
    read_state();

    // 2) Runtime safety checks
    check_runtime_safety();

    // 3) Sample for recording
    if (recording_) {
        auto now = std::chrono::steady_clock::now();
        double since_last = std::chrono::duration<double>(now - last_record_time_).count();
        if (since_last >= record_period_s_) {
            std::lock_guard<std::mutex> state_lock(state_mutex_);
            std::lock_guard<std::mutex> record_lock(record_mutex_);
            if (recording_) {
                record_buffer_.emplace_back(now_seconds(), state_.pos);
            }
            last_record_time_ = now;
        }
    }

    // 4) Get current command
    JointCommand cmd;
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        cmd = command_;
    }

    // 5) Compute inverse dynamics (gravity compensation)
    JointVector tau_id = {};
    if (gravity_model_ && gravity_comp_factor_ > 0.0) {
        tau_id = gravity_model_->compute_inverse_dynamics(
            state_.pos, cmd.vel, cmd.acc
        );

        // Scale and clip
        for (size_t i = 0; i < 6; ++i) {
            tau_id[i] *= gravity_torque_scale_[i] * gravity_comp_factor_;
            if (std::abs(tau_id[i]) > max_gravity_torque_[i]) {
                tau_id[i] = std::copysign(max_gravity_torque_[i], tau_id[i]);
            }
        }
    }

    // 6) Combine torques
    JointVector motor_torques;
    for (size_t i = 0; i < 6; ++i) {
        double torque_urdf = cmd.torque_ff[i] + tau_id[i];
        motor_torques[i] = std::clamp(
            torque_urdf * joint_sign_[i],
            -torque_clip_[i], torque_clip_[i]
        );
    }

    // 7) Send commands to motor chain
    JointVector motor_pos, motor_vel;
    for (size_t i = 0; i < 6; ++i) {
        motor_pos[i] = cmd.pos[i] * joint_sign_[i];
        motor_vel[i] = cmd.vel[i] * joint_sign_[i];
    }

    motor_chain_->send_commands(motor_pos, motor_vel, cmd.kp, cmd.kd, motor_torques);

    // 8) Send gripper command
    if (gripper_) {
        // SOP-06: leave a bus slot for the last arm motor's feedback before
        // the gripper frame, same as the Python SDK.
        double gap = motor_chain_->inter_cmd_gap_s();
        if (gap > 0) {
            std::this_thread::sleep_for(std::chrono::duration<double>(gap));
        }
        gripper_->step();
    }
}

void ArmRobot::read_state() {
    int count = motor_chain_->drain_and_update(1);
    if (count > 0) {
        last_feedback_time_ = std::chrono::steady_clock::now();
    }

    auto positions = motor_chain_->get_positions();
    auto velocities = motor_chain_->get_velocities();
    auto efforts = motor_chain_->get_efforts();
    auto error_codes = motor_chain_->get_error_codes();
    auto [temp_mos, temp_rotor] = motor_chain_->get_temperatures();

    std::lock_guard<std::mutex> lock(state_mutex_);
    for (size_t i = 0; i < 6; ++i) {
        state_.pos[i] = positions[i] * joint_sign_[i];
        state_.vel[i] = velocities[i] * joint_sign_[i];
        state_.eff[i] = efforts[i] * joint_sign_[i];
        state_.error_codes[i] = error_codes[i];
        state_.temp_mos[i] = temp_mos[i];
        state_.temp_rotor[i] = temp_rotor[i];
    }
}

void ArmRobot::check_runtime_safety() {
    check_feedback_stale();
    check_motor_errors();
    check_motor_temps();

    if (estop_latch_) {
        return;
    }

    check_runtime_joint_limits();
    check_velocity_limits();
}

void ArmRobot::check_feedback_stale() {
    auto ages = motor_chain_->get_feedback_ages();
    bool any_stale = false;
    bool any_warning = false;

    for (size_t i = 0; i < 6; ++i) {
        if (ages[i] > stale_feedback_estop_s_) {
            any_stale = true;
        } else if (ages[i] > stale_feedback_warn_s_) {
            any_warning = true;
        }
    }

    if (any_stale) {
        throw std::runtime_error("CAN feedback stale - bus or motor feedback may be down");
    }

    if (any_warning) {
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration<double>(now - last_stale_warn_time_).count() > 1.0) {
            std::cerr << "[ArmRobot] Warning: CAN feedback delayed" << std::endl;
            last_stale_warn_time_ = now;
        }
    }
}

void ArmRobot::check_motor_errors() {
    std::lock_guard<std::mutex> lock(state_mutex_);

    for (size_t i = 0; i < 6; ++i) {
        int code = state_.error_codes[i];
        if (code != 0x0 && code != 0x1) {
            // Hardware fault
            throw std::runtime_error("Motor fault on joint " + std::to_string(i + 1) +
                                     ": error_code=0x" + std::to_string(code));
        }
    }
}

void ArmRobot::check_motor_temps() {
    std::lock_guard<std::mutex> lock(state_mutex_);

    for (size_t i = 0; i < 6; ++i) {
        if (state_.temp_mos[i] > temp_mos_estop_c_) {
            throw std::runtime_error("MOS over-temperature on joint " +
                                     std::to_string(i + 1));
        }
        if (state_.temp_rotor[i] > temp_rotor_estop_c_) {
            throw std::runtime_error("Motor coil over-temperature on joint " +
                                     std::to_string(i + 1));
        }
    }

    // Warning check
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now - last_temp_warn_time_).count() > 1.0) {
        for (size_t i = 0; i < 6; ++i) {
            if (state_.temp_mos[i] > temp_mos_warn_c_ ||
                state_.temp_rotor[i] > temp_rotor_warn_c_) {
                std::cerr << "[ArmRobot] Warning: Motor temperature high on joint "
                          << i + 1 << std::endl;
                last_temp_warn_time_ = now;
                break;
            }
        }
    }
}

void ArmRobot::check_velocity_limits() {
    std::lock_guard<std::mutex> lock(state_mutex_);

    for (size_t i = 0; i < 6; ++i) {
        if (std::abs(state_.vel[i]) > vel_limit_[i]) {
            throw std::runtime_error("Joint velocity limit exceeded on joint " +
                                     std::to_string(i + 1));
        }
    }
}

void ArmRobot::check_runtime_joint_limits() {
    if (!joint_limits_) {
        return;
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    const double buffer = 0.15;  // rad

    for (size_t i = 0; i < 6; ++i) {
        double pos = state_.pos[i];
        double lo = (*joint_limits_)[i].first;
        double hi = (*joint_limits_)[i].second;

        if (pos < lo - buffer || pos > hi + buffer) {
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration<double>(now - last_limit_warn_time_).count() > 1.0) {
                std::cerr << "[ArmRobot] Warning: Joint " << i + 1
                          << " position outside limits: " << pos
                          << " not in [" << lo << ", " << hi << "]" << std::endl;
                last_limit_warn_time_ = now;
            }
        }
    }
}

std::optional<JointVector> ArmRobot::clip_joint_pos(const JointVector& pos,
                                                    const JointVector* q_current,
                                                    double tol_rad) {
    if (!joint_limits_) {
        return pos;
    }

    JointVector result = pos;
    bool rejected = false;

    for (size_t i = 0; i < 6; ++i) {
        double lo = (*joint_limits_)[i].first;
        double hi = (*joint_limits_)[i].second;
        double original = pos[i];

        // S¹-topology circle-unwrap: map target to the nearest equivalent on the circle
        // Joint angles live on a circle. The Leader may represent the same physical
        // pose with different 2π-shifted values (e.g. wrap across ±π). Find the
        // equivalent angle closest to the current command position so a small
        // physical movement never produces a full-stroke jump after clipping.
        if (q_current != nullptr) {
            double diff = original - (*q_current)[i];
            double k = std::round(diff / (2.0 * M_PI));
            original = original - k * 2.0 * M_PI;
        }

        if (original < lo - tol_rad || original > hi + tol_rad) {
            rejected = true;
            break;
        } else if (original < lo || original > hi) {
            result[i] = std::clamp(original, lo, hi);
        } else {
            result[i] = original;
        }
    }

    if (rejected) {
        return std::nullopt;
    }

    return result;
}

JointVector ArmRobot::validate_joint_pos(const JointVector& pos, double tolerance_rad) {
    auto result = clip_joint_pos(pos, nullptr, tolerance_rad);
    if (!result) {
        throw std::runtime_error("Target joint position out of limits");
    }
    return *result;
}

void ArmRobot::set_fault_state(ControlState state, const std::string& code,
                                const std::string& reason) {
    control_state_ = state;
    fault_code_ = code;
    fault_reason_ = reason;
}

void ArmRobot::send_zero_torque_and_disable() {
    JointVector zero = {};
    try {
        motor_chain_->send_commands(zero, zero, zero, zero, zero);
    } catch (...) {
        // Ignore errors during emergency shutdown
    }

    try {
        if (!motor_chain_->disable_all()) {
            std::cerr << "[ArmRobot] ERROR: failed to disable all motors — "
                         "check motor state manually!" << std::endl;
        }
    } catch (...) {
        // Ignore errors during emergency shutdown
    }

    if (gripper_) {
        try {
            gripper_->disable();
        } catch (...) {
            // Ignore errors during emergency shutdown
        }
    }
}

double ArmRobot::now_seconds() const {
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration<double>(now.time_since_epoch()).count();
}

} // namespace a1z
