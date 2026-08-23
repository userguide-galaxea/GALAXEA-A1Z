#include "a1z/arm_robot.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>

namespace a1z {

namespace {
// Command feedforward caps (Python arm_robot.py module constants
// _MAX_CMD_VEL_RAD_S / _MAX_CMD_ACC_RAD_S2 / _MAX_CMD_KP / _MAX_CMD_KD).
constexpr double MAX_CMD_VEL_RAD_S = 4.0;
constexpr double MAX_CMD_ACC_RAD_S2 = 20.0;
constexpr double MAX_CMD_KP = 200.0;
constexpr double MAX_CMD_KD = 5.0;
} // namespace

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
        std::cerr << "[ArmRobot] Command rejected: robot is blocked (state="
                  << to_string(control_state_)
                  << ", fault_code=" << (fault_code_.empty() ? "-" : fault_code_)
                  << ", reason=" << (fault_reason_.empty() ? "-" : fault_reason_)
                  << ")" << std::endl;
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
    bool ok = command_joint_state(cmd);
    if (ok) {
        // Non-streaming entry: the integral only serves the streaming
        // command_joint_state tracker (Python parity, SOP-09 W4).
        reset_integral_state();
    }
    return ok;
}

JointState ArmRobot::get_joint_state() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return state_;
}

void ArmRobot::estop() {
    // Python parity: atomically pin the target to the measured pose with
    // kp=0 and halved kd; gravity compensation keeps running so the arm
    // does not collapse under load.
    {
        std::lock_guard<std::mutex> state_lock(state_mutex_);
        std::lock_guard<std::mutex> cmd_lock(command_mutex_);
        command_.pos = state_.pos;
        command_.vel.fill(0.0);
        command_.acc.fill(0.0);
        command_.kp.fill(0.0);
        for (size_t i = 0; i < 6; ++i) {
            command_.kd[i] = default_kd_[i] * 0.5;
        }
        command_.torque_ff.fill(0.0);
    }
    estop_latch_ = true;
    commands_blocked_ = true;
    set_fault_state(ControlState::SOFT_ESTOP, "MANUAL_ESTOP", "manual soft estop");
    reset_integral_state();
    std::cout << "[ArmRobot] Emergency stop engaged" << std::endl;
}

void ArmRobot::release() {
    // Re-anchor the target at the measured pose before accepting commands.
    {
        std::lock_guard<std::mutex> state_lock(state_mutex_);
        std::lock_guard<std::mutex> cmd_lock(command_mutex_);
        command_.pos = state_.pos;
        command_.vel.fill(0.0);
        command_.acc.fill(0.0);
        command_.torque_ff.fill(0.0);
    }
    estop_latch_ = false;
    commands_blocked_ = false;
    set_fault_state(ControlState::RUNNING, "", "");
    reset_integral_state();
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
    reset_integral_state();
}

void ArmRobot::move_joints(const JointVector& target_pos, double speed,
                           const JointVector* kp, const JointVector* kd,
                           std::optional<double> max_jump_rad) {
    if (commands_blocked_) {
        std::cerr << "[ArmRobot] move_joints rejected: robot is blocked (state="
                  << to_string(control_state_)
                  << ", fault_code=" << (fault_code_.empty() ? "-" : fault_code_)
                  << ", reason=" << (fault_reason_.empty() ? "-" : fault_reason_)
                  << ")" << std::endl;
        return;
    }
    // Minimum-jerk peak velocity is 1.875 x average. Reject upfront so we
    // never generate a feedforward that the update() check would refuse.
    if (!std::isfinite(speed) || speed <= 0.0) {
        throw std::invalid_argument("move_joints speed must be > 0");
    }
    if (speed * 1.875 > MAX_CMD_VEL_RAD_S) {
        throw std::invalid_argument(
            "move_joints speed exceeds feedforward cap: peak vel " +
            std::to_string(speed * 1.875) + " > " +
            std::to_string(MAX_CMD_VEL_RAD_S) + " rad/s");
    }

    JointVector target = validate_joint_pos(target_pos);

    // Gain overrides validated before they reach shared state.
    JointVector kp_cmd = kp != nullptr ? *kp : default_kp_;
    JointVector kd_cmd = kd != nullptr ? *kd : default_kd_;
    for (size_t i = 0; i < 6; ++i) {
        if (!std::isfinite(kp_cmd[i]) || kp_cmd[i] < 0.0 ||
            kp_cmd[i] > MAX_CMD_KP) {
            throw std::invalid_argument(
                "move_joints kp must be within [0, " +
                std::to_string(MAX_CMD_KP) + "]");
        }
        if (!std::isfinite(kd_cmd[i]) || kd_cmd[i] < 0.0 ||
            kd_cmd[i] > MAX_CMD_KD) {
            throw std::invalid_argument(
                "move_joints kd must be within [0, " +
                std::to_string(MAX_CMD_KD) + "]");
        }
    }

    // The jump guard uses the *measured* position — this is what
    // max_jump_rad actually protects (e.g. catching elbow-flipped IK
    // against reality).
    JointVector measured = get_joint_state().pos;
    if (max_jump_rad.has_value()) {
        double limit = *max_jump_rad;
        if (!std::isfinite(limit) || limit < 0.0) {
            throw std::invalid_argument(
                "max_jump_rad must be finite and >= 0");
        }
        for (size_t i = 0; i < 6; ++i) {
            double jump = std::abs(target[i] - measured[i]);
            if (jump > limit) {
                throw std::invalid_argument(
                    "Target too far from current position (max_jump_rad=" +
                    std::to_string(limit) + "), refusing to move: joint " +
                    std::to_string(i + 1) + " off by " +
                    std::to_string(jump) + " rad");
            }
        }
    }

    // Trajectory start uses the *last commanded* position so back-to-back
    // move_joints calls keep command-space continuity. Using the measured
    // position here would inject a backwards step equal to the PD tracking
    // error between consecutive moves, which the PD loop sees as a jerk.
    JointVector current;
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        if (commands_blocked_) {
            std::cerr << "[ArmRobot] move_joints rejected while entering a fault"
                      << std::endl;
            return;
        }
        current = command_.pos;
    }

    JointVector delta;
    double max_dist = 0.0;
    for (size_t i = 0; i < 6; ++i) {
        delta[i] = target[i] - current[i];
        max_dist = std::max(max_dist, std::abs(delta[i]));
    }
    if (max_dist < 0.001) {
        return;
    }

    // Minimum-jerk peak acc is about 5.7735*delta/duration^2. A re-command of
    // an already-reached pose (tiny delta) or a high-speed short-distance
    // move would otherwise spike past the feedforward cap. Clamp duration
    // from below by (a) the fixed MIN_MOVE_DURATION_S and (b)
    // sqrt(6*delta/MAX_ACC) — the slight over-estimate of the 5.7735
    // coefficient gives ~4% acc headroom for float epsilon. The vel cap is
    // already guaranteed by the speed pre-check above.
    const double MIN_MOVE_DURATION_S = 0.3;
    double acc_dur = std::sqrt(6.0 * max_dist / MAX_CMD_ACC_RAD_S2);
    double duration = std::max({max_dist / speed, acc_dur, MIN_MOVE_DURATION_S});
    double dt = control_period_s_;
    int steps = std::max(1, static_cast<int>(duration / dt));

    for (int step = 1; step <= steps; ++step) {
        if (commands_blocked_) {
            std::cerr << "[ArmRobot] move_joints aborted mid-trajectory: "
                         "estop engaged" << std::endl;
            return;
        }
        double t = static_cast<double>(step) / steps;
        // Minimum-jerk profile: pos and vel are zero at t=0 and t=1.
        double t2 = t * t;
        double t3 = t2 * t;
        double t4 = t3 * t;
        double t5 = t4 * t;
        double alpha = 10.0 * t3 - 15.0 * t4 + 6.0 * t5;
        double alpha_dot = (30.0 * t2 - 60.0 * t3 + 30.0 * t4) / duration;
        double alpha_ddot = (60.0 * t - 180.0 * t2 + 120.0 * t3) /
                            (duration * duration);
        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            if (commands_blocked_) {
                std::cerr << "[ArmRobot] move_joints aborted while entering a "
                             "fault" << std::endl;
                return;
            }
            for (size_t i = 0; i < 6; ++i) {
                command_.pos[i] = current[i] + alpha * delta[i];
                command_.vel[i] = alpha_dot * delta[i];
                command_.acc[i] = alpha_ddot * delta[i];
            }
            command_.kp = kp_cmd;
            command_.kd = kd_cmd;
        }
        std::this_thread::sleep_for(std::chrono::duration<double>(dt));
    }

    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        if (commands_blocked_) {
            return;
        }
        command_.pos = target;
        command_.vel.fill(0.0);
        command_.acc.fill(0.0);
    }
    reset_integral_state();
}

void ArmRobot::set_gripper_free_drive(bool enabled) {
    if (!gripper_) {
        return;
    }
    gripper_free_drive_ = enabled;
}

void ArmRobot::reset_integral_state() {
    {
        std::lock_guard<std::mutex> cmd_lock(command_mutex_);
        if (integrator_) {
            integrator_->reset();
        }
    }
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    last_tau_i_.fill(0.0);
}

void ArmRobot::set_integral_config(std::optional<IntegralConfig> cfg) {
    // Swap under the command lock so a mid-run level switch never mixes old
    // accumulator state with new gains.
    {
        std::lock_guard<std::mutex> cmd_lock(command_mutex_);
        if (cfg.has_value()) {
            integrator_ =
                std::make_shared<JointErrorIntegrator>(*cfg, control_period_s_);
        } else {
            integrator_.reset();
        }
    }
    reset_integral_state();
}

void ArmRobot::reset_integral() {
    reset_integral_state();
}

void ArmRobot::set_coulomb_config(std::optional<CoulombConfig> cfg) {
    // Same command-lock discipline as set_integral_config: the config is
    // read on every control cycle, so a mid-run switch must never mix a
    // partially-updated config into a command.
    std::lock_guard<std::mutex> cmd_lock(command_mutex_);
    coulomb_config_ = std::move(cfg);
    coulomb_ff_.reset();
}

void ArmRobot::set_coulomb_ff(std::optional<JointVector> ff) {
    std::lock_guard<std::mutex> cmd_lock(command_mutex_);
    coulomb_ff_ = ff;
    coulomb_config_.reset();
}

JointVector ArmRobot::last_tau_i() const {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    return last_tau_i_;
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
    const int MAX_SLOW_PERIODS = 3;          // Position hold after 3 consecutive slow periods

    auto last_check_time = std::chrono::steady_clock::now();
    int iteration_count = 0;
    int consecutive_slow = 0;
    bool recoverable_active = false;
    auto recoverable_since = std::chrono::steady_clock::now();

    while (!stop_requested_) {
        auto loop_start = std::chrono::steady_clock::now();

        try {
            update();
            recoverable_active = false;
        } catch (const HardSafetyFault& e) {
            // Confirmed hardware/safety fault: the only case where cutting
            // power is safer than holding (the arm has no brake).
            std::cerr << "[ArmRobot] Hard safety fault: " << e.what()
                      << " — disabling motors" << std::endl;
            send_zero_torque_and_disable();
            running_ = false;
            set_fault_state(ControlState::HARD_DISABLED, "SAFETY_HARD_FAULT", e.what());
            return;
        } catch (const std::exception& e) {
            // Recoverable: hold the last safely sent command and retry;
            // escalate to a latched position hold if it persists.
            handle_control_exception(e, recoverable_since, recoverable_active);
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
                    // Python parity: latch a position hold, keep the loop alive
                    // instead of disabling the motors.
                    hold_position("Frequency below " + std::to_string(min_freq_hz_) +
                                      " Hz for " +
                                      std::to_string((int)(consecutive_slow * FREQ_CHECK_INTERVAL)) + "s",
                                  "CONTROL_FREQUENCY_LOW");
                    consecutive_slow = 0;
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

    // 4) Get current command (plus the feedforward configs under the same
    // lock — a mid-run config swap must never mix into a command)
    JointCommand cmd;
    std::shared_ptr<JointErrorIntegrator> integrator;
    std::optional<CoulombConfig> coulomb_config;
    std::optional<JointVector> coulomb_ff;
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        cmd = command_;
        integrator = integrator_;
        coulomb_config = coulomb_config_;
        coulomb_ff = coulomb_ff_;
    }

    // 5) Validate command/feedback finiteness (Python parity)
    for (size_t i = 0; i < 6; ++i) {
        if (!std::isfinite(cmd.pos[i]) || !std::isfinite(cmd.vel[i]) ||
            !std::isfinite(cmd.acc[i]) || !std::isfinite(cmd.kp[i]) ||
            !std::isfinite(cmd.kd[i]) || !std::isfinite(cmd.torque_ff[i])) {
            throw RecoverableControlFault(
                "Non-finite value reached the internal joint command",
                "CMD_NON_FINITE");
        }
        if (!std::isfinite(state_.pos[i])) {
            throw HardSafetyFault("Non-finite motor feedback position");
        }
    }

    // 6) Compute inverse dynamics (gravity compensation)
    JointVector tau_id = {};
    if (gravity_model_ && gravity_comp_factor_ > 0.0) {
        try {
            tau_id = gravity_model_->compute_inverse_dynamics(
                state_.pos, cmd.vel, cmd.acc
            );
        } catch (const std::exception& e) {
            throw RecoverableControlFault(
                std::string("Inverse dynamics computation failed: ") + e.what(),
                "ID_FAILED");
        }
        for (size_t i = 0; i < 6; ++i) {
            if (!std::isfinite(tau_id[i])) {
                throw RecoverableControlFault(
                    "Inverse dynamics returned a non-finite vector", "ID_INVALID");
            }
            // Python treats oversized model torque as a fault (model/pose
            // anomaly), not as a value to silently clip.
            if (std::abs(tau_id[i]) > max_gravity_torque_[i]) {
                throw RecoverableControlFault(
                    "Inverse dynamics torque too large on joint " +
                        std::to_string(i + 1),
                    "ID_TORQUE_LARGE");
            }
            tau_id[i] *= gravity_torque_scale_[i] * gravity_comp_factor_;
        }
    }

    // 7) Error-integral feedforward (SOP-09 W2). e = q_des - q_meas, both in
    // the URDF frame. Only accumulates on the streaming tracker and never
    // while estopped; steps at the control rate. tau_i (already clamped to
    // +/-tau_i_max inside the integrator) joins the torque sum below — ahead
    // of x joint_sign and torque_clip, so it is frame-correct and the global
    // clip stays the mechanical backstop for a runaway (Python parity:
    // _send_command_and_cache_hold_locked).
    JointVector tau_i = {};
    if (integrator && !estop_latch_) {
        JointVector e;
        for (size_t i = 0; i < 6; ++i) {
            e[i] = cmd.pos[i] - state_.pos[i];
        }
        tau_i = integrator->step(e, cmd.vel);
    }
    {
        std::lock_guard<std::mutex> state_lock(state_mutex_);
        last_tau_i_ = tau_i;
    }

    // 8) Coulomb friction feedforward (S1): two paths — CoulombConfig
    // (tanh-smoothed) or bare array (hard sign, legacy); zero while estopped.
    JointVector tau_c = {};
    if (!estop_latch_) {
        if (coulomb_config.has_value()) {
            tau_c = coulomb_config->compute_tau(cmd.vel);
        } else if (coulomb_ff.has_value()) {
            for (size_t i = 0; i < 6; ++i) {
                double e = cmd.pos[i] - state_.pos[i];
                double s = (e > 0.0) ? 1.0 : (e < 0.0 ? -1.0 : 0.0);
                tau_c[i] = s * (*coulomb_ff)[i];
            }
        }
    }

    // 9) Combine torques
    JointVector motor_torques;
    for (size_t i = 0; i < 6; ++i) {
        double torque_urdf = cmd.torque_ff[i] + tau_i[i] + tau_c[i] + tau_id[i];
        motor_torques[i] = std::clamp(
            torque_urdf * joint_sign_[i],
            -torque_clip_[i], torque_clip_[i]
        );
    }

    // 10) Safe-hold fallback frame: with any feedforward motion in the
    // command, the fallback is gravity-only at the measured pose (Python
    // _static_hold_motor_torque); otherwise it is the frame just sent.
    JointVector safe_torque = motor_torques;
    bool has_feedforward = false;
    for (size_t i = 0; i < 6; ++i) {
        if (cmd.vel[i] != 0.0 || cmd.acc[i] != 0.0 || cmd.torque_ff[i] != 0.0) {
            has_feedforward = true;
            break;
        }
    }
    if (has_feedforward && gravity_model_ && gravity_comp_factor_ > 0.0) {
        JointVector zero = {};
        JointVector tau_hold;
        try {
            tau_hold = gravity_model_->compute_inverse_dynamics(state_.pos, zero, zero);
        } catch (const std::exception& e) {
            throw RecoverableControlFault(
                std::string("Static-hold inverse dynamics failed: ") + e.what(),
                "ID_FAILED");
        }
        for (size_t i = 0; i < 6; ++i) {
            double t = tau_hold[i] * gravity_torque_scale_[i] * gravity_comp_factor_;
            safe_torque[i] = std::clamp(t * joint_sign_[i],
                                        -torque_clip_[i], torque_clip_[i]);
        }
    }

    // 11) Send commands to motor chain
    JointVector motor_pos, motor_vel;
    for (size_t i = 0; i < 6; ++i) {
        motor_pos[i] = cmd.pos[i] * joint_sign_[i];
        motor_vel[i] = cmd.vel[i] * joint_sign_[i];
    }

    motor_chain_->send_commands(motor_pos, motor_vel, cmd.kp, cmd.kd, motor_torques);

    // 12) Cache the sent command and its gravity-only fallback for the
    // recoverable-fault policy
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        last_sent_command_ = cmd;
        has_sent_command_ = true;
        last_safe_hold_frame_.pos = motor_pos;
        last_safe_hold_frame_.vel.fill(0.0);
        last_safe_hold_frame_.kp = cmd.kp;
        last_safe_hold_frame_.kd = cmd.kd;
        last_safe_hold_frame_.torque = safe_torque;
    }

    // 13) Send gripper command
    if (gripper_) {
        // SOP-06: leave a bus slot for the last arm motor's feedback before
        // the gripper frame, same as the Python SDK.
        double gap = motor_chain_->inter_cmd_gap_s();
        if (gap > 0) {
            std::this_thread::sleep_for(std::chrono::duration<double>(gap));
        }
        // Free-drive toggle (Python _gripper_free_drive): zero-torque frame
        // for hand teaching instead of the position-tracking hybrid frame.
        if (gripper_free_drive_) {
            gripper_->free_drive_step();
        } else {
            gripper_->step();
        }
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
        throw RecoverableControlFault(
            "CAN feedback stale - bus or motor feedback may be down",
            "FEEDBACK_STALE");
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
            throw HardSafetyFault("Motor fault on joint " + std::to_string(i + 1) +
                                  ": error_code=0x" + std::to_string(code));
        }
    }
}

void ArmRobot::check_motor_temps() {
    std::lock_guard<std::mutex> lock(state_mutex_);

    for (size_t i = 0; i < 6; ++i) {
        if (state_.temp_mos[i] > temp_mos_estop_c_) {
            throw HardSafetyFault("MOS over-temperature on joint " +
                                  std::to_string(i + 1));
        }
        if (state_.temp_rotor[i] > temp_rotor_estop_c_) {
            throw HardSafetyFault("Motor coil over-temperature on joint " +
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
            throw HardSafetyFault("Joint velocity limit exceeded on joint " +
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

void ArmRobot::write_last_sent_hold_locked() {
    // Caller owns command_mutex_. Restore the last fully transmitted target
    // and PD gains; clear motion/torque feedforward.
    if (has_sent_command_) {
        command_.pos = last_sent_command_.pos;
        command_.kp = last_sent_command_.kp;
        command_.kd = last_sent_command_.kd;
    }
    command_.vel.fill(0.0);
    command_.acc.fill(0.0);
    command_.torque_ff.fill(0.0);
}

void ArmRobot::resend_last_safe_hold() {
    MotorCommandFrame frame;
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        if (!has_sent_command_) {
            throw RecoverableControlFault(
                "No successfully queued safe MIT hold frame is available");
        }
        frame = last_safe_hold_frame_;
    }
    motor_chain_->send_commands(frame.pos, frame.vel, frame.kp, frame.kd,
                                frame.torque);
}

void ArmRobot::hold_position(const std::string& reason,
                             const std::string& fault_code) {
    if (control_state_ == ControlState::HARD_DISABLED ||
        control_state_ == ControlState::HARD_DISABLE_UNCONFIRMED ||
        control_state_ == ControlState::STOPPED) {
        return;
    }
    if (control_state_ == ControlState::FAULT_HOLD && fault_code_ == fault_code) {
        return;  // already holding with the same code
    }

    commands_blocked_ = true;
    {
        // Strict MIT position hold at the measured pose; the control loop
        // stays alive and gravity compensation keeps running.
        std::lock_guard<std::mutex> state_lock(state_mutex_);
        std::lock_guard<std::mutex> cmd_lock(command_mutex_);
        command_.pos = state_.pos;
        command_.vel.fill(0.0);
        command_.acc.fill(0.0);
        command_.kp = default_kp_;
        command_.kd = default_kd_;
        command_.torque_ff.fill(0.0);
    }
    set_fault_state(ControlState::FAULT_HOLD, fault_code, reason);
    reset_integral_state();
    std::cerr << "[ArmRobot] Fault hold engaged (" << fault_code << "): "
              << reason << std::endl;
}

bool ArmRobot::handle_control_exception(
        const std::exception& e,
        std::chrono::steady_clock::time_point& recoverable_since,
        bool& recoverable_active) {
    auto now = std::chrono::steady_clock::now();

    std::string code = "INTERNAL_ERROR";
    if (const auto* rcf = dynamic_cast<const RecoverableControlFault*>(&e)) {
        code = rcf->fault_code();
    }

    bool first_error = !recoverable_active;
    if (first_error) {
        recoverable_active = true;
        recoverable_since = now;
        std::cerr << "[ArmRobot] Recoverable control error (" << code
                  << "); holding last command while retrying: " << e.what()
                  << std::endl;
    }

    // Keep the last fully transmitted target, clear motion feedforward.
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        write_last_sent_hold_locked();
    }

    // Resend the cached gravity-only hold frame without using stale dynamics.
    try {
        resend_last_safe_hold();
    } catch (const std::exception& resend_exc) {
        if (first_error) {
            std::cerr << "[ArmRobot] Cached MIT hold resend failed: "
                      << resend_exc.what() << std::endl;
        }
    }

    double elapsed = std::chrono::duration<double>(now - recoverable_since).count();
    if (elapsed >= RECOVERABLE_TIMEOUT_S) {
        hold_position(std::string(e.what()) + " (no successful control update for " +
                          std::to_string(elapsed) + "s)",
                      code + "_PERSISTENT");
    }
    return true;
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
