#pragma once

#include "a1z/types.hpp"
#include "a1z/mixed_motor_chain.hpp"
#include "a1z/gravity_model.hpp"
#include "a1z/gripper.hpp"
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace a1z {

/**
 * @brief Recoverable control fault (transient CAN error, stale feedback,
 * inverse-dynamics failure, ...).
 *
 * The control loop holds the last safely transmitted command and retries;
 * only if the fault persists does it escalate to a latched FAULT_HOLD.
 * Matches the Python SDK's RecoverableControlFault policy.
 */
class RecoverableControlFault : public std::runtime_error {
public:
    explicit RecoverableControlFault(const std::string& msg,
                                     std::string fault_code = "CONTROL_FAULT")
        : std::runtime_error(msg), fault_code_(std::move(fault_code)) {}
    const std::string& fault_code() const { return fault_code_; }

private:
    std::string fault_code_;
};

/**
 * @brief Hard safety fault (motor fault, over-temperature, non-finite
 * feedback, ...). The control loop disables the motors immediately.
 */
class HardSafetyFault : public std::runtime_error {
public:
    explicit HardSafetyFault(const std::string& msg) : std::runtime_error(msg) {}
};

/**
 * @brief Joint command structure.
 */
struct JointCommand {
    JointVector pos = {};
    JointVector vel = {};
    JointVector acc = {};
    JointVector kp = {};
    JointVector kd = {};
    JointVector torque_ff = {};
};

/**
 * @brief Joint state structure.
 */
struct JointState {
    JointVector pos = {};
    JointVector vel = {};
    JointVector eff = {};
    std::array<int, 6> error_codes = {};
    std::array<double, 6> temp_mos = {};
    std::array<double, 6> temp_rotor = {};
};

/**
 * @brief A1Z 6-DOF arm robot with gravity compensation.
 *
 * Manages a MixedMotorChain, a gravity model, and runs a background
 * control loop for gravity compensation + PD control.
 */
class ArmRobot {
public:
    /**
     * @brief Construct an ArmRobot.
     * @param motor_chain MixedMotorChain instance
     * @param gravity_model GravityModel instance (optional, can be nullptr)
     * @param gripper Gripper instance (optional, can be nullptr)
     * @param control_freq_hz Control loop frequency (default 250 Hz)
     * @param min_freq_hz Minimum frequency before emergency stop
     */
    ArmRobot(std::shared_ptr<MixedMotorChain> motor_chain,
             std::shared_ptr<GravityModel> gravity_model = nullptr,
             std::shared_ptr<Gripper> gripper = nullptr,
             int control_freq_hz = 250,
             double min_freq_hz = 80.0);

    ~ArmRobot();

    // Delete copy
    ArmRobot(const ArmRobot&) = delete;
    ArmRobot& operator=(const ArmRobot&) = delete;

    /**
     * @brief Start the control loop.
     * @param initial_kp Optional initial kp gains
     * @param initial_kd Optional initial kd gains
     */
    void start(const JointVector* initial_kp = nullptr,
               const JointVector* initial_kd = nullptr);

    /**
     * @brief Stop the control loop and disable motors.
     */
    void stop();

    /**
     * @brief Check if control loop is running.
     */
    bool is_running() const { return running_; }

    /**
     * @brief Get current control state.
     */
    ControlState control_state() const;

    /**
     * @brief Set target joint state.
     * @param cmd Joint command with pos, vel, and optional kp, kd, acc, torque_ff
     * @return true if command was accepted
     */
    bool command_joint_state(const JointCommand& cmd);

    /**
     * @brief Set target joint positions with default gains.
     * @param pos Target positions (rad)
     * @return true if command was accepted
     */
    bool command_joint_pos(const JointVector& pos);

    /**
     * @brief Get current joint state.
     */
    JointState get_joint_state();

    /**
     * @brief Emergency stop (soft estop - hold position with gravity comp).
     */
    void estop();

    /**
     * @brief Release from estop.
     */
    void release();

    /**
     * @brief Set gravity compensation factor [0.0, 1.0].
     */
    void set_gravity_comp_factor(double factor);

    /**
     * @brief Set joint sign convention (motor -> URDF frame).
     */
    void set_joint_sign(const JointVector& sign);

    /**
     * @brief Set joint limits.
     */
    void set_joint_limits(const std::array<std::pair<double, double>, 6>& limits);

    /**
     * @brief Switch between zero-gravity (floating) and position-hold mode.
     *
     * In zero-gravity mode kp=0 and kd=default*0.5 so the arm follows
     * gravity compensation only; in position-hold mode the default PD
     * gains are used. Can be called before start() or at runtime.
     */
    void set_gravity_mode(bool enabled);

    /**
     * @brief Enable/disable recording for teaching.
     */
    void set_recording(bool enable);

    /**
     * @brief Get recorded trajectory.
     */
    std::vector<std::pair<double, JointVector>> get_recording() const;

    /**
     * @brief Clear recorded trajectory.
     */
    void clear_recording();

private:
    // Control loop
    void control_loop();
    void update();

    // State reading
    void read_state();

    // Safety checks
    void check_runtime_safety();
    void check_feedback_stale();
    void check_motor_errors();
    void check_motor_temps();
    void check_velocity_limits();
    void check_runtime_joint_limits();

    // Command validation
    std::optional<JointVector> clip_joint_pos(const JointVector& pos,
                                               const JointVector* q_current = nullptr,
                                               double tol_rad = 0.05);
    JointVector validate_joint_pos(const JointVector& pos, double tolerance_rad = 0.05);

    // Hardware interface
    std::shared_ptr<MixedMotorChain> motor_chain_;
    std::shared_ptr<GravityModel> gravity_model_;
    std::shared_ptr<Gripper> gripper_;

    // Control parameters
    int control_freq_hz_;
    double control_period_s_;
    double min_freq_hz_;
    double gravity_comp_factor_ = 1.0;
    bool zero_gravity_mode_ = false;

    // Joint configuration (lemo branch defaults)
    JointVector joint_sign_ = {1.0, 1.0, -1.0, 1.0, -1.0, 1.0};
    JointVector gravity_torque_scale_ = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    JointVector max_gravity_torque_ = {50.0, 50.0, 50.0, 24.0, 10.0, 10.0};
    JointVector torque_clip_ = {70.0, 70.0, 70.0, 27.0, 10.0, 10.0};
    JointVector default_kp_ = {146.9, 62.95, 89.24, 120.0, 40.0, 100.0};
    JointVector default_kd_ = {5.0, 5.0, 5.0, 2.078, 1.506, 1.255};
    std::optional<std::array<std::pair<double, double>, 6>> joint_limits_;

    // Safety limits
    JointVector vel_limit_ = {12.0, 12.0, 12.0, 7.0, 20.0, 20.0};
    double temp_mos_warn_c_ = 70.0;
    double temp_mos_estop_c_ = 85.0;
    double temp_rotor_warn_c_ = 75.0;
    double temp_rotor_estop_c_ = 90.0;
    double stale_feedback_warn_s_ = 0.05;
    double stale_feedback_estop_s_ = 0.2;

    // State
    JointState state_;
    JointCommand command_;
    std::atomic<bool> running_{false};
    std::atomic<bool> stop_requested_{false};
    std::atomic<bool> estop_latch_{false};
    std::atomic<bool> commands_blocked_{true};

    // Threading
    std::thread control_thread_;
    mutable std::mutex state_mutex_;
    mutable std::mutex command_mutex_;
    mutable std::mutex lifecycle_mutex_;

    // Timing
    std::chrono::steady_clock::time_point last_feedback_time_;
    std::chrono::steady_clock::time_point last_temp_warn_time_;
    std::chrono::steady_clock::time_point last_stale_warn_time_;
    std::chrono::steady_clock::time_point last_limit_warn_time_;
    std::chrono::steady_clock::time_point last_clip_warn_time_;

    // Recording
    bool recording_ = false;
    std::vector<std::pair<double, JointVector>> record_buffer_;
    mutable std::mutex record_mutex_;
    double record_period_s_ = 0.02;  // 50 Hz
    std::chrono::steady_clock::time_point last_record_time_;

    // Fault state
    ControlState control_state_ = ControlState::STOPPED;
    std::string fault_code_;
    std::string fault_reason_;

    // Fault policy (Python parity): a recoverable error holds the last
    // safely sent command and resends the cached gravity-only hold frame;
    // only after RECOVERABLE_TIMEOUT_S of continuous failure does it latch
    // FAULT_HOLD (position hold at the measured pose, loop stays alive).
    // Hard faults still disable immediately — the arm has no brake, so
    // dropping power is reserved for confirmed hardware/safety faults.
    static constexpr double RECOVERABLE_TIMEOUT_S = 0.2;

    // Motor-space frame cached after every successful send; resent while a
    // recoverable fault is being retried.
    struct MotorCommandFrame {
        JointVector pos = {};
        JointVector vel = {};
        JointVector kp = {};
        JointVector kd = {};
        JointVector torque = {};
    };

    void write_last_sent_hold_locked();  // caller owns command_mutex_
    void resend_last_safe_hold();
    void hold_position(const std::string& reason,
                       const std::string& fault_code);
    // Returns true if the loop should continue. Sets recoverable_since on
    // first error; clears semantics handled by caller.
    bool handle_control_exception(const std::exception& e,
                                  std::chrono::steady_clock::time_point& recoverable_since,
                                  bool& recoverable_active);

    // Helper
    double now_seconds() const;
    void set_fault_state(ControlState state, const std::string& code, const std::string& reason);
    void send_zero_torque_and_disable();

    // Last successfully transmitted command and its gravity-only fallback
    JointCommand last_sent_command_;
    MotorCommandFrame last_safe_hold_frame_;
    bool has_sent_command_ = false;
};

} // namespace a1z
