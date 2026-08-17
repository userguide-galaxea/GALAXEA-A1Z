#pragma once

#include "a1z/motor_a_driver.hpp"
#include "a1z/motor_b_driver.hpp"
#include "a1z/transport.hpp"
#include <array>
#include <map>
#include <memory>
#include <vector>

namespace a1z {

/**
 * @brief Unified motor chain managing MotorA + MotorB motors.
 *
 * Joints are ordered: MotorA motors first (indices 0..n_motor_a-1),
 * then MotorB motors (indices n_motor_a..n_motor_a+n_motor_b-1).
 */
class MixedMotorChain {
public:
    /**
     * @brief Construct a mixed motor chain.
     * @param motor_a_list List of MotorA instances
     * @param motor_b_list List of MotorB instances
     * @param motor_a_joint_indices Joint indices for MotorA motors
     * @param motor_b_joint_indices Joint indices for MotorB motors
     * @param motor_a_kt Torque constant for MotorA current->torque conversion
     * @param inter_cmd_gap_s Delay (s) inserted before each command frame send
     *        after the first in send_commands (SOP-05/SOP-06 CAN pacing)
     */
    MixedMotorChain(std::vector<std::shared_ptr<MotorA>> motor_a_list,
                    std::vector<std::shared_ptr<MotorB>> motor_b_list,
                    std::vector<int> motor_a_joint_indices,
                    std::vector<int> motor_b_joint_indices,
                    double motor_a_kt = 2.8,
                    double inter_cmd_gap_s = 0.0);

    /**
     * @brief Get total number of motors/joints.
     */
    int num_motors() const { return num_motors_; }

    /**
     * @brief Enable all motors.
     */
    void enable_all();

    /**
     * @brief Disable all motors.
     * @return true if all disable commands sent successfully
     */
    bool disable_all();

    /**
     * @brief Drain pending CAN messages and update feedback state.
     * @param timeout_ms Maximum time to spend draining
     * @return Number of valid feedback messages processed
     */
    int drain_and_update(int timeout_ms = 1);

    /**
     * @brief Send MIT commands to all motors.
     */
    void send_commands(const JointVector& pos, const JointVector& vel,
                       const JointVector& kp, const JointVector& kd,
                       const JointVector& torque, int motor_a_mode = 0);

    /**
     * @brief Get current joint positions.
     */
    JointVector get_positions() const;

    /**
     * @brief Get current joint velocities.
     */
    JointVector get_velocities() const;

    /**
     * @brief Get current joint efforts (torques).
     */
    JointVector get_efforts() const;

    /**
     * @brief Get feedback ages in seconds for each joint.
     */
    std::array<double, 6> get_feedback_ages() const;

    /**
     * @brief Get per-joint motor error codes.
     */
    std::array<int, 6> get_error_codes() const;

    /**
     * @brief Get temperatures (mos, rotor) for each joint.
     */
    std::pair<std::array<double, 6>, std::array<double, 6>> get_temperatures() const;

    /**
     * @brief Register external motor for feedback routing (e.g., gripper).
     */
    void register_external_motor(std::shared_ptr<MotorB> motor);

    /**
     * @brief Reset feedback health tracking.
     */
    void reset_feedback_health();

    /**
     * @brief Check if all joints have fresh feedback.
     * @param max_age_s Maximum allowed feedback age
     * @return true if all joints have feedback younger than max_age_s
     */
    bool all_feedback_fresh(double max_age_s = 0.2) const;

    /**
     * @brief Get inter-command gap in seconds.
     */
    double inter_cmd_gap_s() const { return inter_cmd_gap_s_; }

private:
    struct MotorEntry {
        enum class Type { MotorA, MotorB };
        Type type;
        std::shared_ptr<void> motor;
        int joint_index;
    };

    std::vector<std::shared_ptr<MotorA>> motor_a_list_;
    std::vector<std::shared_ptr<MotorB>> motor_b_list_;
    std::vector<int> motor_a_joint_indices_;
    std::vector<int> motor_b_joint_indices_;
    double motor_a_kt_;
    double inter_cmd_gap_s_;
    int num_motors_;

    // State arrays
    JointVector positions_ = {};
    JointVector velocities_ = {};
    JointVector efforts_ = {};
    std::array<double, 6> last_feedback_time_ = {};

    // CAN ID -> motor mapping for feedback dispatch
    std::map<int, MotorEntry> motor_id_map_;

    std::shared_ptr<Transport> transport_;

    void dispatch_feedback(const CanFrame& frame);
    double now_seconds() const;
};

} // namespace a1z
