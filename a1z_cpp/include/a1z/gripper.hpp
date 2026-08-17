#pragma once

#include "a1z/motor_b_driver.hpp"
#include <memory>
#include <mutex>

namespace a1z {

// Gripper hardware configuration
constexpr double GRIPPER_CLOSE_RAD = 2.87;   // rad, fully closed
constexpr double GRIPPER_OPEN_RAD = -2.87;   // rad, fully open
constexpr double GRIPPER_MAX_VEL = 10.0;     // rad/s
constexpr double GRIPPER_HOME_VEL = 5.0;     // rad/s
constexpr double GRIPPER_HOME_TORQUE_NM = 0.5;
constexpr int GRIPPER_CAN_ID = 7;
constexpr double MOTOR_PEAK_TORQUE_NM = 11.0;

/**
 * @brief Gripper control using MotorB force-position hybrid mode.
 *
 * Physical stroke: -2.87 rad (open) → +2.87 rad (closed).
 * External interface uses normalized values: 0.0 = closed, 1.0 = fully open.
 */
class Gripper {
public:
    /**
     * @brief Construct a gripper controller.
     * @param motor MotorB instance for the gripper
     * @param open_rad Open position (rad)
     * @param close_rad Close position (rad)
     * @param max_torque Maximum gripping torque (Nm)
     * @param max_vel Maximum velocity (rad/s)
     */
    explicit Gripper(std::shared_ptr<MotorB> motor,
                     double open_rad = GRIPPER_OPEN_RAD,
                     double close_rad = GRIPPER_CLOSE_RAD,
                     double max_torque = 0.5,
                     double max_vel = GRIPPER_MAX_VEL);

    /**
     * @brief Enable gripper and switch to hybrid control mode.
     */
    void enable();

    /**
     * @brief Disable gripper motor.
     */
    void disable();

    /**
     * @brief Drive gripper to open position and wait for arrival.
     * @param timeout_s Maximum seconds to wait
     * @return true if reached open position
     */
    bool home(double timeout_s = 1.5);

    /**
     * @brief Set gripper target position.
     * @param value Normalized position [0.0 = closed, 1.0 = open]
     */
    void command(double value);

    /**
     * @brief Get current commanded position [0.0, 1.0].
     */
    double get_pos() const;

    /**
     * @brief Get feedback-based normalized position [0.0, 1.0].
     */
    double get_feedback_norm() const;

    /**
     * @brief Send one hybrid command. Call once per control tick.
     */
    void step();

    /**
     * @brief Send zero-current frame for manual movement.
     */
    void free_drive_step();

    /**
     * @brief Get motor instance.
     */
    std::shared_ptr<MotorB> motor() const { return motor_; }

private:
    std::shared_ptr<MotorB> motor_;
    double open_rad_;
    double close_rad_;
    double max_vel_;
    double i_des_;
    double cmd_norm_ = 1.0;  // Start open
    mutable std::mutex mutex_;
};

} // namespace a1z
