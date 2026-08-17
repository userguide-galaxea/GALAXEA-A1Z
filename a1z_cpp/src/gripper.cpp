#include "a1z/gripper.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

namespace a1z {

Gripper::Gripper(std::shared_ptr<MotorB> motor,
                 double open_rad, double close_rad,
                 double max_torque, double max_vel)
    : motor_(std::move(motor))
    , open_rad_(open_rad)
    , close_rad_(close_rad)
    , max_vel_(max_vel) {
    i_des_ = std::clamp(max_torque / MOTOR_PEAK_TORQUE_NM, 0.0, 1.0);
}

void Gripper::enable() {
    motor_->clear_error();
    motor_->enable();
    // Switch to force-position hybrid mode
    motor_->set_ctrl_mode(4);

    // Read actual position and feed as first hybrid frame to prevent snap
    auto fb = motor_->last_feedback();
    if (fb) {
        motor_->send_hybrid_command(fb->position, 0.0, 0.0);
    }
}

void Gripper::disable() {
    motor_->disable();
}

bool Gripper::home(double timeout_s) {
    double i_home = GRIPPER_HOME_TORQUE_NM / MOTOR_PEAK_TORQUE_NM;
    std::cout << "[Gripper] Homing to open (" << open_rad_ << " rad)..." << std::endl;

    auto start = std::chrono::steady_clock::now();
    bool reached = false;

    while (true) {
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - start).count();
        if (elapsed > timeout_s) break;

        motor_->send_hybrid_command(open_rad_, GRIPPER_HOME_VEL, i_home);

        // Try to receive feedback
        // Note: This requires the CAN interface to be in a state where we can read
        // In practice, the caller should ensure feedback is being drained
        auto fb = motor_->last_feedback();
        if (fb && std::abs(fb->position - open_rad_) < 0.1) {
            std::cout << "[Gripper] Open at " << fb->position << " rad ("
                      << elapsed << "s)" << std::endl;
            reached = true;
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (!reached) {
        std::cerr << "[Gripper] Home timed out after " << timeout_s << "s" << std::endl;
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        cmd_norm_ = 1.0;
    }
    return reached;
}

void Gripper::command(double value) {
    std::lock_guard<std::mutex> lock(mutex_);
    cmd_norm_ = std::clamp(value, 0.0, 1.0);
}

double Gripper::get_pos() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return cmd_norm_;
}

double Gripper::get_feedback_norm() const {
    auto fb = motor_->last_feedback();
    if (!fb) {
        return get_pos();
    }
    double span = open_rad_ - close_rad_;  // negative (open < close)
    double norm = (fb->position - close_rad_) / span;
    return std::clamp(norm, 0.0, 1.0);
}

void Gripper::step() {
    double norm;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        norm = cmd_norm_;
    }
    double target_rad = close_rad_ + norm * (open_rad_ - close_rad_);
    motor_->send_hybrid_command(target_rad, max_vel_, i_des_);
}

void Gripper::free_drive_step() {
    auto fb = motor_->last_feedback();
    double pos = fb ? fb->position : open_rad_;
    motor_->send_hybrid_command(pos, 0.0, 0.0);

    std::lock_guard<std::mutex> lock(mutex_);
    cmd_norm_ = std::clamp((pos - close_rad_) / (open_rad_ - close_rad_), 0.0, 1.0);
}

} // namespace a1z
