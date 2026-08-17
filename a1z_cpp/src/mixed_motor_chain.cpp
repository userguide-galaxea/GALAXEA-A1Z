#include "a1z/mixed_motor_chain.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

namespace a1z {

MixedMotorChain::MixedMotorChain(std::vector<std::shared_ptr<MotorA>> motor_a_list,
                                 std::vector<std::shared_ptr<MotorB>> motor_b_list,
                                 std::vector<int> motor_a_joint_indices,
                                 std::vector<int> motor_b_joint_indices,
                                 double motor_a_kt,
                                 double inter_cmd_gap_s)
    : motor_a_list_(std::move(motor_a_list))
    , motor_b_list_(std::move(motor_b_list))
    , motor_a_joint_indices_(std::move(motor_a_joint_indices))
    , motor_b_joint_indices_(std::move(motor_b_joint_indices))
    , motor_a_kt_(motor_a_kt)
    , inter_cmd_gap_s_(inter_cmd_gap_s) {

    num_motors_ = motor_a_list_.size() + motor_b_list_.size();

    // Initialize feedback times to -inf (never received)
    for (auto& t : last_feedback_time_) {
        t = -std::numeric_limits<double>::infinity();
    }

    // Build motor_id -> entry mapping
    for (size_t i = 0; i < motor_a_list_.size(); ++i) {
        auto& motor = motor_a_list_[i];
        MotorEntry entry;
        entry.type = MotorEntry::Type::MotorA;
        entry.motor = motor;
        entry.joint_index = motor_a_joint_indices_[i];
        motor_id_map_[motor->motor_id()] = entry;
        if (!transport_) transport_ = motor->transport();
    }

    for (size_t i = 0; i < motor_b_list_.size(); ++i) {
        auto& motor = motor_b_list_[i];
        MotorEntry entry;
        entry.type = MotorEntry::Type::MotorB;
        entry.motor = motor;
        entry.joint_index = motor_b_joint_indices_[i];
        motor_id_map_[motor->motor_id()] = entry;
        if (!transport_) transport_ = motor->transport();
    }
}

void MixedMotorChain::enable_all() {
    for (auto& motor : motor_a_list_) {
        motor->enable();
    }
    for (auto& motor : motor_b_list_) {
        // DaMiao motors may have wrong mode in flash - set MIT mode in RAM
        motor->set_ctrl_mode(1);
        motor->enable();
    }
}

bool MixedMotorChain::disable_all() {
    bool success = true;
    // Send twice - a single frame can be missed on a busy bus
    for (int i = 0; i < 2; ++i) {
        for (auto& motor : motor_a_list_) {
            try {
                motor->disable();
            } catch (...) {
                success = false;
            }
        }
        for (auto& motor : motor_b_list_) {
            try {
                motor->disable();
            } catch (...) {
                success = false;
            }
        }
    }
    return success;
}

int MixedMotorChain::drain_and_update(int timeout_ms) {
    if (!transport_) return 0;

    int valid_count = 0;
    auto start = std::chrono::steady_clock::now();
    auto deadline = start + std::chrono::milliseconds(timeout_ms);

    while (std::chrono::steady_clock::now() < deadline) {
        auto frame = transport_->receive(0);
        if (!frame) break;

        dispatch_feedback(*frame);
        valid_count++;
    }

    // Update state arrays from last_feedback
    for (size_t i = 0; i < motor_a_list_.size(); ++i) {
        int idx = motor_a_joint_indices_[i];
        const auto& fb = motor_a_list_[i]->last_feedback();
        if (fb) {
            positions_[idx] = fb->position;
            velocities_[idx] = fb->velocity;
            efforts_[idx] = fb->current * motor_a_kt_;
        }
    }

    for (size_t i = 0; i < motor_b_list_.size(); ++i) {
        int idx = motor_b_joint_indices_[i];
        const auto& fb = motor_b_list_[i]->last_feedback();
        if (fb) {
            positions_[idx] = fb->position;
            velocities_[idx] = fb->velocity;
            efforts_[idx] = fb->torque;
        }
    }

    return valid_count;
}

void MixedMotorChain::dispatch_feedback(const CanFrame& frame) {
    auto it = motor_id_map_.find(frame.id);
    if (it == motor_id_map_.end()) {
        return;
    }

    auto& entry = it->second;
    bool parsed = false;

    if (entry.type == MotorEntry::Type::MotorA) {
        auto motor = std::static_pointer_cast<MotorA>(entry.motor);
        parsed = motor->parse_feedback(frame).has_value();
    } else {
        auto motor = std::static_pointer_cast<MotorB>(entry.motor);
        parsed = motor->parse_feedback(frame).has_value();
    }

    if (parsed && entry.joint_index >= 0 && entry.joint_index < 6) {
        last_feedback_time_[entry.joint_index] = now_seconds();
    }
}

void MixedMotorChain::send_commands(const JointVector& pos, const JointVector& vel,
                                    const JointVector& kp, const JointVector& kd,
                                    const JointVector& torque, int motor_a_mode) {
    bool first = true;

    for (size_t i = 0; i < motor_a_list_.size(); ++i) {
        if (!first && inter_cmd_gap_s_ > 0) {
            std::this_thread::sleep_for(
                std::chrono::duration<double>(inter_cmd_gap_s_));
        }
        first = false;
        int idx = motor_a_joint_indices_[i];
        motor_a_list_[i]->send_mit_command(
            pos[idx], vel[idx], kp[idx], kd[idx], torque[idx], motor_a_mode
        );
    }

    for (size_t i = 0; i < motor_b_list_.size(); ++i) {
        if (!first && inter_cmd_gap_s_ > 0) {
            std::this_thread::sleep_for(
                std::chrono::duration<double>(inter_cmd_gap_s_));
        }
        first = false;
        int idx = motor_b_joint_indices_[i];
        motor_b_list_[i]->send_mit_command(
            pos[idx], vel[idx], kp[idx], kd[idx], torque[idx]
        );
    }
}

JointVector MixedMotorChain::get_positions() const {
    return positions_;
}

JointVector MixedMotorChain::get_velocities() const {
    return velocities_;
}

JointVector MixedMotorChain::get_efforts() const {
    return efforts_;
}

std::array<double, 6> MixedMotorChain::get_feedback_ages() const {
    std::array<double, 6> ages;
    double now = now_seconds();
    for (size_t i = 0; i < 6; ++i) {
        if (std::isinf(last_feedback_time_[i])) {
            ages[i] = std::numeric_limits<double>::infinity();
        } else {
            ages[i] = now - last_feedback_time_[i];
        }
    }
    return ages;
}

std::array<int, 6> MixedMotorChain::get_error_codes() const {
    std::array<int, 6> codes = {};

    for (size_t i = 0; i < motor_a_list_.size(); ++i) {
        int idx = motor_a_joint_indices_[i];
        const auto& fb = motor_a_list_[i]->last_feedback();
        if (fb) codes[idx] = fb->error;
    }

    for (size_t i = 0; i < motor_b_list_.size(); ++i) {
        int idx = motor_b_joint_indices_[i];
        const auto& fb = motor_b_list_[i]->last_feedback();
        if (fb) codes[idx] = fb->error;
    }

    return codes;
}

std::pair<std::array<double, 6>, std::array<double, 6>>
MixedMotorChain::get_temperatures() const {
    std::array<double, 6> temp_mos = {};
    std::array<double, 6> temp_rotor = {};

    for (size_t i = 0; i < motor_a_list_.size(); ++i) {
        int idx = motor_a_joint_indices_[i];
        const auto& fb = motor_a_list_[i]->last_feedback();
        if (fb) {
            temp_mos[idx] = fb->temperature_mos;
            temp_rotor[idx] = fb->temperature;
        }
    }

    for (size_t i = 0; i < motor_b_list_.size(); ++i) {
        int idx = motor_b_joint_indices_[i];
        const auto& fb = motor_b_list_[i]->last_feedback();
        if (fb) {
            temp_mos[idx] = fb->temperature_mos;
            temp_rotor[idx] = fb->temperature_rotor;
        }
    }

    return {temp_mos, temp_rotor};
}

void MixedMotorChain::register_external_motor(std::shared_ptr<MotorB> motor) {
    MotorEntry entry;
    entry.type = MotorEntry::Type::MotorB;
    entry.motor = motor;
    entry.joint_index = -1;  // External, not part of joint chain
    motor_id_map_[motor->motor_id()] = entry;
}

void MixedMotorChain::reset_feedback_health() {
    for (auto& t : last_feedback_time_) {
        t = -std::numeric_limits<double>::infinity();
    }
}

bool MixedMotorChain::all_feedback_fresh(double max_age_s) const {
    auto ages = get_feedback_ages();
    for (double age : ages) {
        if (age > max_age_s) {
            return false;
        }
    }
    return true;
}

double MixedMotorChain::now_seconds() const {
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration<double>(now.time_since_epoch()).count();
}

} // namespace a1z
