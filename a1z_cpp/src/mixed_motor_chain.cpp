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
    std::vector<int> failed;
    // Send twice - a single frame can be missed on a busy bus
    for (int i = 0; i < 2; ++i) {
        for (auto& motor : motor_a_list_) {
            try {
                if (!motor->disable() && i == 1) failed.push_back(motor->motor_id());
            } catch (...) {
                if (i == 1) failed.push_back(motor->motor_id());
            }
        }
        for (auto& motor : motor_b_list_) {
            try {
                if (!motor->disable() && i == 1) failed.push_back(motor->motor_id());
            } catch (...) {
                if (i == 1) failed.push_back(motor->motor_id());
            }
        }
    }
    if (!failed.empty()) {
        std::cerr << "[MixedMotorChain] WARNING: disable command failed for motor IDs:";
        for (int id : failed) std::cerr << " " << id;
        std::cerr << " — these motors may still be ENABLED" << std::endl;
    }

    // MotorB 失能确认：达妙反馈的状态字段 0x0=disabled, 0x1=enabled，
    // 电机执行失能后会回一帧状态帧。发送成功 ≠ 电机执行（2026-08 实测
    // MotorB 在 stop 后仍保持使能），未确认的补发重试，最多 3 轮。
    // MotorA（ENCOS）反馈不携带使能状态且失能是广播帧，无法验证，跳过。
    bool all_confirmed = true;
    for (int round = 0; round < 3; ++round) {
        drain_and_update(120);  // 收确认帧
        std::vector<int> unconfirmed;
        for (auto& motor : motor_b_list_) {
            const auto& fb = motor->last_feedback();
            if (!fb || fb->error != 0x0) {
                unconfirmed.push_back(motor->motor_id());
            }
        }
        if (unconfirmed.empty()) {
            all_confirmed = true;
            break;
        }
        all_confirmed = false;
        for (auto& motor : motor_b_list_) {
            if (std::find(unconfirmed.begin(), unconfirmed.end(),
                          motor->motor_id()) != unconfirmed.end()) {
                try {
                    motor->disable();
                } catch (...) {
                }
            }
        }
    }
    if (!all_confirmed) {
        std::cerr << "[MixedMotorChain] WARNING: MotorB disable NOT confirmed "
                     "after retries for motor IDs:";
        for (auto& motor : motor_b_list_) {
            const auto& fb = motor->last_feedback();
            if (!fb || fb->error != 0x0) {
                std::cerr << " " << motor->motor_id();
            }
        }
        std::cerr << " — check motor state manually before powering!" << std::endl;
    }
    return failed.empty() && all_confirmed;
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
        auto fb = motor->parse_feedback(frame);
        // Non-type-1 report frames only latch the error code; they are not
        // valid position feedback and must not refresh freshness.
        parsed = fb.has_value() && fb->valid_position;
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
        // Error codes from non-type-1 report frames are latched separately
        // and take priority over the regular feedback error field.
        int latched = motor_a_list_[i]->last_reported_error();
        if (latched) codes[idx] = latched;
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
