#include "a1z/motor_a_driver.hpp"

#include <chrono>
#include <cmath>
#include <cstring>
#include <thread>

namespace a1z {

MotorA::MotorA(int motor_id, std::shared_ptr<Transport> transport,
               const MotorARanges& ranges, bool use_new_enable_protocol)
    : motor_id_(motor_id)
    , transport_(std::move(transport))
    , ranges_(ranges)
    , use_new_enable_protocol_(use_new_enable_protocol) {}

bool MotorA::enable() {
    CanFrame frame;
    if (use_new_enable_protocol_) {
        // New protocol: 0x7FF config frame
        frame.id = 0x7FF;
        frame.dlc = 4;
        frame.data[0] = (motor_id_ >> 8) & 0xFF;
        frame.data[1] = motor_id_ & 0xFF;
        frame.data[2] = 0x00;
        frame.data[3] = 0x01;
    } else {
        // Legacy: per-motor 0xFC frame
        frame.id = motor_id_;
        frame.dlc = 8;
        std::memset(frame.data.data(), 0xFF, 7);
        frame.data[7] = 0xFC;
    }
    bool ok = transport_->send(frame);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    return ok;
}

bool MotorA::disable() {
    CanFrame frame;
    if (use_new_enable_protocol_) {
        // New protocol: 0x7FF config frame
        frame.id = 0x7FF;
        frame.dlc = 4;
        frame.data[0] = (motor_id_ >> 8) & 0xFF;
        frame.data[1] = motor_id_ & 0xFF;
        frame.data[2] = 0x00;
        frame.data[3] = 0x02;
    } else {
        // Legacy: per-motor 0xFD frame
        frame.id = motor_id_;
        frame.dlc = 8;
        std::memset(frame.data.data(), 0xFF, 7);
        frame.data[7] = 0xFD;
    }
    bool ok = transport_->send(frame);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    return ok;
}

void MotorA::send_mit_command(double pos, double vel, double kp, double kd,
                              double torque, int mode) {
    uint16_t kp_u12 = float_to_uint(kp, ranges_.kp_min, ranges_.kp_max, 12);
    uint16_t kd_u9 = float_to_uint(kd, ranges_.kd_min, ranges_.kd_max, 9);
    uint16_t pos_u16 = float_to_uint(pos, ranges_.pos_min, ranges_.pos_max, 16);
    uint16_t vel_u12 = float_to_uint(vel, ranges_.vel_min, ranges_.vel_max, 12);
    uint16_t torque_u12 = float_to_uint(torque, ranges_.torque_min, ranges_.torque_max, 12);

    auto data = pack_mit(mode, kp_u12, kd_u9, pos_u16, vel_u12, torque_u12);

    CanFrame frame;
    frame.id = motor_id_;
    frame.dlc = 8;
    std::memcpy(frame.data.data(), data.data(), 8);
    transport_->send(frame);
}

std::optional<MotorAFeedback> MotorA::parse_feedback(const CanFrame& frame) {
    if (frame.dlc < 8) {
        return std::nullopt;
    }

    // Parse 64-bit big-endian frame
    uint64_t raw = 0;
    for (int i = 0; i < 8; ++i) {
        raw = (raw << 8) | frame.data[i];
    }

    int report_type = (raw >> 61) & 0x7;
    int error_code = (raw >> 56) & 0x1F;

    if (report_type != 0x1) {
        // ENCOS report frames come in 6 types (manual V1.12 §10); only type 1
        // carries uint16 pos/vel/current. Types 2/3 use a float32 layout and
        // 0/4/5/6 are reserved/config/query/brake replies. Never parse these
        // as position — that would read values near the range endpoints
        // (feedback jump). Only latch the error code.
        MotorAFeedback fb;
        fb.motor_id = frame.id;
        fb.error = error_code;
        fb.valid_position = false;
        if (error_code) {
            last_reported_error_ = error_code;
        }
        return fb;
    }

    uint16_t pos_raw = (raw >> 40) & 0xFFFF;
    uint16_t vel_raw = (raw >> 28) & 0xFFF;
    uint16_t curr_raw = (raw >> 16) & 0xFFF;
    uint8_t motor_temp_raw = (raw >> 8) & 0xFF;
    uint8_t mos_temp_raw = raw & 0xFF;

    MotorAFeedback fb;
    fb.motor_id = frame.id;
    fb.position = uint_to_float(pos_raw, ranges_.pos_min, ranges_.pos_max, 16);
    fb.velocity = uint_to_float(vel_raw, ranges_.vel_min, ranges_.vel_max, 12);
    fb.current = uint_to_float(curr_raw, ranges_.current_fb_min, ranges_.current_fb_max, 12);
    fb.error = error_code;
    // Temperature encoding: raw = actual_°C * 2 + 50
    fb.temperature = (motor_temp_raw - 50) / 2.0;
    fb.temperature_mos = (mos_temp_raw - 50) / 2.0;

    last_feedback_ = fb;
    return fb;
}

uint16_t MotorA::float_to_uint(double x, double x_min, double x_max, int bits) {
    double span = x_max - x_min;
    double offset = x_min;
    double clamped = std::max(x_min, std::min(x, x_max));
    uint32_t max_val = (1 << bits) - 1;
    return static_cast<uint16_t>((clamped - offset) * max_val / span);
}

double MotorA::uint_to_float(uint16_t x, double x_min, double x_max, int bits) {
    double span = x_max - x_min;
    uint32_t max_val = (1 << bits) - 1;
    return static_cast<double>(x) * span / max_val + x_min;
}

std::array<uint8_t, 8> MotorA::pack_mit(int mode, uint16_t kp_u12, uint16_t kd_u9,
                                        uint16_t pos_u16, uint16_t vel_u12,
                                        uint16_t torque_u12) {
    // Byte layout (aligned with CAN-H7 send_motor_ctrl_cmd):
    //   data[0] = (mode << 5) | (kp >> 7)
    //   data[1] = ((kp & 0x7F) << 1) | (kd >> 8)
    //   data[2] = kd & 0xFF
    //   data[3] = pos >> 8
    //   data[4] = pos & 0xFF
    //   data[5] = vel >> 4
    //   data[6] = ((vel & 0x0F) << 4) | (torque >> 8)
    //   data[7] = torque & 0xFF
    std::array<uint8_t, 8> data;
    data[0] = ((mode & 0x7) << 5) | ((kp_u12 >> 7) & 0x1F);
    data[1] = ((kp_u12 & 0x7F) << 1) | ((kd_u9 >> 8) & 0x01);
    data[2] = kd_u9 & 0xFF;
    data[3] = (pos_u16 >> 8) & 0xFF;
    data[4] = pos_u16 & 0xFF;
    data[5] = (vel_u12 >> 4) & 0xFF;
    data[6] = ((vel_u12 & 0x0F) << 4) | ((torque_u12 >> 8) & 0x0F);
    data[7] = torque_u12 & 0xFF;
    return data;
}

} // namespace a1z
