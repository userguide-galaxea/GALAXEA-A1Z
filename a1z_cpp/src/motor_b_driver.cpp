#include "a1z/motor_b_driver.hpp"

#include <chrono>
#include <cmath>
#include <cstring>
#include <thread>

namespace a1z {

const std::map<int, std::string> MOTOR_B_ERROR_CODES = {
    {0x0, "disabled"},
    {0x1, "normal"},
    {0x8, "over voltage"},
    {0x9, "under voltage"},
    {0xA, "over current"},
    {0xB, "mos over temperature"},
    {0xC, "motor coil over temperature"},
    {0xD, "communication lost"},
    {0xE, "overload"},
    {0xF, "position out of range"},
};

MotorB::MotorB(int motor_id, std::shared_ptr<CanInterface> can,
               const MotorBRanges& ranges)
    : motor_id_(motor_id)
    , can_(std::move(can))
    , ranges_(ranges) {}

void MotorB::enable() {
    CanFrame frame;
    frame.id = motor_id_;
    frame.dlc = 8;
    std::memset(frame.data.data(), 0xFF, 7);
    frame.data[7] = 0xFC;
    can_->send(frame);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

void MotorB::disable() {
    CanFrame frame;
    frame.id = motor_id_;
    frame.dlc = 8;
    std::memset(frame.data.data(), 0xFF, 7);
    frame.data[7] = 0xFD;
    can_->send(frame);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

void MotorB::clear_error() {
    CanFrame frame;
    frame.id = motor_id_;
    frame.dlc = 8;
    std::memset(frame.data.data(), 0xFF, 7);
    frame.data[7] = 0xFB;
    can_->send(frame);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

void MotorB::set_zero_ram() {
    CanFrame frame;
    frame.id = motor_id_;
    frame.dlc = 8;
    std::memset(frame.data.data(), 0xFF, 7);
    frame.data[7] = 0xFE;
    can_->send(frame);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

void MotorB::set_ctrl_mode(int mode) {
    write_register(0x0A, static_cast<uint32_t>(mode));
}

void MotorB::write_register(int reg_id, uint32_t value) {
    CanFrame frame;
    frame.id = 0x7FF;
    frame.dlc = 8;
    frame.data[0] = motor_id_ & 0xFF;
    frame.data[1] = (motor_id_ >> 8) & 0xFF;
    frame.data[2] = 0x55;  // write command
    frame.data[3] = reg_id & 0xFF;
    std::memcpy(&frame.data[4], &value, 4);
    can_->send(frame);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

void MotorB::write_register_float(int reg_id, float value) {
    CanFrame frame;
    frame.id = 0x7FF;
    frame.dlc = 8;
    frame.data[0] = motor_id_ & 0xFF;
    frame.data[1] = (motor_id_ >> 8) & 0xFF;
    frame.data[2] = 0x55;  // write command
    frame.data[3] = reg_id & 0xFF;
    std::memcpy(&frame.data[4], &value, 4);
    can_->send(frame);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

void MotorB::send_mit_command(double pos, double vel, double kp, double kd,
                              double torque) {
    uint16_t pos_u16 = float_to_uint(pos, ranges_.pos_min, ranges_.pos_max, 16);
    uint16_t vel_u12 = float_to_uint(vel, ranges_.vel_min, ranges_.vel_max, 12);
    uint16_t kp_u12 = float_to_uint(kp, ranges_.kp_min, ranges_.kp_max, 12);
    uint16_t kd_u12 = float_to_uint(kd, ranges_.kd_min, ranges_.kd_max, 12);
    uint16_t tor_u12 = float_to_uint(torque, ranges_.torque_min, ranges_.torque_max, 12);

    CanFrame frame;
    frame.id = motor_id_;
    frame.dlc = 8;
    frame.data[0] = (pos_u16 >> 8) & 0xFF;
    frame.data[1] = pos_u16 & 0xFF;
    frame.data[2] = (vel_u12 >> 4) & 0xFF;
    frame.data[3] = ((vel_u12 & 0xF) << 4) | ((kp_u12 >> 8) & 0xF);
    frame.data[4] = kp_u12 & 0xFF;
    frame.data[5] = (kd_u12 >> 4) & 0xFF;
    frame.data[6] = ((kd_u12 & 0xF) << 4) | ((tor_u12 >> 8) & 0xF);
    frame.data[7] = tor_u12 & 0xFF;
    can_->send(frame);
}

void MotorB::send_hybrid_command(double pos, double vel, double i_des) {
    // Clamp inputs
    vel = std::max(0.0, std::min(100.0, vel));
    i_des = std::max(0.0, std::min(1.0, i_des));

    uint16_t v_int = static_cast<uint16_t>(vel * 100.0);
    uint16_t i_int = static_cast<uint16_t>(i_des * 10000.0);

    CanFrame frame;
    frame.id = 0x300 + motor_id_;
    frame.dlc = 8;
    // p_des: float32 little-endian
    float p = static_cast<float>(pos);
    std::memcpy(&frame.data[0], &p, 4);
    // v_des: uint16 little-endian, ×100 → rad/s
    frame.data[4] = v_int & 0xFF;
    frame.data[5] = (v_int >> 8) & 0xFF;
    // i_des: uint16 little-endian, ×10000 → fraction
    frame.data[6] = i_int & 0xFF;
    frame.data[7] = (i_int >> 8) & 0xFF;
    can_->send(frame);
}

std::optional<MotorBFeedback> MotorB::parse_feedback(const CanFrame& frame) {
    if (frame.dlc < 8) {
        return std::nullopt;
    }

    const auto& data = frame.data;

    int error_int = (data[0] & 0xF0) >> 4;
    uint16_t p_int = (data[1] << 8) | data[2];
    uint16_t v_int = (data[3] << 4) | (data[4] >> 4);
    uint16_t t_int = ((data[4] & 0xF) << 8) | data[5];

    MotorBFeedback fb;
    fb.motor_id = frame.id;
    fb.position = uint_to_float(p_int, ranges_.pos_min, ranges_.pos_max, 16);
    fb.velocity = uint_to_float(v_int, ranges_.vel_min, ranges_.vel_max, 12);
    fb.torque = uint_to_float(t_int, ranges_.torque_min, ranges_.torque_max, 12);
    fb.error = error_int;
    fb.error_message = error_to_string(error_int);
    fb.temperature_mos = static_cast<double>(data[6]);
    fb.temperature_rotor = static_cast<double>(data[7]);

    last_feedback_ = fb;
    return fb;
}

uint16_t MotorB::float_to_uint(double x, double x_min, double x_max, int bits) {
    double span = x_max - x_min;
    double offset = x_min;
    double clamped = std::max(x_min, std::min(x, x_max));
    uint32_t max_val = (1 << bits) - 1;
    return static_cast<uint16_t>((clamped - offset) * max_val / span);
}

double MotorB::uint_to_float(uint16_t x, double x_min, double x_max, int bits) {
    double span = x_max - x_min;
    uint32_t max_val = (1 << bits) - 1;
    return static_cast<double>(x) * span / max_val + x_min;
}

std::string MotorB::error_to_string(int error_code) {
    auto it = MOTOR_B_ERROR_CODES.find(error_code);
    if (it != MOTOR_B_ERROR_CODES.end()) {
        return it->second;
    }
    return "unknown(" + std::to_string(error_code) + ")";
}

} // namespace a1z
