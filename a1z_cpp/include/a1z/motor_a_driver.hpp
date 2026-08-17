#pragma once

#include "a1z/types.hpp"
#include "a1z/can_interface.hpp"
#include <memory>
#include <optional>

namespace a1z {

/**
 * @brief MotorA physical ranges and limits.
 */
struct MotorARanges {
    double kp_min = 0.0;
    double kp_max = 500.0;
    double kd_min = 0.0;
    double kd_max = 5.0;
    double pos_min = -12.5;
    double pos_max = 12.5;
    double vel_min = -18.0;
    double vel_max = 18.0;
    double torque_min = -90.0;
    double torque_max = 90.0;
    double current_fb_min = -30.0;
    double current_fb_max = 30.0;
};

/**
 * @brief MotorA feedback data.
 */
struct MotorAFeedback {
    int motor_id = 0;
    double position = 0.0;
    double velocity = 0.0;
    double current = 0.0;
    int error = 0;
    double temperature = 0.0;      // Motor coil temperature (°C)
    double temperature_mos = 0.0;  // MOS temperature (°C)
};

/**
 * @brief MotorA CAN driver (MIT mixed control).
 *
 * Bit layout (64 bits, big-endian, MSB first):
 *   mode:   uint3
 *   kp:     uint12   (0..4095 -> kp_min..kp_max)
 *   kd:     uint9    (0..511  -> kd_min..kd_max)
 *   pos:    uint16   (0..65535 -> pos_min..pos_max)
 *   vel:    uint12   (0..4095  -> vel_min..vel_max)
 *   torque: uint12   (0..4095  -> torque_min..torque_max)
 */
class MotorA {
public:
    /**
     * @brief Construct a MotorA driver.
     * @param motor_id CAN ID of the motor (0x01-0x7FF)
     * @param can CAN interface reference
     * @param ranges Physical ranges for scaling
     * @param use_new_enable_protocol Use 0x7FF config frame instead of legacy
     */
    explicit MotorA(int motor_id,
                    std::shared_ptr<CanInterface> can,
                    const MotorARanges& ranges = MotorARanges(),
                    bool use_new_enable_protocol = false);

    /**
     * @brief Send motor enable command.
     */
    void enable();

    /**
     * @brief Send motor disable command.
     */
    void disable();

    /**
     * @brief Send MIT mixed-control command.
     * @param pos Target position (rad)
     * @param vel Target velocity (rad/s)
     * @param kp Position gain
     * @param kd Velocity gain
     * @param torque Feedforward torque (Nm)
     * @param mode MotorA mode field (usually 0)
     */
    void send_mit_command(double pos, double vel, double kp, double kd,
                          double torque, int mode = 0);

    /**
     * @brief Parse feedback CAN frame.
     * @param frame CAN frame
     * @return Parsed feedback or nullopt if invalid
     */
    std::optional<MotorAFeedback> parse_feedback(const CanFrame& frame);

    /**
     * @brief Get last parsed feedback.
     */
    const std::optional<MotorAFeedback>& last_feedback() const { return last_feedback_; }

    /**
     * @brief Get motor ID.
     */
    int motor_id() const { return motor_id_; }

    /**
     * @brief Get CAN interface.
     */
    std::shared_ptr<CanInterface> can() const { return can_; }

private:
    int motor_id_;
    std::shared_ptr<CanInterface> can_;
    MotorARanges ranges_;
    bool use_new_enable_protocol_;
    std::optional<MotorAFeedback> last_feedback_;

    // Utility functions
    static uint16_t float_to_uint(double x, double x_min, double x_max, int bits);
    static double uint_to_float(uint16_t x, double x_min, double x_max, int bits);
    static std::array<uint8_t, 8> pack_mit(int mode, uint16_t kp_u12, uint16_t kd_u9,
                                           uint16_t pos_u16, uint16_t vel_u12,
                                           uint16_t torque_u12);
};

} // namespace a1z
