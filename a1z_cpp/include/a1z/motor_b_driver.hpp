#pragma once

#include "a1z/types.hpp"
#include "a1z/transport.hpp"
#include <map>
#include <memory>
#include <optional>
#include <string>

namespace a1z {

/**
 * @brief MotorB physical ranges and limits.
 */
struct MotorBRanges {
    double pos_min = -12.5;
    double pos_max = 12.5;
    double vel_min = -30.0;
    double vel_max = 30.0;
    double torque_min = -10.0;
    double torque_max = 10.0;
    double kp_min = 0.0;
    double kp_max = 500.0;
    double kd_min = 0.0;
    double kd_max = 5.0;
};

/**
 * @brief MotorB feedback data.
 */
struct MotorBFeedback {
    int motor_id = 0;
    double position = 0.0;
    double velocity = 0.0;
    double torque = 0.0;
    int error = 0;
    std::string error_message;
    double temperature_mos = 0.0;
    double temperature_rotor = 0.0;
};

/**
 * @brief MotorB CAN driver (MIT mixed control).
 *
 * MotorB MIT command bit layout (64 bits):
 *   pos(16) | vel(12) | kp(12) | kd(12) | torque(12)
 *
 * Feedback layout:
 *   error(4) | pos(16) | vel(12) | torque(12) | temp_mos(8) | temp_rotor(8)
 */
class MotorB {
public:
    /**
     * @brief Construct a MotorB driver.
     * @param motor_id CAN ID of the motor
     * @param transport Transport interface (SocketCAN or G4Ros)
     * @param ranges Physical ranges for scaling
     */
    explicit MotorB(int motor_id,
                    std::shared_ptr<Transport> transport,
                    const MotorBRanges& ranges = MotorBRanges());

    /**
     * @brief Send motor enable command (0xFC).
     */
    bool enable();

    /**
     * @brief Send motor disable command (0xFD).
     */
    bool disable();

    /**
     * @brief Clear motor error (0xFB).
     */
    void clear_error();

    /**
     * @brief Set current position as zero in RAM only (0xFE).
     */
    void set_zero_ram();

    /**
     * @brief Switch control mode in RAM.
     * @param mode 1=MIT, 2=position-speed cascade, 3=speed, 4=force-position hybrid
     */
    void set_ctrl_mode(int mode);

    /**
     * @brief Send MIT mixed-control command.
     */
    void send_mit_command(double pos, double vel, double kp, double kd, double torque);

    /**
     * @brief Send force-position hybrid command (mode 4).
     * @param pos Target position (rad)
     * @param vel Speed limit (rad/s)
     * @param i_des Torque current fraction [0.0, 1.0]
     */
    void send_hybrid_command(double pos, double vel, double i_des);

    /**
     * @brief Parse feedback CAN frame.
     */
    std::optional<MotorBFeedback> parse_feedback(const CanFrame& frame);

    /**
     * @brief Get last parsed feedback.
     */
    const std::optional<MotorBFeedback>& last_feedback() const { return last_feedback_; }

    /**
     * @brief Get motor ID.
     */
    int motor_id() const { return motor_id_; }

    /**
     * @brief Get transport interface.
     */
    std::shared_ptr<Transport> transport() const { return transport_; }

    /**
     * @brief Write register via 0x7FF broadcast frame.
     */
    void write_register(int reg_id, uint32_t value);

    /**
     * @brief Write float register via 0x7FF broadcast frame.
     */
    void write_register_float(int reg_id, float value);

private:
    int motor_id_;
    std::shared_ptr<Transport> transport_;
    MotorBRanges ranges_;
    std::optional<MotorBFeedback> last_feedback_;

    static uint16_t float_to_uint(double x, double x_min, double x_max, int bits);
    static double uint_to_float(uint16_t x, double x_min, double x_max, int bits);
    static std::string error_to_string(int error_code);
};

// Error code lookup table
extern const std::map<int, std::string> MOTOR_B_ERROR_CODES;

} // namespace a1z
