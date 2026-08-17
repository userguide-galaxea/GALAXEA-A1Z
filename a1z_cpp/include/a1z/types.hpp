#pragma once

#include <array>
#include <cstdint>
#include <vector>

namespace a1z {

// CAN message structure (Linux SocketCAN compatible)
struct CanFrame {
    uint32_t id = 0;
    uint8_t dlc = 0;
    std::array<uint8_t, 8> data = {};
    bool is_extended = false;
    bool is_error = false;
    bool is_rtr = false;
};

// Joint state vectors
using JointVector = std::array<double, 6>;
using JointArray = std::vector<double>;

// Motor types
enum class MotorType {
    MotorA,
    MotorB
};

// Control states matching Python SDK
enum class ControlState {
    STOPPED,
    RUNNING,
    SOFT_ESTOP,
    COMMAND_HOLD,
    FAULT_HOLD,
    HARD_DISABLED,
    HARD_DISABLE_UNCONFIRMED
};

} // namespace a1z
