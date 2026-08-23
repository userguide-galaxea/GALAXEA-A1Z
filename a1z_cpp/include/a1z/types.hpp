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

inline const char* to_string(ControlState s) {
    switch (s) {
    case ControlState::STOPPED: return "STOPPED";
    case ControlState::RUNNING: return "RUNNING";
    case ControlState::SOFT_ESTOP: return "SOFT_ESTOP";
    case ControlState::COMMAND_HOLD: return "COMMAND_HOLD";
    case ControlState::FAULT_HOLD: return "FAULT_HOLD";
    case ControlState::HARD_DISABLED: return "HARD_DISABLED";
    case ControlState::HARD_DISABLE_UNCONFIRMED: return "HARD_DISABLE_UNCONFIRMED";
    }
    return "UNKNOWN";
}

} // namespace a1z
