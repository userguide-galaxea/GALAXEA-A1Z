/**
 * @file motor_diag.cpp
 * @brief A1Z motor communication diagnostic tool (C++ version)
 *
 * Usage:
 *   motor_diag --scan [--arm-side left|right] [--transport g4ros|socketcan]
 *   motor_diag --monitor [--arm-side left|right]
 *   motor_diag --listen [--duration 5]
 *   motor_diag --probe JOINT_ID
 */

#include "a1z/transport.hpp"
#include "a1z/motor_a_driver.hpp"
#include "a1z/motor_b_driver.hpp"

#include <chrono>
#include <cstring>
#include <iostream>
#include <map>
#include <thread>

using namespace a1z;

// Joint configuration (matches Python version)
struct JointConfig {
    std::string name;
    std::string type;
    int can_id;
};

const std::map<int, JointConfig> JOINT_CONFIG = {
    {0, {"arm_joint1", "MOTOR_A", 0x01}},
    {1, {"arm_joint2", "MOTOR_A", 0x02}},
    {2, {"arm_joint3", "MOTOR_A", 0x03}},
    {3, {"arm_joint4", "MotorB4340", 0x04}},
    {4, {"arm_joint5", "MotorB4310", 0x05}},
    {5, {"arm_joint6", "MotorB4310", 0x06}},
    {6, {"gripper", "MotorB4310", 0x07}},
};

void print_usage(const char* prog) {
    std::cout << "Usage: " << prog << " [OPTIONS]\n"
              << "\nModes:\n"
              << "  --scan              Scan all motors and check communication\n"
              << "  --monitor           Continuously monitor motor states\n"
              << "  --listen            Passively listen to bus messages\n"
              << "  --probe JOINT_ID    Probe specific joint (enable -> zero cmd -> read -> disable)\n"
              << "\nOptions:\n"
              << "  --transport TYPE    Transport type: g4ros (default) or socketcan\n"
              << "  --arm-side SIDE     Arm side: left (default) or right (g4ros only)\n"
              << "  --can CHANNEL       CAN channel (default: can0, socketcan only)\n"
              << "  --duration SECONDS  Duration for listen mode (default: 5)\n"
              << "  --help              Show this help\n";
}

// Probe one motor through the SDK drivers, using the same zero-gain probe
// the Python SDK sends at startup (kp=0, kd=0.05, pos=0 properly encoded).
// NEVER hand-craft an all-zero MIT frame: raw-zero fields can be treated by
// the firmware as "keep previous value" — a stale nonzero kp/kd combined
// with p_des at the range endpoint drives the joint to its limit.
// MotorB gets set_ctrl_mode(1) before enable, mirroring Python enable_all.
static bool probe_motor(std::shared_ptr<Transport> transport,
                        const JointConfig& config, double* pos_out) {
    bool got = false;

    if (config.type == "MOTOR_A") {
        MotorA motor(config.can_id, transport, MotorARanges(),
                     /*use_new_enable_protocol=*/true);
        motor.enable();
        motor.send_mit_command(0.0, 0.0, 0.0, 0.05, 0.0);
        for (int retry = 0; retry < 10 && !got; ++retry) {
            auto frame = transport->receive(20);
            if (frame && frame->id == static_cast<uint32_t>(config.can_id)) {
                auto fb = motor.parse_feedback(*frame);
                if (fb && fb->valid_position) {
                    *pos_out = fb->position;
                    got = true;
                }
            }
        }
        motor.disable();
        return got;
    }

    MotorB motor(config.can_id, transport);
    motor.set_ctrl_mode(1);  // MIT mode
    motor.enable();
    motor.send_mit_command(0.0, 0.0, 0.0, 0.05, 0.0);
    for (int retry = 0; retry < 10 && !got; ++retry) {
        auto frame = transport->receive(20);
        if (frame && frame->id == static_cast<uint32_t>(config.can_id)) {
            auto fb = motor.parse_feedback(*frame);
            if (fb) {
                *pos_out = fb->position;
                got = true;
            }
        }
    }
    motor.disable();
    return got;
}

int cmd_scan(std::shared_ptr<Transport> transport) {
    std::cout << "Scanning motors..." << std::endl;
    std::cout << "NOTE: each motor is enabled briefly with zero gains — "
                 "keep the arm supported." << std::endl;
    std::cout << "===================" << std::endl;

    int found = 0;
    int missing = 0;

    for (const auto& [joint_idx, config] : JOINT_CONFIG) {
        std::cout << "Joint " << joint_idx << " (" << config.name
                  << ", " << config.type << ", CAN ID 0x"
                  << std::hex << config.can_id << std::dec << "): ";

        double pos = 0.0;
        if (probe_motor(transport, config, &pos)) {
            std::cout << "OK (pos=" << pos << " rad)" << std::endl;
            found++;
        } else {
            std::cout << "NO RESPONSE" << std::endl;
            missing++;
        }

        // Small delay between motors
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "===================" << std::endl;
    std::cout << "Found: " << found << ", Missing: " << missing << std::endl;
    return missing > 0 ? 1 : 0;
}

int cmd_monitor(std::shared_ptr<Transport> transport) {
    std::cout << "Monitoring motors (Ctrl+C to exit)..." << std::endl;
    std::cout << "Time       Joint  Type      Pos(rad)   Vel(rad/s)  Cur/Torque  Temp" << std::endl;
    std::cout << "------------------------------------------------------------------------" << std::endl;

    auto start_time = std::chrono::steady_clock::now();

    while (true) {
        auto frame = transport->receive(100);
        if (!frame) {
            continue;
        }

        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - start_time).count();

        // Find joint config
        int joint_idx = -1;
        std::string joint_name = "unknown";
        std::string joint_type = "unknown";

        for (const auto& [idx, config] : JOINT_CONFIG) {
            if (config.can_id == static_cast<int>(frame->id)) {
                joint_idx = idx;
                joint_name = config.name;
                joint_type = config.type;
                break;
            }
        }

        if (joint_idx < 0) {
            continue;  // Unknown CAN ID
        }

        printf("%8.2f  J%d    %-8s", elapsed, joint_idx, joint_type.c_str());

        if (joint_type == "MOTOR_A") {
            // Parse MotorA feedback
            if (frame->dlc >= 8) {
                uint64_t raw = 0;
                for (int i = 0; i < 8; ++i) {
                    raw = (raw << 8) | frame->data[i];
                }
                uint16_t pos_raw = (raw >> 40) & 0xFFFF;
                uint16_t vel_raw = (raw >> 28) & 0xFFF;
                uint16_t cur_raw = (raw >> 16) & 0xFFF;
                uint8_t temp_raw = (raw >> 8) & 0xFF;

                double pos = (pos_raw / 65535.0) * 25.0 - 12.5;
                double vel = (vel_raw / 4095.0) * 36.0 - 18.0;
                double cur = (cur_raw / 4095.0) * 60.0 - 30.0;
                double temp = (temp_raw - 50) / 2.0;

                printf("  %8.4f  %8.4f  %8.3fA  %5.1fC\n", pos, vel, cur, temp);
            }
        } else {
            // Parse MotorB feedback
            if (frame->dlc >= 8) {
                uint16_t pos_raw = (frame->data[1] << 8) | frame->data[2];
                uint16_t vel_raw = (frame->data[3] << 4) | (frame->data[4] >> 4);
                uint16_t tor_raw = ((frame->data[4] & 0xF) << 8) | frame->data[5];
                uint8_t temp_mos = frame->data[6];
                uint8_t temp_rotor = frame->data[7];

                double pos = (pos_raw / 65535.0) * 25.0 - 12.5;
                double vel = (vel_raw / 4095.0) * 60.0 - 30.0;
                double tor = (tor_raw / 4095.0) * 20.0 - 10.0;

                printf("  %8.4f  %8.4f  %8.3fNm  %d/%dC\n", pos, vel, tor,
                       static_cast<int>(temp_mos), static_cast<int>(temp_rotor));
            }
        }
    }

    return 0;
}

int cmd_listen(std::shared_ptr<Transport> transport, int duration_s) {
    std::cout << "Listening to CAN bus for " << duration_s << " seconds..." << std::endl;

    auto start = std::chrono::steady_clock::now();
    int count = 0;

    while (true) {
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration<double>(now - start).count() > duration_s) {
            break;
        }

        auto frame = transport->receive(100);
        if (frame) {
            std::cout << "CAN ID 0x" << std::hex << frame->id << std::dec
                      << " [" << static_cast<int>(frame->dlc) << " bytes]: ";
            for (int i = 0; i < frame->dlc; ++i) {
                printf("%02X ", frame->data[i]);
            }
            std::cout << std::endl;
            count++;
        }
    }

    std::cout << "Received " << count << " messages" << std::endl;
    return 0;
}

int cmd_probe(std::shared_ptr<Transport> transport, int joint_id) {
    auto it = JOINT_CONFIG.find(joint_id);
    if (it == JOINT_CONFIG.end()) {
        std::cerr << "Invalid joint ID: " << joint_id << std::endl;
        return 1;
    }

    const auto& config = it->second;
    std::cout << "Probing joint " << joint_id << " (" << config.name << ")..." << std::endl;

    std::cout << "  Enabling and sending zero-gain probe (kp=0, kd=0.05)..." << std::endl;
    double pos = 0.0;
    if (probe_motor(transport, config, &pos)) {
        std::cout << "  Feedback received: pos=" << pos << " rad" << std::endl;
    } else {
        std::cout << "  No feedback received!" << std::endl;
        return 1;
    }
    std::cout << "  Disabled." << std::endl;

    return 0;
}

int main(int argc, char* argv[]) {
    std::string transport_type = "g4ros";
    std::string arm_side = "left";
    std::string can_channel = "can0";
    int duration = 5;
    int probe_joint = -1;

    enum Mode { NONE, SCAN, MONITOR, LISTEN, PROBE };
    Mode mode = NONE;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help") {
            print_usage(argv[0]);
            return 0;
        } else if (arg == "--scan") {
            mode = SCAN;
        } else if (arg == "--monitor") {
            mode = MONITOR;
        } else if (arg == "--listen") {
            mode = LISTEN;
        } else if (arg == "--probe" && i + 1 < argc) {
            mode = PROBE;
            probe_joint = std::atoi(argv[++i]);
        } else if (arg == "--transport" && i + 1 < argc) {
            transport_type = argv[++i];
        } else if (arg == "--arm-side" && i + 1 < argc) {
            arm_side = argv[++i];
        } else if (arg == "--can" && i + 1 < argc) {
            can_channel = argv[++i];
        } else if (arg == "--duration" && i + 1 < argc) {
            duration = std::atoi(argv[++i]);
        }
    }

    if (mode == NONE) {
        print_usage(argv[0]);
        return 1;
    }

    // Create transport
    std::shared_ptr<Transport> transport;
    try {
        if (transport_type == "g4ros") {
            transport = create_g4ros_transport(arm_side);
        } else if (transport_type == "socketcan") {
            transport = create_socketcan_transport(can_channel);
        } else {
            std::cerr << "Invalid transport type: " << transport_type << std::endl;
            return 1;
        }
    } catch (const std::exception& e) {
        std::cerr << "Failed to create transport: " << e.what() << std::endl;
        return 1;
    }

    // Execute command
    int result = 0;
    switch (mode) {
        case SCAN:
            result = cmd_scan(transport);
            break;
        case MONITOR:
            result = cmd_monitor(transport);
            break;
        case LISTEN:
            result = cmd_listen(transport, duration);
            break;
        case PROBE:
            result = cmd_probe(transport, probe_joint);
            break;
        default:
            break;
    }

    transport->close();
    return result;
}
