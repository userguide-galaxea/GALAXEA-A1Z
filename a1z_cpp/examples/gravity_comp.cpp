/**
 * @file gravity_comp.cpp
 * @brief A1Z gravity compensation example (C++ version)
 *
 * Usage:
 *   gravity_comp [--mode gravity|hold] [--gravity_factor 0.3] [--arm-side left|right]
 *
 * Modes:
 *   gravity  - Zero-gravity (floating) mode
 *   hold     - Position hold with gravity compensation
 */

#include "a1z/transport.hpp"
#include "a1z/arm_robot.hpp"
#include "a1z/gravity_model.hpp"
#include "a1z/gripper.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <thread>
#include <vector>

using namespace a1z;

std::atomic<bool> g_running{true};

void signal_handler(int signum) {
    (void)signum;
    g_running = false;
}

void print_usage(const char* prog) {
    std::cout << "Usage: " << prog << " [OPTIONS]\n"
              << "\nModes:\n"
              << "  --mode MODE         gravity (default) or hold\n"
              << "  --gravity_factor F  Gravity compensation scale 0.0-1.0 (default: 1.0)\n"
              << "  --freq HZ           Control loop frequency (default: 250)\n"
              << "  --transport TYPE    g4ros (default) or socketcan\n"
              << "  --arm-side SIDE     left (default) or right (g4ros only)\n"
              << "  --can CHANNEL       CAN channel (default: can0, socketcan only)\n"
              << "  --urdf PATH         URDF file path (default: auto-detect A1Z_G1Z.urdf)\n"
              << "  --help              Show this help\n";
}

int main(int argc, char* argv[]) {
    std::string mode = "gravity";
    double gravity_factor = 1.0;
    int freq = 250;
    std::string transport_type = "g4ros";
    std::string arm_side = "left";
    std::string can_channel = "can0";
    std::string urdf_path;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help") {
            print_usage(argv[0]);
            return 0;
        } else if (arg == "--mode" && i + 1 < argc) {
            mode = argv[++i];
        } else if (arg == "--gravity_factor" && i + 1 < argc) {
            gravity_factor = std::atof(argv[++i]);
        } else if (arg == "--freq" && i + 1 < argc) {
            freq = std::atoi(argv[++i]);
        } else if (arg == "--transport" && i + 1 < argc) {
            transport_type = argv[++i];
        } else if (arg == "--arm-side" && i + 1 < argc) {
            arm_side = argv[++i];
        } else if (arg == "--can" && i + 1 < argc) {
            can_channel = argv[++i];
        } else if (arg == "--urdf" && i + 1 < argc) {
            urdf_path = argv[++i];
        }
    }

    bool zero_gravity = (mode == "gravity");

    std::cout << "============================================================" << std::endl;
    std::cout << "  A1Z Gravity Compensation (C++)" << std::endl;
    std::cout << "  Mode:            " << (zero_gravity ? "Zero-gravity (floating)" : "Position hold + gravity comp") << std::endl;
    std::cout << "  Gravity factor:  " << gravity_factor << std::endl;
    std::cout << "  Control freq:    " << freq << " Hz" << std::endl;
    std::cout << "  Transport:       " << transport_type;
    if (transport_type == "g4ros") {
        std::cout << " (arm side: " << arm_side << ")";
    } else {
        std::cout << " (CAN: " << can_channel << ")";
    }
    std::cout << std::endl;
    std::cout << "============================================================" << std::endl;

    // Create transport
    std::shared_ptr<Transport> transport;
    try {
        if (transport_type == "g4ros") {
            transport = create_g4ros_transport(arm_side);
        } else {
            transport = create_socketcan_transport(can_channel);
        }
    } catch (const std::exception& e) {
        std::cerr << "Failed to create transport: " << e.what() << std::endl;
        return 1;
    }

    // Create motors
    std::vector<std::shared_ptr<MotorA>> motor_a_list;
    std::vector<std::shared_ptr<MotorB>> motor_b_list;

    // MotorA: joints 1-3 (CAN ID 0x01-0x03)
    for (int i = 1; i <= 3; ++i) {
        auto motor = std::make_shared<MotorA>(i, transport, MotorARanges(),
                                              /*use_new_enable_protocol=*/true);
        motor_a_list.push_back(motor);
    }

    // MotorB: joints 4-6 (CAN ID 0x04-0x06)
    for (int i = 4; i <= 6; ++i) {
        MotorBRanges ranges;
        if (i == 4) {
            // Joint 3 (arm_joint4) uses the high-torque range (matches the
            // Python SDK's _MOTOR_B_RANGES_JOINT3).
            ranges.vel_min = -10.0;
            ranges.vel_max = 10.0;
            ranges.torque_min = -28.0;
            ranges.torque_max = 28.0;
        }
        auto motor = std::make_shared<MotorB>(i, transport, ranges);
        motor_b_list.push_back(motor);
    }

    // Joint indices: MotorA 0-2, MotorB 3-5
    std::vector<int> motor_a_joint_indices = {0, 1, 2};
    std::vector<int> motor_b_joint_indices = {3, 4, 5};

    // Create motor chain with 250us inter-command gap (lemo default)
    auto motor_chain = std::make_shared<MixedMotorChain>(
        motor_a_list, motor_b_list,
        motor_a_joint_indices, motor_b_joint_indices,
        2.8,      // motor_a_kt
        0.00025   // 250us gap
    );

    // Create gravity model. Without --urdf, try the default locations
    // (repo checkout and the board's ~/a1z layout) before giving up.
    if (urdf_path.empty()) {
        const char* home = std::getenv("HOME");
        std::vector<std::string> candidates = {
            "a1z/robot_models/a1z/A1Z_G1Z.urdf",
            "../a1z/robot_models/a1z/A1Z_G1Z.urdf",
        };
        if (home) {
            candidates.push_back(std::string(home) + "/a1z/a1z/robot_models/a1z/A1Z_G1Z.urdf");
        }
        for (const auto& path : candidates) {
            std::ifstream f(path);
            if (f.good()) {
                urdf_path = path;
                break;
            }
        }
    }

    std::shared_ptr<GravityModel> gravity_model;
    if (!urdf_path.empty()) {
        try {
            gravity_model = std::make_shared<GravityModel>(urdf_path);
            std::cout << "  URDF:            " << urdf_path << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "Failed to load URDF: " << e.what() << std::endl;
            std::cerr << "Continuing without gravity compensation..." << std::endl;
        }
    } else {
        std::cerr << "No URDF found (use --urdf PATH); running without gravity compensation." << std::endl;
    }

    // Create gripper (optional)
    std::shared_ptr<Gripper> gripper;
    // auto gripper_motor = std::make_shared<MotorB>(7, nullptr);
    // gripper = std::make_shared<Gripper>(gripper_motor);

    // Create arm robot
    auto robot = std::make_shared<ArmRobot>(
        motor_chain, gravity_model, gripper,
        freq,   // control_freq_hz
        80.0    // min_freq_hz
    );

    // Set gravity compensation factor
    robot->set_gravity_comp_factor(gravity_factor);

    // Zero-gravity (floating) vs position-hold mode
    robot->set_gravity_mode(zero_gravity);

    // Joint limits: needed for command validation and the ±π unwrap
    if (gravity_model) {
        robot->set_joint_limits(gravity_model->get_joint_limits());
    }

    // Set joint sign convention (lemo defaults)
    JointVector joint_sign = {1.0, 1.0, -1.0, 1.0, -1.0, 1.0};
    robot->set_joint_sign(joint_sign);

    // Setup signal handler
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    std::cout << "\nStarting robot..." << std::endl;
    std::cout << "Press Ctrl+C to exit" << std::endl;

    try {
        // Start control loop
        robot->start();

        std::cout << "Robot running. ";
        if (zero_gravity) {
            std::cout << "Arm is in zero-gravity mode - you can move it by hand." << std::endl;
        } else {
            std::cout << "Arm is holding position with gravity compensation." << std::endl;
        }

        // Main loop - just print status
        while (g_running) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            auto state = robot->get_joint_state();
            std::cout << "\rPositions: [";
            for (size_t i = 0; i < 6; ++i) {
                printf("%7.3f", state.pos[i]);
                if (i < 5) std::cout << ", ";
            }
            std::cout << "] rad" << std::flush;
        }

        std::cout << "\n\nStopping robot..." << std::endl;
        robot->stop();

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        robot->stop();
        return 1;
    }

    transport->close();
    std::cout << "Done." << std::endl;
    return 0;
}
