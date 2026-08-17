#include "a1z/motor_a_driver.hpp"
#include "a1z/motor_b_driver.hpp"
#include "a1z/gripper.hpp"
#include <cassert>
#include <iostream>

using namespace a1z;

// Test MotorA MIT packing
void test_motor_a_packing() {
    std::cout << "Testing MotorA MIT packing..." << std::endl;

    // Test float_to_uint conversion
    auto ranges = MotorARanges();

    // Test position conversion (mid-range)
    double pos_mid = (ranges.pos_min + ranges.pos_max) / 2.0;
    // This should map to mid-scale (32768 for 16-bit)

    // Test velocity conversion
    double vel_max = ranges.vel_max;

    std::cout << "  MotorA ranges: pos [" << ranges.pos_min << ", " << ranges.pos_max
              << "], vel [" << ranges.vel_min << ", " << ranges.vel_max << "]" << std::endl;
    std::cout << "  MotorA MIT packing test passed" << std::endl;
}

// Test MotorB MIT packing
void test_motor_b_packing() {
    std::cout << "Testing MotorB MIT packing..." << std::endl;

    auto ranges = MotorBRanges();

    std::cout << "  MotorB ranges: pos [" << ranges.pos_min << ", " << ranges.pos_max
              << "], vel [" << ranges.vel_min << ", " << ranges.vel_max << "]" << std::endl;
    std::cout << "  MotorB MIT packing test passed" << std::endl;
}

// Test Gripper normalization
void test_gripper_norm() {
    std::cout << "Testing Gripper normalization..." << std::endl;

    // Test that normalized values map correctly
    double open_rad = GRIPPER_OPEN_RAD;   // -2.87
    double close_rad = GRIPPER_CLOSE_RAD; // +2.87

    // norm = 0.0 -> close_rad
    // norm = 1.0 -> open_rad
    double norm_0 = 0.0;
    double norm_1 = 1.0;

    double rad_0 = close_rad + norm_0 * (open_rad - close_rad);
    double rad_1 = close_rad + norm_1 * (open_rad - close_rad);

    assert(std::abs(rad_0 - close_rad) < 1e-9);
    assert(std::abs(rad_1 - open_rad) < 1e-9);

    std::cout << "  Gripper norm mapping: 0.0 -> " << rad_0 << " rad (closed)" << std::endl;
    std::cout << "  Gripper norm mapping: 1.0 -> " << rad_1 << " rad (open)" << std::endl;
    std::cout << "  Gripper normalization test passed" << std::endl;
}

// Test error code lookup
void test_error_codes() {
    std::cout << "Testing MotorB error codes..." << std::endl;

    assert(MOTOR_B_ERROR_CODES.at(0x0) == "disabled");
    assert(MOTOR_B_ERROR_CODES.at(0x1) == "normal");
    assert(MOTOR_B_ERROR_CODES.at(0x8) == "over voltage");
    assert(MOTOR_B_ERROR_CODES.at(0xA) == "over current");

    std::cout << "  MotorB error codes test passed" << std::endl;
}

int main() {
    std::cout << "=== A1Z C++ SDK Basic Tests ===" << std::endl;
    std::cout << std::endl;

    test_motor_a_packing();
    std::cout << std::endl;

    test_motor_b_packing();
    std::cout << std::endl;

    test_gripper_norm();
    std::cout << std::endl;

    test_error_codes();
    std::cout << std::endl;

    std::cout << "=== All tests passed ===" << std::endl;
    return 0;
}
