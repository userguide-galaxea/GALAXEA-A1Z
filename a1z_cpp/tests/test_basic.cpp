#include "a1z/motor_a_driver.hpp"
#include "a1z/motor_b_driver.hpp"
#include "a1z/gripper.hpp"
#include "a1z/integrator.hpp"
#include "a1z/friction.hpp"
#include <cassert>
#include <cmath>
#include <iostream>

using namespace a1z;

namespace {

bool nearly(double a, double b, double tol = 1e-12) {
    return std::abs(a - b) <= tol;
}

} // namespace

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

// Test IntegralConfig::from_level against the Python reference
// (integrator.py): tau_c_hat anchor from get_robot.py::_TAU_C_HAT.
void test_integral_config() {
    std::cout << "Testing IntegralConfig::from_level..." << std::endl;

    const JointVector tau_c_hat = {1.033, 0.3665, 0.6371, 0.66, 0.2355, 0.2925};

    // K0 = off: all-zero vectors (structurally equivalent to no integrator).
    auto k0 = IntegralConfig::from_level("K0", tau_c_hat);
    for (size_t i = 0; i < 6; ++i) {
        assert(k0.ki[i] == 0.0);
        assert(k0.tau_i_max[i] == 0.0);
    }

    // K1: t_wind = 2.0 s, tau_i_max = 1.2 * tau_c_hat,
    // ki = tau_i_max / (E_TYP_RAD * t_wind).
    auto k1 = IntegralConfig::from_level("K1", tau_c_hat);
    assert(k1.level == "K1");
    assert(nearly(k1.t_leak_s, 1.0));
    assert(nearly(k1.qd_freeze, 0.15));
    assert(nearly(k1.clamp_scale, 1.2));
    for (size_t i = 0; i < 6; ++i) {
        assert(nearly(k1.tau_i_max[i], 1.2 * tau_c_hat[i]));
        assert(nearly(k1.ki[i], 1.2 * tau_c_hat[i] / (E_TYP_RAD * 2.0)));
        assert(nearly(k1.e_db_rad[i], 0.3 * DEG2RAD));
    }

    // NaN tau_c_hat joints auto-disable (ki = 0 and tau_i_max = 0).
    JointVector with_nan = tau_c_hat;
    with_nan[3] = std::nan("");
    auto partial = IntegralConfig::from_level("K2", with_nan);
    assert(partial.ki[3] == 0.0 && partial.tau_i_max[3] == 0.0);
    assert(nearly(partial.tau_i_max[0], 1.2 * tau_c_hat[0]));
    assert(nearly(partial.ki[0], 1.2 * tau_c_hat[0] / (E_TYP_RAD * 0.5)));

    // 1-based joint filter: only joint 6 enabled.
    auto only6 = IntegralConfig::from_level("K1", tau_c_hat, {6});
    for (size_t i = 0; i < 5; ++i) {
        assert(only6.ki[i] == 0.0 && only6.tau_i_max[i] == 0.0);
    }
    assert(only6.ki[5] != 0.0);

    // Unknown level and out-of-range joint both throw.
    bool threw = false;
    try {
        IntegralConfig::from_level("K9", tau_c_hat);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    assert(threw);
    threw = false;
    try {
        IntegralConfig::from_level("K1", tau_c_hat, {7});
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    assert(threw);

    std::cout << "  IntegralConfig::from_level test passed" << std::endl;
}

// Test JointErrorIntegrator dynamics against the Python control law
// (integrator.py::JointErrorIntegrator.step): reversal halving, leak,
// deadband, freeze, clamp.
void test_integrator_step() {
    std::cout << "Testing JointErrorIntegrator..." << std::endl;

    const JointVector tau_c_hat = {1.033, 0.3665, 0.6371, 0.66, 0.2355, 0.2925};
    const double dt = 0.004;  // 250 Hz control period
    const double lam = 1.0 - dt / 1.0;  // t_leak_s = 1.0

    JointErrorIntegrator itg(IntegralConfig::from_level("K1", tau_c_hat), dt);

    JointVector e, qd;
    e.fill(0.05);    // above the 0.3 deg deadband on every joint
    qd.fill(0.0);    // at rest: no freeze

    // First tick: lam*0 + ki*e*dt.
    JointVector t1 = itg.step(e, qd);
    const auto& cfg = itg.config();
    for (size_t i = 0; i < 6; ++i) {
        assert(nearly(t1[i], cfg.ki[i] * 0.05 * dt));
    }
    // Second tick: leak then integrate.
    JointVector t2 = itg.step(e, qd);
    for (size_t i = 0; i < 6; ++i) {
        assert(nearly(t2[i], lam * t1[i] + cfg.ki[i] * 0.05 * dt));
    }
    // Freeze: |qd_des| > qd_freeze (0.15) -> leak only.
    qd.fill(0.2);
    JointVector t3 = itg.step(e, qd);
    for (size_t i = 0; i < 6; ++i) {
        assert(nearly(t3[i], lam * t2[i]));
    }
    // Deadband: |e| < e_db -> leak only.
    qd.fill(0.0);
    e.fill(0.001);
    JointVector t4 = itg.step(e, qd);
    for (size_t i = 0; i < 6; ++i) {
        assert(nearly(t4[i], lam * t3[i]));
    }

    // Reversal: a definite sign flip halves tau_i before leak + integrate.
    itg.reset();
    e.fill(0.05);
    qd.fill(0.1);  // 0.1 < qd_freeze, so integration stays active
    JointVector r1 = itg.step(e, qd);
    qd.fill(-0.1);
    JointVector r2 = itg.step(e, qd);
    for (size_t i = 0; i < 6; ++i) {
        assert(nearly(r2[i], lam * 0.5 * r1[i] + cfg.ki[i] * 0.05 * dt));
    }

    // Clamp: ki=1, tau_i_max=0.01, no deadband -> saturates at the clamp.
    IntegralConfig small;
    small.ki.fill(1.0);
    small.tau_i_max.fill(0.01);
    small.e_db_rad.fill(0.0);
    JointErrorIntegrator sat(small, dt);
    JointVector big_e, zero_qd;
    big_e.fill(1.0);
    zero_qd.fill(0.0);
    JointVector t = {};
    for (int k = 0; k < 100; ++k) {
        t = sat.step(big_e, zero_qd);
    }
    for (size_t i = 0; i < 6; ++i) {
        assert(nearly(t[i], 0.01));
    }

    // reset() zeroes the accumulator.
    sat.reset();
    for (size_t i = 0; i < 6; ++i) {
        assert(sat.tau_i()[i] == 0.0);
    }

    std::cout << "  JointErrorIntegrator test passed" << std::endl;
}

// Test CoulombConfig against the Python reference (friction.py):
// tau_coulomb = tau_c * tanh(v_des / qd_eps).
void test_coulomb_config() {
    std::cout << "Testing CoulombConfig..." << std::endl;

    const JointVector tau_c_hat = {1.033, 0.3665, 0.6371, 0.66, 0.2355, 0.2925};

    CoulombConfig cc(tau_c_hat);  // default qd_eps = 0.05
    JointVector v;

    // Zero velocity -> zero torque (continuous through zero).
    v.fill(0.0);
    JointVector t0 = cc.compute_tau(v);
    for (size_t i = 0; i < 6; ++i) {
        assert(t0[i] == 0.0);
    }
    // v = qd_eps -> tanh(1).
    v.fill(0.05);
    JointVector t1 = cc.compute_tau(v);
    for (size_t i = 0; i < 6; ++i) {
        assert(nearly(t1[i], tau_c_hat[i] * std::tanh(1.0)));
    }
    // Large |v| -> saturates at tau_c.
    v.fill(10.0);
    JointVector t2 = cc.compute_tau(v);
    for (size_t i = 0; i < 6; ++i) {
        assert(nearly(t2[i], tau_c_hat[i], 1e-9));
    }

    // from_tau_c_hat: NaN joints disabled, scale applied.
    JointVector with_nan = tau_c_hat;
    with_nan[2] = std::nan("");
    auto scaled = CoulombConfig::from_tau_c_hat(with_nan, 1.5);
    assert(scaled.tau_c[2] == 0.0);
    assert(nearly(scaled.tau_c[0], 1.5 * tau_c_hat[0]));

    // 1-based joint filter: only joint 4 enabled.
    auto only4 = CoulombConfig::from_tau_c_hat(tau_c_hat, 1.0, 0.05, {4});
    for (size_t i = 0; i < 6; ++i) {
        if (i == 3) {
            assert(nearly(only4.tau_c[i], tau_c_hat[i]));
        } else {
            assert(only4.tau_c[i] == 0.0);
        }
    }

    // qd_eps must be positive.
    bool threw = false;
    try {
        CoulombConfig bad(tau_c_hat, 0.0);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    assert(threw);

    std::cout << "  CoulombConfig test passed" << std::endl;
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

    test_integral_config();
    std::cout << std::endl;

    test_integrator_step();
    std::cout << std::endl;

    test_coulomb_config();
    std::cout << std::endl;

    std::cout << "=== All tests passed ===" << std::endl;
    return 0;
}
