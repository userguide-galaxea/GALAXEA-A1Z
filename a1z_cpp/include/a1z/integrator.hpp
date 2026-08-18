#pragma once

// Per-joint error-integral feedforward (leaky integrator) for the A1Z arm.
// Header-only port of a1z/robots/integrator.py (SOP-09 §3). No hardware
// dependency; unit-testable. The host-side 250 Hz outer loop folds tau_i
// into the torque sum ahead of x joint_sign and torque_clip.
//
// Control law (SOP-09 §3 / devlog 2026-07-22 Q8(2)):
//
//   tau_i[k] = clamp(lam*tau_i[k-1] + ki*e[k]*dt, +/-tau_i_max),
//   e[k] = q_des[k] - q_meas[k]
//
// Anti-hunting set (all four required on hardware):
//   1. clamp    tau_i_max = 1.2 * tau_c_hat (per joint)
//   2. leak     lam = 1 - dt/T_leak (integral decays once at rest)
//   3. deadband |e| < e_db: leak only this tick (no quantization ripple)
//   4. schedule |qd_des| > qd_freeze freezes integration;
//               command-direction reversal halves tau_i

#include "a1z/types.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace a1z {

// deg2rad as a literal so we do not depend on M_PI under -std=c++17.
constexpr double DEG2RAD = 0.017453292519943295;  // pi / 180

// Typical steady-state error used for level calibration:
// e_typ = 0.5 deg (SOP-09 §3). Matches integrator.py::E_TYP_RAD.
constexpr double E_TYP_RAD = 0.5 * DEG2RAD;

/**
 * @brief Per-joint integrator configuration.
 *
 * Port of integrator.py::IntegralConfig. The active full vectors are the
 * only source of truth (SOP-09 P0-4).
 */
struct IntegralConfig {
    JointVector ki{};         // Nm/(rad*s); 0 = joint disabled
    JointVector tau_i_max{};  // Nm = clamp_scale * tau_c_hat (0 on disabled joints)
    JointVector e_db_rad{};   // error deadband (default 0.3 deg)
    double t_leak_s = 1.0;    // lam = 1 - dt/T_leak
    double qd_freeze = 0.15;  // rad/s: |qd_des| above this freezes integration
    std::string level = "K0"; // level name (bookkeeping)
    double clamp_scale = 1.2; // tau_i_max = clamp_scale * tau_c_hat

    /// Python __post_init__: disabled joints must carry a finite (zero)
    /// clamp, otherwise clamping would pollute tau_i with NaN.
    void validate() const {
        for (size_t i = 0; i < 6; ++i) {
            if (!std::isfinite(tau_i_max[i])) {
                throw std::invalid_argument(
                    "tau_i_max has non-finite entries; disabled joints must be "
                    "0 (use IntegralConfig::from_level which zeroes them)");
            }
        }
    }

    /**
     * @brief Build a config from a level. Port of IntegralConfig.from_level.
     *
     * ki = tau_i_max / (E_TYP_RAD * t_wind); joints with NaN tau_c_hat or
     * outside `joints` get ki=0 and tau_i_max=0 (enable-mask form).
     * K0 -> all zeros (equivalent to no integrator).
     *
     * Levels (t_wind s): K0 = off, K1 = 2.0, K2 = 0.5, K3 = 0.2.
     *
     * @param joints 1-based joint indices to enable; empty = all.
     * @param t_wind_s Continuous override for the level's t_wind (s);
     *                 std::nullopt (default) uses the level's canonical value.
     * @param clamp_scale tau_i_max = clamp_scale * tau_c_hat (default 1.2).
     */
    static IntegralConfig from_level(
        const std::string& level,
        const JointVector& tau_c_hat,
        const std::vector<int>& joints = {},
        double t_leak_s = 1.0,
        double e_db_deg = 0.3,
        double qd_freeze = 0.15,
        std::optional<double> t_wind_s = std::nullopt,
        double clamp_scale = 1.2) {
        // LEVELS: K0 -> off (ki=0), K1 -> 2.0, K2 -> 0.5, K3 -> 0.2.
        std::optional<double> level_t_wind;
        if (level == "K0") {
            level_t_wind = std::nullopt;
        } else if (level == "K1") {
            level_t_wind = 2.0;
        } else if (level == "K2") {
            level_t_wind = 0.5;
        } else if (level == "K3") {
            level_t_wind = 0.2;
        } else {
            throw std::invalid_argument(
                "unknown integral level '" + level +
                "'; choose from {K0, K1, K2, K3}");
        }

        // 1-based joint numbers -> enable mask; empty = all joints.
        std::array<bool, 6> mask;
        mask.fill(joints.empty());
        for (int j : joints) {
            if (j < 1 || j > 6) {
                throw std::invalid_argument("integral joint out of range 1..6");
            }
            mask[j - 1] = true;
        }

        std::optional<double> t_wind =
            t_wind_s.has_value() ? t_wind_s : level_t_wind;

        IntegralConfig cfg;
        cfg.t_leak_s = t_leak_s;
        cfg.qd_freeze = qd_freeze;
        cfg.level = level;
        cfg.clamp_scale = clamp_scale;
        for (size_t i = 0; i < 6; ++i) {
            bool enabled = std::isfinite(tau_c_hat[i]) && mask[i];
            double tau_i_max = enabled ? clamp_scale * tau_c_hat[i] : 0.0;
            double ki = 0.0;
            if (t_wind.has_value()) {  // not K0
                ki = enabled ? tau_i_max / (E_TYP_RAD * (*t_wind)) : 0.0;
            }
            // Keep ki / tau_i_max consistent: a joint with ki==0 (K0, or
            // unstandardised / out-of-scope) carries no clamp either.
            cfg.ki[i] = ki;
            cfg.tau_i_max[i] = (ki != 0.0) ? tau_i_max : 0.0;
            cfg.e_db_rad[i] = e_db_deg * DEG2RAD;
        }
        cfg.validate();
        return cfg;
    }
};

/**
 * @brief tau_i[k] = clamp(lam*tau_i[k-1] + ki*e[k]*dt, +/-tau_i_max) with
 * the full anti-hunting set (SOP-09 §3).
 *
 * Port of integrator.py::JointErrorIntegrator. Not thread-safe by itself;
 * ArmRobot drives it from the control thread only.
 */
class JointErrorIntegrator {
public:
    JointErrorIntegrator(IntegralConfig cfg, double dt)
        : cfg_(std::move(cfg)), dt_(dt) {
        if (dt_ <= 0.0) {
            throw std::invalid_argument("dt must be > 0");
        }
        cfg_.validate();
        // Leak coefficient lam = 1 - dt/T_leak, clamped to [0, 1] so the
        // degenerate T_leak < dt case never amplifies.
        lam_ = std::clamp(1.0 - dt_ / cfg_.t_leak_s, 0.0, 1.0);
        for (size_t i = 0; i < 6; ++i) {
            enabled_[i] = cfg_.ki[i] != 0.0;
        }
        reset();
    }

    /// Zero the integral state (non-streaming entry / estop / config swap).
    void reset() {
        tau_i_.fill(0.0);
        last_qd_sign_.fill(0.0);
    }

    /**
     * @brief Advance one tick and return tau_i.
     * @param e q_des - q_meas (URDF frame)
     * @param qd_des commanded velocity
     */
    JointVector step(const JointVector& e, const JointVector& qd_des) {
        for (size_t i = 0; i < 6; ++i) {
            // Reversal: joints whose commanded velocity sign flipped get
            // tau_i <- tau_i/2 (drag-tail suppression).
            double cur =
                (qd_des[i] > 0.0) ? 1.0 : (qd_des[i] < 0.0 ? -1.0 : 0.0);
            if (cur != 0.0 && last_qd_sign_[i] != 0.0 &&
                cur != last_qd_sign_[i]) {
                tau_i_[i] *= 0.5;
            }
            // Only update the memory on a definite sign, keeping the last
            // non-zero sign (a stationary segment is not a reversal).
            if (cur != 0.0) {
                last_qd_sign_[i] = cur;
            }
        }

        // Leak: the integral decays automatically once at rest (always on).
        for (size_t i = 0; i < 6; ++i) {
            tau_i_[i] *= lam_;
        }

        // Deadband + freeze: joints with |e| < e_db or |qd_des| > qd_freeze
        // leak only this tick, never integrate.
        for (size_t i = 0; i < 6; ++i) {
            bool freeze = std::abs(qd_des[i]) > cfg_.qd_freeze;
            bool deadband = std::abs(e[i]) < cfg_.e_db_rad[i];
            if (enabled_[i] && !freeze && !deadband) {
                tau_i_[i] += cfg_.ki[i] * e[i] * dt_;
            }
        }

        // Clamp: windup risk is capped mechanically.
        for (size_t i = 0; i < 6; ++i) {
            tau_i_[i] =
                std::clamp(tau_i_[i], -cfg_.tau_i_max[i], cfg_.tau_i_max[i]);
        }
        return tau_i_;
    }

    JointVector tau_i() const { return tau_i_; }
    const IntegralConfig& config() const { return cfg_; }

private:
    IntegralConfig cfg_;
    double dt_;
    double lam_;
    std::array<bool, 6> enabled_{};
    JointVector tau_i_{};
    JointVector last_qd_sign_{};
};

} // namespace a1z
