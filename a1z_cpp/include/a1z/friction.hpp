#pragma once

// CoulombConfig — config object for tanh-smoothed Coulomb friction
// feedforward. Header-only port of a1z/robots/friction.py (SOP-11 §7.2):
// per-joint tau_c + smooth-sign bandwidth qd_eps + enable mask. The legacy
// bare-array hard-sign path stays in ArmRobot (set_coulomb_ff).

#include "a1z/types.hpp"

#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace a1z {

/**
 * @brief Per-joint Coulomb friction feedforward configuration.
 *
 * Port of friction.py::CoulombConfig. tau_c is in Nm, 0 = disabled for
 * that joint; qd_eps is the velocity bandwidth for tanh smoothing (rad/s):
 * tau_coulomb = tau_c * tanh(v_des / qd_eps).
 */
struct CoulombConfig {
    JointVector tau_c{};
    double qd_eps = 0.05;

    CoulombConfig() = default;

    /// @param enable_mask joints with false have tau_c forced to 0
    ///                    (Python __post_init__).
    explicit CoulombConfig(const JointVector& tau_c_vec, double eps = 0.05,
                           const std::array<bool, 6>* enable_mask = nullptr)
        : tau_c(tau_c_vec), qd_eps(eps) {
        if (enable_mask != nullptr) {
            for (size_t i = 0; i < 6; ++i) {
                if (!(*enable_mask)[i]) {
                    tau_c[i] = 0.0;
                }
            }
        }
        if (qd_eps <= 0.0) {
            throw std::invalid_argument("qd_eps must be positive");
        }
    }

    /**
     * @brief Construct from static Coulomb torque estimates (_TAU_C_HAT).
     * Port of CoulombConfig.from_tau_c_hat.
     *
     * @param scale multiplier (default 1.0 = use the estimate as-is)
     * @param joints 1-based joint indices to enable (empty = all finite)
     */
    static CoulombConfig from_tau_c_hat(const JointVector& tau_c_hat,
                                        double scale = 1.0,
                                        double qd_eps = 0.05,
                                        const std::vector<int>& joints = {}) {
        std::array<bool, 6> mask;
        for (size_t i = 0; i < 6; ++i) {
            mask[i] = std::isfinite(tau_c_hat[i]);
        }
        if (!joints.empty()) {
            std::array<bool, 6> jmask{};
            for (int j : joints) {
                if (j < 1 || j > 6) {
                    throw std::invalid_argument(
                        "coulomb joint out of range 1..6");
                }
                jmask[j - 1] = true;
            }
            for (size_t i = 0; i < 6; ++i) {
                mask[i] = mask[i] && jmask[i];
            }
        }
        JointVector tau_c;
        for (size_t i = 0; i < 6; ++i) {
            tau_c[i] = mask[i] ? tau_c_hat[i] * scale : 0.0;
        }
        return CoulombConfig(tau_c, qd_eps, &mask);
    }

    /**
     * @brief Coulomb feedforward torque for commanded velocity v_des.
     *
     * Uses tanh(v_des / qd_eps) instead of hard sign(e), so the torque is
     * continuous through zero velocity (SOP-11 §7.2).
     */
    JointVector compute_tau(const JointVector& v_des) const {
        JointVector out;
        for (size_t i = 0; i < 6; ++i) {
            out[i] = tau_c[i] * std::tanh(v_des[i] / qd_eps);
        }
        return out;
    }
};

} // namespace a1z
