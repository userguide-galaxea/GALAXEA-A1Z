#pragma once

#include "a1z/types.hpp"

#ifdef A1Z_HAS_PINOCCHIO
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#endif

#include <string>

namespace a1z {

/**
 * @brief Gravity compensation model using Pinocchio RNEA.
 *
 * When Pinocchio is available, computes full inverse dynamics.
 * When not available, provides a stub implementation that returns zeros.
 */
class GravityModel {
public:
    /**
     * @brief Load URDF and build dynamics model.
     * @param urdf_path Path to URDF file
     * @param mesh_dir Root directory for mesh resources (optional)
     */
    explicit GravityModel(const std::string& urdf_path, const std::string& mesh_dir = "");

    /**
     * @brief Compute gravity compensation torque tau_g(q).
     * @param q Joint angles (rad), shape (6,)
     * @return Gravity torques (Nm), shape (6,)
     */
    JointVector compute_gravity_torque(const JointVector& q);

    /**
     * @brief Compute full inverse dynamics torque via RNEA.
     * @param q Joint positions (rad)
     * @param qd Joint velocities (rad/s)
     * @param qdd Joint accelerations (rad/s^2)
     * @return Inverse dynamics torques (Nm)
     */
    JointVector compute_inverse_dynamics(const JointVector& q,
                                          const JointVector& qd,
                                          const JointVector& qdd);

    /**
     * @brief Get joint limits [lower, upper] for each joint.
     */
    std::array<std::pair<double, double>, 6> get_joint_limits() const;

    /**
     * @brief Set gravity vector (for non-standard mounting).
     */
    void set_gravity(const std::array<double, 3>& gravity);

    /**
     * @brief Check if Pinocchio is available.
     */
    static bool is_available();

private:
#ifdef A1Z_HAS_PINOCCHIO
    pinocchio::Model model_;
    pinocchio::Data data_;
#endif
    bool has_model_ = false;
    int nq_ = 6;
    int nv_ = 6;
};

} // namespace a1z
