#include "a1z/gravity_model.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace a1z {

GravityModel::GravityModel(const std::string& urdf_path, const std::string& mesh_dir) {
#ifdef A1Z_HAS_PINOCCHIO
    try {
        pinocchio::urdf::buildModel(urdf_path, model_);
        data_ = pinocchio::Data(model_);
        nq_ = model_.nq;
        nv_ = model_.nv;
        has_model_ = true;

        // Verify gravity direction
        const auto& g = model_.gravity.linear();
        if (std::abs(g[0]) > 1e-6 || std::abs(g[1]) > 1e-6 || g[2] > 0) {
            std::cerr << "[GravityModel] Warning: unexpected gravity vector: ["
                      << g[0] << ", " << g[1] << ", " << g[2] << "]" << std::endl;
        }
    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to load URDF: " + std::string(e.what()));
    }
#else
    (void)urdf_path;
    (void)mesh_dir;
    std::cerr << "[GravityModel] Warning: Pinocchio not available, gravity compensation disabled"
              << std::endl;
    has_model_ = false;
#endif
}

JointVector GravityModel::compute_gravity_torque(const JointVector& q) {
    JointVector zero = {};
    return compute_inverse_dynamics(q, zero, zero);
}

JointVector GravityModel::compute_inverse_dynamics(const JointVector& q,
                                                    const JointVector& qd,
                                                    const JointVector& qdd) {
    JointVector tau = {};

#ifdef A1Z_HAS_PINOCCHIO
    if (!has_model_) {
        return tau;
    }

    Eigen::VectorXd q_eigen(nq_), qd_eigen(nv_), qdd_eigen(nv_);
    for (int i = 0; i < nq_; ++i) q_eigen[i] = q[i];
    for (int i = 0; i < nv_; ++i) {
        qd_eigen[i] = qd[i];
        qdd_eigen[i] = qdd[i];
    }

    Eigen::VectorXd tau_eigen = pinocchio::rnea(model_, data_, q_eigen, qd_eigen, qdd_eigen);

    for (int i = 0; i < nv_ && i < 6; ++i) {
        tau[i] = tau_eigen[i];
    }
#else
    (void)q;
    (void)qd;
    (void)qdd;
#endif

    return tau;
}

std::array<std::pair<double, double>, 6> GravityModel::get_joint_limits() const {
    std::array<std::pair<double, double>, 6> limits;

#ifdef A1Z_HAS_PINOCCHIO
    if (has_model_) {
        for (int i = 0; i < nq_ && i < 6; ++i) {
            limits[i] = {model_.lowerPositionLimit[i], model_.upperPositionLimit[i]};
        }
        return limits;
    }
#endif

    // Default A1Z limits
    limits[0] = {-2.094, 2.094};   // J1
    limits[1] = {0.0, 3.142};      // J2
    limits[2] = {-3.142, 0.0};     // J3
    limits[3] = {-1.484, 1.484};   // J4
    limits[4] = {-1.484, 1.484};   // J5
    limits[5] = {-2.007, 2.007};   // J6
    return limits;
}

void GravityModel::set_gravity(const std::array<double, 3>& gravity) {
#ifdef A1Z_HAS_PINOCCHIO
    if (has_model_) {
        model_.gravity.linear(Eigen::Vector3d(gravity[0], gravity[1], gravity[2]));
    }
#else
    (void)gravity;
#endif
}

bool GravityModel::is_available() {
#ifdef A1Z_HAS_PINOCCHIO
    return true;
#else
    return false;
#endif
}

} // namespace a1z
