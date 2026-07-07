// @file kalman_ct.hpp
// @brief Coordinated-turn Kalman filter with 7-state (pos, vel, omega).
#pragma once

#include "cuas_fusion/common/fixed_types.hpp"

#include <Eigen/Dense>

namespace cuas {

class KalmanCT {
public:
    KalmanCT() = default;

    void init(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0);
    // IMM mixing injection: overwrites only the shared (pos, vel) block.
    // Turn rate x_(6), its variance, and the cross terms are preserved —
    // re-init()ing here reset omega to 0 every predict cycle, so this model
    // could never develop a turn rate.
    void setMixedState(const Eigen::VectorXd& x6, const Eigen::MatrixXd& P6) {
        x_.head(6) = x6.head(6);
        P_.topLeftCorner(6, 6) = P6.topLeftCorner(6, 6);
    }
    void predict(float64_t dt);
    void update(const Eigen::VectorXd& z, const Eigen::MatrixXd& R);
    Eigen::VectorXd getState() const;
    Eigen::MatrixXd getCovariance() const;
    float64_t likelihood(const Eigen::VectorXd& z, const Eigen::MatrixXd& R) const;
    Eigen::MatrixXd getF(float64_t dt) const;
    Eigen::MatrixXd getQ(float64_t dt) const;
    void setVelocity(const Eigen::Vector3d& v) { x_.segment<3>(3) = v; }

    static constexpr int32_t kStateDim = 7;

private:
    Eigen::VectorXd x_ = Eigen::VectorXd::Zero(7);
    Eigen::MatrixXd P_ = Eigen::MatrixXd::Identity(7, 7);
    bool initialized_ = false;

    static constexpr float64_t sigma_a_     = 0.5;
    static constexpr float64_t sigma_omega_ = 0.1;
    // Below this, use the analytic omega->0 limit of the Jacobian. 1e-3 rad/s
    // (a full turn in ~105 min) is physically negligible for a C-UAS target;
    // the old 1e-6 guard let /omega^2 terms amplify by up to 1e12 at the
    // boundary, risking covariance blow-up.
    static constexpr float64_t omega_guard_ = 1e-3;
};

} // namespace cuas
