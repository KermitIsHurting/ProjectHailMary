// @file kalman_ca.hpp
// @brief Constant-acceleration Kalman filter with 9-state (pos, vel, acc).
#pragma once

#include "cuas_fusion/common/fixed_types.hpp"

#include <Eigen/Dense>

namespace cuas {

class KalmanCA {
public:
    KalmanCA() = default;

    void init(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0);
    // IMM mixing injection: overwrites only the shared (pos, vel) block.
    // Acceleration states x_(6..8), their covariance, and the cross terms
    // are preserved — re-init()ing here zeroed them every predict cycle and
    // degenerated this model to CV.
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
    void setSigmaJ(float64_t sigma) { sigma_j_ = sigma; }
    void setVelocity(const Eigen::Vector3d& v) { x_.segment<3>(3) = v; }

    static constexpr int32_t kStateDim = 9;

private:
    Eigen::VectorXd x_ = Eigen::VectorXd::Zero(9);
    Eigen::MatrixXd P_ = Eigen::MatrixXd::Identity(9, 9);
    bool initialized_ = false;

    float64_t sigma_j_ = 1.0;
};

} // namespace cuas
