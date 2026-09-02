// @file kalman_ca.hpp
// @brief Constant-acceleration Kalman filter with 9-state (pos, vel, acc).
#pragma once

#include "cuas_fusion/common/eigen_types.hpp"
#include "cuas_fusion/common/fixed_types.hpp"

#include <Eigen/Dense>

namespace cuas {

class KalmanCA {
public:
    KalmanCA() = default;

    void init(const Vector6d& x0, const Matrix6d& P0);
    // IMM mixing injection, block-diagonal (RC-34): the shared (pos, vel)
    // block takes the mixed estimate, the private acceleration block keeps
    // its variance, and the cross terms are zeroed. A cross-covariance
    // between the NEW shared state and the OLD acceleration is not the
    // covariance of anything; keeping it left P indefinite within 5 s of
    // straight flight. PSD by construction; predict() rebuilds the coupling
    // through F. (re-init()ing here zeroed the acceleration every cycle and
    // degenerated this model to CV.)
    void setMixedState(const Vector6d& x6, const Matrix6d& P6) {
        x_.head<6>() = x6;
        P_.topLeftCorner<6, 6>() = P6;
        P_.topRightCorner<6, 3>().setZero();
        P_.bottomLeftCorner<3, 6>().setZero();
    }
    void predict(float64_t dt);
    void update(const Eigen::Vector3d& z, const Eigen::Matrix3d& R);
    Vector6d getState() const;
    Matrix6d getCovariance() const;
    float64_t likelihood(const Eigen::Vector3d& z, const Eigen::Matrix3d& R) const;
    Matrix6d getF(float64_t dt) const;
    Matrix6d getQ(float64_t dt) const;
    void setSigmaJ(float64_t sigma) { sigma_j_ = sigma; }
    void setVelocity(const Eigen::Vector3d& v) { x_.segment<3>(3) = v; }

    static constexpr int32_t kStateDim = 9;

private:
    Vector9d x_ = Vector9d::Zero();
    Matrix9d P_ = Matrix9d::Identity();
    bool initialized_ = false;

    float64_t sigma_j_ = 1.0;
};

} // namespace cuas
