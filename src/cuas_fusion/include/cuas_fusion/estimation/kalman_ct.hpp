// @file kalman_ct.hpp
// @brief Coordinated-turn Kalman filter with 7-state (pos, vel, omega).
#pragma once

#include "cuas_fusion/common/eigen_types.hpp"
#include "cuas_fusion/common/fixed_types.hpp"

#include <Eigen/Dense>

namespace cuas {

class KalmanCT {
public:
    KalmanCT() = default;

    void init(const Vector6d& x0, const Matrix6d& P0);
    // IMM mixing injection, block-diagonal (RC-34): the shared (pos, vel)
    // block takes the mixed estimate, the turn rate x_(6) keeps its
    // variance, and the cross terms are zeroed — see KalmanCA. predict()
    // rebuilds the omega coupling through F each cycle. (re-init()ing here
    // reset omega to 0, so this model could never develop a turn rate.)
    void setMixedState(const Vector6d& x6, const Matrix6d& P6) {
        x_.head<6>() = x6;
        P_.topLeftCorner<6, 6>() = P6;
        P_.topRightCorner<6, 1>().setZero();
        P_.bottomLeftCorner<1, 6>().setZero();
    }
    void predict(float64_t dt);
    void update(const Eigen::Vector3d& z, const Eigen::Matrix3d& R);
    Vector6d getState() const;
    Matrix6d getCovariance() const;
    float64_t likelihood(const Eigen::Vector3d& z, const Eigen::Matrix3d& R) const;
    Matrix6d getF(float64_t dt) const;
    Matrix6d getQ(float64_t dt) const;
    void setVelocity(const Eigen::Vector3d& v) { x_.segment<3>(3) = v; }

    static constexpr int32_t kStateDim = 7;

private:
    Vector7d x_ = Vector7d::Zero();
    Matrix7d P_ = Matrix7d::Identity();
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
