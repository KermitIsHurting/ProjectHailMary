// @file kalman_cv.hpp
// @brief Constant-velocity Kalman filter with 6-state (pos, vel).
#pragma once

#include "cuas_fusion/common/eigen_types.hpp"
#include "cuas_fusion/common/fixed_types.hpp"

#include <Eigen/Dense>

namespace cuas {

class KalmanCV {
public:
    KalmanCV() = default;

    void init(const Vector6d& x0, const Matrix6d& P0);
    // IMM mixing injection: CV has no model-private state, so this is a full
    // overwrite; provided for API symmetry with KalmanCA/KalmanCT.
    void setMixedState(const Vector6d& x6, const Matrix6d& P6) {
        x_ = x6;
        P_ = P6;
    }
    void predict(float64_t dt);
    void update(const Eigen::Vector3d& z, const Eigen::Matrix3d& R);
    Vector6d getState() const;
    Matrix6d getCovariance() const;
    float64_t likelihood(const Eigen::Vector3d& z, const Eigen::Matrix3d& R) const;
    Matrix6d getF(float64_t dt) const;
    Matrix6d getQ(float64_t dt) const;
    void setSigmaA(float64_t sigma) { sigma_a_ = sigma; }
    void setVelocity(const Eigen::Vector3d& v) { x_.segment<3>(3) = v; }

    static constexpr int32_t kStateDim = 6;

private:
    Vector6d x_ = Vector6d::Zero();
    Matrix6d P_ = Matrix6d::Identity();
    bool initialized_ = false;

    float64_t sigma_a_ = 0.5;
};

} // namespace cuas
