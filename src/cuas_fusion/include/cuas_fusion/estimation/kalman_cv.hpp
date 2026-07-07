// @file kalman_cv.hpp
// @brief Constant-velocity Kalman filter with 6-state (pos, vel).
#pragma once

#include "cuas_fusion/common/fixed_types.hpp"

#include <Eigen/Dense>

namespace cuas {

class KalmanCV {
public:
    KalmanCV() = default;

    void init(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0);
    // IMM mixing injection: CV has no model-private state, so this is a full
    // overwrite; provided for API symmetry with KalmanCA/KalmanCT.
    void setMixedState(const Eigen::VectorXd& x6, const Eigen::MatrixXd& P6) {
        x_ = x6.head(6);
        P_ = P6.topLeftCorner(6, 6);
    }
    void predict(float64_t dt);
    void update(const Eigen::VectorXd& z, const Eigen::MatrixXd& R);
    Eigen::VectorXd getState() const;
    Eigen::MatrixXd getCovariance() const;
    float64_t likelihood(const Eigen::VectorXd& z, const Eigen::MatrixXd& R) const;
    Eigen::MatrixXd getF(float64_t dt) const;
    Eigen::MatrixXd getQ(float64_t dt) const;
    void setSigmaA(float64_t sigma) { sigma_a_ = sigma; }
    void setVelocity(const Eigen::Vector3d& v) { x_.segment<3>(3) = v; }

    static constexpr int32_t kStateDim = 6;

private:
    Eigen::VectorXd x_ = Eigen::VectorXd::Zero(6);
    Eigen::MatrixXd P_ = Eigen::MatrixXd::Identity(6, 6);
    bool initialized_ = false;

    float64_t sigma_a_ = 0.5;
};

} // namespace cuas
