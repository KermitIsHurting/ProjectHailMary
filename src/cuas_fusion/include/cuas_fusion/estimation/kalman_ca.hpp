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
    void predict(float64_t dt);
    void update(const Eigen::VectorXd& z, const Eigen::MatrixXd& R);
    Eigen::VectorXd getState() const;
    Eigen::MatrixXd getCovariance() const;
    float64_t likelihood(const Eigen::VectorXd& z, const Eigen::MatrixXd& R) const;
    Eigen::MatrixXd getF(float64_t dt) const;
    Eigen::MatrixXd getQ(float64_t dt) const;
    void setSigmaJ(float64_t sigma) { sigma_j_ = sigma; }

    static constexpr int32_t kStateDim = 9;

private:
    Eigen::VectorXd x_ = Eigen::VectorXd::Zero(9);
    Eigen::MatrixXd P_ = Eigen::MatrixXd::Identity(9, 9);
    bool initialized_ = false;

    float64_t sigma_j_ = 1.0;
};

} // namespace cuas
