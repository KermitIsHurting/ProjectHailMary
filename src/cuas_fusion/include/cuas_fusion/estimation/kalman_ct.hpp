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
    void predict(float64_t dt);
    void update(const Eigen::VectorXd& z, const Eigen::MatrixXd& R);
    Eigen::VectorXd getState() const;
    Eigen::MatrixXd getCovariance() const;
    float64_t likelihood(const Eigen::VectorXd& z, const Eigen::MatrixXd& R) const;
    Eigen::MatrixXd getF(float64_t dt) const;
    Eigen::MatrixXd getQ(float64_t dt) const;

    static constexpr int32_t kStateDim = 7;

private:
    Eigen::VectorXd x_ = Eigen::VectorXd::Zero(7);
    Eigen::MatrixXd P_ = Eigen::MatrixXd::Identity(7, 7);
    bool initialized_ = false;

    static constexpr float64_t sigma_a_     = 0.5;
    static constexpr float64_t sigma_omega_ = 0.1;
    static constexpr float64_t omega_guard_ = 1e-6;  // below this, linearise around zero turn
};

} // namespace cuas
