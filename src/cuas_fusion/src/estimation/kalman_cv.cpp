// @file kalman_cv.cpp
// @brief Constant-velocity Kalman filter implementation.
#include "cuas_fusion/estimation/kalman_cv.hpp"
#include "cuas_fusion/common/fixed_types.hpp"

#include <algorithm>
#include <cmath>

namespace cuas {

void KalmanCV::init(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0)
{
    x_ = x0.head(6);
    P_ = P0.topLeftCorner(6, 6);
    initialized_ = true;
}

Eigen::MatrixXd KalmanCV::getF(float64_t dt) const
{
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6, 6);
    F(0, 3) = dt;
    F(1, 4) = dt;
    F(2, 5) = dt;
    return F;
}

Eigen::MatrixXd KalmanCV::getQ(float64_t dt) const
{
    const float64_t dt2 = dt * dt;
    const float64_t dt3 = dt2 * dt;
    const float64_t dt4 = dt3 * dt;
    const float64_t sa2 = sigma_a_ * sigma_a_;

    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(6, 6);
    for (int32_t i = 0; i < 3; ++i) {
        Q(i, i)         = 0.25 * dt4 * sa2;
        Q(i, i + 3)     = 0.5  * dt3 * sa2;
        Q(i + 3, i)     = 0.5  * dt3 * sa2;
        Q(i + 3, i + 3) = dt2 * sa2;
    }
    return Q;
}

void KalmanCV::predict(float64_t dt)
{
    const Eigen::MatrixXd F = getF(dt);
    const Eigen::MatrixXd Q = getQ(dt);
    x_ = F * x_;
    P_ = F * P_ * F.transpose() + Q;
}

void KalmanCV::update(const Eigen::VectorXd& z, const Eigen::MatrixXd& R)
{
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 6);
    H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

    const Eigen::VectorXd y = z - H * x_;
    const Eigen::MatrixXd S = H * P_ * H.transpose() + R;
    const Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
    x_ = x_ + K * y;
    P_ = (Eigen::MatrixXd::Identity(6, 6) - K * H) * P_;
}

Eigen::VectorXd KalmanCV::getState() const { return x_; }

Eigen::MatrixXd KalmanCV::getCovariance() const { return P_; }

float64_t KalmanCV::likelihood(const Eigen::VectorXd& z, const Eigen::MatrixXd& R) const
{
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 6);
    H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

    const Eigen::VectorXd y = z - H * x_;
    const Eigen::MatrixXd S = H * P_ * H.transpose() + R;
    const float64_t det = S.determinant();
    if (det < 1e-12) {
        return 1e-12;
    }

    const float64_t n = static_cast<float64_t>(z.size());
    const float64_t exponent = -0.5 * (y.transpose() * S.inverse() * y)(0, 0);
    const float64_t norm = std::pow(2.0 * M_PI, n / 2.0) * std::sqrt(det);
    return std::max(std::exp(exponent) / norm, 1e-12);
}

} // namespace cuas
