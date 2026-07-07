// @file kalman_ca.cpp
// @brief Constant-acceleration Kalman filter implementation.
#include "cuas_fusion/estimation/kalman_ca.hpp"
#include "cuas_fusion/common/fixed_types.hpp"

#include <algorithm>
#include <cmath>

namespace cuas {

void KalmanCA::init(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0)
{
    x_ = Eigen::VectorXd::Zero(9);
    x_.head(std::min<int32_t>(static_cast<int32_t>(x0.size()), 9)) =
        x0.head(std::min<int32_t>(static_cast<int32_t>(x0.size()), 9));

    P_ = Eigen::MatrixXd::Identity(9, 9);
    const int32_t sz = std::min<int32_t>(static_cast<int32_t>(P0.rows()), 9);
    P_.topLeftCorner(sz, sz) = P0.topLeftCorner(sz, sz);
    initialized_ = true;
}

Eigen::MatrixXd KalmanCA::getF(float64_t dt) const
{
    Eigen::MatrixXd F6 = Eigen::MatrixXd::Identity(6, 6);
    F6(0, 3) = dt;
    F6(1, 4) = dt;
    F6(2, 5) = dt;
    return F6;
}

Eigen::MatrixXd KalmanCA::getQ(float64_t dt) const
{
    const float64_t dt2 = dt * dt;
    const float64_t dt3 = dt2 * dt;
    const float64_t dt4 = dt3 * dt;
    const float64_t sj2 = sigma_j_ * sigma_j_;

    Eigen::MatrixXd Q6 = Eigen::MatrixXd::Zero(6, 6);
    for (int32_t i = 0; i < 3; ++i) {
        Q6(i, i)         = 0.25 * dt4 * sj2;
        Q6(i, i + 3)     = 0.5  * dt3 * sj2;
        Q6(i + 3, i)     = 0.5  * dt3 * sj2;
        Q6(i + 3, i + 3) = dt2 * sj2;
    }
    return Q6;
}

void KalmanCA::predict(float64_t dt)
{
    const float64_t dt2 = 0.5 * dt * dt;

    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(9, 9);
    for (int32_t i = 0; i < 3; ++i) {
        F(i, i + 3) = dt;
        F(i, i + 6) = dt2;
        F(i + 3, i + 6) = dt;
    }

    const float64_t sj2 = sigma_j_ * sigma_j_;
    const float64_t dt3 = dt * dt * dt;
    const float64_t dt4 = dt3 * dt;
    const float64_t dt5 = dt4 * dt;

    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(9, 9);
    for (int32_t i = 0; i < 3; ++i) {
        Q(i, i)         = dt5 / 20.0 * sj2;
        Q(i, i + 3)     = dt4 / 8.0  * sj2;
        Q(i, i + 6)     = dt3 / 6.0  * sj2;
        Q(i + 3, i)     = dt4 / 8.0  * sj2;
        Q(i + 3, i + 3) = dt3 / 3.0  * sj2;
        Q(i + 3, i + 6) = dt * dt / 2.0 * sj2;
        Q(i + 6, i)     = dt3 / 6.0  * sj2;
        Q(i + 6, i + 3) = dt * dt / 2.0 * sj2;
        Q(i + 6, i + 6) = dt * sj2;
    }

    x_ = F * x_;
    P_ = F * P_ * F.transpose() + Q;
}

void KalmanCA::update(const Eigen::VectorXd& z, const Eigen::MatrixXd& R)
{
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 9);
    H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

    const Eigen::VectorXd y = z - H * x_;
    const Eigen::MatrixXd S = H * P_ * H.transpose() + R;
    const Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
    x_ = x_ + K * y;
    P_ = (Eigen::MatrixXd::Identity(9, 9) - K * H) * P_;
}

Eigen::VectorXd KalmanCA::getState() const
{
    return x_.head(6);
}

Eigen::MatrixXd KalmanCA::getCovariance() const
{
    return P_.topLeftCorner(6, 6);
}

float64_t KalmanCA::likelihood(const Eigen::VectorXd& z, const Eigen::MatrixXd& R) const
{
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 9);
    H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

    const Eigen::VectorXd y = z - H * x_;
    const Eigen::MatrixXd S = H * P_ * H.transpose() + R;
    // Negated comparison so a NaN determinant (numerically broken S) takes
    // the floor branch instead of flowing into exp(NaN) and poisoning the
    // IMM weights permanently (Dir 0.3.1).
    const float64_t det = S.determinant();
    if (!(det > 1e-12)) {
        return 1e-12;
    }

    const float64_t n = static_cast<float64_t>(z.size());
    const float64_t exponent = -0.5 * (y.transpose() * S.inverse() * y)(0, 0);
    const float64_t norm = std::pow(2.0 * M_PI, n / 2.0) * std::sqrt(det);
    return std::max(std::exp(exponent) / norm, 1e-12);
}

} // namespace cuas
