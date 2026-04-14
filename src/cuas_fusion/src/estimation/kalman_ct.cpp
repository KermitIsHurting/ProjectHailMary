// @file kalman_ct.cpp
// @brief Coordinated-turn Kalman filter with omega-rate state.
#include "cuas_fusion/estimation/kalman_ct.hpp"
#include "cuas_fusion/common/fixed_types.hpp"

#include <algorithm>
#include <cmath>

namespace cuas {

void KalmanCT::init(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0)
{
    x_ = Eigen::VectorXd::Zero(7);
    x_.head(std::min<int32_t>(static_cast<int32_t>(x0.size()), 7)) =
        x0.head(std::min<int32_t>(static_cast<int32_t>(x0.size()), 7));

    P_ = Eigen::MatrixXd::Identity(7, 7);
    const int32_t sz = std::min<int32_t>(static_cast<int32_t>(P0.rows()), 7);
    P_.topLeftCorner(sz, sz) = P0.topLeftCorner(sz, sz);
    initialized_ = true;
}

void KalmanCT::predict(float64_t dt)
{
    const float64_t omega = x_(6);
    Eigen::VectorXd x_new(7);

    // Straight-line linearisation when turn rate is below the guard threshold
    if (std::abs(omega) < omega_guard_) {
        const float64_t vx = x_(3);
        const float64_t vy = x_(4);
        const float64_t vz = x_(5);
        x_new(0) = x_(0) + vx * dt;
        x_new(1) = x_(1) + vy * dt;
        x_new(2) = x_(2) + vz * dt;
        x_new(3) = vx;
        x_new(4) = vy;
        x_new(5) = vz;
        x_new(6) = omega;
    } else {
        const float64_t s  = std::sin(omega * dt);
        const float64_t c  = std::cos(omega * dt);
        const float64_t vx = x_(3);
        const float64_t vy = x_(4);
        const float64_t vz = x_(5);

        x_new(0) = x_(0) + (vx * s - vy * (1.0 - c)) / omega;
        x_new(1) = x_(1) + (vx * (1.0 - c) + vy * s) / omega;
        x_new(2) = x_(2) + vz * dt;
        x_new(3) = vx * c - vy * s;
        x_new(4) = vx * s + vy * c;
        x_new(5) = vz;
        x_new(6) = omega;
    }

    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(7, 7);
    if (std::abs(omega) < omega_guard_) {
        F(0, 3) = dt;
        F(1, 4) = dt;
        F(2, 5) = dt;
    } else {
        const float64_t s  = std::sin(omega * dt);
        const float64_t c  = std::cos(omega * dt);
        const float64_t vx = x_(3);
        const float64_t vy = x_(4);

        F(0, 3) = s / omega;
        F(0, 4) = -(1.0 - c) / omega;
        F(0, 6) = (vx * c * dt * omega - vx * s + vy * s * dt * omega - vy * (1.0 - c))
                   / (omega * omega);
        F(1, 3) = (1.0 - c) / omega;
        F(1, 4) = s / omega;
        F(1, 6) = (vx * s * dt * omega - vx * (1.0 - c) - vy * c * dt * omega + vy * s)
                   / (omega * omega);
        F(2, 5) = dt;
        F(3, 3) = c;
        F(3, 4) = -s;
        F(3, 6) = -vx * s * dt - vy * c * dt;
        F(4, 3) = s;
        F(4, 4) = c;
        F(4, 6) = vx * c * dt - vy * s * dt;
    }

    const float64_t sa2 = sigma_a_ * sigma_a_;
    const float64_t so2 = sigma_omega_ * sigma_omega_;
    const float64_t dt2 = dt * dt;

    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(7, 7);
    for (int32_t i = 0; i < 3; ++i) {
        Q(i, i)         = 0.25 * dt2 * dt2 * sa2;
        Q(i, i + 3)     = 0.5  * dt2 * dt  * sa2;
        Q(i + 3, i)     = 0.5  * dt2 * dt  * sa2;
        Q(i + 3, i + 3) = dt2 * sa2;
    }
    Q(6, 6) = dt2 * so2;

    x_ = x_new;
    P_ = F * P_ * F.transpose() + Q;
}

void KalmanCT::update(const Eigen::VectorXd& z, const Eigen::MatrixXd& R)
{
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 7);
    H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

    const Eigen::VectorXd y = z - H * x_;
    const Eigen::MatrixXd S = H * P_ * H.transpose() + R;
    const Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
    x_ = x_ + K * y;
    P_ = (Eigen::MatrixXd::Identity(7, 7) - K * H) * P_;
}

Eigen::VectorXd KalmanCT::getState() const
{
    return x_.head(6);
}

Eigen::MatrixXd KalmanCT::getCovariance() const
{
    return P_.topLeftCorner(6, 6);
}

Eigen::MatrixXd KalmanCT::getF(float64_t dt) const
{
    Eigen::MatrixXd F6 = Eigen::MatrixXd::Identity(6, 6);
    const float64_t omega = x_(6);
    if (std::abs(omega) < omega_guard_) {
        F6(0, 3) = dt;
        F6(1, 4) = dt;
        F6(2, 5) = dt;
    } else {
        const float64_t s = std::sin(omega * dt);
        const float64_t c = std::cos(omega * dt);
        F6(0, 3) = s / omega;
        F6(0, 4) = -(1.0 - c) / omega;
        F6(1, 3) = (1.0 - c) / omega;
        F6(1, 4) = s / omega;
        F6(2, 5) = dt;
        F6(3, 3) = c;
        F6(3, 4) = -s;
        F6(4, 3) = s;
        F6(4, 4) = c;
    }
    return F6;
}

Eigen::MatrixXd KalmanCT::getQ(float64_t dt) const
{
    const float64_t sa2 = sigma_a_ * sigma_a_;
    const float64_t dt2 = dt * dt;

    Eigen::MatrixXd Q6 = Eigen::MatrixXd::Zero(6, 6);
    for (int32_t i = 0; i < 3; ++i) {
        Q6(i, i)         = 0.25 * dt2 * dt2 * sa2;
        Q6(i, i + 3)     = 0.5  * dt2 * dt  * sa2;
        Q6(i + 3, i)     = 0.5  * dt2 * dt  * sa2;
        Q6(i + 3, i + 3) = dt2 * sa2;
    }
    return Q6;
}

float64_t KalmanCT::likelihood(const Eigen::VectorXd& z, const Eigen::MatrixXd& R) const
{
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 7);
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
