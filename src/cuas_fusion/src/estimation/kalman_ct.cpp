// @file kalman_ct.cpp
// @brief Coordinated-turn Kalman filter with omega-rate state.
#include "cuas_fusion/estimation/kalman_ct.hpp"
#include "cuas_fusion/common/fixed_types.hpp"
#include "cuas_fusion/estimation/position_update.hpp"

#include <algorithm>
#include <cmath>

namespace cuas {

void KalmanCT::init(const Vector6d& x0, const Matrix6d& P0)
{
    x_ = Vector7d::Zero();
    x_.head<6>() = x0;

    P_ = Matrix7d::Identity();
    P_.topLeftCorner<6, 6>() = P0;
    initialized_ = true;
}

void KalmanCT::predict(float64_t dt)
{
    const float64_t omega = x_(6);
    Vector7d x_new;

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

    Matrix7d F = Matrix7d::Identity();
    if (std::abs(omega) < omega_guard_) {
        const float64_t vx = x_(3);
        const float64_t vy = x_(4);
        F(0, 3) = dt;
        F(1, 4) = dt;
        F(2, 5) = dt;
        // Analytic omega->0 limits of the coupling terms. Without them the
        // position/velocity covariance never correlates with omega, so a
        // filter starting at omega = 0 could never estimate a turn rate from
        // position measurements at all.
        F(0, 6) = -0.5 * vy * dt * dt;
        F(1, 6) =  0.5 * vx * dt * dt;
        F(3, 6) = -vy * dt;
        F(4, 6) =  vx * dt;
    } else {
        const float64_t s  = std::sin(omega * dt);
        const float64_t c  = std::cos(omega * dt);
        const float64_t vx = x_(3);
        const float64_t vy = x_(4);

        F(0, 3) = s / omega;
        F(0, 4) = -(1.0 - c) / omega;
        // d(px')/domega by quotient rule on (vx*s - vy*(1-c))/omega:
        // (N' * omega - N) / omega^2 with N' = vx*dt*c - vy*dt*s.
        // The previous version had the vy terms sign-flipped, biasing the
        // omega update whenever the track had a cross-track velocity
        // component (Taylor limit check: must approach -vy*dt^2/2, the
        // guard-branch value above).
        F(0, 6) = (vx * c * dt * omega - vx * s - vy * s * dt * omega + vy * (1.0 - c))
                   / (omega * omega);
        F(1, 3) = (1.0 - c) / omega;
        F(1, 4) = s / omega;
        // d(py')/domega on (vx*(1-c) + vy*s)/omega, N' = vx*dt*s + vy*dt*c.
        F(1, 6) = (vx * s * dt * omega - vx * (1.0 - c) + vy * c * dt * omega - vy * s)
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

    Matrix7d Q = Matrix7d::Zero();
    for (int32_t i = 0; i < 3; ++i) {
        Q(i, i)         = 0.25 * dt2 * dt2 * sa2;
        Q(i, i + 3)     = 0.5  * dt2 * dt  * sa2;
        Q(i + 3, i)     = 0.5  * dt2 * dt  * sa2;
        Q(i + 3, i + 3) = dt2 * sa2;
    }
    Q(6, 6) = dt2 * so2;

    x_ = x_new;
    P_ = F * P_ * F.transpose() + Q;
    estimation::symmetrize<7>(P_);
}

void KalmanCT::update(const Eigen::Vector3d& z, const Eigen::Matrix3d& R)
{
    // Skipped (prediction kept) when S is not positive definite (B3).
    (void)estimation::positionUpdate<7>(x_, P_, z, R);
}

Vector6d KalmanCT::getState() const
{
    return x_.head<6>();
}

Matrix6d KalmanCT::getCovariance() const
{
    return P_.topLeftCorner<6, 6>();
}

Matrix6d KalmanCT::getF(float64_t dt) const
{
    Matrix6d F6 = Matrix6d::Identity();
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

Matrix6d KalmanCT::getQ(float64_t dt) const
{
    const float64_t sa2 = sigma_a_ * sigma_a_;
    const float64_t dt2 = dt * dt;

    Matrix6d Q6 = Matrix6d::Zero();
    for (int32_t i = 0; i < 3; ++i) {
        Q6(i, i)         = 0.25 * dt2 * dt2 * sa2;
        Q6(i, i + 3)     = 0.5  * dt2 * dt  * sa2;
        Q6(i + 3, i)     = 0.5  * dt2 * dt  * sa2;
        Q6(i + 3, i + 3) = dt2 * sa2;
    }
    return Q6;
}

float64_t KalmanCT::likelihood(const Eigen::Vector3d& z, const Eigen::Matrix3d& R) const
{
    return estimation::positionLikelihood<7>(x_, P_, z, R);
}

} // namespace cuas
