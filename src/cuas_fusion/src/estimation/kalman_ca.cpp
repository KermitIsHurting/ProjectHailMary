// @file kalman_ca.cpp
// @brief Constant-acceleration Kalman filter implementation.
#include "cuas_fusion/estimation/kalman_ca.hpp"
#include "cuas_fusion/common/fixed_types.hpp"
#include "cuas_fusion/estimation/position_update.hpp"

#include <algorithm>
#include <cmath>

namespace cuas {

void KalmanCA::init(const Vector6d& x0, const Matrix6d& P0)
{
    x_ = Vector9d::Zero();
    x_.head<6>() = x0;

    P_ = Matrix9d::Identity();
    P_.topLeftCorner<6, 6>() = P0;
    initialized_ = true;
}

Matrix6d KalmanCA::getF(float64_t dt) const
{
    Matrix6d F6 = Matrix6d::Identity();
    F6(0, 3) = dt;
    F6(1, 4) = dt;
    F6(2, 5) = dt;
    return F6;
}

Matrix6d KalmanCA::getQ(float64_t dt) const
{
    const float64_t dt2 = dt * dt;
    const float64_t dt3 = dt2 * dt;
    const float64_t dt4 = dt3 * dt;
    const float64_t sj2 = sigma_j_ * sigma_j_;

    Matrix6d Q6 = Matrix6d::Zero();
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

    Matrix9d F = Matrix9d::Identity();
    for (int32_t i = 0; i < 3; ++i) {
        F(i, i + 3) = dt;
        F(i, i + 6) = dt2;
        F(i + 3, i + 6) = dt;
    }

    const float64_t sj2 = sigma_j_ * sigma_j_;
    const float64_t dt3 = dt * dt * dt;
    const float64_t dt4 = dt3 * dt;
    const float64_t dt5 = dt4 * dt;

    Matrix9d Q = Matrix9d::Zero();
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
    estimation::symmetrize<9>(P_);
}

void KalmanCA::update(const Eigen::Vector3d& z, const Eigen::Matrix3d& R)
{
    // Skipped (prediction kept) when S is not positive definite (B3).
    (void)estimation::positionUpdate<9>(x_, P_, z, R);
}

Vector6d KalmanCA::getState() const
{
    return x_.head<6>();
}

Matrix6d KalmanCA::getCovariance() const
{
    return P_.topLeftCorner<6, 6>();
}

float64_t KalmanCA::likelihood(const Eigen::Vector3d& z, const Eigen::Matrix3d& R) const
{
    return estimation::positionLikelihood<9>(x_, P_, z, R);
}

} // namespace cuas
