// @file kalman_cv.cpp
// @brief Constant-velocity Kalman filter implementation.
#include "cuas_fusion/estimation/kalman_cv.hpp"
#include "cuas_fusion/common/fixed_types.hpp"
#include "cuas_fusion/estimation/position_update.hpp"

#include <algorithm>
#include <cmath>

namespace cuas {

void KalmanCV::init(const Vector6d& x0, const Matrix6d& P0)
{
    x_ = x0;
    P_ = P0;
    initialized_ = true;
}

Matrix6d KalmanCV::getF(float64_t dt) const
{
    Matrix6d F = Matrix6d::Identity();
    F(0, 3) = dt;
    F(1, 4) = dt;
    F(2, 5) = dt;
    return F;
}

Matrix6d KalmanCV::getQ(float64_t dt) const
{
    const float64_t dt2 = dt * dt;
    const float64_t dt3 = dt2 * dt;
    const float64_t dt4 = dt3 * dt;
    const float64_t sa2 = sigma_a_ * sigma_a_;

    Matrix6d Q = Matrix6d::Zero();
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
    const Matrix6d F = getF(dt);
    const Matrix6d Q = getQ(dt);
    x_ = F * x_;
    P_ = F * P_ * F.transpose() + Q;
    estimation::symmetrize<6>(P_);
}

void KalmanCV::update(const Eigen::Vector3d& z, const Eigen::Matrix3d& R)
{
    // Skipped (prediction kept) when S is not positive definite (B3).
    (void)estimation::positionUpdate<6>(x_, P_, z, R);
}

Vector6d KalmanCV::getState() const { return x_; }

Matrix6d KalmanCV::getCovariance() const { return P_; }

float64_t KalmanCV::likelihood(const Eigen::Vector3d& z, const Eigen::Matrix3d& R) const
{
    return estimation::positionLikelihood<6>(x_, P_, z, R);
}

} // namespace cuas
