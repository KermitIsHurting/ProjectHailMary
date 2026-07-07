// @file kalman_cv.cpp
// @brief Constant-velocity Kalman filter implementation.
#include "cuas_fusion/estimation/kalman_cv.hpp"
#include "cuas_fusion/common/fixed_types.hpp"

#include <algorithm>
#include <cmath>

namespace cuas {

namespace {
using Matrix3x6 = Eigen::Matrix<float64_t, 3, 6>;
using Matrix6x3 = Eigen::Matrix<float64_t, 6, 3>;
} // namespace

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
}

void KalmanCV::update(const Eigen::Vector3d& z, const Eigen::Matrix3d& R)
{
    Matrix3x6 H = Matrix3x6::Zero();
    H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

    const Eigen::Vector3d y = z - H * x_;
    const Eigen::Matrix3d S = H * P_ * H.transpose() + R;
    const Matrix6x3 K = P_ * H.transpose() * S.inverse();
    x_ = x_ + K * y;
    P_ = (Matrix6d::Identity() - K * H) * P_;
}

Vector6d KalmanCV::getState() const { return x_; }

Matrix6d KalmanCV::getCovariance() const { return P_; }

float64_t KalmanCV::likelihood(const Eigen::Vector3d& z, const Eigen::Matrix3d& R) const
{
    Matrix3x6 H = Matrix3x6::Zero();
    H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

    const Eigen::Vector3d y = z - H * x_;
    const Eigen::Matrix3d S = H * P_ * H.transpose() + R;
    // Negated comparison so a NaN determinant (numerically broken S) takes
    // the floor branch instead of flowing into exp(NaN) and poisoning the
    // IMM weights permanently (Dir 0.3.1).
    const float64_t det = S.determinant();
    if (!(det > 1e-12)) {
        return 1e-12;
    }

    const float64_t n = 3.0;
    const float64_t exponent = -0.5 * (y.transpose() * S.inverse() * y)(0, 0);
    const float64_t norm = std::pow(2.0 * M_PI, n / 2.0) * std::sqrt(det);
    return std::max(std::exp(exponent) / norm, 1e-12);
}

} // namespace cuas
