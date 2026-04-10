#include "cuas_fusion/estimation/kalman_cv.hpp"

#include <cmath>

namespace cuas {

void KalmanCV::init(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0)
{
    x_ = x0.head(6);
    P_ = P0.topLeftCorner(6, 6);
    initialized_ = true;
}

Eigen::MatrixXd KalmanCV::getF(double dt) const
{
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6, 6);
    F(0, 3) = dt;
    F(1, 4) = dt;
    F(2, 5) = dt;
    return F;
}

Eigen::MatrixXd KalmanCV::getQ(double dt) const
{
    double dt2 = dt * dt;
    double dt3 = dt2 * dt;
    double dt4 = dt3 * dt;
    double sa2 = sigma_a_ * sigma_a_;

    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(6, 6);
    for (int i = 0; i < 3; ++i) {
        Q(i, i)         = 0.25 * dt4 * sa2;
        Q(i, i + 3)     = 0.5  * dt3 * sa2;
        Q(i + 3, i)     = 0.5  * dt3 * sa2;
        Q(i + 3, i + 3) = dt2 * sa2;
    }
    return Q;
}

void KalmanCV::predict(double dt)
{
    Eigen::MatrixXd F = getF(dt);
    Eigen::MatrixXd Q = getQ(dt);
    x_ = F * x_;
    P_ = F * P_ * F.transpose() + Q;
}

void KalmanCV::update(const Eigen::VectorXd& z, const Eigen::MatrixXd& R)
{
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 6);
    H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

    Eigen::VectorXd y = z - H * x_;
    Eigen::MatrixXd S = H * P_ * H.transpose() + R;
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
    x_ = x_ + K * y;
    P_ = (Eigen::MatrixXd::Identity(6, 6) - K * H) * P_;
}

Eigen::VectorXd KalmanCV::getState() const { return x_; }

Eigen::MatrixXd KalmanCV::getCovariance() const { return P_; }

double KalmanCV::likelihood(const Eigen::VectorXd& z, const Eigen::MatrixXd& R) const
{
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 6);
    H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

    Eigen::VectorXd y = z - H * x_;
    Eigen::MatrixXd S = H * P_ * H.transpose() + R;
    double det = S.determinant();
    if (det < 1e-12) return 1e-12;

    double n = static_cast<double>(z.size());
    double exponent = -0.5 * (y.transpose() * S.inverse() * y)(0, 0);
    double norm = std::pow(2.0 * M_PI, n / 2.0) * std::sqrt(det);
    return std::max(std::exp(exponent) / norm, 1e-12);
}

} // namespace cuas
