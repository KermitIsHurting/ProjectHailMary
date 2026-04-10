#include "cuas_fusion/estimation/kalman_ca.hpp"

#include <cmath>

namespace cuas {

void KalmanCA::init(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0)
{
    x_ = Eigen::VectorXd::Zero(9);
    x_.head(std::min<int>(x0.size(), 9)) = x0.head(std::min<int>(x0.size(), 9));

    P_ = Eigen::MatrixXd::Identity(9, 9);
    int sz = std::min<int>(P0.rows(), 9);
    P_.topLeftCorner(sz, sz) = P0.topLeftCorner(sz, sz);
    initialized_ = true;
}

Eigen::MatrixXd KalmanCA::getF(double dt) const
{
    double dt2 = 0.5 * dt * dt;
    Eigen::MatrixXd F6 = Eigen::MatrixXd::Identity(6, 6);
    F6(0, 3) = dt;
    F6(1, 4) = dt;
    F6(2, 5) = dt;
    return F6;
}

Eigen::MatrixXd KalmanCA::getQ(double dt) const
{
    double dt2 = dt * dt;
    double dt3 = dt2 * dt;
    double dt4 = dt3 * dt;
    double sj2 = sigma_j_ * sigma_j_;

    Eigen::MatrixXd Q6 = Eigen::MatrixXd::Zero(6, 6);
    for (int i = 0; i < 3; ++i) {
        Q6(i, i)         = 0.25 * dt4 * sj2;
        Q6(i, i + 3)     = 0.5  * dt3 * sj2;
        Q6(i + 3, i)     = 0.5  * dt3 * sj2;
        Q6(i + 3, i + 3) = dt2 * sj2;
    }
    return Q6;
}

void KalmanCA::predict(double dt)
{
    double dt2 = 0.5 * dt * dt;

    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(9, 9);
    for (int i = 0; i < 3; ++i) {
        F(i, i + 3) = dt;
        F(i, i + 6) = dt2;
        F(i + 3, i + 6) = dt;
    }

    double sj2 = sigma_j_ * sigma_j_;
    double dt3 = dt * dt * dt;
    double dt4 = dt3 * dt;
    double dt5 = dt4 * dt;

    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(9, 9);
    for (int i = 0; i < 3; ++i) {
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

    Eigen::VectorXd y = z - H * x_;
    Eigen::MatrixXd S = H * P_ * H.transpose() + R;
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
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

double KalmanCA::likelihood(const Eigen::VectorXd& z, const Eigen::MatrixXd& R) const
{
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 9);
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
