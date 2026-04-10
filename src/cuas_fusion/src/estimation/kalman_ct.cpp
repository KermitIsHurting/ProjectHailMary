#include "cuas_fusion/estimation/kalman_ct.hpp"

#include <cmath>

namespace cuas {

void KalmanCT::init(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0)
{
    x_ = Eigen::VectorXd::Zero(7);
    x_.head(std::min<int>(x0.size(), 7)) = x0.head(std::min<int>(x0.size(), 7));

    P_ = Eigen::MatrixXd::Identity(7, 7);
    int sz = std::min<int>(P0.rows(), 7);
    P_.topLeftCorner(sz, sz) = P0.topLeftCorner(sz, sz);
    initialized_ = true;
}

void KalmanCT::predict(double dt)
{
    double omega = x_(6);
    Eigen::VectorXd x_new(7);

    if (std::abs(omega) < omega_guard_) {
        double vx = x_(3), vy = x_(4), vz = x_(5);
        x_new(0) = x_(0) + vx * dt;
        x_new(1) = x_(1) + vy * dt;
        x_new(2) = x_(2) + vz * dt;
        x_new(3) = vx;
        x_new(4) = vy;
        x_new(5) = vz;
        x_new(6) = omega;
    } else {
        double s = std::sin(omega * dt);
        double c = std::cos(omega * dt);
        double vx = x_(3), vy = x_(4), vz = x_(5);

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
        double s = std::sin(omega * dt);
        double c = std::cos(omega * dt);
        double vx = x_(3), vy = x_(4);

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

    double sa2 = sigma_a_ * sigma_a_;
    double so2 = sigma_omega_ * sigma_omega_;
    double dt2 = dt * dt;

    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(7, 7);
    for (int i = 0; i < 3; ++i) {
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

    Eigen::VectorXd y = z - H * x_;
    Eigen::MatrixXd S = H * P_ * H.transpose() + R;
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
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

Eigen::MatrixXd KalmanCT::getF(double dt) const
{
    Eigen::MatrixXd F6 = Eigen::MatrixXd::Identity(6, 6);
    double omega = x_(6);
    if (std::abs(omega) < omega_guard_) {
        F6(0, 3) = dt;
        F6(1, 4) = dt;
        F6(2, 5) = dt;
    } else {
        double s = std::sin(omega * dt);
        double c = std::cos(omega * dt);
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

Eigen::MatrixXd KalmanCT::getQ(double dt) const
{
    double sa2 = sigma_a_ * sigma_a_;
    double dt2 = dt * dt;

    Eigen::MatrixXd Q6 = Eigen::MatrixXd::Zero(6, 6);
    for (int i = 0; i < 3; ++i) {
        Q6(i, i)         = 0.25 * dt2 * dt2 * sa2;
        Q6(i, i + 3)     = 0.5  * dt2 * dt  * sa2;
        Q6(i + 3, i)     = 0.5  * dt2 * dt  * sa2;
        Q6(i + 3, i + 3) = dt2 * sa2;
    }
    return Q6;
}

double KalmanCT::likelihood(const Eigen::VectorXd& z, const Eigen::MatrixXd& R) const
{
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 7);
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
