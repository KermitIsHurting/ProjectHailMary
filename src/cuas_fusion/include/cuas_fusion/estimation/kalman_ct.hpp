#pragma once

#include <Eigen/Dense>

namespace cuas {

/// Coordinated-Turn 7-state EKF with turn-rate estimation
class KalmanCT {
public:
    KalmanCT() = default;

    /// Initialize state and covariance
    void init(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0);

    /// Propagate state forward by dt seconds (nonlinear)
    void predict(double dt);

    /// Incorporate a position measurement z with noise R
    void update(const Eigen::VectorXd& z, const Eigen::MatrixXd& R);

    /// Current state projected to 6-DOF [px, py, pz, vx, vy, vz]
    Eigen::VectorXd getState() const;

    /// Current covariance projected to 6x6
    Eigen::MatrixXd getCovariance() const;

    /// Measurement likelihood
    double likelihood(const Eigen::VectorXd& z, const Eigen::MatrixXd& R) const;

    /// State-transition Jacobian for given dt (projected to 6x6)
    Eigen::MatrixXd getF(double dt) const;

    /// Process-noise matrix for given dt (projected to 6x6)
    Eigen::MatrixXd getQ(double dt) const;

    /// Dimension of internal state
    static constexpr int kStateDim = 7;

private:
    Eigen::VectorXd x_ = Eigen::VectorXd::Zero(7);
    Eigen::MatrixXd P_ = Eigen::MatrixXd::Identity(7, 7);
    bool initialized_ = false;

    static constexpr double sigma_a_ = 0.5;
    static constexpr double sigma_omega_ = 0.1;
    static constexpr double omega_guard_ = 1e-6;
};

} // namespace cuas
