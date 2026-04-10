#pragma once

#include <Eigen/Dense>

namespace cuas {

/// Constant-Acceleration 9-state linear Kalman filter
class KalmanCA {
public:
    KalmanCA() = default;

    /// Initialize state and covariance
    void init(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0);

    /// Propagate state forward by dt seconds
    void predict(double dt);

    /// Incorporate a position measurement z with noise R
    void update(const Eigen::VectorXd& z, const Eigen::MatrixXd& R);

    /// Current state projected to 6-DOF [px, py, pz, vx, vy, vz]
    Eigen::VectorXd getState() const;

    /// Current covariance projected to 6x6
    Eigen::MatrixXd getCovariance() const;

    /// Measurement likelihood
    double likelihood(const Eigen::VectorXd& z, const Eigen::MatrixXd& R) const;

    /// State-transition matrix for given dt (projected to 6x6)
    Eigen::MatrixXd getF(double dt) const;

    /// Process-noise matrix for given dt (projected to 6x6)
    Eigen::MatrixXd getQ(double dt) const;

    /// Dimension of internal state
    static constexpr int kStateDim = 9;

private:
    Eigen::VectorXd x_ = Eigen::VectorXd::Zero(9);
    Eigen::MatrixXd P_ = Eigen::MatrixXd::Identity(9, 9);
    bool initialized_ = false;

    static constexpr double sigma_j_ = 1.0;
};

} // namespace cuas
