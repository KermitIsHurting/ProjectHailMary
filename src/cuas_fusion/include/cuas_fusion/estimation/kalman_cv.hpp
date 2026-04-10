#pragma once

#include <Eigen/Dense>

namespace cuas {

/// Constant-Velocity 6-state linear Kalman filter
class KalmanCV {
public:
    KalmanCV() = default;

    /// Initialize state and covariance
    void init(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0);

    /// Propagate state forward by dt seconds
    void predict(double dt);

    /// Incorporate a position measurement z with noise R
    void update(const Eigen::VectorXd& z, const Eigen::MatrixXd& R);

    /// Current state [px, py, pz, vx, vy, vz]
    Eigen::VectorXd getState() const;

    /// Current covariance (6x6)
    Eigen::MatrixXd getCovariance() const;

    /// Measurement likelihood for model-probability update
    double likelihood(const Eigen::VectorXd& z, const Eigen::MatrixXd& R) const;

    /// State-transition matrix for given dt
    Eigen::MatrixXd getF(double dt) const;

    /// Process-noise matrix for given dt
    Eigen::MatrixXd getQ(double dt) const;

    /// Dimension of internal state
    static constexpr int kStateDim = 6;

private:
    Eigen::VectorXd x_ = Eigen::VectorXd::Zero(6);
    Eigen::MatrixXd P_ = Eigen::MatrixXd::Identity(6, 6);
    bool initialized_ = false;

    static constexpr double sigma_a_ = 0.5;
};

} // namespace cuas
