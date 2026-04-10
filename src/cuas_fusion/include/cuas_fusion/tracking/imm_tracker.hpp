#pragma once

#include "cuas_fusion/estimation/imm_filter.hpp"

#include <Eigen/Dense>
#include <array>
#include <cstdint>
#include <string>

namespace cuas {

/// Per-track IMM-backed tracker with lifecycle management
class IMMTracker {
public:
    IMMTracker(uint32_t track_id, double x, double y, double z, double timestamp);

    /// Propagate filter forward by dt seconds
    void predict(double dt);

    /// Incorporate a new position measurement
    void update(double x, double y, double z, double timestamp);

    /// Current lifecycle state string
    std::string getState() const;

    /// Current estimated position (3x1)
    Eigen::VectorXd getPosition() const;

    /// Current estimated velocity (3x1)
    Eigen::VectorXd getVelocity() const;

    /// Full 6x6 covariance
    Eigen::MatrixXd getCovariance() const;

    /// IMM model weights [CV, CA, CT]
    std::array<double, 3> getModelWeights() const;

    /// Track identifier
    uint32_t getTrackId() const;

    /// Blended F matrix for predictor
    Eigen::MatrixXd getMixedF(double dt) const;

    /// Blended Q matrix for predictor
    Eigen::MatrixXd getMixedQ(double dt) const;

    /// Timestamp of last successful update
    double lastUpdateTime() const;

private:
    ImmFilter imm_;
    uint32_t track_id_;
    std::string track_state_;
    double last_update_time_;
    int consecutive_hit_count_;

    static constexpr int kConfirmHits = 5;
    static constexpr double kOccludedTimeout = 0.5;
    static constexpr double kLostTimeout = 5.0;
};

} // namespace cuas
