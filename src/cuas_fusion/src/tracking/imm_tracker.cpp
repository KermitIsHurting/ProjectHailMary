#include "cuas_fusion/tracking/imm_tracker.hpp"

namespace cuas {

IMMTracker::IMMTracker(uint32_t track_id, double x, double y, double z, double timestamp)
    : track_id_(track_id)
    , track_state_("TENTATIVE")
    , last_update_time_(timestamp)
    , consecutive_hit_count_(1)
{
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(6);
    x0(0) = x;
    x0(1) = y;
    x0(2) = z;

    Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(6, 6);
    P0.block<3, 3>(0, 0) *= 1.0;
    P0.block<3, 3>(3, 3) *= 0.25;

    imm_.init(x0, P0);
}

void IMMTracker::predict(double dt)
{
    imm_.predict(dt);

    double time_since = dt;
    if (track_state_ == "CONFIRMED" || track_state_ == "REACQUIRED") {
        // stays in current state during predict
    } else if (track_state_ == "OCCLUDED") {
        // check for LOST handled by caller via elapsed time
    }
}

void IMMTracker::update(double x, double y, double z, double timestamp)
{
    Eigen::VectorXd z_meas(3);
    z_meas << x, y, z;
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(3, 3);

    imm_.update(z_meas, R);
    last_update_time_ = timestamp;
    consecutive_hit_count_++;

    if (track_state_ == "OCCLUDED") {
        track_state_ = "REACQUIRED";
        consecutive_hit_count_ = 1;
    } else if (track_state_ == "TENTATIVE") {
        if (consecutive_hit_count_ >= kConfirmHits) {
            track_state_ = "CONFIRMED";
        }
    }
}

std::string IMMTracker::getState() const { return track_state_; }

Eigen::VectorXd IMMTracker::getPosition() const
{
    return imm_.getState().head(3);
}

Eigen::VectorXd IMMTracker::getVelocity() const
{
    return imm_.getState().tail(3);
}

Eigen::MatrixXd IMMTracker::getCovariance() const
{
    return imm_.getCovariance();
}

std::array<double, 3> IMMTracker::getModelWeights() const
{
    return imm_.getModelWeights();
}

uint32_t IMMTracker::getTrackId() const { return track_id_; }

Eigen::MatrixXd IMMTracker::getMixedF(double dt) const
{
    return imm_.getMixedF(dt);
}

Eigen::MatrixXd IMMTracker::getMixedQ(double dt) const
{
    return imm_.getMixedQ(dt);
}

double IMMTracker::lastUpdateTime() const { return last_update_time_; }

} // namespace cuas
