// @file imm_tracker.cpp
// @brief IMM tracker state machine with confidence decay and CT-driven maneuver detection.
#include "cuas_fusion/tracking/imm_tracker.hpp"
#include "cuas_fusion/common/constants.hpp"

#include <algorithm>
#include <cmath>

namespace cuas {

IMMTracker::IMMTracker(uint32_t track_id,
                       float64_t x, float64_t y, float64_t z, float64_t timestamp)
    : track_id_(track_id)
    , track_state_(TrackState::TENTATIVE)
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

void IMMTracker::predict(float64_t dt)
{
    imm_.predict(dt);

    // Decay fires every tick; update() adds a larger gain so a hit net-increases.
    const float32_t decayed =
        confidence_ - kConfidenceDecayRate * static_cast<float32_t>(dt);
    confidence_ = std::max(kMinConfidence, decayed);

    switch (track_state_) {
        case TrackState::CONFIRMED:
        case TrackState::REACQUIRED:
        case TrackState::OCCLUDED:
        case TrackState::TENTATIVE:
        case TrackState::COASTED:
        case TrackState::LOST:
        case TrackState::DELETED:
        default: {
            break;
        }
    }
}

void IMMTracker::update(float64_t x, float64_t y, float64_t z, float64_t timestamp)
{
    Eigen::VectorXd z_meas(3);
    z_meas << x, y, z;
    const Eigen::MatrixXd R = Eigen::MatrixXd::Identity(3, 3);

    imm_.update(z_meas, R);

    // Gain uses time since last measurement, not predict dt, so bursts of
    // quick updates don't saturate confidence in a single tick.
    const float64_t dt_since_hit = std::max(0.0, timestamp - last_update_time_);

    // Reject physically implausible velocity jumps (e.g. from arm-swing
    // reflections) while keeping the position correction and the grown
    // covariance from this update. Skip the gate on the first post-init
    // measurement because the baseline prev_velocity_ is an arbitrary zero.
    const Eigen::Vector3d new_velocity = imm_.getState().tail(3);
    if (!has_prev_velocity_) {
        prev_velocity_     = new_velocity;
        has_prev_velocity_ = true;
    } else {
        const float64_t delta_v     = (new_velocity - prev_velocity_).norm();
        const float64_t max_delta_v = kMaxPhysicalAcceleration * dt_since_hit;
        if (delta_v > max_delta_v) {
            imm_.setVelocity(prev_velocity_);
            ++velocity_reject_count_;
        } else {
            prev_velocity_ = new_velocity;
        }
    }

    last_update_time_ = timestamp;
    ++consecutive_hit_count_;

    const float32_t gained =
        confidence_ + kConfidenceGainRate * static_cast<float32_t>(dt_since_hit);
    confidence_ = std::min(1.0F, gained);

    switch (track_state_) {
        case TrackState::OCCLUDED: {
            track_state_ = TrackState::REACQUIRED;
            consecutive_hit_count_ = 1;
            break;
        }
        case TrackState::TENTATIVE: {
            if (consecutive_hit_count_ >= kConfirmHits) {
                track_state_ = TrackState::CONFIRMED;
                confidence_ = kConfirmedConfidence;
            }
            break;
        }
        case TrackState::CONFIRMED:
        case TrackState::REACQUIRED:
        case TrackState::COASTED:
        case TrackState::LOST:
        case TrackState::DELETED:
        default: {
            break;
        }
    }

    const std::array<float64_t, 3> weights = imm_.getModelWeights();
    const float64_t ct_prob = weights[2];
    imm_ct_probability_ = static_cast<float32_t>(ct_prob);

    if (ct_prob > kManeuverCtThreshold) {
        ++ct_dominant_frames_;
        if ((ct_dominant_frames_ >= kManeuverFramesRequired) && !is_maneuvering_) {
            is_maneuvering_ = true;
            imm_.setModelNoise(0U, kManeuverCvSigmaA);
            imm_.setModelNoise(1U, kManeuverCaSigmaJ);
        }
    } else {
        ct_dominant_frames_ = 0U;
        if (is_maneuvering_) {
            is_maneuvering_ = false;
            imm_.setModelNoise(0U, kNominalCvSigmaA);
            imm_.setModelNoise(1U, kNominalCaSigmaJ);
        }
    }
}

TrackState IMMTracker::getState() const { return track_state_; }

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

std::array<float64_t, 3> IMMTracker::getModelWeights() const
{
    return imm_.getModelWeights();
}

uint32_t IMMTracker::getTrackId() const { return track_id_; }

Eigen::MatrixXd IMMTracker::getMixedF(float64_t dt) const
{
    return imm_.getMixedF(dt);
}

Eigen::MatrixXd IMMTracker::getMixedQ(float64_t dt) const
{
    return imm_.getMixedQ(dt);
}

float64_t IMMTracker::lastUpdateTime() const { return last_update_time_; }

float64_t IMMTracker::distance_to(float64_t px, float64_t py, float64_t pz) const
{
    const Eigen::VectorXd pos = imm_.getState().head(3);
    const float64_t dx = pos(0) - px;
    const float64_t dy = pos(1) - py;
    const float64_t dz = pos(2) - pz;
    return std::sqrt((dx * dx) + (dy * dy) + (dz * dz));
}

float64_t IMMTracker::speed() const
{
    return imm_.getState().tail(3).norm();
}

} // namespace cuas
