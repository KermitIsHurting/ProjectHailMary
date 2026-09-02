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
    , state_time_(timestamp)
    , consecutive_hit_count_(1)
{
    Vector6d x0 = Vector6d::Zero();
    x0(0) = x;
    x0(1) = y;
    x0(2) = z;

    // Born from one return: position known to the sensor sigma, velocity
    // unknown. The old P0 (1 m, 0.5 m/s) claimed a velocity the filter did
    // not have, so the jump gate below pinned fast targets near 0 m/s and
    // the association gate never held them (RC-2, RC-3).
    Matrix6d P0 = Matrix6d::Identity();
    P0.block<3, 3>(0, 0) *= kRadarDetectionSigmaM * kRadarDetectionSigmaM;
    P0.block<3, 3>(3, 3) *= kTrackInitVelSigmaMps * kTrackInitVelSigmaMps;

    imm_.init(x0, P0);
}

void IMMTracker::predict(float64_t dt)
{
    imm_.predict(dt);
    state_time_ += dt;

    // Decay fires every tick; update() adds a larger gain so a hit net-increases.
    const float32_t decayed =
        confidence_ - kConfidenceDecayRate * static_cast<float32_t>(dt);
    confidence_ = std::max(kMinConfidence, decayed);
}

void IMMTracker::update(float64_t x, float64_t y, float64_t z, float64_t timestamp)
{
    const Eigen::Vector3d z_meas{x, y, z};
    // Same sensor model as TrackManager (kRadarDetectionSigmaM = 0.15 m):
    // the old R = 1 m^2/axis disagreed with it by 44x with no rationale
    // (A1.14).
    const Eigen::Matrix3d R = Eigen::Matrix3d::Identity() *
        (kRadarDetectionSigmaM * kRadarDetectionSigmaM);

    // Prior velocity uncertainty scales the jump gate (RC-2, D-9): a fresh
    // track admits any speed, a converged one tightens to the kinematic floor.
    const float64_t sigma_v_prior = velocitySigma();

    imm_.update(z_meas, R);

    // Gain uses time since last measurement, not predict dt, so bursts of
    // quick updates don't saturate confidence in a single tick.
    const float64_t dt_since_hit = std::max(0.0, timestamp - last_update_time_);

    // Reject physically implausible velocity jumps (e.g. from arm-swing
    // reflections) while keeping the position correction and the grown
    // covariance from this update. Skip the gate on the first post-init
    // measurement because the baseline prev_velocity_ is an arbitrary zero.
    const Eigen::Vector3d new_velocity = imm_.getState().tail<3>();
    if (!has_prev_velocity_) {
        prev_velocity_     = new_velocity;
        has_prev_velocity_ = true;
    } else {
        const float64_t delta_v     = (new_velocity - prev_velocity_).norm();
        // Floor the gate dt: several points of one radar cloud arrive with
        // an identical stamp, so dt=0 made max_delta_v 0 and every velocity
        // innovation was rejected precisely when data was densest (A1.14).
        const float64_t gate_dt     = std::max(dt_since_hit, kMinGateDtS);
        const float64_t max_delta_v = std::max(kMaxPhysicalAcceleration * gate_dt,
                                               kGateSigmas * sigma_v_prior);
        if (delta_v > max_delta_v) {
            imm_.setVelocity(prev_velocity_);
            ++velocity_reject_count_;
        } else {
            prev_velocity_ = new_velocity;
        }
    }

    // One hit per stamp, and a gap restarts confirmation (RC-37): several
    // returns of one cloud share a stamp and used to confirm a track from a
    // single frame.
    if (timestamp > last_update_time_) {
        if (dt_since_hit > kTrackHitGapResetS) {
            consecutive_hit_count_ = 0;
        }
        ++consecutive_hit_count_;
    }
    last_update_time_ = timestamp;
    state_time_       = std::max(state_time_, timestamp);

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

Eigen::Vector3d IMMTracker::getPosition() const
{
    return imm_.getState().head<3>();
}

Eigen::Vector3d IMMTracker::getVelocity() const
{
    return imm_.getState().tail<3>();
}

Matrix6d IMMTracker::getCovariance() const
{
    return imm_.getCovariance();
}

std::array<float64_t, 3> IMMTracker::getModelWeights() const
{
    return imm_.getModelWeights();
}

uint32_t IMMTracker::getTrackId() const { return track_id_; }

Matrix6d IMMTracker::getMixedF(float64_t dt) const
{
    return imm_.getMixedF(dt);
}

Matrix6d IMMTracker::getMixedQ(float64_t dt) const
{
    return imm_.getMixedQ(dt);
}

float64_t IMMTracker::lastUpdateTime() const { return last_update_time_; }

float64_t IMMTracker::distance_to(float64_t px, float64_t py, float64_t pz) const
{
    const Eigen::Vector3d pos = imm_.getState().head<3>();
    const float64_t dx = pos(0) - px;
    const float64_t dy = pos(1) - py;
    const float64_t dz = pos(2) - pz;
    return std::sqrt((dx * dx) + (dy * dy) + (dz * dz));
}

float64_t IMMTracker::speed() const
{
    return imm_.getState().tail<3>().norm();
}

float64_t IMMTracker::stateTime() const { return state_time_; }

float64_t IMMTracker::positionSigma() const
{
    const Matrix6d P = imm_.getCovariance();
    const float64_t v = std::max({P(0, 0), P(1, 1), P(2, 2)});
    return std::sqrt(std::max(0.0, v));
}

float64_t IMMTracker::velocitySigma() const
{
    const Matrix6d P = imm_.getCovariance();
    const float64_t v = std::max({P(3, 3), P(4, 4), P(5, 5)});
    return std::sqrt(std::max(0.0, v));
}

Eigen::Vector3d IMMTracker::predictedPositionAt(float64_t t) const
{
    const Vector6d  s  = imm_.getState();
    const float64_t dt = std::clamp(t - state_time_, -kAssocMaxExtrapS, kAssocMaxExtrapS);
    return s.head<3>() + (s.tail<3>() * dt);
}

float64_t IMMTracker::associationGateM(float64_t t) const
{
    const float64_t dt   = std::min(std::abs(t - state_time_), kAssocMaxExtrapS);
    const float64_t gate = kAssocBaseGateM +
        (kGateSigmas * (positionSigma() + (velocitySigma() * dt)));
    return std::min(gate, kAssocMaxGateM);
}

float64_t IMMTracker::distanceAt(float64_t px, float64_t py, float64_t pz, float64_t t) const
{
    const Eigen::Vector3d p{px, py, pz};
    return (predictedPositionAt(t) - p).norm();
}

bool IMMTracker::gates(float64_t px, float64_t py, float64_t pz, float64_t t) const
{
    return distanceAt(px, py, pz, t) <= associationGateM(t);
}

float64_t IMMTracker::radialSpeed() const
{
    const Vector6d        s = imm_.getState();
    const Eigen::Vector3d p = s.head<3>();
    const float64_t       r = p.norm();
    if (r < kRadialSpeedMinRangeM) {
        return 0.0;
    }
    return p.dot(s.tail<3>()) / r;
}

} // namespace cuas
