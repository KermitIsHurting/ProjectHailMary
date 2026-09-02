// @file imm_tracker.hpp
// @brief IMM-filter-backed single-target tracker with confidence and maneuver outputs.
#pragma once

#include "cuas_fusion/common/constants.hpp"
#include "cuas_fusion/common/eigen_types.hpp"
#include "cuas_fusion/common/fixed_types.hpp"
#include "cuas_fusion/common/types.hpp"
#include "cuas_fusion/estimation/imm_filter.hpp"

#include <Eigen/Dense>
#include <array>

namespace cuas {

class IMMTracker {
public:
    IMMTracker() = default;
    IMMTracker(uint32_t track_id,
               float64_t x, float64_t y, float64_t z, float64_t timestamp);

    void predict(float64_t dt);
    void update(float64_t x, float64_t y, float64_t z, float64_t timestamp);
    TrackState getState() const;
    Eigen::Vector3d getPosition() const;
    Eigen::Vector3d getVelocity() const;
    Matrix6d getCovariance() const;
    std::array<float64_t, 3> getModelWeights() const;
    uint32_t getTrackId() const;
    Matrix6d getMixedF(float64_t dt) const;
    Matrix6d getMixedQ(float64_t dt) const;
    float64_t lastUpdateTime() const;

    float32_t getConfidence()          const { return confidence_; }
    bool      isManeuvering()          const { return is_maneuvering_; }
    float32_t getCtProbability()       const { return imm_ct_probability_; }
    uint32_t  getVelocityRejectCount() const { return velocity_reject_count_; }

    float64_t distance_to(float64_t px, float64_t py, float64_t pz) const;
    float64_t speed() const;

    // Association (RC-3). The state refers to stateTime(); a return stamped
    // t is compared against the position extrapolated over (t - stateTime())
    // with a gate that widens with this track's position and velocity
    // uncertainty, so a fresh track admits any speed and a converged one
    // tightens to kAssocBaseGateM.
    float64_t       stateTime() const;
    Eigen::Vector3d predictedPositionAt(float64_t t) const;
    float64_t       associationGateM(float64_t t) const;
    float64_t       distanceAt(float64_t px, float64_t py, float64_t pz, float64_t t) const;
    bool            gates(float64_t px, float64_t py, float64_t pz, float64_t t) const;

    // Largest per-axis 1-sigma of the blended estimate.
    float64_t positionSigma() const;
    float64_t velocitySigma() const;

    // Line-of-sight speed, m/s: positive = receding, negative = closing
    // (TI convention, audit D-7). Feeds Track.doppler_mps (RC-1).
    float64_t radialSpeed() const;

private:
    ImmFilter  imm_{};
    uint32_t   track_id_              = 0U;
    TrackState track_state_           = TrackState::TENTATIVE;
    float64_t  last_update_time_      = 0.0;
    float64_t  state_time_            = 0.0;
    int32_t    consecutive_hit_count_ = 0;

    bool       is_maneuvering_       = false;
    uint32_t   ct_dominant_frames_   = 0U;
    float32_t  imm_ct_probability_   = 0.0F;
    float32_t  confidence_           = kInitConfidence;

    Eigen::Vector3d prev_velocity_        = Eigen::Vector3d::Zero();
    bool            has_prev_velocity_    = false;
    uint32_t        velocity_reject_count_ = 0U;

    // Velocity-gate floor: one radar frame period, so same-stamp points in
    // a single cloud still get a physically meaningful gate (A1.14).
    static constexpr float64_t kMinGateDtS      = 0.05;
    // NOTE: an OCCLUDED/LOST timeout lifecycle was sketched here but never
    // implemented; the node-level 5 s erase_if is the actual track reaper.
    static constexpr int32_t   kConfirmHits     = 5;
};

} // namespace cuas
