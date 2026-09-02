// @file track_manager.cpp
// @brief Hungarian assignment with Mahalanobis gating over a fixed slot array.
#include "cuas_fusion/tracking/track_manager.hpp"

#include "cuas_fusion/common/constants.hpp"
#include "cuas_fusion/common/fixed_containers.hpp"
#include "cuas_fusion/common/fixed_types.hpp"

#include <array>
#include <cmath>

namespace cuas {

namespace {

// Fresh tracks have no prior observation, so uncertainty starts at 1 m on each axis.
constexpr float64_t kInitialPosVar = 1.0;
// Without an explicit predict step, a missed frame grows uncertainty linearly
// so the gate reopens as the track drifts.
constexpr float64_t kMissProcessVar = 0.25;

} // namespace

bool TrackManager::init()
{
    for (uint32_t i = 0U; i < TRACK_MAX_TRACKS; ++i) {
        entries_[i].active     = false;
        entries_[i].hit_count  = 0;
        entries_[i].miss_count = 0;
        entries_[i].P.setIdentity();
    }
    next_id_ = 1U;

    R_detection_ = Eigen::Matrix3d::Identity() *
                   (kRadarDetectionSigmaM * kRadarDetectionSigmaM);
    return true;
}

uint32_t TrackManager::allocateSlot() const
{
    for (uint32_t i = 0U; i < TRACK_MAX_TRACKS; ++i) {
        if (!entries_[i].active) {
            return i;
        }
    }
    return TRACK_MAX_TRACKS;
}

void TrackManager::initiateTrack(uint32_t slot, const FusedDetection& det)
{
    TrackEntry& e = entries_[slot];
    e.active     = true;
    e.hit_count  = 1;
    e.miss_count = 0;
    e.P          = Eigen::Matrix3d::Identity() * kInitialPosVar;

    Track& t = e.track;
    t.track_id_     = next_id_;
    ++next_id_;
    t.position_x_m_ = det.position_x_m;
    t.position_y_m_ = det.position_y_m;
    t.position_z_m_ = det.position_z_m;
    t.velocity_mps_ = std::abs(det.velocity_mps);
    t.doppler_mps_  = det.velocity_mps;
    t.class_id_     = det.class_id;
    t.confidence_   = det.confidence;
    t.state_        = TrackState::TENTATIVE;
    t.timestamp_ns_ = det.timestamp_ns;
}

void TrackManager::applyDetection(uint32_t slot, const FusedDetection& det)
{
    TrackEntry& e = entries_[slot];
    Track&      t = e.track;

    // Standard linear Kalman measurement update on the 3-DOF position block.
    // The gain must be applied to the state: shrinking P while overwriting
    // the state with the raw measurement made P overconfident about a
    // full-noise state and tightened the gate around it (A1.9).
    const Eigen::Matrix3d S = e.P + R_detection_;
    const Eigen::LLT<Eigen::Matrix3d> llt(S);
    if (llt.info() == Eigen::Success) {
        const Eigen::Matrix3d K =
            e.P * llt.solve(Eigen::Matrix3d::Identity());
        const Eigen::Vector3d pos{
            static_cast<float64_t>(t.position_x_m_),
            static_cast<float64_t>(t.position_y_m_),
            static_cast<float64_t>(t.position_z_m_)
        };
        const Eigen::Vector3d meas{
            static_cast<float64_t>(det.position_x_m),
            static_cast<float64_t>(det.position_y_m),
            static_cast<float64_t>(det.position_z_m)
        };
        const Eigen::Vector3d updated = pos + K * (meas - pos);
        e.P = (Eigen::Matrix3d::Identity() - K) * e.P;
        t.position_x_m_ = static_cast<float32_t>(updated.x());
        t.position_y_m_ = static_cast<float32_t>(updated.y());
        t.position_z_m_ = static_cast<float32_t>(updated.z());
    } else {
        // S is PD by construction, so a failed factorization means the
        // covariance is corrupt (NaN): re-anchor on the measurement.
        e.P = Eigen::Matrix3d::Identity() * kInitialPosVar;
        t.position_x_m_ = det.position_x_m;
        t.position_y_m_ = det.position_y_m;
        t.position_z_m_ = det.position_z_m;
    }
    t.velocity_mps_ = std::abs(det.velocity_mps);
    t.doppler_mps_  = det.velocity_mps;
    t.class_id_     = det.class_id;
    t.confidence_   = det.confidence;
    t.timestamp_ns_ = det.timestamp_ns;

    // Saturate: hit history is only meaningful up to the confirm threshold,
    // and it is no longer reset by misses, so it must not run away.
    if (e.hit_count < TRACK_CONFIRM_HITS) {
        ++e.hit_count;
    }
    e.miss_count = 0;

    if (e.hit_count >= TRACK_CONFIRM_HITS) {
        t.state_ = TrackState::CONFIRMED;
    }
}

void TrackManager::applyMiss(uint32_t slot)
{
    TrackEntry& e = entries_[slot];
    ++e.miss_count;
    e.P += Eigen::Matrix3d::Identity() * kMissProcessVar;

    if (e.miss_count >= TRACK_MAX_MISSES) {
        e.active = false;
        return;
    }
    // M-of-N demotion (A1.9): hit history is never reset, and a confirmed
    // track keeps publishing through short dropouts — only
    // TRACK_COAST_MISSES consecutive misses demote it to COASTED, from
    // which one hit re-confirms it immediately.
    if (e.track.state_ != TrackState::CONFIRMED ||
        e.miss_count >= TRACK_COAST_MISSES)
    {
        e.track.state_ = TrackState::COASTED;
    }
}

bool TrackManager::update(const FusedDetection* detections,
                          uint32_t det_count,
                          FixedVector<Track, TRACK_MAX_TRACKS>& confirmed_out)
{
    confirmed_out.clear();

    // Row ordering in the cost matrix must map back to slot indices after the
    // solve, so snapshot active slots before any slot state can change.
    FixedVector<uint32_t, TRACK_MAX_TRACKS> active_slots;
    for (uint32_t i = 0U; i < TRACK_MAX_TRACKS; ++i) {
        if (entries_[i].active) {
            (void)active_slots.push_back(i);
        }
    }

    uint32_t m_val = det_count;
    if (det_count > TRACK_MAX_TRACKS) {
        m_val = TRACK_MAX_TRACKS;
    }
    const uint32_t M = m_val;
    const uint32_t N = active_slots.size();

    auto cost = cost_matrix_.topLeftCorner(static_cast<Eigen::Index>(N),
                                           static_cast<Eigen::Index>(M));

    for (uint32_t i = 0U; i < N; ++i) {
        const TrackEntry& e = entries_[active_slots[i]];
        const Eigen::Vector3d track_pos{
            static_cast<float64_t>(e.track.position_x_m_),
            static_cast<float64_t>(e.track.position_y_m_),
            static_cast<float64_t>(e.track.position_z_m_)
        };

        // One factorization per track instead of two inversions per pair
        // (A3.3). A failed LLT means a corrupt (NaN) covariance: price the
        // whole row out so the solver's finite-cost precondition holds.
        const Eigen::Matrix3d S = e.P + R_detection_;
        const Eigen::LLT<Eigen::Matrix3d> llt(S);
        const bool s_ok = (llt.info() == Eigen::Success);
        Eigen::Matrix3d S_inv = Eigen::Matrix3d::Identity();
        if (s_ok) {
            S_inv = llt.solve(Eigen::Matrix3d::Identity());
        }

        // Confirmed tracks get a wider gate so arm/leg returns from the same
        // body don't fall outside and spawn fragment tracks; tentative tracks
        // keep the tight gate so clutter cannot promote itself to confirmed.
        const float64_t gate_chi2 =
            (e.track.state_ == TrackState::CONFIRMED)
                ? kConfirmedGateChi2
                : kTentativeGateChi2;

        for (uint32_t j = 0U; j < M; ++j) {
            const FusedDetection& det = detections[j];
            const Eigen::Vector3d det_pos{
                static_cast<float64_t>(det.position_x_m),
                static_cast<float64_t>(det.position_y_m),
                static_cast<float64_t>(det.position_z_m)
            };
            const Eigen::Vector3d innovation = det_pos - track_pos;

            const Eigen::Index ri = static_cast<Eigen::Index>(i);
            const Eigen::Index cj = static_cast<Eigen::Index>(j);
            if (s_ok) {
                // NaN-safe gate: a non-finite distance fails the comparison
                // and prices the pair out.
                const float64_t md2 = innovation.dot(S_inv * innovation);
                cost(ri, cj) =
                    (md2 < gate_chi2) ? md2 : HungarianSolver::kLargeCost;
            } else {
                cost(ri, cj) = HungarianSolver::kLargeCost;
            }
        }
    }

    FixedVector<int32_t, TRACK_MAX_TRACKS> assignment;
    FixedVector<int32_t, TRACK_MAX_TRACKS> unassigned_dets;
    if (!solver_.solve(cost, assignment, unassigned_dets)) {
        return false;
    }

    for (uint32_t i = 0U; i < N; ++i) {
        const uint32_t slot = active_slots[i];
        const int32_t  j    = assignment[i];
        if (j < 0) {
            applyMiss(slot);
        } else {
            applyDetection(slot, detections[static_cast<uint32_t>(j)]);
        }
    }

    for (uint32_t k = 0U; k < unassigned_dets.size(); ++k) {
        const uint32_t slot = allocateSlot();
        if (slot >= TRACK_MAX_TRACKS) {
            break;
        }
        const uint32_t det_idx = static_cast<uint32_t>(unassigned_dets[k]);
        initiateTrack(slot, detections[det_idx]);
    }

    for (uint32_t i = 0U; i < TRACK_MAX_TRACKS; ++i) {
        if (entries_[i].active &&
            entries_[i].track.state_ == TrackState::CONFIRMED)
        {
            (void)confirmed_out.push_back(entries_[i].track);
        }
    }

    return true;
}

} // namespace cuas
