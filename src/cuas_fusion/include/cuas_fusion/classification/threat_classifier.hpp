// @file threat_classifier.hpp
// @brief Threat level and escalation-state classifier over tracks.
#pragma once

#include "cuas_fusion/common/constants.hpp"
#include "cuas_fusion/common/fixed_containers.hpp"
#include "cuas_fusion/common/fixed_types.hpp"
#include "cuas_fusion/common/types.hpp"
#include "cuas_fusion/tracking/track.hpp"

namespace cuas {

enum class EscalationState {
    UNKNOWN,
    TRACKED,
    IDENTIFIED,
    THREATENING,
    ENGAGED
};

inline const char* escalationStateToString(EscalationState state)
{
    switch (state) {
        case EscalationState::UNKNOWN:     { return "UNKNOWN"; }
        case EscalationState::TRACKED:     { return "TRACKED"; }
        case EscalationState::IDENTIFIED:  { return "IDENTIFIED"; }
        case EscalationState::THREATENING: { return "THREATENING"; }
        case EscalationState::ENGAGED:     { return "ENGAGED"; }
        default:                           { return "UNKNOWN"; }
    }
}

struct ClassificationResult {
    ThreatLevel     threat_level     = ThreatLevel::UNKNOWN;
    EscalationState escalation_state = EscalationState::UNKNOWN;
    float32_t       quality_score    = 0.0F;
    float32_t       dwell_time_s     = 0.0F;
};

class ThreatClassifier {
public:
    ThreatClassifier() = default;

    bool init();

    ClassificationResult classify(const Track& track,
                                  float64_t current_time_s,
                                  float32_t threatening_range_m     = 4.0F,
                                  float32_t threatening_velocity_mps = 0.3F,
                                  float32_t escalation_dwell_s       = 1.0F);

    void pruneStale(float64_t current_time_s, float64_t timeout_s = 5.0);

    // Drop every per-track state whose id is not in `ids` (the ids of the
    // TrackArray just processed). The tracker reaps a track 5 s after its
    // last return; keeping its state here until a timeout meant the 32-slot
    // map filled with dead ids and new tracks got no escalation state at
    // all (RC-4). O(states x n_ids), both bounded by TRACK_MAX_TRACKS.
    void retainOnly(const FixedVector<uint32_t, TRACK_MAX_TRACKS>& ids);

    struct ImpactPoint {
        float32_t x_m;
        float32_t y_m;
    };

    float32_t bearing_deg(float32_t x_m, float32_t y_m) const;
    float32_t predicted_range(float32_t x_m, float32_t y_m,
                              float32_t doppler_mps, float32_t horizon_s) const;
    ImpactPoint predicted_impact(float32_t x_m, float32_t y_m,
                                 float32_t doppler_mps,
                                 float32_t horizon_s) const;

private:
    struct PerTrackState {
        EscalationState state          = EscalationState::UNKNOWN;
        float64_t       first_seen_s   = 0.0;
        float64_t       identified_s   = 0.0;
        float64_t       threatening_s  = 0.0;
        float64_t       last_seen_s    = 0.0;
    };

    FixedMap<uint32_t, PerTrackState, TRACK_MAX_TRACKS> states_{};
};

} // namespace cuas
