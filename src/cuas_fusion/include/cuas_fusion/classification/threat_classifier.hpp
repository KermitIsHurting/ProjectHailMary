#pragma once

#include "cuas_fusion/common/types.hpp"
#include "cuas_fusion/tracking/track.hpp"

#include <cstdint>
#include <string>
#include <unordered_map>

namespace cuas {

enum class EscalationState {
    UNKNOWN,
    TRACKED,
    IDENTIFIED,
    THREATENING,
    ENGAGED
};

std::string escalationStateToString(EscalationState state);

struct ClassificationResult {
    ThreatLevel threat_level = ThreatLevel::UNKNOWN;
    EscalationState escalation_state = EscalationState::UNKNOWN;
    float quality_score = 0.0f;
    float dwell_time_s = 0.0f;
};

class ThreatClassifier {
public:
    ThreatClassifier() = default;

    bool init();

    ClassificationResult classify(const Track& track, double current_time_s,
                                  float threatening_range_m = 4.0f,
                                  float threatening_velocity_mps = 0.3f,
                                  float escalation_dwell_s = 1.0f);

    void pruneStale(double current_time_s, double timeout_s = 5.0);

private:
    struct PerTrackState {
        EscalationState state = EscalationState::UNKNOWN;
        double first_seen_s = 0.0;
        double identified_s = 0.0;
        double threatening_s = 0.0;
        double last_seen_s = 0.0;
    };

    std::unordered_map<uint32_t, PerTrackState> states_;
};

} // namespace cuas
