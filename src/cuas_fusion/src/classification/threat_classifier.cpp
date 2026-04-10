#include "cuas_fusion/classification/threat_classifier.hpp"

#include <cmath>

namespace cuas {

std::string escalationStateToString(EscalationState state)
{
    switch (state) {
        case EscalationState::UNKNOWN:     return "UNKNOWN";
        case EscalationState::TRACKED:     return "TRACKED";
        case EscalationState::IDENTIFIED:  return "IDENTIFIED";
        case EscalationState::THREATENING: return "THREATENING";
        case EscalationState::ENGAGED:     return "ENGAGED";
    }
    return "UNKNOWN";
}

bool ThreatClassifier::init()
{
    return true;
}

ClassificationResult ThreatClassifier::classify(
    const Track& track, double current_time_s,
    float threatening_range_m, float threatening_velocity_mps,
    float escalation_dwell_s)
{
    ClassificationResult result;

    // --- Threat level (per-frame, same logic as before) ---
    if (track.class_label_.empty()) {
        result.threat_level = ThreatLevel::UNKNOWN;
    } else if (track.class_label_ == "0") {
        float abs_vel = std::abs(track.velocity_mps_);
        result.threat_level = (abs_vel <= 2.0f) ? ThreatLevel::BENIGN : ThreatLevel::SUSPECT;
    } else if (track.confidence_ > 0.5f) {
        result.threat_level = ThreatLevel::THREAT;
    } else {
        result.threat_level = ThreatLevel::UNKNOWN;
    }

    // --- Escalation state machine (stateful, per track_id) ---
    auto& ts = states_[track.track_id_];
    if (ts.first_seen_s == 0.0) {
        ts.first_seen_s = current_time_s;
        ts.state = EscalationState::UNKNOWN;
    }
    ts.last_seen_s = current_time_s;

    double dwell = current_time_s - ts.first_seen_s;
    result.dwell_time_s = static_cast<float>(dwell);

    float range = std::sqrt(track.position_x_m_ * track.position_x_m_ +
                            track.position_y_m_ * track.position_y_m_);

    switch (ts.state) {
        case EscalationState::UNKNOWN:
            if (dwell > static_cast<double>(escalation_dwell_s)) {
                ts.state = EscalationState::TRACKED;
            }
            break;

        case EscalationState::TRACKED:
            if (!track.class_label_.empty() && track.class_label_ == "0") {
                if (ts.identified_s == 0.0) ts.identified_s = current_time_s;
                if (current_time_s - ts.identified_s > 0.5) {
                    ts.state = EscalationState::IDENTIFIED;
                }
            }
            break;

        case EscalationState::IDENTIFIED:
            if (range < threatening_range_m &&
                track.doppler_mps_ < -threatening_velocity_mps) {
                if (ts.threatening_s == 0.0) ts.threatening_s = current_time_s;
                if (current_time_s - ts.threatening_s > static_cast<double>(escalation_dwell_s)) {
                    ts.state = EscalationState::THREATENING;
                    ts.threatening_s = current_time_s;
                }
            } else {
                ts.threatening_s = 0.0;
            }
            break;

        case EscalationState::THREATENING:
            if (range < 2.0f &&
                (current_time_s - ts.threatening_s) > 2.0) {
                ts.state = EscalationState::ENGAGED;
            }
            break;

        case EscalationState::ENGAGED:
            break;
    }

    result.escalation_state = ts.state;

    // --- Quality score ---
    float q = 0.3f;
    if (!track.class_label_.empty()) q += 0.3f;
    if (track.state_ == TrackState::CONFIRMED) q += 0.2f;
    if (dwell > 3.0) q += 0.2f;
    result.quality_score = std::min(1.0f, std::max(0.0f, q));

    return result;
}

void ThreatClassifier::pruneStale(double current_time_s, double timeout_s)
{
    for (auto it = states_.begin(); it != states_.end(); ) {
        if (current_time_s - it->second.last_seen_s > timeout_s) {
            it = states_.erase(it);
        } else {
            ++it;
        }
    }
}

} // namespace cuas
