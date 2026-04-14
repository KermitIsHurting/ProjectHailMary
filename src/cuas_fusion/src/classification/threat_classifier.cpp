// @file threat_classifier.cpp
// @brief Threat classification and escalation state machine.
#include "cuas_fusion/classification/threat_classifier.hpp"

#include <algorithm>
#include <cmath>

namespace cuas {

bool ThreatClassifier::init()
{
    return true;
}

ClassificationResult ThreatClassifier::classify(
    const Track& track, float64_t current_time_s,
    float32_t threatening_range_m, float32_t threatening_velocity_mps,
    float32_t escalation_dwell_s)
{
    ClassificationResult result;

    if (track.class_label_.empty()) {
        result.threat_level = ThreatLevel::UNKNOWN;
    } else if (track.class_label_ == "0") {
        const float32_t abs_vel = std::abs(track.velocity_mps_);
        result.threat_level = (abs_vel <= 2.0F) ? ThreatLevel::BENIGN : ThreatLevel::SUSPECT;
    } else if (track.confidence_ > 0.5F) {
        result.threat_level = ThreatLevel::THREAT;
    } else {
        result.threat_level = ThreatLevel::UNKNOWN;
    }

    PerTrackState* ts_ptr = states_.find(track.track_id_);
    if (ts_ptr == nullptr) {
        PerTrackState fresh;
        fresh.first_seen_s = current_time_s;
        fresh.state        = EscalationState::UNKNOWN;
        fresh.last_seen_s  = current_time_s;
        (void)states_.insert_or_assign(track.track_id_, fresh);
        ts_ptr = states_.find(track.track_id_);
    }

    if (ts_ptr == nullptr) {
        result.escalation_state = EscalationState::UNKNOWN;
        return result;
    }

    PerTrackState& ts = *ts_ptr;
    ts.last_seen_s = current_time_s;

    const float64_t dwell = current_time_s - ts.first_seen_s;
    result.dwell_time_s = static_cast<float32_t>(dwell);

    const float32_t range = std::sqrt(
        track.position_x_m_ * track.position_x_m_ +
        track.position_y_m_ * track.position_y_m_);

    switch (ts.state) {
        case EscalationState::UNKNOWN: {
            if (dwell > static_cast<float64_t>(escalation_dwell_s)) {
                ts.state = EscalationState::TRACKED;
            }
            break;
        }

        case EscalationState::TRACKED: {
            if (!track.class_label_.empty() && track.class_label_ == "0") {
                if (ts.identified_s == 0.0) {
                    ts.identified_s = current_time_s;
                }
                if (current_time_s - ts.identified_s > 0.5) {
                    ts.state = EscalationState::IDENTIFIED;
                }
            }
            break;
        }

        case EscalationState::IDENTIFIED: {
            if (range < threatening_range_m &&
                track.doppler_mps_ < -threatening_velocity_mps) {
                if (ts.threatening_s == 0.0) {
                    ts.threatening_s = current_time_s;
                }
                if (current_time_s - ts.threatening_s >
                    static_cast<float64_t>(escalation_dwell_s)) {
                    ts.state = EscalationState::THREATENING;
                    ts.threatening_s = current_time_s;
                }
            } else {
                ts.threatening_s = 0.0;
            }
            break;
        }

        case EscalationState::THREATENING: {
            if (range < 2.0F &&
                (current_time_s - ts.threatening_s) > 2.0) {
                ts.state = EscalationState::ENGAGED;
            }
            break;
        }

        case EscalationState::ENGAGED: {
            break;
        }

        default: {
            break;
        }
    }

    result.escalation_state = ts.state;

    float32_t q = 0.3F;
    if (!track.class_label_.empty()) {
        q += 0.3F;
    }
    if (track.state_ == TrackState::CONFIRMED) {
        q += 0.2F;
    }
    if (dwell > 3.0) {
        q += 0.2F;
    }
    result.quality_score = std::min(1.0F, std::max(0.0F, q));

    return result;
}

void ThreatClassifier::pruneStale(float64_t current_time_s, float64_t timeout_s)
{
    states_.erase_if(
        [&](uint32_t /*id*/, const PerTrackState& s) {
            return (current_time_s - s.last_seen_s) > timeout_s;
        });
}

} // namespace cuas
