// @file threat_classifier.cpp
// @brief Threat classification and escalation state machine.
#include "cuas_fusion/common/bearing.hpp"
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

    if (track.class_id_ < 0) {
        result.threat_level = ThreatLevel::UNKNOWN;
    } else if (track.class_id_ == 0) {
        const float32_t abs_vel = std::abs(track.velocity_mps_);
        if (abs_vel <= 2.0F) {
            result.threat_level = ThreatLevel::BENIGN;
        } else {
            result.threat_level = ThreatLevel::SUSPECT;
        }
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
            if (track.class_id_ == 0) {
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
    if (track.class_id_ >= 0) {
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

void ThreatClassifier::retainOnly(const FixedVector<uint32_t, TRACK_MAX_TRACKS>& ids)
{
    states_.erase_if(
        [&](uint32_t id, const PerTrackState& s) {
            (void)s;
            for (uint32_t i = 0U; i < ids.size(); ++i) {
                if (ids[i] == id) {
                    return false;
                }
            }
            return true;
        });
}

void ThreatClassifier::pruneStale(float64_t current_time_s, float64_t timeout_s)
{
    states_.erase_if(
        [&](uint32_t id, const PerTrackState& s) {
            (void)id;
            return (current_time_s - s.last_seen_s) > timeout_s;
        });
}

float32_t ThreatClassifier::bearing_deg(float32_t x_m, float32_t y_m) const
{
    return bearingDegBoresightZero(x_m, y_m);
}

float32_t ThreatClassifier::predicted_range(float32_t x_m, float32_t y_m,
                                            float32_t doppler_mps,
                                            float32_t horizon_s) const
{
    const ImpactPoint p = predicted_impact(x_m, y_m, doppler_mps, horizon_s);
    return std::sqrt((p.x_m * p.x_m) + (p.y_m * p.y_m));
}

ThreatClassifier::ImpactPoint ThreatClassifier::predicted_impact(
    float32_t x_m, float32_t y_m,
    float32_t doppler_mps, float32_t horizon_s) const
{
    const float32_t az_rad = std::atan2(x_m, y_m);
    ImpactPoint out;
    out.x_m = x_m + (doppler_mps * std::sin(az_rad) * horizon_s);
    out.y_m = y_m + (doppler_mps * std::cos(az_rad) * horizon_s);
    return out;
}

} // namespace cuas
