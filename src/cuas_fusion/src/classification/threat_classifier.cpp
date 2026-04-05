#include "cuas_fusion/classification/threat_classifier.hpp"
#include "cuas_fusion/common/constants.hpp"

#include <algorithm>
#include <cstdio>

namespace cuas {

bool ThreatClassifier::init()
{
    return true;
}

ThreatLevel ThreatClassifier::classify(const Track& track) const
{
    ThreatLevel result = ThreatLevel::UNKNOWN;

    // Rule 1 — UNKNOWN: low confidence, empty label, or tentative state.
    if (track.confidence_ < THREAT_MIN_CONFIDENCE ||
        track.class_label_.empty() ||
        track.state_ == TrackState::TENTATIVE)
    {
        result = ThreatLevel::UNKNOWN;
    }
    // Rule 2 — BENIGN: known non-aerial class, slow, not approaching.
    else if ([&]() {
                 for (const auto& cls : THREAT_BENIGN_CLASSES) {
                     if (track.class_label_ == cls) { return true; }
                 }
                 return false;
             }() &&
             track.velocity_mps_ < THREAT_VELOCITY_SUSPECT_MPS &&
             track.doppler_mps_ > THREAT_APPROACH_THRESHOLD_MPS)
    {
        result = ThreatLevel::BENIGN;
    }
    // Rule 3 — THREAT: fast AND approaching.
    else if (track.velocity_mps_ >= THREAT_VELOCITY_THREAT_MPS &&
             track.doppler_mps_ <= THREAT_APPROACH_THRESHOLD_MPS)
    {
        result = ThreatLevel::THREAT;
    }
    // Rule 4 — SUSPECT: fast, drone class, or approaching.
    else if (track.velocity_mps_ >= THREAT_VELOCITY_SUSPECT_MPS ||
             [&]() {
                 for (const auto& cls : THREAT_DRONE_CLASSES) {
                     if (track.class_label_ == cls) { return true; }
                 }
                 return false;
             }() ||
             track.doppler_mps_ <= THREAT_APPROACH_THRESHOLD_MPS)
    {
        result = ThreatLevel::SUSPECT;
    }
    // Rule 5 — default.
    else {
        result = ThreatLevel::UNKNOWN;
    }

    fprintf(stderr,
            "[ThreatClassifier] track_id=%u class=%s vel=%.2f → %s\n",
            track.track_id_,
            track.class_label_.c_str(),
            track.velocity_mps_,
            threatLevelToString(result).c_str());

    return result;
}

} // namespace cuas
