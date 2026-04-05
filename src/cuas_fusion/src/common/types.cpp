#include "cuas_fusion/common/types.hpp"

namespace cuas {

std::string trackStateToString(TrackState state)
{
    switch (state) {
        case TrackState::TENTATIVE: return "TENTATIVE";
        case TrackState::CONFIRMED: return "CONFIRMED";
        case TrackState::COASTED:   return "COASTED";
        case TrackState::DELETED:   return "DELETED";
    }
    return "UNKNOWN";
}

std::string threatLevelToString(ThreatLevel level)
{
    switch (level) {
        case ThreatLevel::BENIGN:  return "BENIGN";
        case ThreatLevel::UNKNOWN: return "UNKNOWN";
        case ThreatLevel::SUSPECT: return "SUSPECT";
        case ThreatLevel::THREAT:  return "THREAT";
    }
    return "UNKNOWN";
}

} // namespace cuas
