#pragma once

#include "cuas_fusion/common/types.hpp"
#include "cuas_fusion/tracking/track.hpp"

namespace cuas {

class ThreatClassifier {
public:
    ThreatClassifier() = default;

    bool init();
    ThreatLevel classify(const Track& track) const;
};

} // namespace cuas
