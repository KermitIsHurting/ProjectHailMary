// @file track_state_ids.hpp
// @brief Numeric constants matching Track.msg track_state_id and ThreatReport.msg threat_level_id fields.
#pragma once

// WHY: string comparisons at runtime violate JSF AV Rule 206 and IR-3. These
// constants let all nodes compare integers instead of std::string, eliminating
// runtime heap use and satisfying the enum-class-for-state-machines
// requirement via strongly-typed constexpr values.

#include "cuas_fusion/common/fixed_types.hpp"

namespace cuas {

namespace track_state {
    static constexpr uint8_t kUnknown    = 0U;
    static constexpr uint8_t kTentative  = 1U;
    static constexpr uint8_t kConfirmed  = 2U;
    static constexpr uint8_t kOccluded   = 3U;
    static constexpr uint8_t kReacquired = 4U;
    static constexpr uint8_t kLost       = 5U;
}  // namespace track_state

namespace threat_level {
    static constexpr uint8_t kUnknown     = 0U;
    static constexpr uint8_t kBenign      = 1U;
    static constexpr uint8_t kSuspect     = 2U;
    static constexpr uint8_t kIdentified  = 3U;
    static constexpr uint8_t kThreatening = 4U;
}  // namespace threat_level

}  // namespace cuas
