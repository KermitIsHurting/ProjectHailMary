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
    static constexpr uint8_t kCoasted    = 6U;
    static constexpr uint8_t kDeleted    = 7U;
}  // namespace track_state

// Track.msg source_mask bits: which sensors contributed to the current
// track state (P3.1). A bearing-only candidate that has never seen radar
// is kCamera alone; the legacy radar cascade is kRadar alone.
namespace track_source {
    static constexpr uint8_t kRadar  = 1U;
    static constexpr uint8_t kCamera = 2U;
}  // namespace track_source

namespace threat_level {
    static constexpr uint8_t kUnknown     = 0U;
    static constexpr uint8_t kBenign      = 1U;
    static constexpr uint8_t kSuspect     = 2U;
    static constexpr uint8_t kIdentified  = 3U;
    static constexpr uint8_t kThreatening = 4U;
}  // namespace threat_level

}  // namespace cuas
