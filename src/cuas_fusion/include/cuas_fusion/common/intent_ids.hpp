// @file intent_ids.hpp
// @brief Numeric constants matching IntentReport.msg intent field.
#pragma once

// WHY: typed constants prevent string-based intent state and satisfy IR-3
// from first implementation rather than requiring a later compliance fix.

#include "cuas_fusion/common/fixed_types.hpp"

namespace cuas {

namespace intent_class {
    static constexpr uint8_t kUnknown     = 0U;
    static constexpr uint8_t kApproaching = 1U;
    static constexpr uint8_t kLoitering   = 2U;
    static constexpr uint8_t kOrbiting    = 3U;
    static constexpr uint8_t kDeparting   = 4U;
    static constexpr uint8_t kTransiting  = 5U;
}  // namespace intent_class

}  // namespace cuas
