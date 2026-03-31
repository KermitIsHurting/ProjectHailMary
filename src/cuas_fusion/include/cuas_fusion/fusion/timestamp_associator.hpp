// timestamp_associator.hpp
// Aligns detections from sensors with different update rates and latencies
// by interpolating or extrapolating to a common reference timestamp.

#pragma once

namespace cuas {

class TimestampAssociator {
public:
    TimestampAssociator() = default;
};

} // namespace cuas
