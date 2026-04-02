// types.hpp
// Shared data structures used across all CUAS fusion components:
// detections, tracks, sensor metadata, and geometry primitives.

#pragma once

#include <cstdint>

namespace cuas {

struct BoundingBox {
    float   x, y, w, h;    // pixel coords, origin top-left
    float   confidence;
    int     class_id;
    int64_t timestamp_ns;
};

} // namespace cuas
