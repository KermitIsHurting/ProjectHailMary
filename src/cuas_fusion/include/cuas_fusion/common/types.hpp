// types.hpp

#pragma once

#include <cstdint>
#include <string>

namespace cuas {

struct BoundingBox {
    float   x, y, w, h;
    float   confidence;
    int     class_id;
    int64_t timestamp_ns;
};

struct ExtrinsicTransform {
    float x_m;
    float y_m;
    float z_m;
};

struct RadarDetection {
    float   x;
    float   y;
    float   z;
    float   velocity;
    int64_t timestamp_ns;
};

struct FusedDetection {
    float       position_x_m;
    float       position_y_m;
    float       position_z_m;
    float       velocity_mps;
    std::string class_label;
    float       confidence;
    float       pixel_u;
    float       pixel_v;
    int64_t     timestamp_ns;
};

} // namespace cuas
