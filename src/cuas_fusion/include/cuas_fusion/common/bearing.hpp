// @file bearing.hpp
// @brief The one radar-frame bearing convention used on the wire.
#pragma once

#include "cuas_fusion/common/fixed_types.hpp"

#include <cmath>

namespace cuas {

// Radar-frame bearing in degrees: 0 at boresight (+Y, the range axis per
// ICD radar_frame), positive toward +X. The ONE producer of every azimuth
// or bearing on the wire (RC-21): the predictor and the RViz label used
// atan2(y, x), which put boresight at 90 deg while fusion, the classifier
// and CoT used this convention.
inline float32_t bearingDegBoresightZero(float32_t x_m, float32_t y_m)
{
    return std::atan2(x_m, y_m) * (180.0F / 3.14159265358979F);
}

} // namespace cuas
