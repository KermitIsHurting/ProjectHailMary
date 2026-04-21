// @file intent_classifier.hpp
// @brief Pure math class computing track behavioral intent from position and velocity.
#pragma once

#include "cuas_fusion/common/fixed_types.hpp"

namespace cuas {

struct IntentInput {
    float32_t x_m;
    float32_t y_m;
    float32_t vx_mps;
    float32_t vy_mps;
    float32_t speed_mps;
    uint32_t  track_id;
};

struct IntentResult {
    uint8_t   intent;
    float32_t confidence;
    float32_t loiter_radius_m;
    float32_t approach_rate_mps;
};

class IntentClassifier {
public:
    IntentClassifier() = default;

    IntentResult classify(const IntentInput & input) const;

private:
    static constexpr float32_t kApproachRateThresh =  0.3F;
    static constexpr float32_t kDepartRateThresh   = -0.3F;
    static constexpr float32_t kLoiterSpeedMax     =  0.5F;
    static constexpr float32_t kOrbitSpeedMin      =  0.3F;
    static constexpr float32_t kOrbitRadiusMin     =  1.0F;
    static constexpr float32_t kOrbitRadiusMax     = 10.0F;
    static constexpr float32_t kTransitSpeedMin    =  0.8F;
    static constexpr float32_t kOriginEpsilon      =  0.01F;
};

} // namespace cuas
