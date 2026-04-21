// @file intent_classifier.cpp
// @brief Implements radial/tangential intent classification with explicit clamps.
#include "cuas_fusion/intent_classifier.hpp"
#include "cuas_fusion/common/intent_ids.hpp"

#include <cmath>

namespace cuas {

IntentResult IntentClassifier::classify(const IntentInput & input) const
{
    IntentResult result;
    result.intent            = cuas::intent_class::kUnknown;
    result.confidence        = 0.0F;
    result.loiter_radius_m   = 0.0F;
    result.approach_rate_mps = 0.0F;

    const float32_t dist_sq  = (input.x_m * input.x_m) + (input.y_m * input.y_m);
    const float32_t distance = std::sqrt(dist_sq);

    if (distance < kOriginEpsilon) {
        return result;
    }

    const float32_t bx = -input.x_m / distance;
    const float32_t by = -input.y_m / distance;
    const float32_t approach_rate =
        (input.vx_mps * bx) + (input.vy_mps * by);

    if (approach_rate > kApproachRateThresh) {
        result.intent = cuas::intent_class::kApproaching;
        float32_t conf = approach_rate / 2.0F;
        if (conf < 0.0F) {
            conf = 0.0F;
        }
        if (conf > 1.0F) {
            conf = 1.0F;
        }
        result.confidence        = conf;
        result.approach_rate_mps = approach_rate;
    } else if (approach_rate < kDepartRateThresh) {
        result.intent = cuas::intent_class::kDeparting;
        float32_t conf = -approach_rate / 2.0F;
        if (conf < 0.0F) {
            conf = 0.0F;
        }
        if (conf > 1.0F) {
            conf = 1.0F;
        }
        result.confidence = conf;
    } else if ((input.speed_mps >= kOrbitSpeedMin) &&
               (distance         >= kOrbitRadiusMin) &&
               (distance         <= kOrbitRadiusMax)) {
        result.intent          = cuas::intent_class::kOrbiting;
        result.loiter_radius_m = distance;
        result.confidence      = 0.7F;
    } else if (input.speed_mps <= kLoiterSpeedMax) {
        result.intent          = cuas::intent_class::kLoitering;
        result.loiter_radius_m = distance;
        float32_t conf = 1.0F - (input.speed_mps / kLoiterSpeedMax);
        if (conf < 0.0F) {
            conf = 0.0F;
        }
        if (conf > 1.0F) {
            conf = 1.0F;
        }
        result.confidence = conf;
    } else if (input.speed_mps >= kTransitSpeedMin) {
        result.intent = cuas::intent_class::kTransiting;
        float32_t conf = input.speed_mps / 3.0F;
        if (conf < 0.0F) {
            conf = 0.0F;
        }
        if (conf > 1.0F) {
            conf = 1.0F;
        }
        result.confidence = conf;
    } else {
        result.intent     = cuas::intent_class::kUnknown;
        result.confidence = 0.0F;
    }

    return result;
}

} // namespace cuas
