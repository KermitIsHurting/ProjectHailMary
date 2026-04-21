// @file fusion_engine.hpp
// @brief Radar detection to YOLO box projector and associator.
#pragma once

#include "cuas_fusion/common/constants.hpp"
#include "cuas_fusion/common/fixed_containers.hpp"
#include "cuas_fusion/common/fixed_types.hpp"
#include "cuas_fusion/common/types.hpp"

namespace cuas {

class FusionEngine {
public:
    FusionEngine() = default;

    bool init(const ExtrinsicTransform& extrinsic);

    bool projectAndAssociate(
        const FixedVector<RadarDetection, TRACK_MAX_TRACKS>& radar_pts,
        const FixedVector<BoundingBox, 128U>& yolo_boxes,
        FixedVector<FusedDetection, TRACK_MAX_TRACKS>& fused_out);

private:
    static constexpr float32_t kEmaAlpha = 0.35F;

    struct EmaState {
        float32_t x   = 0.0F;
        float32_t y   = 0.0F;
        float32_t z   = 0.0F;
        float32_t u   = 0.0F;
        float32_t v   = 0.0F;
        float32_t vel = 0.0F;
        bool      valid = false;
    };

    ExtrinsicTransform extrinsic_{};
    bool               initialized_ = false;
    uint32_t           miss_count_  = 0U;
    FixedMap<int32_t, EmaState, FUSION_MAX_CLASSES> ema_per_class_{};
};

} // namespace cuas
