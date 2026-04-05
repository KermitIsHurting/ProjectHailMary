// fusion_engine.hpp
// Pure C++ — zero ROS dependency.

#pragma once

#include <cstdint>
#include <vector>

#include "cuas_fusion/common/types.hpp"

namespace cuas {

class FusionEngine {
public:
    FusionEngine() = default;

    bool init(const ExtrinsicTransform& extrinsic);

    bool projectAndAssociate(
        const std::vector<RadarDetection>& radar_pts,
        const std::vector<BoundingBox>& yolo_boxes,
        std::vector<FusedDetection>& fused_out);

private:
    ExtrinsicTransform extrinsic_{};
    bool initialized_ = false;
    size_t miss_count_ = 0;
};

} // namespace cuas
