// @file topk_boxes.hpp
// @brief Bounded keeper of the K highest-confidence boxes seen (pre-NMS).
#pragma once

#include "cuas_fusion/common/fixed_containers.hpp"
#include "cuas_fusion/common/fixed_types.hpp"
#include "cuas_fusion/common/types.hpp"

namespace cuas {

// The YOLO head emits 8400 anchors; only K survive to NMS. Taking the
// first K above threshold in anchor order dropped the nearest, largest
// target whenever clutter filled the cap first (RC-19). offer() keeps the
// K best by confidence in O(K) per call with no allocation.
template <uint32_t K>
class TopKBoxes {
public:
    void offer(const BoundingBox& b)
    {
        if (boxes_.size() < K) {
            (void)boxes_.push_back(b);
            if (boxes_.size() == K) {
                refreshMin();
            }
            return;
        }
        if (!(b.confidence > boxes_[min_idx_].confidence)) {
            return;
        }
        boxes_[min_idx_] = b;
        refreshMin();
    }

    const FixedVector<BoundingBox, K>& boxes() const { return boxes_; }
    uint32_t size() const { return boxes_.size(); }

private:
    void refreshMin()
    {
        min_idx_ = 0U;
        for (uint32_t i = 1U; i < boxes_.size(); ++i) {
            if (boxes_[i].confidence < boxes_[min_idx_].confidence) {
                min_idx_ = i;
            }
        }
    }

    FixedVector<BoundingBox, K> boxes_;
    uint32_t min_idx_ = 0U;
};

} // namespace cuas
