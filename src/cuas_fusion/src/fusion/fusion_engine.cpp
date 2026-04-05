// fusion_engine.cpp

#include "cuas_fusion/fusion/fusion_engine.hpp"
#include "cuas_fusion/common/constants.hpp"

#include <cstdio>
#include <cmath>
#include <unordered_map>

namespace cuas {

bool FusionEngine::init(const ExtrinsicTransform& extrinsic)
{
    extrinsic_ = extrinsic;
    miss_count_ = 0;
    initialized_ = true;
    return true;
}

bool FusionEngine::projectAndAssociate(
    const std::vector<RadarDetection>& radar_pts,
    const std::vector<BoundingBox>& yolo_boxes,
    std::vector<FusedDetection>& fused_out)
{
    if (!initialized_) {
        fprintf(stderr, "FusionEngine: not initialized\n");
        return false;
    }

    fused_out.clear();

    struct Candidate {
        FusedDetection detection;
        float range;
    };
    std::unordered_map<size_t, Candidate> best_per_box;

    for (const auto& rpt : radar_pts) {
        float x_cam = rpt.x + extrinsic_.x_m;
        float y_cam = rpt.y + extrinsic_.y_m;
        float z_cam = rpt.z + extrinsic_.z_m;

        if (z_cam <= 0.0f) {
            continue;
        }

        // Pinhole projection
        float u = CAMERA_FX * (x_cam / z_cam) + CAMERA_CX;
        float v = CAMERA_FY * (y_cam / z_cam) + CAMERA_CY;

        if (u < 0.0f || u >= static_cast<float>(CAMERA_IMAGE_W) ||
            v < 0.0f || v >= static_cast<float>(CAMERA_IMAGE_H)) {
            fprintf(stderr, "FusionEngine: radar point projected out of bounds "
                    "(u=%.1f, v=%.1f)\n", u, v);
            continue;
        }

        bool matched = false;
        for (size_t bi = 0; bi < yolo_boxes.size(); ++bi) {
            const auto& box = yolo_boxes[bi];
            if (u >= box.x && u <= box.x + box.w &&
                v >= box.y && v <= box.y + box.h) {
                FusedDetection fd;
                fd.position_x_m = rpt.x;
                fd.position_y_m = rpt.y;
                fd.position_z_m = rpt.z;
                fd.velocity_mps = rpt.velocity;
                fd.class_label  = std::to_string(box.class_id);
                fd.confidence   = box.confidence;
                fd.pixel_u      = u;
                fd.pixel_v      = v;
                fd.timestamp_ns = rpt.timestamp_ns;

                float range = rpt.y;

                auto it = best_per_box.find(bi);
                if (it == best_per_box.end() || range < it->second.range) {
                    best_per_box[bi] = {fd, range};
                }

                matched = true;
                break;
            }
        }

        if (!matched) {
            ++miss_count_;
            if (miss_count_ % 100 == 1) {
                fprintf(stderr, "FusionEngine: radar point at (u=%.1f, v=%.1f) "
                        "matched no YOLO box (miss #%zu)\n", u, v, miss_count_);
            }
        }
    }

    fused_out.reserve(best_per_box.size());
    for (auto& [box_idx, candidate] : best_per_box) {
        fused_out.push_back(std::move(candidate.detection));
    }

    return true;
}

} // namespace cuas
