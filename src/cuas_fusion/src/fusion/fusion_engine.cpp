// @file fusion_engine.cpp
// @brief Projects radar detections into camera frame and joins them with YOLO boxes.
#include "cuas_fusion/fusion/fusion_engine.hpp"
#include "cuas_fusion/common/constants.hpp"
#include "cuas_fusion/common/fixed_containers.hpp"
#include "cuas_fusion/common/fixed_types.hpp"

#include <cmath>

namespace cuas {

bool FusionEngine::init(const ExtrinsicTransform& extrinsic)
{
    extrinsic_   = extrinsic;
    miss_count_  = 0U;
    initialized_ = true;
    return true;
}

bool FusionEngine::projectAndAssociate(
    const FixedVector<RadarDetection, TRACK_MAX_TRACKS>& radar_pts,
    const FixedVector<BoundingBox, 128U>& yolo_boxes,
    FixedVector<FusedDetection, TRACK_MAX_TRACKS>& fused_out)
{
    if (!initialized_) {
        return false;
    }

    fused_out.clear();

    struct Accumulator {
        float32_t sum_x = 0.0F;
        float32_t sum_y = 0.0F;
        float32_t sum_z = 0.0F;
        float32_t sum_vel = 0.0F;
        float32_t sum_u = 0.0F;
        float32_t sum_v = 0.0F;
        int32_t   count = 0;
        int64_t   timestamp_ns = 0;
        const BoundingBox* box = nullptr;
    };

    FixedMap<uint32_t, Accumulator, FUSION_MAX_DETECTIONS> accum_per_box{};

    for (uint32_t rpti = 0U; rpti < radar_pts.size(); ++rpti) {
        const RadarDetection& rpt = radar_pts[rpti];
        // Radar frame (x right, y forward, z up) to camera frame (x right, y down, z forward)
        const float32_t x_cam = rpt.x + extrinsic_.x_m;
        const float32_t z_cam = rpt.y + extrinsic_.y_m;
        const float32_t y_cam = -(rpt.z + extrinsic_.z_m);

        if (z_cam <= 0.0F) {
            continue;
        }

        const float32_t u = CAMERA_FX * (x_cam / z_cam) + CAMERA_CX;
        const float32_t v = CAMERA_FY * (y_cam / z_cam) + CAMERA_CY;

        if (u < 0.0F || u >= static_cast<float32_t>(CAMERA_IMAGE_W) ||
            v < 0.0F || v >= static_cast<float32_t>(CAMERA_IMAGE_H)) {
            continue;
        }

        bool matched = false;
        for (uint32_t bi = 0U; bi < yolo_boxes.size(); ++bi) {
            const BoundingBox& box = yolo_boxes[bi];
            // 25% padding accounts for YOLO box jitter between frames
            const float32_t pad_x = box.w * 0.25F;
            const float32_t pad_y = box.h * 0.25F;
            if (u >= box.x - pad_x && u <= box.x + box.w + pad_x &&
                v >= box.y - pad_y && v <= box.y + box.h + pad_y) {
                const uint32_t key = static_cast<uint32_t>(bi);
                Accumulator* acc = accum_per_box.find(key);
                if (acc == nullptr) {
                    Accumulator fresh;
                    fresh.box = &box;
                    (void)accum_per_box.insert_or_assign(key, fresh);
                    acc = accum_per_box.find(key);
                }
                if (acc != nullptr) {
                    acc->sum_x   += rpt.x;
                    acc->sum_y   += rpt.y;
                    acc->sum_z   += rpt.z;
                    acc->sum_vel += rpt.velocity;
                    acc->sum_u   += u;
                    acc->sum_v   += v;
                    ++acc->count;
                    acc->timestamp_ns = rpt.timestamp_ns;
                    acc->box = &box;
                }
                matched = true;
                break;
            }
        }

        if (!matched) {
            ++miss_count_;
        }
    }

    for (uint32_t s = 0U; s < accum_per_box.slot_count(); ++s) {
        const auto& slot = accum_per_box.slots()[s];
        if (!slot.occupied) {
            continue;
        }
        const Accumulator& acc = slot.value;
        if (acc.box == nullptr || acc.count <= 0) {
            continue;
        }

        const float32_t n = static_cast<float32_t>(acc.count);
        float32_t rx   = acc.sum_x   / n;
        float32_t ry   = acc.sum_y   / n;
        float32_t rz   = acc.sum_z   / n;
        float32_t rvel = acc.sum_vel / n;

        // YOLO bbox centre used for pixel location; radar z is too noisy for display
        const float32_t yu = acc.box->x + acc.box->w * 0.5F;
        const float32_t yv = acc.box->y + acc.box->h * 0.5F;

        // EMA smooths only the radar 3D position; YOLO centre is already stable
        const int32_t key = acc.box->class_id;
        EmaState* ema = ema_per_class_.find(key);
        if (ema == nullptr) {
            EmaState fresh;
            (void)ema_per_class_.insert_or_assign(key, fresh);
            ema = ema_per_class_.find(key);
        }
        if (ema != nullptr) {
            if (ema->valid) {
                rx   = kEmaAlpha * rx   + (1.0F - kEmaAlpha) * ema->x;
                ry   = kEmaAlpha * ry   + (1.0F - kEmaAlpha) * ema->y;
                rz   = kEmaAlpha * rz   + (1.0F - kEmaAlpha) * ema->z;
                rvel = kEmaAlpha * rvel + (1.0F - kEmaAlpha) * ema->vel;
            }
            ema->x = rx;
            ema->y = ry;
            ema->z = rz;
            ema->u = yu;
            ema->v = yv;
            ema->vel = rvel;
            ema->valid = true;
        }

        FusedDetection fd;
        fd.position_x_m = rx;
        fd.position_y_m = ry;
        fd.position_z_m = rz;
        fd.velocity_mps = rvel;
        fd.class_label  = std::to_string(acc.box->class_id);
        fd.confidence   = acc.box->confidence;
        fd.pixel_u      = yu;
        fd.pixel_v      = yv;
        fd.timestamp_ns = acc.timestamp_ns;
        fd.range_m      = ry;
        fd.azimuth_deg  = std::atan2(rx, ry) * 180.0F / static_cast<float32_t>(M_PI);
        fd.bbox_width_px  = acc.box->w;
        fd.bbox_height_px = acc.box->h;
        (void)fused_out.push_back(fd);
    }

    return true;
}

} // namespace cuas
