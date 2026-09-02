// @file fusion_engine.cpp
// @brief Projects radar detections into camera frame and joins them with YOLO boxes.
#include "cuas_fusion/common/bearing.hpp"
#include "cuas_fusion/fusion/fusion_engine.hpp"
#include "cuas_fusion/common/constants.hpp"
#include "cuas_fusion/common/fixed_containers.hpp"
#include "cuas_fusion/common/fixed_types.hpp"

#include <cmath>

namespace cuas {

bool FusionEngine::init(const ExtrinsicTransform& extrinsic)
{
    if (!extrinsicRotationMatrix(extrinsic, rot_)) {
        return false;
    }
    // Negated form: a NaN offset must fail init, not poison every projection.
    const bool t_ok = (std::abs(extrinsic.t_x_m) < 1.0e6F) &&
                      (std::abs(extrinsic.t_y_m) < 1.0e6F) &&
                      (std::abs(extrinsic.t_z_m) < 1.0e6F);
    if (!t_ok) {
        return false;
    }
    trans_[0]    = extrinsic.t_x_m;
    trans_[1]    = extrinsic.t_y_m;
    trans_[2]    = extrinsic.t_z_m;
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
        // Full SE(3) radar→camera projection; the default extrinsic
        // quaternion reduces this to the old axis permutation exactly.
        const float32_t x_cam = (rot_[0] * rpt.x) + (rot_[1] * rpt.y) +
                                (rot_[2] * rpt.z) + trans_[0];
        const float32_t y_cam = (rot_[3] * rpt.x) + (rot_[4] * rpt.y) +
                                (rot_[5] * rpt.z) + trans_[1];
        const float32_t z_cam = (rot_[6] * rpt.x) + (rot_[7] * rpt.y) +
                                (rot_[8] * rpt.z) + trans_[2];

        // Negated: a NaN depth takes the skip branch (behind-camera guard).
        if (!(z_cam > 0.0F)) {
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
        EmaState* ema = associateEma(acc.box->class_id, rx, ry, rz,
                                     acc.timestamp_ns);
        if (ema != nullptr) {
            if (ema->valid) {
                rx   = kEmaAlpha * rx   + (1.0F - kEmaAlpha) * ema->x;
                ry   = kEmaAlpha * ry   + (1.0F - kEmaAlpha) * ema->y;
                rz   = kEmaAlpha * rz   + (1.0F - kEmaAlpha) * ema->z;
                rvel = kEmaAlpha * rvel + (1.0F - kEmaAlpha) * ema->vel;
            }
            ema->x   = rx;
            ema->y   = ry;
            ema->z   = rz;
            ema->vel = rvel;
            ema->class_id       = acc.box->class_id;
            ema->last_update_ns = acc.timestamp_ns;
            ema->valid          = true;
        }

        FusedDetection fd;
        fd.position_x_m = rx;
        fd.position_y_m = ry;
        fd.position_z_m = rz;
        fd.velocity_mps = rvel;
        fd.class_id     = acc.box->class_id;
        fd.confidence   = acc.box->confidence;
        fd.pixel_u      = yu;
        fd.pixel_v      = yv;
        fd.timestamp_ns = acc.timestamp_ns;
        // Euclidean range, not the forward coordinate (RC-29).
        fd.range_m      = std::sqrt((rx * rx) + (ry * ry) + (rz * rz));
        fd.azimuth_deg  = bearingDegBoresightZero(rx, ry);
        fd.bbox_width_px  = acc.box->w;
        fd.bbox_height_px = acc.box->h;
        (void)fused_out.push_back(fd);
    }

    return true;
}

FusionEngine::EmaState* FusionEngine::associateEma(
    int32_t class_id, float32_t x, float32_t y, float32_t z, int64_t now_ns)
{
    for (EmaState& s : ema_states_) {
        if (s.valid && (now_ns - s.last_update_ns) > kEmaTimeoutNs) {
            s.valid = false;
        }
    }

    EmaState* best   = nullptr;
    float32_t best_d = kEmaGateM;
    for (EmaState& s : ema_states_) {
        if (!s.valid || s.class_id != class_id) {
            continue;
        }
        const float32_t dx = x - s.x;
        const float32_t dy = y - s.y;
        const float32_t dz = z - s.z;
        const float32_t d  = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (d <= best_d) {
            best_d = d;
            best   = &s;
        }
    }
    if (best != nullptr) {
        return best;
    }

    // No gate match: recycle a free slot, else the longest-unrefreshed one.
    EmaState* oldest = &ema_states_[0];
    for (EmaState& s : ema_states_) {
        if (!s.valid) {
            s = EmaState{};
            return &s;
        }
        if (s.last_update_ns < oldest->last_update_ns) {
            oldest = &s;
        }
    }
    *oldest = EmaState{};
    return oldest;
}

} // namespace cuas
