// @file types.hpp
// @brief Common data structures and state enums shared across the pipeline.
#pragma once

#include "cuas_fusion/common/fixed_types.hpp"

#include <array>
#include <charconv>
#include <cmath>
#include <string>
#include <system_error>

namespace cuas {

enum class TrackState {
    TENTATIVE,
    CONFIRMED,
    OCCLUDED,
    REACQUIRED,
    COASTED,
    LOST,
    DELETED
};

enum class ThreatLevel {
    BENIGN,
    UNKNOWN,
    SUSPECT,
    THREAT
};

inline const char* trackStateToString(TrackState state)
{
    switch (state) {
        case TrackState::TENTATIVE:  { return "TENTATIVE"; }
        case TrackState::CONFIRMED:  { return "CONFIRMED"; }
        case TrackState::OCCLUDED:   { return "OCCLUDED"; }
        case TrackState::REACQUIRED: { return "REACQUIRED"; }
        case TrackState::COASTED:    { return "COASTED"; }
        case TrackState::LOST:       { return "LOST"; }
        case TrackState::DELETED:    { return "DELETED"; }
        default:                     { return "UNKNOWN"; }
    }
}

// `recognized` (optional) reports whether the input matched a known state,
// so callers can warn instead of silently treating garbage as a live
// TENTATIVE track (A6.6).
inline TrackState trackStateFromString(const std::string& s,
                                       bool* recognized = nullptr)
{
    if (recognized != nullptr) { *recognized = true; }
    if (s == "TENTATIVE")  { return TrackState::TENTATIVE; }
    if (s == "CONFIRMED")  { return TrackState::CONFIRMED; }
    if (s == "OCCLUDED")   { return TrackState::OCCLUDED; }
    if (s == "REACQUIRED") { return TrackState::REACQUIRED; }
    if (s == "COASTED")    { return TrackState::COASTED; }
    if (s == "LOST")       { return TrackState::LOST; }
    if (s == "DELETED")    { return TrackState::DELETED; }
    if (recognized != nullptr) { *recognized = false; }
    return TrackState::TENTATIVE;
}

inline const char* threatLevelToString(ThreatLevel level)
{
    switch (level) {
        case ThreatLevel::BENIGN:  { return "BENIGN"; }
        case ThreatLevel::UNKNOWN: { return "UNKNOWN"; }
        case ThreatLevel::SUSPECT: { return "SUSPECT"; }
        case ThreatLevel::THREAT:  { return "THREAT"; }
        default:                   { return "UNKNOWN"; }
    }
}

struct BoundingBox {
    float32_t x = 0.0F;
    float32_t y = 0.0F;
    float32_t w = 0.0F;
    float32_t h = 0.0F;
    float32_t confidence  = 0.0F;
    int32_t   class_id    = 0;
    int64_t   timestamp_ns = 0;
};

// Radar-to-camera rigid transform: p_cam = R(q) · p_radar + t.
//
// Frames: radar (x right, y forward/boresight, z up); camera optical
// (x right, y down, z forward). The default quaternion is +90° about
// radar x — exactly the radar→camera axis convention for a perfectly
// aligned mount, so a default-constructed transform reproduces the
// legacy hardcoded projection. Calibration (tools/calibrate_extrinsics.py)
// folds real mount misalignment into q. Translation is the radar origin
// expressed in CAMERA axes (not the old radar-frame offsets).
struct ExtrinsicTransform {
    float32_t q_w = 0.70710678F;  // unit quaternion, w-x-y-z order
    float32_t q_x = 0.70710678F;
    float32_t q_y = 0.0F;
    float32_t q_z = 0.0F;
    float32_t t_x_m = 0.0F;       // camera-frame translation
    float32_t t_y_m = 0.0F;
    float32_t t_z_m = 0.0F;
};

// Row-major 3x3 rotation matrix from the extrinsic quaternion, normalized
// here so callers may store a non-unit (e.g. hand-edited) quaternion.
// Returns false — R untouched — for a non-finite or near-zero quaternion,
// so a corrupt config fails init instead of poisoning every projection.
inline bool extrinsicRotationMatrix(const ExtrinsicTransform& e,
                                    std::array<float32_t, 9>& R)
{
    const float32_t n2 = (e.q_w * e.q_w) + (e.q_x * e.q_x) +
                         (e.q_y * e.q_y) + (e.q_z * e.q_z);
    // Negated comparison: a NaN norm takes the reject branch.
    if (!(n2 > 1.0e-6F) || !(n2 < 1.0e12F)) {
        return false;
    }
    const float32_t s = 1.0F / std::sqrt(n2);
    const float32_t w = e.q_w * s;
    const float32_t x = e.q_x * s;
    const float32_t y = e.q_y * s;
    const float32_t z = e.q_z * s;
    R[0] = 1.0F - 2.0F * ((y * y) + (z * z));
    R[1] = 2.0F * ((x * y) - (w * z));
    R[2] = 2.0F * ((x * z) + (w * y));
    R[3] = 2.0F * ((x * y) + (w * z));
    R[4] = 1.0F - 2.0F * ((x * x) + (z * z));
    R[5] = 2.0F * ((y * z) - (w * x));
    R[6] = 2.0F * ((x * z) - (w * y));
    R[7] = 2.0F * ((y * z) + (w * x));
    R[8] = 1.0F - 2.0F * ((x * x) + (y * y));
    return true;
}

struct RadarDetection {
    float32_t x        = 0.0F;
    float32_t y        = 0.0F;
    float32_t z        = 0.0F;
    float32_t velocity = 0.0F;
    int64_t   timestamp_ns = 0;
};

struct FusedDetection {
    float32_t   position_x_m = 0.0F;
    float32_t   position_y_m = 0.0F;
    float32_t   position_z_m = 0.0F;
    float32_t   velocity_mps = 0.0F;
    // -1 = unlabeled. Numeric id only in hot-path value types (A3.8);
    // stringify at the ROS publish boundary (DEV-005 chokepoint).
    int32_t     class_id   = -1;
    float32_t   confidence = 0.0F;
    float32_t   pixel_u    = 0.0F;
    float32_t   pixel_v    = 0.0F;
    int64_t     timestamp_ns = 0;
    float32_t   range_m     = 0.0F;
    float32_t   azimuth_deg = 0.0F;
    float32_t   bbox_width_px  = 0.0F;
    float32_t   bbox_height_px = 0.0F;
};

// DEV-005 chokepoint helpers: class ids cross the ROS boundary as strings;
// empty or unparseable = unlabeled = -1 (A3.8).
inline int32_t parseClassId(const std::string& s)
{
    if (s.empty()) {
        return -1;
    }
    int32_t value = 0;
    const auto result = std::from_chars(s.data(), s.data() + s.size(), value);
    if (result.ec != std::errc{}) {
        return -1;
    }
    return value;
}

inline std::string classIdToLabel(int32_t class_id)
{
    return (class_id < 0) ? std::string{} : std::to_string(class_id);
}

} // namespace cuas
