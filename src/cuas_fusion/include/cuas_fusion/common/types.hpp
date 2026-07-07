// @file types.hpp
// @brief Common data structures and state enums shared across the pipeline.
#pragma once

#include "cuas_fusion/common/fixed_types.hpp"

#include <charconv>
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

struct ExtrinsicTransform {
    float32_t x_m = 0.0F;
    float32_t y_m = 0.0F;
    float32_t z_m = 0.0F;
};

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
