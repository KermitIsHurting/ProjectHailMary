// constants.hpp
// System-wide compile-time constants: coordinate frame IDs, timing thresholds,
// filter tuning defaults, and physical constants used by the fusion pipeline.

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace cuas {

// ---------------------------------------------------------------------------
// Camera (Arducam AR0234 B0429, MIPI CSI-2)
// ---------------------------------------------------------------------------
static constexpr const char * CAMERA_DEVICE_PATH      = "/dev/video0";
static constexpr int          CAMERA_WIDTH             = 1920;
static constexpr int          CAMERA_HEIGHT            = 1080;
static constexpr int          CAMERA_FPS               = 30;
static constexpr const char * CAMERA_TOPIC             = "/camera/image_raw";
static constexpr int          CAMERA_RETRY_INTERVAL_MS = 1000;

// Sensor black level (pedestal) in raw 16-bit units.
// AR0234 via Arducam CSI2 reports ~2752 in optically dark pixels.
static constexpr uint16_t     CAMERA_BLACK_LEVEL        = 2752;

// White-balance gains calibrated from a neutral grey surface.
// The sensor lacks an IR-cut filter so blue is ~3x weaker than green/red.
static constexpr float        CAMERA_WB_GAIN_B         = 2.987f;
static constexpr float        CAMERA_WB_GAIN_G         = 1.000f;
static constexpr float        CAMERA_WB_GAIN_R         = 0.861f;

// Tone-mapping scale: maps black-subtracted 16-bit Bayer data to 8-bit.
// Derived empirically: 255 / 99.5th-percentile of typical scene (~9800).
static constexpr float        CAMERA_TONE_SCALE        = 0.026f;

// ---------------------------------------------------------------------------
// Inference (YOLOv8s TensorRT INT8)
// ---------------------------------------------------------------------------
static constexpr int   INFERENCE_INPUT_W      = 640;
static constexpr int   INFERENCE_INPUT_H      = 640;
static constexpr int   INFERENCE_INPUT_C      = 3;
static constexpr float INFERENCE_CONF_THRESH  = 0.45f;
static constexpr float INFERENCE_NMS_THRESH   = 0.45f;
static constexpr int   INFERENCE_MAX_DET      = 100;
static constexpr int   INFERENCE_NUM_CLASSES  = 80;
static constexpr int   INFERENCE_NUM_ANCHORS  = 8400;

// ---------------------------------------------------------------------------
// Timestamp association
// ---------------------------------------------------------------------------
static constexpr size_t   TIMESTAMP_BUFFER_SIZE   = 6;
static constexpr int64_t  MAX_TIMESTAMP_DELTA_NS  = 50'000'000LL;  // 50 ms

// ---------------------------------------------------------------------------
// Fusion (radar-camera association)
// ---------------------------------------------------------------------------
static constexpr float  FUSION_IOU_THRESHOLD       = 0.15f;
static constexpr float  FUSION_MAX_PROJ_ERROR_PX   = 50.0f;
static constexpr float  CAMERA_FX                  = 800.0f;   // estimate — update after calibration
static constexpr float  CAMERA_FY                  = 800.0f;
static constexpr float  CAMERA_CX                  = 960.0f;
static constexpr float  CAMERA_CY                  = 600.0f;
static constexpr int    CAMERA_IMAGE_W             = 1920;
static constexpr int    CAMERA_IMAGE_H             = 1080;

// ---------------------------------------------------------------------------
// Tracking
// ---------------------------------------------------------------------------
static constexpr size_t TRACK_MAX_TRACKS            = 32;
static constexpr float  TRACK_ASSOCIATION_DIST_M    = 3.0f;
static constexpr int    TRACK_CONFIRM_HITS          = 3;
static constexpr int    TRACK_MAX_MISSES            = 5;

// ---------------------------------------------------------------------------
// Threat classifier thresholds
// ---------------------------------------------------------------------------
static constexpr float THREAT_VELOCITY_SUSPECT_MPS   = 2.0f;
static constexpr float THREAT_VELOCITY_THREAT_MPS    = 5.0f;
static constexpr float THREAT_APPROACH_THRESHOLD_MPS = -0.5f;
static constexpr float THREAT_MIN_CONFIDENCE         = 0.45f;

static constexpr std::array<std::string_view, 3> THREAT_DRONE_CLASSES = {
    "bird", "airplane", "kite"
};

static constexpr std::array<std::string_view, 5> THREAT_BENIGN_CLASSES = {
    "person", "car", "truck", "motorcycle", "bicycle"
};

} // namespace cuas
