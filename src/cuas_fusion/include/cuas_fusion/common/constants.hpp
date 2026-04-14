// @file constants.hpp
// @brief Project-wide compile-time numerical constants.
#pragma once

#include "cuas_fusion/common/fixed_types.hpp"

#include <array>
#include <cstddef>
#include <string_view>

namespace cuas {

static constexpr const char * CAMERA_DEVICE_PATH      = "/dev/video0";
static constexpr int32_t      CAMERA_WIDTH             = 1920;
static constexpr int32_t      CAMERA_HEIGHT            = 1080;
static constexpr int32_t      CAMERA_FPS               = 30;
static constexpr const char * CAMERA_TOPIC             = "/camera/image_raw";
static constexpr int32_t      CAMERA_RETRY_INTERVAL_MS = 1000;

// AR0234 via Arducam CSI2 reports ~2752 in optically dark pixels
static constexpr uint16_t     CAMERA_BLACK_LEVEL        = 2752U;

// Sensor lacks an IR-cut filter so blue is ~3x weaker than green/red
static constexpr float32_t    CAMERA_WB_GAIN_B         = 2.987F;
static constexpr float32_t    CAMERA_WB_GAIN_G         = 1.000F;
static constexpr float32_t    CAMERA_WB_GAIN_R         = 0.861F;

// Maps black-subtracted 16-bit Bayer to 8-bit; 255 / 99.5th percentile (~9800)
static constexpr float32_t    CAMERA_TONE_SCALE        = 0.026F;

static constexpr int32_t   INFERENCE_INPUT_W      = 640;
static constexpr int32_t   INFERENCE_INPUT_H      = 640;
static constexpr int32_t   INFERENCE_INPUT_C      = 3;
static constexpr float32_t INFERENCE_CONF_THRESH  = 0.25F;
static constexpr float32_t INFERENCE_NMS_THRESH   = 0.45F;
static constexpr int32_t   INFERENCE_MAX_DET      = 100;
static constexpr int32_t   INFERENCE_NUM_CLASSES  = 80;
static constexpr int32_t   INFERENCE_NUM_ANCHORS  = 8400;

static constexpr std::size_t TIMESTAMP_BUFFER_SIZE   = 6U;
static constexpr int64_t     MAX_TIMESTAMP_DELTA_NS  = 150'000'000LL; // 150 ms

static constexpr float32_t FUSION_IOU_THRESHOLD       = 0.15F;
static constexpr float32_t FUSION_MAX_PROJ_ERROR_PX   = 50.0F;
static constexpr float32_t CAMERA_FX                  = 1862.7F;  // checkerboard calibration 2026-04-09
static constexpr float32_t CAMERA_FY                  = 1877.7F;
static constexpr float32_t CAMERA_CX                  = 1032.8F;
static constexpr float32_t CAMERA_CY                  = 426.8F;
static constexpr int32_t   CAMERA_IMAGE_W             = 1920;
static constexpr int32_t   CAMERA_IMAGE_H             = 1080;

static constexpr uint32_t  TRACK_MAX_TRACKS            = 32U;
static constexpr float32_t TRACK_ASSOCIATION_DIST_M    = 3.0F;
static constexpr int32_t   TRACK_CONFIRM_HITS          = 3;
static constexpr int32_t   TRACK_MAX_MISSES            = 5;
static constexpr float64_t kRadarDetectionSigmaM       = 0.15;

static constexpr float32_t kConfidenceDecayRate        = 0.05F;
static constexpr float32_t kConfidenceGainRate         = 0.20F;
static constexpr float32_t kMinConfidence              = 0.10F;
static constexpr float32_t kInitConfidence             = 0.10F;
static constexpr float32_t kConfirmedConfidence        = 0.60F;

static constexpr float64_t kManeuverCtThreshold        = 0.6;
static constexpr uint32_t  kManeuverFramesRequired     = 3U;
static constexpr float64_t kManeuverCvSigmaA           = 0.2;
static constexpr float64_t kManeuverCaSigmaJ           = 0.3;
static constexpr float64_t kNominalCvSigmaA            = 0.5;
static constexpr float64_t kNominalCaSigmaJ            = 1.0;

static constexpr float32_t THREAT_VELOCITY_SUSPECT_MPS   = 2.0F;
static constexpr float32_t THREAT_VELOCITY_THREAT_MPS    = 5.0F;
static constexpr float32_t THREAT_APPROACH_THRESHOLD_MPS = -0.5F;
static constexpr float32_t THREAT_MIN_CONFIDENCE         = 0.25F;

static constexpr uint32_t  PREDICTION_MAX_STEPS        = 128U;

static constexpr uint32_t  FUSION_MAX_DETECTIONS       = 128U;
static constexpr uint32_t  FUSION_MAX_CLASSES          = 80U;

static constexpr std::array<std::string_view, 3> THREAT_DRONE_CLASSES = {
    "14", "4", "33"   // bird, airplane, kite (COCO numeric IDs)
};

static constexpr std::array<std::string_view, 5> THREAT_BENIGN_CLASSES = {
    "0", "2", "7", "3", "1"  // person, car, truck, motorcycle, bicycle (COCO numeric IDs)
};

} // namespace cuas
