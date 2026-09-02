// @file constants.hpp
// @brief Project-wide compile-time numerical constants.
#pragma once

#include "cuas_fusion/common/fixed_types.hpp"

#include <cstddef>

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
// Derived, not duplicate literals: these must always equal the capture
// resolution (values unchanged, R12g).
static constexpr int32_t   CAMERA_IMAGE_W             = CAMERA_WIDTH;
static constexpr int32_t   CAMERA_IMAGE_H             = CAMERA_HEIGHT;

static constexpr uint32_t  TRACK_MAX_TRACKS            = 32U;
// Raw radar returns kept per frame BEFORE clustering. Was borrowed from
// TRACK_MAX_TRACKS, which capped a dense scene at its first 32 returns in
// TLV order and dropped the distant target first (RC-10). Detections out
// of the clusterer stay capped at TRACK_MAX_TRACKS.
static constexpr uint32_t  RADAR_MAX_POINTS_PER_FRAME  = 256U;
static constexpr float32_t TRACK_ASSOCIATION_DIST_M    = 3.0F;
static constexpr int32_t   TRACK_CONFIRM_HITS          = 3;
static constexpr int32_t   TRACK_MAX_MISSES            = 5;
// Consecutive misses before a CONFIRMED track stops publishing (COASTED);
// deletion happens at TRACK_MAX_MISSES.
static constexpr int32_t   TRACK_COAST_MISSES          = 2;
static constexpr float64_t kRadarDetectionSigmaM       = 0.15;
// P3.2 measurement models. Doppler sigma is an indoor-profile estimate
// (P4.1 derives it from the radar profile YAML); pixel sigma is detector
// box-centre jitter, to be refined against the P0 bag corpus.
static constexpr float64_t kRadarDopplerSigmaMps       = 0.2;
static constexpr float64_t kRadarMinRangeM             = 0.5;
static constexpr float64_t kCameraPixelSigmaPx         = 4.0;
static constexpr float64_t kCameraMinDepthM            = 0.1;

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

// Floors the centroid weight on near-stationary returns so they still contribute
// position but get outvoted by torso-speed doppler returns inside the cluster.
static constexpr float32_t kDopplerWeightFloor         = 0.1F;

// Human locomotion acceleration cap, used to reject arm-swing velocity spikes
// that leak into the IMM blended state between frames.
static constexpr float64_t kMaxPhysicalAcceleration    = 3.0;
// Track birth and gating (audit B2, D-9 engineering defaults). A track is
// born from ONE return: position known to the sensor sigma, velocity
// unknown. The velocity-jump gate is max(kMaxPhysicalAcceleration*dt,
// kGateSigmas*sigma_v); the association gate is kAssocBaseGateM +
// kGateSigmas*(sigma_pos + sigma_v*|dt|), capped at kAssocMaxGateM, where
// dt is the extrapolation from the state time to the return's stamp.
static constexpr float64_t kTrackInitVelSigmaMps       = 10.0;
static constexpr float64_t kGateSigmas                 = 3.0;
static constexpr float64_t kAssocBaseGateM             = 0.8;
static constexpr float64_t kAssocMaxGateM              = 5.0;
static constexpr float64_t kAssocMaxExtrapS            = 0.25;
// One hit per stamp; a gap longer than this restarts confirmation (RC-37).
static constexpr float64_t kTrackHitGapResetS          = 0.25;
// Tracks with no return for this long are dropped by the tracker node.
static constexpr float64_t kTrackReapAfterS            = 5.0;
// Below this range the line-of-sight direction is undefined.
static constexpr float64_t kRadialSpeedMinRangeM       = 1.0e-3;
// Camera-label join in the threat classifier (D-10): a fused detection
// set older than this is not applied, and a label joins a track only when
// the fused position is within this distance AND the bearing agrees.
static constexpr int64_t   kFusedLabelMaxAgeNs         = 250'000'000LL;
static constexpr float32_t kLabelJoinMaxDistM          = 1.0F;
static constexpr float32_t kLabelJoinMaxBearingDeg     = 15.0F;

// Tight gate for new tracks keeps clutter from initiating confirmed IDs;
// wider gate for confirmed tracks absorbs extended body returns.
static constexpr float64_t kTentativeGateChi2          = 9.0;
static constexpr float64_t kConfirmedGateChi2          = 16.0;

static constexpr float32_t THREAT_VELOCITY_SUSPECT_MPS   = 2.0F;
static constexpr float32_t THREAT_VELOCITY_THREAT_MPS    = 5.0F;
static constexpr float32_t THREAT_APPROACH_THRESHOLD_MPS = -0.5F;
static constexpr float32_t THREAT_MIN_CONFIDENCE         = 0.25F;

// WHY: cap on forecast position uncertainty — without a measurement update
// the open-loop covariance trace inflates across ticks, producing nonsense
// 50+ m radii for a target at 3-5 m range; 5 m is the overlay drawable ceiling.
static constexpr float32_t kMaxUncertaintyRadiusM      = 5.0F;

// WHY: max age of a cached mux entry before it is evicted. Predictors run at
// 20 Hz (50 ms), so 500 ms tolerates ~10 missed ticks without dropping live
// tracks, but purges stale IDs left behind when a track exits /tracks.
static constexpr float64_t kPredictionStaleSec         = 0.5;

static constexpr uint32_t  FUSION_MAX_DETECTIONS       = 128U;

static constexpr float32_t kOverlayLabelFontScale   = 1.2F;
static constexpr float32_t kOverlayTrackIdFontScale = 0.9F;
static constexpr int32_t   kOverlayLabelPadding     = 6;
static constexpr int32_t   kOverlayArcMaxWaypoints  = 8;
static constexpr int32_t   kOverlayArcDotRadius     = 4;
static constexpr int32_t   kOverlayArcThickness     = 2;

static constexpr float32_t kOverlayConfirmedFontScale  = 0.85F;
static constexpr float32_t kOverlayTentativeFontScale  = 0.45F;
static constexpr float32_t kOverlayTentativeAlpha      = 0.50F;

static constexpr float32_t kPpiFovDegrees               = 120.0F;
static constexpr float32_t kPpiFovHalfRad               = 1.0472F;
static constexpr float32_t kPpiMaxRangeM                = 15.0F;

static constexpr float32_t kSimRadarMaxRangeM          = 15.0F;
static constexpr float32_t kSimRadarNoiseSigmaM        = 0.10F;
static constexpr uint32_t  kSimRadarMaxTargets         = 8U;
static constexpr uint32_t  kSimRadarMaxPoints          = 64U;

// WHY: mirrors the cv::arrowedLine thickness used for the velocity vector in
// cuas_visualizer_node. Keeping them in lockstep means the forecast arc and
// the velocity arrow read at the same visual weight instead of the arc
// looking like a fainter, disconnected overlay.
static constexpr int32_t   kVelocityArrowThicknessPx = 3;
static constexpr int32_t   kArcLineThickness         = kVelocityArrowThicknessPx;

// WHY: dot floor at 10 px so nearest waypoint stays visible at 1920x1080;
// ceiling at 18 px so close-range dots do not dominate the track label.
static constexpr int32_t   kArcDotMinRadius = 10;
static constexpr int32_t   kArcDotMaxRadius = 18;

// WHY: furthest dot fades to 30 % so the arc reads as "fading away" without
// the far dot blending completely into background clutter.
static constexpr float32_t kArcDotMinAlpha = 0.30F;

// WHY: 5 steps keeps arc within frame at 2-5 m indoor range; longer forecasts
// project off-screen and rejoin later in the sequence, which reads as a
// rendering glitch at the overlay level.
static constexpr int32_t   kArcMaxWaypoints = 5;

// WHY: if a waypoint projects into this edge margin we stop drawing the arc.
// Continuing past a margin violation produces an arc that exits the frame and
// snaps back on a later waypoint.
static constexpr int32_t   kArcBoundsMarginPx = 50;

} // namespace cuas
