# ProjectHailMary Coding Standard

## Applicable Standards

1. JSF AV C++ (Joint Strike Fighter Air Vehicle C++ Coding Standards, Lockheed Martin Doc 2RDU00001 Rev C).
2. MISRA C++:2023 (Guidelines for the use of C++17 in critical systems, October 2023, targeting ISO/IEC 14882:2017).
3. DO-178C (awareness level; this is a portfolio project, not certified avionics software).

## Compliance Approach

Application code under `src/cuas_fusion/` follows JSF AV C++ and MISRA C++:2023. Third-party frameworks are treated as adopted code per MISRA Compliance:2020 Section 6.5, with the deviations recorded below.

## Rules Enforced

### Memory

1. No dynamic heap allocation after initialization.
2. Pre-allocated containers `FixedVector` and `FixedMap` live in `include/cuas_fusion/common/fixed_containers.hpp`.
3. `std::vector` appears only at the ROS message boundary (DEV-005).

### Types

1. Arithmetic types use explicit-width aliases (`float32_t`, `uint32_t`, and so on) defined in `include/cuas_fusion/common/fixed_types.hpp`.
2. Application code does not use bare `int` or `double`.

### Errors

1. Application code contains no `try`, `catch`, or `throw`.
2. Errors are reported through return codes.
3. `-fno-exceptions` is not applied because `rclcpp` propagates `std::exception` internally (DEV-001).
4. `std::stoi` is replaced with `std::from_chars`. `cv_bridge::toCvShare` is replaced with `cuas::rosImageToBgr`.

### RTTI

1. Application code does not use `dynamic_cast` or `typeid`.
2. `-fno-rtti` is not applied because `rclcpp` requires RTTI internally (DEV-001).

### State

1. State machines use `enum class`, never strings.
2. `TrackState`, `ThreatLevel`, and `EscalationState` live in `types.hpp` and `threat_classifier.hpp`.
3. String conversion happens only at the ROS publish boundary via `trackStateToString` / `trackStateFromString`.

### I/O

1. Application code contains no `fprintf`, `printf`, `stdout`, `stderr`, or `std::snprintf`.
2. Node wrappers log via the `RCLCPP_*` macros.
3. Pure math classes format output through `std::ostringstream`.

### Control Flow

1. All control bodies are braced, including single-statement branches.
2. All `switch` statements contain a `default` clause.
3. Fallthrough is explicit.
4. Conditionals do not contain assignments.
5. Cyclomatic complexity target is 20 per function.

### Architecture

1. Math classes have no ROS dependency and can be tested in isolation: `KalmanCV`, `KalmanCA`, `KalmanCT`, `ImmFilter`, `KinematicPredictor`, `OcclusionPredictor`, `TrackManager`, `ThreatClassifier`, `FusionEngine`.
2. Nodes are thin wrappers: subscribe, call the math class, publish.
3. Linear algebra uses `Eigen3`.

### Capacity Constants

All fixed bounds live in `include/cuas_fusion/common/constants.hpp`:

1. `TRACK_MAX_TRACKS` maximum simultaneous active tracks.
2. `PREDICTION_MAX_STEPS` maximum prediction horizon steps.
3. `FUSION_MAX_DETECTIONS` maximum detections per frame.
4. `FUSION_MAX_CLASSES` bound on COCO class cache.
5. `TIMESTAMP_BUFFER_SIZE` camera frame ring buffer depth.
6. `VIZ_MAX_CACHE_ENTRIES` visualizer pixel-bin cache depth.

New constants for future upgrades (Hungarian assignment, geofencing, health monitor, intent classifier) go here.

## Documented Deviations

### DEV-001: ROS 2 Framework

Rules deviated: JSF AV 206 (dynamic alloc), 208 (exceptions), 178 (RTTI).

Rationale: ROS 2 Humble is the middleware and uses exceptions, dynamic allocation, and RTTI internally.

Mitigation: application logic lives in pure C++ classes. ROS nodes are thin adapters. Exception-throwing library calls at the boundary (`cv_bridge::toCvShare`, `std::stoi`) are replaced with non-throwing equivalents.

### DEV-002: Eigen3

Rules deviated: JSF AV 206.

Rationale: Eigen may allocate heap memory for large matrices.

Mitigation: all matrices use fixed dimensions (6x6, 9x9, 7x7). `EIGEN_NO_MALLOC` can be defined in debug builds.

### DEV-003: OpenCV

Rules deviated: JSF AV 206, 208.

Rationale: OpenCV manages image buffers with dynamic allocation and uses exceptions.

Mitigation: OpenCV is confined to the camera driver, inference node, and visualization node. Failures surface at the node boundary through return codes.

### DEV-004: TensorRT

Rules deviated: JSF AV 206, 208.

Rationale: TensorRT manages GPU memory and inference buffers. The `nvinfer1::ILogger::log` callback signature is imposed by the framework.

Mitigation: TensorRT is encapsulated in `TrtDetector` and `inference_node`. The engine loads once at init. The logger callback is a silent sink; severity surfaces through the `bool init()` and `bool infer()` return codes.

### DEV-005: std::vector and std::string at ROS Boundary

Rules deviated: JSF AV 206.

Rationale: ROS 2 message types require `std::vector` for variable-length arrays and `std::string` for text.

Mitigation: internal pipeline uses `FixedVector` and `FixedMap`. Conversion to `std::vector` happens only in node wrapper code. `FusedDetection::class_label` keeps `std::string` for ROS message compatibility.

## Forward Compatibility

The standard accommodates these planned upgrades without structural change.

1. Hungarian assignment receives the same `FusedDetection*` and `uint32_t` count.
2. Mahalanobis gating reads `TrackEntry` covariance already present.
3. Confidence decay uses the existing `float32_t confidence_` on `Track`.
4. Maneuver detection reads `ImmFilter::getModelWeights()`.
5. An intent classifier node consumes `/tracks` and reuses the enum helpers.
6. Geofencing uses `FixedVector` for zone definitions.
7. A reachability node consumes `FixedVector<Track>`.
8. Engagement logic reuses `EscalationState`.
9. A health monitor emits `RCLCPP_WARN` with per-sensor state.
10. GPS georeferencing adds WGS84 fields to `Track` and `ThreatReport`.

All fixed-size bounds live in `constants.hpp` so future limits can be tuned in one place.
