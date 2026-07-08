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

1. Node code logs via the `RCLCPP_*` macros; pure math classes format output through `std::ostringstream`.
2. Sanctioned `stderr` uses only: the per-process `main()` exception boundary (DEV-001 precaution — no logger is guaranteed alive there), the TensorRT `ILogger` sink, and the no-ROS `CameraDriver` teardown diagnostics.

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
2. `kMaxTrajectorySteps` (kinematic_predictor.hpp) maximum prediction horizon steps.
3. `FUSION_MAX_DETECTIONS` maximum detections per frame.
4. `FUSION_MAX_CLASSES` bound on COCO class cache.
5. `TIMESTAMP_BUFFER_SIZE` camera frame ring buffer depth.
6. `VIZ_MAX_CACHE_ENTRIES` visualizer pixel-bin cache depth.

New constants for future upgrades (Hungarian assignment, geofencing, health monitor, intent classifier) go here.

## Documented Deviations

Format per MISRA Compliance:2020 §4.2 — each record states the guideline(s),
the reason category (R1 code quality, R2 access to hardware/system interface,
R3 adopted-code integration, R4 non-compliant adopted code), the
circumstances, the risk assessment with precautions, and the locations.

**Sign-off**: J. Lopez (project owner), 2026-07-07. A new or widened
deviation requires re-approval; a shrunken one only a changelog entry.

### DEV-001: exceptions and RTTI not disabled (ROS 2 framework)

- **Guidelines**: JSF AV 208 (exceptions), JSF AV 178 (RTTI), MISRA 18.x family.
- **Reason**: R3 adopted code.
- **Circumstances**: rclcpp/rosidl require exceptions and RTTI; `-fno-exceptions` does not link. Owned code contains zero `throw`/`try`/`catch` and no `dynamic_cast`/`typeid` (audit-verified).
- **Risk & precautions**: a library throw inside a callback would otherwise reach `std::terminate` with no fault record. Precaution (enforced): every `main()` wraps init/spin in the single sanctioned catch-all, logging FATAL to stderr with a defined exit code, so the fault path is deterministic and visible to the health monitor. Throwing boundary calls are replaced with non-throwing equivalents (`std::from_chars`, `cuas::rosImageToBgr`).
- **Locations**: every node `main()` (tagged by the shared comment block).

### DEV-002: Eigen internal allocation

- **Guidelines**: JSF AV 206 (dynamic allocation), JPL P3.
- **Reason**: R3 adopted code.
- **Circumstances**: Eigen allocates for dynamic-size matrices. As of A3.1 the estimation/tracking/prediction hot path uses only fixed-size types (`Vector6d`/`Matrix6d`/`Vector7d`/`Matrix7d`/`Vector9d`/`Matrix9d`, measurement `Vector3d`/`Matrix3d`), which live on the stack.
- **Risk & precautions**: a future edit could silently reintroduce a heap-allocating dynamic temporary. Precaution (enforced, not aspirational): `tracking_lib` compiles with `EIGEN_NO_MALLOC` in Debug builds, so any Eigen heap allocation aborts at the allocation site. Dynamic `MatrixXd` remains only in node-boundary glue (predictor F/Q builders) and test fixtures.
- **Locations**: `src/estimation/*`, `src/tracking/*`, `src/prediction/kinematic_predictor.cpp`; enforcement in `CMakeLists.txt` (tracking_lib).

### DEV-003: OpenCV

- **Guidelines**: JSF AV 206, 208.
- **Reason**: R3 adopted code.
- **Circumstances**: OpenCV manages image buffers dynamically and reports errors via exceptions.
- **Risk & precautions**: confined to the camera driver, inference preprocess, and visualization; per-frame Mats are preallocated members (A3.4, R12e) so steady-state allocation is zero; `cv::Exception` escape paths are covered by the DEV-001 process boundary, and hot-path callbacks validate geometry/encoding before invoking cv (clutter/visualizer guards).
- **Locations**: `drivers/camera_driver.cpp`, `inference/trt_detector.cpp`, `visualization/*`, `fusion/timestamp_associator.cpp`.

### DEV-004: TensorRT

- **Guidelines**: JSF AV 206, 208.
- **Reason**: R3 adopted code.
- **Circumstances**: TensorRT owns GPU memory and imposes the `nvinfer1::ILogger` callback signature.
- **Risk & precautions**: encapsulated in `TrtDetector`; the engine loads once at init; engine tensor count and shapes are validated against the compile-time buffers (A2.8) so a mismatched engine fails init instead of overflowing device buffers; the logger latches kERROR+ to stderr (a silent sink hid every load failure). Buffers are RAII with stateless functor deleters.
- **Locations**: `inference/trt_detector.{hpp,cpp}` (one-shot init allocation tagged DEV-004).

### DEV-005: std::vector/std::string at the ROS boundary

- **Guidelines**: JSF AV 206.
- **Reason**: R3 adopted code (rosidl-generated types).
- **Circumstances**: ROS 2 messages require `std::vector`/`std::string`.
- **Risk & precautions**: the internal pipeline uses `FixedVector`/`FixedMap` and integer ids; strings and vectors appear only in node wrapper code at single chokepoints (`parseClassId`/`classIdToLabel`, `trackStateToString`/`FromString`, `track_state_to_id`). Since A3.8, no hot-path value type carries `std::string`.
- **Locations**: node wrapper files; chokepoint helpers in `common/types.hpp`.

### DEV-006: rclcpp/rosidl allocation in the publish/subscribe machinery

- **Guidelines**: MISRA 21.6.1 (Advisory), JPL P3, DO-178C D2.
- **Reason**: R3 adopted code.
- **Circumstances**: the executor and DDS allocate internally per publish/take; owned code minimizes traffic (FixedVector state, `reserve()`, ConstSharedPtr snapshots, member-message reuse) but cannot remove the middleware residue.
- **Risk & precautions**: allocator jitter inside callbacks, bounded in practice by small message sizes. Full cure (TLSF executor allocator, loaned messages, bounded IDL) is a tracked roadmap item, not a blocker.
- **Locations**: every `publish()`/subscription in node files.

### DEV-007: const_cast in the zero-copy image wrap

- **Guidelines**: MISRA 8.2.3 (Required).
- **Reason**: R3 (OpenCV external-buffer `cv::Mat` ctor takes non-const `void*`); R1 also applies — the compliant alternative is a ~6 MB/frame copy.
- **Risk & precautions**: the returned Mat is treated as read-only by contract (documented at the helper); consumers verified read-only.
- **Locations**: `common/ros_image_adapter.hpp:25,32`.

### DEV-008: reinterpret_cast to sockaddr*

- **Guidelines**: MISRA 8.2.5 (Required).
- **Reason**: R2 access to system interface — the BSD sockets ABI requires it; no conforming alternative exists.
- **Risk & precautions**: confined to the single `sendto` call site.
- **Locations**: `output/cot_publisher_node.cpp` (sendUdp).

### DEV-009: type-erased callbacks (std::function) in the rclcpp API

- **Guidelines**: JPL P9 (no function pointers).
- **Reason**: R3 — subscriptions/timers are `std::function` by API.
- **Risk & precautions**: callbacks are bound member functions or explicit-capture lambdas (analyzable capture set); no owned function-pointer tables exist.
- **Locations**: every `create_subscription`/`create_wall_timer` call.

### DEV-010: unsynchronized shared state under the single-threaded executor

- **Guidelines**: DO-178C D1 / concurrency.
- **Reason**: R1.
- **Circumstances**: `latest_*` members are written in subscription callbacks and read in timer callbacks without locks — safe iff the node spins on a single-threaded executor, which every launch file uses. Nodes with their own capture/parse threads (camera, radar, fusion, overlay) do lock.
- **Risk & precautions**: this record is void if a MultiThreadedExecutor is ever adopted; mutexes then become mandatory. Adding them preemptively is the recommended eventual state.
- **Locations**: geofence, reachability, kinematic/occlusion predictor, imm_tracker, intent classifier nodes.

### DEV-011: unbounded strings in cuas_msgs (sequences bounded in 0.9.0-alpha)

- **Guidelines**: DO-178C D1, JPL P2 at the interface.
- **Reason**: R1 (deferred interface change).
- **Circumstances**: originally every .msg sequence and string was unbounded. OVERHAUL P3.1 (2026-07-07, 0.9.0-alpha) bounded all sequences to their producer-side capacity constants (32/64/128/256); the CDR wire form is unchanged, so old bags replay. The deviation now covers only the **string** fields (`class_label`, `track_state`, `threat_level`, `escalation_state`, `zone_id`), which remain unbounded: they cross the wire only via the `common/types.hpp` chokepoint helpers, and the `*_id` integer fields already present are the migration path that retires them.
- **Risk & precautions**: middleware-side string allocation per publish (subsumed by DEV-006); sequence bounds are enforced by construction (fixed-capacity producer pools).
- **Locations**: string fields in `msgs/cuas_msgs/msg/*.msg`.

### DEV-012: display/test-utility path allocations

- **Guidelines**: JPL P3.
- **Reason**: R1 code quality.
- **Circumstances**: visualizer/overlay/capture per-frame clones, ostringstream label text, and marker vector growth live on operator-display paths that tolerate allocator jitter; the tracking hot path is unaffected.
- **Risk & precautions**: display latency jitter only; excluded from this record is anything feeding the CoT or track outputs.
- **Locations**: `visualization/*`, capture node.

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
10. GPS georeferencing adds WGS84 fields to `Track` and `ThreatReport` (the CoT lat/lon stub in docs/ICD.md §4 is the consumer waiting on it).

All fixed-size bounds live in `constants.hpp` so future limits can be tuned in one place.
