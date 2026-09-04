# PROJECT_MAP — cuas_fusion Counter-UAS Ground Station

Jetson Orin Nano · ROS 2 Humble · C++17 · TI IWR6843ISK radar · Arducam AR0234 camera ·
TensorRT YOLOv8 · ~11,150 LOC C++ across 2 packages, 21 executables.

## 1. End-to-end dataflow

```
TI IWR6843ISK ──UART──> radar_parser_node ──/radar/detections──┐
  (or sim_radar_node / radar_sim for bench work)               │
                                                               ▼
AR0234 ──V4L2──> camera_node ──/camera/image_raw──> color_correct_node
                                        │           /camera/image_corrected
                                        ▼                      │
                              inference_node (TensorRT YOLOv8) │
                                        │ /detections          │
                                        ▼                      ▼
                              fusion_node  <── timestamp_associator (50 ms gate)
                                        │ /fused_detections
                                        ▼
                     tracker: Hungarian assignment + IMM (CV/CA/CT Kalman bank)
                     (track_manager_node, imm_tracker_node)
                                        │ /tracks
              ┌─────────────┬───────────┼───────────────┬──────────────┐
              ▼             ▼           ▼               ▼              ▼
      threat_classifier  intent    geofence_node   prediction     reachability
              │        classifier      │        (kinematic /         │
              │             │          │         occlusion /         │
              │             │          │         mux)                │
              └─────────────┴──────────┴───────┬───────┴──────────────┘
                                               ▼
                overlay/visualizer nodes (RViz, image overlay) · cot_publisher (CoT XML out)
                          health_monitor_node watches everything · clutter_map feeds gating
```

Support: georef (ENU↔WGS84), clock (monotonic), fixed_containers (no-alloc), capture_node (frame dump).

## 2. Packages

| Package | Path | Role |
|---|---|---|
| `cuas_fusion` | `src/cuas_fusion/` | All application code (ament_cmake, 21 executables, ~11 static libs) |
| `cuas_msgs` | `msgs/cuas_msgs/` | 19 .msg interface definitions (rosidl). No .srv/.action |

`src/cuas_msgs/` is an empty leftover directory (candidate for removal).
`graphify-out/` is generated analysis output — excluded from audit.

## 3. File inventory

**[HOT]** = executes per frame (30 Hz camera), per radar scan, or per tracker cycle → JPL P3/JSF AV 206
"no heap after init" and bounded-execution rules apply in full.
**[init]** = startup/config/glue — allocation and flexibility acceptable.
**[PROT]** = PROTECTED camera configuration — audit-only, never edited (see §5).

### Drivers (`src/cuas_fusion/src/drivers/`)
| File | LOC | Tag | Role |
|---|---|---|---|
| camera_driver.cpp / .hpp | 175/34 | HOT PROT | V4L2 capture: S_FMT (10-bit Bayer BA10), S_PARM, mmap ring, grabFrame, black-level, WB, Bayer→BGR |
| camera_node.cpp | 110 | HOT PROT | Capture thread → /camera/image_raw (per-frame msg.data.resize + memcpy) |
| radar_parser_node.cpp | 533 | HOT | TI mmWave TLV UART parser → RadarFrame (largest driver; byte-level parsing) |
| radar_driver.cpp/.hpp | 7/12 | init | Stub |
| sim_radar.cpp / sim_radar_node.cpp | 194/158 | init/sim | Simulated radar source for bench testing |

### Inference (`src/inference/`)
| trt_detector.cpp/.hpp | 331/99 | HOT | TensorRT engine load (init) + per-frame enqueue/decode; smart-ptr TRT deleters |
| inference_node.cpp | 104 | HOT | Image sub → detector → /detections |

### Fusion (`src/fusion/`)
| fusion_engine.cpp/.hpp | 165/42 | HOT | Radar↔camera detection fusion core |
| fusion_node.cpp | 214 | HOT | Message plumbing per cycle |
| timestamp_associator.cpp/.hpp | 50/33 | HOT | 50 ms time-gate association (2 baseline test FAILURES — gate boundary bug) |

### Tracking (`src/tracking/`) & Estimation (`src/estimation/`)
| hungarian_solver.cpp/.hpp | 139/46 | HOT | Hungarian assignment (pure logic — fully unit-testable) |
| track_manager.cpp/.hpp + node | 212/56/98 | HOT | Track lifecycle, gating, confidence decay |
| imm_tracker.cpp/.hpp + node | 185/62/210 | HOT | IMM multi-model tracker |
| imm_filter.cpp/.hpp | 206/45 | HOT | IMM mixing/interaction (pure logic — fully unit-testable) |
| kalman_cv/ca/ct.cpp | 86/122/180 | HOT | Constant-velocity / -acceleration / -turn filters |
| track.cpp/.hpp | 7/28 | HOT | Track struct |

### Downstream engines
| threat_classifier.cpp/.hpp + node | 167/78/236 | HOT | Threat scoring (1 baseline test FAILURE — radar-only quality score) |
| intent_classifier.cpp + node | 86/111 | HOT | Intent classification |
| geofence_engine.cpp + node | 189/295 | HOT | Zone checks (config from geofence_zones.yaml) |
| reachability_engine.cpp + node | 87/191 | HOT | Intercept/reachability |
| kinematic_predictor.cpp + node | 157/211 | HOT | Trajectory prediction |
| occlusion_predictor.cpp + node | 61/221 | HOT | Occlusion-gap prediction |
| prediction_mux_node.cpp | 200 | HOT | Predictor selection |
| clutter_map.cpp + node | 143/180 | HOT | False-alarm/clutter map |
| color_correct_engine.cpp/.hpp + node | 66/…/77 | HOT PROT | ISP-style WB/color correction between camera and inference |

### Output / visualization / support
| overlay_engine.cpp/.hpp | 317/67 | HOT | Overlay rendering |
| cuas_visualizer_node.cpp | 941 | HOT | Largest file; RViz markers etc. |
| cuas_overlay_node.cpp | 190 | HOT | Overlay publisher |
| capture_node.cpp | 123 | semi | Frame dump to disk |
| cot_publisher_node.cpp | 152 | semi | Cursor-on-Target XML out |
| health_monitor.cpp + node | 105/156 | semi | Heartbeat/health aggregation |
| georef/wgs84_transform.cpp + georef_node | — | HOT | ENU↔WGS84 |
| common/: constants.hpp (158, lines 13–50 PROT), fixed_containers.hpp (149 — FixedVector/FixedMap, deliberate no-alloc design), types, fixed_types, clock, intent_ids, track_state_ids, ros_image_adapter | init | Shared foundations |

### Launch (`launch/`): sim, full, full_system, occlusion, radar_only, kinematic, replay (.py)
### Config (`config/`): system_params.yaml, predictor_params.yaml, geofence_zones.yaml, camera_calibration*.yaml [PROT]
### Messages (`msgs/cuas_msgs/msg/`): RadarDetection/Frame, FusedDetection(+Array), Track(+Array), PredictedTrack, TrajectoryWaypoints, ThreatReport/IntentReport/InterceptReport/GeofenceEvent(+Arrays), SystemHealth(+Array), ClutterStatus

## 4. Hot path vs init — where the strict rules apply

Hot path (rules P2/P3/M1/D1/D2 in STANDARDS_CHECKLIST.md apply fully): every subscription callback and
the camera capture thread. Init (constructors, `main`, engine load, YAML parse): allocation and
exceptions-from-libraries are tolerable and covered by deviation records.

Known hot-path allocation pressure to audit: `std::vector` growth in fusion_engine, hungarian_solver,
track_manager, imm_tracker_node, radar_parser_node; per-frame `resize`+`memcpy` in camera_node [PROT —
report only].

## 5. PROTECTED camera files (never edited — HARD RULE)

| Path | Why |
|---|---|
| src/cuas_fusion/src/drivers/camera_driver.cpp + include/cuas_fusion/drivers/camera_driver.hpp | V4L2 fmt/rate/mmap and Bayer pipeline — fragile, bricks camera if wrong |
| src/cuas_fusion/src/drivers/camera_node.cpp | Device path, capture loop, retry behavior |
| include/cuas_fusion/common/constants.hpp lines 13–50 | Device path, 1920×1080@30, black level 2752, WB gains, intrinsics |
| config/camera_calibration.yaml, config/camera_calibration_result.yaml | Extrinsics/intrinsics + distortion |
| src/color_correct_engine.cpp, include/…/color_correct_engine.hpp, src/cuas_color_correct_node.cpp | ISP-style correction feeding inference |
| scripts/calibrate_camera.py, test/camera_calibration/calibrate_camera.py | Calibration procedure |
| launch/sim.launch.py lines 66–67; launch/full.launch.py lines 40–42, 90–91, 119, 184, 192 | Camera launch params/remaps |

Change protocol: backup to ./camera_backups/<file>.<ts>.bak → side-by-side proposal file (never wired
into build) → ROLLBACK.md → explicit owner approval BEFORE any of it.

## 6. Build & test infrastructure

- **Build**: colcon/ament_cmake; C++17 required, extensions off. Per-target `CUAS_SAFETY_FLAGS`
  (-Wall -Wextra -Wpedantic -Wcast-align -Wdouble-promotion -Wformat=2 -Wnull-dereference
  -Wimplicit-fallthrough -Wswitch-enum); `CUAS_STRICT_FLAGS` (+-Wsign-conversion -Wconversion) on 5
  non-ROS libs only. **No -Werror. No CI. No .clang-tidy/.clang-format/cppcheck config.**
  CMakeLists documents JSF-AV/MISRA intent + deviations DEV-001..005.
- **Machine-specific paths**: OpenCV cmake dir and CUDA 12.6 include are hardcoded to this Jetson.
- **Tests**: 8 gtest suites registered (timestamp_associator, threat_classifier, geofence, reachability,
  health_monitor, intent, clutter_map, color_correct). **4 orphaned test files NOT in CMake**:
  test/unit/test_kalman_cv.cpp, test_wgs84_transform.cpp, test_imm_filter.cpp,
  test/integration/test_fusion_pipeline.cpp — currently dead.
- **Baseline (clean rebuild, this Jetson, branch misra-audit-fixes @ f39d7d1)**:
  - Build: OK, **90 warnings** (88 = Eigen PacketMath.h via non-SYSTEM include; 2 in radar_parser_node.cpp)
  - Tests: **3 failing cases in 2 suites** (colcon reports "5 failures" by double-counting the two
    failing suite binaries at CTest level): TimestampAssociator.SingleFrameRespects50msLimit and
    .RejectsFramesBeyond50ms (50 ms gate accepts out-of-window frames — association-correctness bug),
    ThreatClassifierTest.QualityScoreRadarOnly (returns 0.5, test expects 0.3)
  - cppcheck 2.7: 68 native findings (2017 additional from MISRA-C addon — **not valid C++ evidence**,
    see STANDARDS_CHECKLIST.md §4)

## 7. Verifiable locally vs NEEDS-HARDWARE

| Verifiable HERE (this Jetson, no sensors) | NEEDS HARDWARE (live sensors) |
|---|---|
| Full colcon build (this is the target machine) | Camera pipeline end-to-end (V4L2 → Bayer → inference) |
| All gtest suites; new IMM/Hungarian/Kalman tests (pure math) | Radar UART parsing against live IWR6843 TLV stream |
| cppcheck / clang-tidy static analysis | TensorRT engine inference on live frames (engine files exist; a synthetic-input smoke test is possible but real latency/accuracy needs frames) |
| Simulation launch (sim_radar) partially — nodes start, topics flow | Multi-node timing/latency budget (docs/latency_budget.md) |
| Logic-level behavior of every engine (fusion, geofence, threat…) | CoT output consumed by real ATAK endpoint |
