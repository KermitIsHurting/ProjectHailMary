# ProjectHailMary — System Architecture

## 1. System Overview

ProjectHailMary is a counter-unmanned-aerial-system (C-UAS) sensor fusion ground station that fuses millimetre-wave radar with electro-optical imagery to detect, track, classify, and forecast small uncooperative aerial targets in a short-range engagement envelope (nominally inside fifteen metres). The system ingests raw radar TLV frames and Bayer camera frames, projects them into a common base-link frame, and produces stable confirmed tracks with adaptive trajectory predictions, behavioural intent, geofence violations, time-to-intercept estimates, and a Cursor-on-Target multicast feed compatible with ATAK clients. It also produces an annotated camera image and an RViz2 marker stream for an operator console.

The reference deployment runs entirely on an NVIDIA Jetson Orin Nano Super, with a Texas Instruments IWR6843ISK 60 GHz mmWave radar evaluation board on `/dev/radar_data` and `/dev/radar_config` and an Arducam AR0234 global-shutter camera on `/dev/video0`. Inference uses a YOLOv8s INT8 TensorRT engine that runs at approximately 35 Hz on the Jetson's iGPU; everything else is CPU-bound. There is no off-board compute and no network dependency at runtime, so the same binary set boots into a working pipeline whenever the Jetson powers up. A simulation launch substitutes a software radar source for the IWR6843ISK so the entire pipeline can be exercised on a development workstation without hardware.

The codebase is organised around a strict separation between pure C++ math classes and ROS 2 node wrappers. All filtering, fusion, classification, prediction, geofencing, reachability, intent, and health logic is implemented as plain C++ with no ROS dependency, sized against fixed-bound containers (`FixedVector`, `FixedMap`) drawn from `include/cuas_fusion/common/fixed_containers.hpp`. The ROS layer is a thin adapter: nodes subscribe, marshal messages into the math-class signature, call a pure function, and republish. This decomposition is enforced by the JSF AV C++ (Joint Strike Fighter Air Vehicle) coding standard with MISRA C++:2023 cross-rules; documented deviations live in `docs/CODING_STANDARD.md` (DEV-001 through DEV-005). Capacity bounds (`TRACK_MAX_TRACKS = 32`, `PREDICTION_MAX_STEPS = 128`, `FUSION_MAX_DETECTIONS = 128`, etc.) are compile-time constants in `include/cuas_fusion/common/constants.hpp` so the reachable state space is statically auditable.

## 2. Pipeline Overview

The full hardware pipeline as launched by `src/cuas_fusion/launch/full.launch.py`:

```
                          +---------------------+
                          |    IWR6843ISK       |
                          |  (mmWave radar)     |
                          +----------+----------+
                                     |
                                     v
                       [radar_parser_node]
                                     |
                              /radar/detections
                                     |
                                     v
                         [clutter_map_node]
                                     |
                              /radar/filtered
                                     |                  +-------------------+
                                     v                  |  /threat/reports  |
                          [imm_tracker_node] <----------+   (horizon feedback)
                                     |                  +-------------------+
                                     |
                                  /tracks
                  +------+------+----+----+-----+------+------+------+------+
                  |      |      |        |     |      |      |      |      |
                  v      v      v        v     v      v      v      v      v
        [fusion_node] [kinematic_   [occlusion_  [threat_       [intent_  [geofence_
              ^        predictor_    predictor_   classifier_    classifier_ node]
              |        node]         node]        node]          node]
              |          |              |             |              |        |
       /inference/        \            /              |              |        |
        detections         v          v               |              |        |
              ^      /predicted_tracks/{kinematic,    |              |        |
              |     occlusion}, /trajectory_waypoints/|              |        |
              |        {kinematic,occlusion}          |              |        |
              |             |                         |              |        |
       [inference_node]     v                         |              |        |
              ^    [prediction_mux_node]              |              |        v
              |             |                         |              |  /geofence/
              |        /predicted_tracks              |              |   violations
              |        /trajectory_waypoints          |              v
              |             |                         |        /intent/
              |             |                         v         reports
       /camera/image_raw    |                  /threat/reports
              ^             |                         |
              |             |                         +--------------+--------+
       [camera_node]        |                                        |        |
              |             |                                        v        v
              v             |                              [cot_publisher_  [reachability_
       (optional)           |                                node]           node]
       [cuas_color_correct_node]                              |                |
              |             |                                 v                v
       /camera/image_corrected                          UDP 239.2.3.1:6969  /reachability/
              |             |                                                 warnings
              v             v
       [inference_node]  [cuas_visualizer_node] <---- /tracks, /predicted_tracks,
                                       |              /trajectory_waypoints,
                                       |              /threat/reports,
                                       |              /fusion/detections,
                                       |              /camera/image_raw
                                       |
                            /visualization/track_markers
                            /visualization/trajectory_markers
                            /visualization/uncertainty_markers
                            /visualization/track_labels
                            /camera/annotated
                                       |
                                       v
                          [cuas_overlay_node] <--- /tracks, /threat/reports,
                                       |          /trajectory_waypoints
                                       |
                            /camera/annotated_enhanced --> [rviz2]

       [health_monitor_node] <--- /radar/detections, /camera/image_raw,
                       |          /tracks, /threat/reports, /predicted_tracks
                       v
                /health/status
```

The arrows represent ROS 2 topic connections. Every transformation crosses exactly one node boundary and is described in section 3.

## 3. Node Descriptions

### 3.1 radar_parser_node

- **Source**: `src/cuas_fusion/src/drivers/radar_parser_node.cpp`
- **Description**: Reads the IWR6843ISK TLV stream from `/dev/radar_data`, decodes detected-object and side-info TLV blocks, applies DBSCAN clustering with doppler-weighted centroids, and republishes the cluster centroids as a `sensor_msgs/PointCloud2`. This node is the only component that touches the radar serial port.
- **Subscribes to**: none (reads the serial device directly).
- **Publishes**:
  - `/radar/detections` (`sensor_msgs/PointCloud2`) at approximately 16 Hz (driven by the radar profile uploaded by `send_radar_config`).
- **Dependencies**: the `send_radar_config` `ExecuteProcess` action must complete first so the radar is in streaming mode; udev symlinks `/dev/radar_data` and `/dev/radar_config` must exist.

### 3.2 clutter_map_node

- **Source**: `src/cuas_fusion/src/clutter_map_node.cpp`
- **Description**: Maintains a static occupancy grid learned from the first N frames of radar returns and filters subsequent returns against the learned map to reject stationary clutter (walls, furniture, fixed structure). During the learning window it can pass detections through unchanged; after learning it removes any return that falls in a high-occupancy cell.
- **Subscribes to**:
  - `/radar/detections` (`sensor_msgs/PointCloud2`).
- **Publishes**:
  - `/radar/filtered` (`sensor_msgs/PointCloud2`) at the radar input rate (~16 Hz).
  - `/clutter/status` (`cuas_msgs/ClutterStatus`) at 1 Hz (1000 ms timer).
- **Dependencies**: `radar_parser_node`.

### 3.3 camera_node

- **Source**: `src/cuas_fusion/src/drivers/camera_node.cpp`, `src/cuas_fusion/src/drivers/camera_driver.cpp`
- **Description**: Opens the AR0234 sensor at `/dev/video0` through the V4L2 backend (no GStreamer dependency at runtime) and publishes raw `bgr8` frames at 1920×1080. Capture runs on a dedicated thread; black-level subtraction, white-balance gains, and tone scale are configured via constants.
- **Subscribes to**: none.
- **Publishes**:
  - `/camera/image_raw` (`sensor_msgs/Image`) at `CAMERA_FPS` = 30 Hz.
- **Dependencies**: `/dev/video0` exists with V4L2 driver loaded; AR0234 calibration constants in `constants.hpp`.

### 3.4 cuas_color_correct_node

- **Source**: `src/cuas_fusion/src/cuas_color_correct_node.cpp`
- **Description**: Optional pre-processing node enabled by the `color_correct:=true` launch argument. It applies per-channel white-balance gain (B=2.987, G=1.000, R=0.861) tuned for the AR0234's missing IR-cut filter and publishes a corrected image stream that downstream consumers (inference, visualizer, overlay) can be remapped to.
- **Subscribes to**:
  - `/camera/image_raw` (`sensor_msgs/Image`).
- **Publishes**:
  - `/camera/image_corrected` (`sensor_msgs/Image`) at the camera input rate (~30 Hz).
- **Dependencies**: `camera_node`.

### 3.5 inference_node

- **Source**: `src/cuas_fusion/src/inference/inference_node.cpp`
- **Description**: Loads a pre-built YOLOv8s INT8 TensorRT engine from `models/yolov8s_int8.engine` and runs detection on each incoming camera frame. Output is the standard `vision_msgs/Detection2DArray` with COCO class IDs in `hypothesis.class_id` and the confidence in `hypothesis.score`.
- **Subscribes to**:
  - `/camera/image_raw` (`sensor_msgs/Image`) — when `color_correct` is false. When `color_correct` is true the launch file remaps this to `/camera/image_corrected`.
- **Publishes**:
  - `/inference/detections` (`vision_msgs/Detection2DArray`) at approximately 35 Hz on a Jetson Orin Nano Super with INT8 weights.
- **Dependencies**: `camera_node` (and `cuas_color_correct_node` when color correction is enabled); TensorRT runtime; the engine file at `models/yolov8s_int8.engine`.

### 3.6 fusion_node

- **Source**: `src/cuas_fusion/src/fusion/fusion_node.cpp`
- **Description**: Wraps `FusionEngine` to project radar tracks into the camera image plane using the static extrinsic offsets `(x=-0.0075, y=0.017, z=-0.079)` and the AR0234 intrinsics from `constants.hpp` (`CAMERA_FX=1862.7`, `CAMERA_FY=1877.7`, `CAMERA_CX=1032.8`, `CAMERA_CY=426.8`), then associates the projected radar centroids with YOLO bounding boxes by IoU. It also broadcasts the static `radar_frame → camera_frame` transform on tf2 at startup.
- **Subscribes to**:
  - `/tracks` (`cuas_msgs/TrackArray`).
  - `/inference/detections` (`vision_msgs/Detection2DArray`).
- **Publishes**:
  - `/fusion/detections` (`cuas_msgs/FusedDetectionArray`) gated by the `/tracks` callback (~20 Hz).
- **Dependencies**: `imm_tracker_node`, `inference_node`.

### 3.7 imm_tracker_node

- **Source**: `src/cuas_fusion/src/tracking/imm_tracker_node.cpp`
- **Description**: The narrow waist of the system. Reads the filtered radar point cloud, performs nearest-neighbour gating against the active track set with a fixed association gate, and updates per-track Interacting Multiple Model filters (constant velocity + constant acceleration + coordinated turn). On a 50 ms timer it predicts each track forward, prunes tracks idle for more than five seconds, and publishes the active set. It also enriches each `Track` with the adaptive `prediction_horizon_s` looked up from the most recent `ThreatReportArray`, so predictors do not need their own threat-level subscription.
- **Subscribes to**:
  - `/radar/detections` (`sensor_msgs/PointCloud2`) — the launch file remaps this to `/radar/filtered`, so in practice it consumes the clutter-filtered cloud.
  - `/threat/reports` (`cuas_msgs/ThreatReportArray`) — used only to retrieve the per-track adaptive prediction horizon.
- **Publishes**:
  - `/tracks` (`cuas_msgs/TrackArray`) at 20 Hz (50 ms wall timer).
- **Dependencies**: `clutter_map_node`; `threat_classifier_node` should also be running for the horizon feedback to update (the tracker degrades gracefully to a 5 s default horizon if no threat reports arrive).

### 3.8 track_manager_node

- **Source**: `src/cuas_fusion/src/tracking/track_manager_node.cpp` (executable name `tracker_node` in `CMakeLists.txt`).
- **Description**: An alternative track manager that consumes already-fused detections rather than raw radar. It runs the simpler `TrackManager` association/lifecycle logic and publishes a confirmed-only track set. Not enabled in `full.launch.py`; retained as a debug aid and as the consumer used by some unit tests.
- **Subscribes to**:
  - `/fusion/detections` (`cuas_msgs/FusedDetectionArray`).
- **Publishes**:
  - `/tracks/confirmed` (`cuas_msgs/TrackArray`) at the input rate (~20 Hz).
- **Dependencies**: `fusion_node`. Inactive in the standard hardware deployment.

### 3.9 kinematic_predictor_node

- **Source**: `src/cuas_fusion/src/prediction/kinematic_predictor_node.cpp`
- **Description**: Projects each track forward over the adaptive horizon using the IMM-blended state and a constant-velocity/constant-acceleration model. The horizon used per track is the `prediction_horizon_s` already attached to the Track message by the IMM tracker (3 s for benign behaviour, scaling up to 10 s for high-threat). Step size is fixed by `predictor_params.yaml`.
- **Subscribes to**:
  - `/tracks` (`cuas_msgs/TrackArray`).
- **Publishes**:
  - `/predicted_tracks/kinematic` (`cuas_msgs/PredictedTrack`) at `publish_rate_hz` = 20 Hz from `predictor_params.yaml`.
  - `/trajectory_waypoints/kinematic` (`cuas_msgs/TrajectoryWaypoints`) at the same rate.
- **Dependencies**: `imm_tracker_node`.

### 3.10 occlusion_predictor_node

- **Source**: `src/cuas_fusion/src/prediction/occlusion_predictor_node.cpp`
- **Description**: Same input contract as the kinematic predictor but extrapolates a track's last-known state through gaps when `track_state` is `OCCLUDED`. Mahalanobis-gated reacquisition (gate = 3.0 from `predictor_params.yaml`) is used internally. Maximum occlusion duration is bounded by `max_occlusion_sec` = 4.0.
- **Subscribes to**:
  - `/tracks` (`cuas_msgs/TrackArray`).
- **Publishes**:
  - `/predicted_tracks/occlusion` (`cuas_msgs/PredictedTrack`) at 20 Hz.
  - `/trajectory_waypoints/occlusion` (`cuas_msgs/TrajectoryWaypoints`) at 20 Hz.
- **Dependencies**: `imm_tracker_node`.

### 3.11 prediction_mux_node

- **Source**: `src/cuas_fusion/src/prediction/prediction_mux_node.cpp`
- **Description**: Arbitrates between the two predictor streams per track. When a track is `OCCLUDED` the mux forwards the occlusion stream; otherwise it forwards the kinematic stream. Cached entries are evicted after `kPredictionStaleSec = 0.5` seconds of silence on either input. Runs on a 50 ms merge tick.
- **Subscribes to**:
  - `/predicted_tracks/kinematic` (`cuas_msgs/PredictedTrack`).
  - `/predicted_tracks/occlusion` (`cuas_msgs/PredictedTrack`).
  - `/trajectory_waypoints/kinematic` (`cuas_msgs/TrajectoryWaypoints`).
  - `/trajectory_waypoints/occlusion` (`cuas_msgs/TrajectoryWaypoints`).
- **Publishes**:
  - `/predicted_tracks` (`cuas_msgs/PredictedTrack`) at 20 Hz (50 ms timer).
  - `/trajectory_waypoints` (`cuas_msgs/TrajectoryWaypoints`) at 20 Hz.
- **Dependencies**: `kinematic_predictor_node`, `occlusion_predictor_node`.

### 3.12 threat_classifier_node

- **Source**: `src/cuas_fusion/src/classification/threat_classifier_node.cpp` (executable name `classifier_node`).
- **Description**: Wraps the pure `ThreatClassifier` with a five-state escalation FSM (`UNKNOWN → BENIGN → SUSPECT → THREAT → THREATENING → ENGAGED`). It evaluates each track against the configured `threatening_range_m`, `threatening_velocity_mps`, `zone_radius_m`, `escalation_dwell_s`, and `track_timeout_s` parameters from `system_params.yaml`, and joins fused detections by nearest-neighbour to attach the COCO class label.
- **Subscribes to**:
  - `/tracks` (`cuas_msgs/TrackArray`).
  - `/fusion/detections` (`cuas_msgs/FusedDetectionArray`).
- **Publishes**:
  - `/threat/reports` (`cuas_msgs/ThreatReportArray`) gated by the `/tracks` callback (~20 Hz).
- **Dependencies**: `imm_tracker_node`, `fusion_node`.

### 3.13 intent_classifier_node

- **Source**: `src/cuas_fusion/src/intent_classifier_node.cpp`
- **Description**: Behavioural intent classifier that reads the track stream and assigns one of `APPROACHING / LOITERING / ORBITING / DEPARTING / TRANSITING` based on heading change, range trend, and dwell. Loiter radius and approach rate are emitted alongside the enum so downstream consumers can apply a confidence-weighted threshold.
- **Subscribes to**:
  - `/tracks` (`cuas_msgs/TrackArray`).
- **Publishes**:
  - `/intent/reports` (`cuas_msgs/IntentReportArray`) at `publish_rate_hz` = 10 Hz (default in launch parameters).
- **Dependencies**: `imm_tracker_node`.

### 3.14 geofence_node

- **Source**: `src/cuas_fusion/src/geofence_node.cpp`
- **Description**: Loads circular and polygonal exclusion zones from `config/geofence_zones.yaml` and emits an `ENTRY`/`EXIT`/`DWELL` event per track per zone. The engine maintains per-track per-zone state machines so transitions are reported once and dwell is reported continuously while inside.
- **Subscribes to**:
  - `/tracks` (`cuas_msgs/TrackArray`).
  - `/threat/reports` (`cuas_msgs/ThreatReportArray`) — used to weight events by threat level.
- **Publishes**:
  - `/geofence/violations` (`cuas_msgs/GeofenceEventArray`) at `publish_rate_hz` = 10 Hz from the launch file.
- **Dependencies**: `imm_tracker_node`, `threat_classifier_node`; `config/geofence_zones.yaml` must be readable.

### 3.15 reachability_node

- **Source**: `src/cuas_fusion/src/reachability_node.cpp`
- **Description**: Computes time-to-intercept and the covariance ellipse for any track at or above the configured `min_threat_level` (default 1, i.e. SUSPECT and higher). It uses the IMM-derived velocity and a forward Kalman-style covariance projection to bound the impact location.
- **Subscribes to**:
  - `/tracks` (`cuas_msgs/TrackArray`).
  - `/threat/reports` (`cuas_msgs/ThreatReportArray`).
- **Publishes**:
  - `/reachability/warnings` (`cuas_msgs/InterceptReportArray`) at `publish_rate_hz` = 20 Hz from the launch file.
- **Dependencies**: `imm_tracker_node`, `threat_classifier_node`.

### 3.16 cot_publisher_node

- **Source**: `src/cuas_fusion/src/output/cot_publisher_node.cpp`
- **Description**: Emits Cursor-on-Target XML events over UDP multicast to `239.2.3.1:6969` (TTL 32, LAN-scoped). Threatening events (`escalation_state` ∈ {`THREATENING`, `ENGAGED`}) fire at 1 Hz; a complete sweep of every active threat report fires every 5 s. Each event is uniquely identified by `CUAS-TRACK-<track_id>` and includes class label, threat level, escalation state, quality score, speed, and course.
- **Subscribes to**:
  - `/threat/reports` (`cuas_msgs/ThreatReportArray`).
- **Publishes**: no ROS topics. Emits CoT XML over UDP multicast at 1 Hz / 0.2 Hz.
- **Dependencies**: `threat_classifier_node`; the `239.2.3.1` multicast group must be reachable on the LAN.

### 3.17 health_monitor_node

- **Source**: `src/cuas_fusion/src/health_monitor_node.cpp`
- **Description**: Watches the five canonical pipeline streams and computes an EMA-smoothed measured rate against the per-topic expected rate (radar 16 Hz, camera 30 Hz, tracker 20 Hz, classifier 20 Hz, predictor 20 Hz). Each stream maps to a `STATUS_OK / STATUS_DEGRADED / STATUS_FAULT` enum, and the overall status is the worst of the five.
- **Subscribes to**:
  - `/radar/detections` (`sensor_msgs/PointCloud2`).
  - `/camera/image_raw` (`sensor_msgs/Image`).
  - `/tracks` (`cuas_msgs/TrackArray`).
  - `/threat/reports` (`cuas_msgs/ThreatReportArray`).
  - `/predicted_tracks` (`cuas_msgs/PredictedTrack`).
- **Publishes**:
  - `/health/status` (`cuas_msgs/SystemHealth`) at `publish_rate_hz` = 1 Hz.
- **Dependencies**: none — operates correctly even if upstream nodes are absent (it will report `STATUS_FAULT` for the missing streams).

### 3.18 cuas_visualizer_node

- **Source**: `src/cuas_fusion/src/visualization/cuas_visualizer_node.cpp`
- **Description**: Operator console renderer. Builds the RViz2 marker arrays (track spheres, ID/state labels, predicted polylines, uncertainty spheres) on a 50 ms timer and renders the OpenCV camera HUD (range/azimuth/velocity readouts, 2D bounding box with threat-coloured outline, velocity vector arrow, prediction arc, 120-degree PPI sector with threat-coloured dots, top-left track-table, zone dwell timer) on every camera frame. Produces an annotated `bgr8` image consumed by `cuas_overlay_node`.
- **Subscribes to**:
  - `/tracks` (`cuas_msgs/TrackArray`).
  - `/predicted_tracks` (`cuas_msgs/PredictedTrack`).
  - `/trajectory_waypoints` (`cuas_msgs/TrajectoryWaypoints`).
  - `/threat/reports` (`cuas_msgs/ThreatReportArray`).
  - `/fusion/detections` (`cuas_msgs/FusedDetectionArray`).
  - `camera/image_raw` (`sensor_msgs/Image`) — the launch file remaps this to `/camera/image_corrected` when color correction is enabled.
- **Publishes**:
  - `/visualization/track_markers` (`visualization_msgs/MarkerArray`) at 20 Hz (50 ms timer).
  - `/visualization/trajectory_markers` (`visualization_msgs/MarkerArray`) at 20 Hz.
  - `/visualization/uncertainty_markers` (`visualization_msgs/MarkerArray`) at 20 Hz.
  - `/visualization/track_labels` (`visualization_msgs/MarkerArray`) at 20 Hz.
  - `/camera/annotated` (`sensor_msgs/Image`) at the camera input rate (~30 Hz).
- **Dependencies**: `imm_tracker_node`, `prediction_mux_node`, `threat_classifier_node`, `fusion_node`, `camera_node` (and optionally `cuas_color_correct_node`).

### 3.19 cuas_overlay_node

- **Source**: `src/cuas_fusion/src/visualization/cuas_overlay_node.cpp`
- **Description**: Downstream enhancer that runs after `cuas_visualizer_node`. It re-decodes the annotated image using the local `rosImageToBgr` adapter (no `cv_bridge` dependency, per DEV-001), applies `OverlayEngine::render` to add trajectory arcs, prediction dots, and weighted track labels using the constants in `constants.hpp` (`kArcMaxWaypoints`, `kArcDotMinRadius`, `kArcLineThickness`, etc.), and republishes. It is intentionally a separate node so iteration on overlay logic cannot break the verified visualizer rendering.
- **Subscribes to**:
  - `/camera/annotated` (`sensor_msgs/Image`).
  - `/tracks` (`cuas_msgs/TrackArray`).
  - `/threat/reports` (`cuas_msgs/ThreatReportArray`).
  - `/trajectory_waypoints` (`cuas_msgs/TrajectoryWaypoints`).
- **Publishes**:
  - `/camera/annotated_enhanced` (`sensor_msgs/Image`) at the input image rate (~30 Hz). FPS is logged once every 5 s over a 30-frame sliding window.
- **Dependencies**: `cuas_visualizer_node`, `imm_tracker_node`, `threat_classifier_node`, `prediction_mux_node`.

### 3.20 sim_radar_node (simulation mode only)

- **Source**: `src/cuas_fusion/src/drivers/sim_radar_node.cpp`
- **Description**: Software-in-the-loop substitute for `radar_parser_node`. Replays a scripted scenario (default `approach`, configurable through the `scenario` parameter) at the configured rate with deterministic noise (`noise_seed=42`) and emits the same `PointCloud2` topic the hardware parser would. Bounds: `kSimRadarMaxRangeM = 15`, `kSimRadarMaxTargets = 8`, `kSimRadarMaxPoints = 64`, `kSimRadarNoiseSigmaM = 0.10`.
- **Subscribes to**: none.
- **Publishes**:
  - `/radar/detections` (`sensor_msgs/PointCloud2`) at the `publish_rate_hz` parameter (16 Hz in `sim.launch.py`).
- **Dependencies**: none. This node only runs in `sim.launch.py`; in `full.launch.py` the hardware `radar_parser_node` produces the same topic.

### 3.21 georef_node

- **Status**: planned, not yet implemented.
- **Source**: not present in `src/cuas_fusion/CMakeLists.txt` or any launch file as of this revision.
- **Description (planned)**: When fielded, this node will read GPS NMEA / time-pulse from a co-located receiver and publish WGS84-georeferenced track positions, replacing the placeholder `lat=0.0 lon=0.0` in the CoT XML and adding WGS84 fields to the `Track` and `ThreatReport` messages as called out in the forward-compatibility section of `docs/CODING_STANDARD.md`. Until implemented, the system operates in body-frame coordinates only.
- **Subscribes to (planned)**: `/tracks`, `/gps/fix` (or vendor-specific NMEA topic).
- **Publishes (planned)**: a georeferenced track stream and a corrected CoT `<point>` payload.
- **Dependencies**: `imm_tracker_node`, an external GNSS source.

## 4. Topic Reference Table

| Topic | Message Type | Producer | Consumers | Rate |
|---|---|---|---|---|
| `/radar/detections` | `sensor_msgs/PointCloud2` | `radar_parser_node` (or `sim_radar_node` in sim) | `clutter_map_node`, `health_monitor_node` | ~16 Hz |
| `/radar/filtered` | `sensor_msgs/PointCloud2` | `clutter_map_node` | `imm_tracker_node` (via launch remap) | ~16 Hz |
| `/clutter/status` | `cuas_msgs/ClutterStatus` | `clutter_map_node` | reserved (visualizer/health hookup planned) | 1 Hz |
| `/camera/image_raw` | `sensor_msgs/Image` | `camera_node` | `cuas_color_correct_node`, `inference_node` (if not corrected), `cuas_visualizer_node`, `health_monitor_node` | 30 Hz |
| `/camera/image_corrected` | `sensor_msgs/Image` | `cuas_color_correct_node` | `inference_node`, `cuas_visualizer_node`, `cuas_overlay_node` (when `color_correct:=true`) | ~30 Hz |
| `/inference/detections` | `vision_msgs/Detection2DArray` | `inference_node` | `fusion_node` | ~35 Hz |
| `/tracks` | `cuas_msgs/TrackArray` | `imm_tracker_node` | `fusion_node`, `kinematic_predictor_node`, `occlusion_predictor_node`, `threat_classifier_node`, `intent_classifier_node`, `geofence_node`, `reachability_node`, `cuas_visualizer_node`, `cuas_overlay_node`, `health_monitor_node` | 20 Hz |
| `/tracks/confirmed` | `cuas_msgs/TrackArray` | `track_manager_node` (debug, not in `full.launch.py`) | none in standard deployment | ~20 Hz |
| `/fusion/detections` | `cuas_msgs/FusedDetectionArray` | `fusion_node` | `threat_classifier_node`, `cuas_visualizer_node`, `track_manager_node` (when launched) | ~20 Hz |
| `/threat/reports` | `cuas_msgs/ThreatReportArray` | `threat_classifier_node` | `cot_publisher_node`, `geofence_node`, `reachability_node`, `cuas_visualizer_node`, `cuas_overlay_node`, `imm_tracker_node` (horizon feedback), `health_monitor_node` | ~20 Hz |
| `/predicted_tracks/kinematic` | `cuas_msgs/PredictedTrack` | `kinematic_predictor_node` | `prediction_mux_node` | 20 Hz |
| `/predicted_tracks/occlusion` | `cuas_msgs/PredictedTrack` | `occlusion_predictor_node` | `prediction_mux_node` | 20 Hz |
| `/predicted_tracks` | `cuas_msgs/PredictedTrack` | `prediction_mux_node` | `cuas_visualizer_node`, `health_monitor_node` | 20 Hz |
| `/trajectory_waypoints/kinematic` | `cuas_msgs/TrajectoryWaypoints` | `kinematic_predictor_node` | `prediction_mux_node` | 20 Hz |
| `/trajectory_waypoints/occlusion` | `cuas_msgs/TrajectoryWaypoints` | `occlusion_predictor_node` | `prediction_mux_node` | 20 Hz |
| `/trajectory_waypoints` | `cuas_msgs/TrajectoryWaypoints` | `prediction_mux_node` | `cuas_visualizer_node`, `cuas_overlay_node` | 20 Hz |
| `/intent/reports` | `cuas_msgs/IntentReportArray` | `intent_classifier_node` | reserved (operator console / future C2 link) | 10 Hz |
| `/geofence/violations` | `cuas_msgs/GeofenceEventArray` | `geofence_node` | reserved (operator console / CoT extension) | 10 Hz |
| `/reachability/warnings` | `cuas_msgs/InterceptReportArray` | `reachability_node` | reserved (operator console / engagement gate) | 20 Hz |
| `/health/status` | `cuas_msgs/SystemHealth` | `health_monitor_node` | reserved (operator console BIT panel) | 1 Hz |
| `/visualization/track_markers` | `visualization_msgs/MarkerArray` | `cuas_visualizer_node` | `rviz2` | 20 Hz |
| `/visualization/trajectory_markers` | `visualization_msgs/MarkerArray` | `cuas_visualizer_node` | `rviz2` | 20 Hz |
| `/visualization/uncertainty_markers` | `visualization_msgs/MarkerArray` | `cuas_visualizer_node` | `rviz2` | 20 Hz |
| `/visualization/track_labels` | `visualization_msgs/MarkerArray` | `cuas_visualizer_node` | `rviz2` | 20 Hz |
| `/camera/annotated` | `sensor_msgs/Image` | `cuas_visualizer_node` | `cuas_overlay_node` | ~30 Hz |
| `/camera/annotated_enhanced` | `sensor_msgs/Image` | `cuas_overlay_node` | `rviz2` | ~30 Hz |

## 5. Message Type Summary

The 19 message types in `msgs/cuas_msgs/msg/` cover the full pipeline.

1. **`RadarDetection`** — single radar return (x, y, z, velocity, timestamp_ns); raw sensor stage.
2. **`RadarFrame`** — `Header` + `RadarDetection[]`; raw sensor stage container.
3. **`FusedDetection`** — radar+camera fused observation (3D position, velocity, COCO class label, confidence, pixel u/v, range, azimuth, bbox dims); fusion stage.
4. **`FusedDetectionArray`** — `Header` + `FusedDetection[]`; fusion stage container.
5. **`Track`** — confirmed track (id, position, velocity, doppler, IMM CT probability, maneuvering flag, adaptive horizon, track-state enum + string, vx/vy components, confidence); the system's narrow-waist data type.
6. **`TrackArray`** — `Header` + `Track[]`; tracking stage container.
7. **`ThreatReport`** — per-track threat assessment (level, escalation FSM state, predicted impact xy, exclusion-zone violations list, prediction horizon, threat-level enum); classification stage.
8. **`ThreatReportArray`** — `Header` + `ThreatReport[]`; classification stage container.
9. **`IntentReport`** — per-track behavioural intent (intent enum, confidence, loiter radius, approach rate); intent stage.
10. **`IntentReportArray`** — `IntentReport[]` + stamp; intent stage container.
11. **`PredictedTrack`** — per-track forward state with full 6×6 covariance, IMM model weights (CV/CA/CT), bearing/elevation; prediction stage.
12. **`TrajectoryWaypoints`** — per-track polyline of forecast waypoints with per-step uncertainty radii and bearing/elevation; prediction stage.
13. **`GeofenceEvent`** — single zone violation (zone_id, track_id, event_type, distance); geofence stage.
14. **`GeofenceEventArray`** — `GeofenceEvent[]` + stamp; geofence stage container.
15. **`InterceptReport`** — per-track time-to-intercept, intercept confidence, covariance ellipse (major/minor/heading), intercept-possible flag; reachability stage.
16. **`InterceptReportArray`** — `InterceptReport[]` + stamp; reachability stage container.
17. **`SystemHealth`** — overall + per-subsystem status enums and measured Hz; health-monitor stage.
18. **`SystemHealthArray`** — `SystemHealth[]` + stamp; health-monitor stage container (reserved for multi-station aggregation).
19. **`ClutterStatus`** — clutter-map state (state enum, frames learned/required, occupancy ratio); pre-track filter stage.

## 6. Coordinate Frames

The system uses three rigid frames connected by static tf2 transforms.

- **`base_link`** — system origin. All track positions and threat impact predictions are reported in this frame. The launch file publishes a static identity transform `base_link → radar_frame` so a single-radar deployment treats the radar boresight as the platform origin.
- **`radar_frame`** — IWR6843ISK boresight. Convention: `X` = azimuth (positive right of boresight), `Y` = range / depth (positive away from the radar), `Z` = elevation (positive up). The radar parser emits PointCloud2 returns directly in this frame; in the standard deployment it coincides numerically with `base_link`.
- **`camera_frame`** — AR0234 sensor frame. Origin is offset from `radar_frame` by `(x = -0.0075 m, y = 0.017 m, z = -0.079 m)` — the lens optical centre sits 7.5 mm to the left of, 17 mm above, and 79 mm below the radar boresight on the assembled fixture. `fusion_node` declares these offsets as ROS parameters (`extrinsic.x_offset_m`, `extrinsic.y_offset_m`, `extrinsic.z_offset_m`) and broadcasts a static `radar_frame → camera_frame` transform on tf2 at startup.

The camera intrinsics used for projection live in `include/cuas_fusion/common/constants.hpp` (`CAMERA_FX = 1862.7`, `CAMERA_FY = 1877.7`, `CAMERA_CX = 1032.8`, `CAMERA_CY = 426.8`, image 1920×1080) and were obtained from a checkerboard calibration on 2026-04-09.

## 7. Key Architectural Decisions

**Math classes have zero ROS dependencies.** Every algorithmic component — `KalmanCV`, `KalmanCA`, `KalmanCT`, `ImmFilter`, `KinematicPredictor`, `OcclusionPredictor`, `TrackManager`, `ThreatClassifier`, `FusionEngine`, `GeofenceEngine`, `ReachabilityEngine`, `HealthMonitor`, `IntentClassifier`, `ClutterMap`, `OverlayEngine`, `ColorCorrectEngine` — is a pure C++ class compiled into a static library that links against Eigen and the standard library and nothing else. Each is unit-tested under `colcon test` with `ament_cmake_gtest`. The ROS layer is the thinnest possible adapter: subscribe, marshal the message into the math-class signature, call the function, marshal the result back into a message, publish. The reason is testability and substitutability — the same `FusionEngine` runs inside `fusion_node`, the unit-test harness, and the planned offline replay tool, so any change is verified deterministically before it lands on the live pipeline.

**`Track` is the narrow waist of the system.** Exactly one node produces `/tracks` (`imm_tracker_node`), and ten consumers read from it (`fusion_node`, `kinematic_predictor_node`, `occlusion_predictor_node`, `threat_classifier_node`, `intent_classifier_node`, `geofence_node`, `reachability_node`, `cuas_visualizer_node`, `cuas_overlay_node`, `health_monitor_node`). This 1-producer/10-consumer topology is deliberate: the tracker is the single point where confidence decay, IMM mode mixing, Hungarian association, Mahalanobis gating, and adaptive prediction horizon are computed, and every downstream consumer sees the same authoritative state. Adding a new feature (intent classification, geofence, reachability) reduces to writing a new node that subscribes to `/tracks` rather than reaching back into the tracker, which keeps the tracker's invariants intact and bounds the cyclomatic complexity per node.

**The overlay node is downstream of the visualizer, not a replacement.** `cuas_visualizer_node` produces the canonical `/camera/annotated` stream with the verified bounding boxes, range/azimuth/velocity readouts, PPI, track table, and zone timer. `cuas_overlay_node` subscribes to that already-rendered stream and adds the trajectory arc, prediction dots, and weighted track labels on top. The two nodes are decoupled deliberately: iteration on overlay aesthetics, font scales, dot radii, and arc-length heuristics happens entirely in the overlay node, and a regression there cannot corrupt the operator's primary track-and-status display. This is the same "never modify a verified component" pattern that motivates the static math-class libraries.

**JSF AV C++ was chosen because the project targets defense-program conventions.** The Joint Strike Fighter Air Vehicle C++ standard combined with MISRA C++:2023 cross-rules is the lingua franca for ground-station and avionics-adjacent software in the United States defense ecosystem; reviewers from Northrop Grumman, Lockheed Martin, Raytheon, and L3Harris will have audited code under one or both standards. Adopting them up front (no heap allocation after init, no exceptions in application code, no RTTI in application code, all state machines as `enum class`, fixed-bound containers everywhere, every switch with a `default`, no bare `int`/`double`) means the codebase is immediately familiar to a defense reviewer and the deviations that the ROS 2 framework forces (DEV-001 through DEV-005 in `docs/CODING_STANDARD.md`) are documented at the same level of formality as a real MISRA Compliance:2020 deviation report.

## 8. Hardware Configuration

**NVIDIA Jetson Orin Nano Super.** The compute platform. Runs Ubuntu 22.04 with JetPack and ROS 2 Humble. The TensorRT INT8 engine is built once for the on-board iGPU and cached at `/home/zork/ProjectHailMarry/models/yolov8s_int8.engine`. CUDA 12.6 headers and `libnvinfer` are linked from `/usr/local/cuda` and `/usr/lib/aarch64-linux-gnu`. All other application logic is CPU-bound and fits comfortably in the platform's eight Cortex-A78AE cores.

**Texas Instruments IWR6843ISK.** A 60 GHz mmWave radar evaluation module. Two USB serial endpoints are exposed: a configuration port that accepts the radar profile (chirp configuration, frame rate, range/doppler bins) and a data port that streams TLV-formatted detected-object frames at the profile's frame rate. The standard profile in `config/radar_profile.cfg` produces approximately sixteen frames per second. Stable device naming is achieved through the udev rules:

```
# /etc/udev/rules.d/99-radar.rules (paraphrased)
SUBSYSTEM=="tty", ATTRS{idVendor}=="0451", ATTRS{idProduct}=="bef3", \
    ATTRS{interface}=="XDS110 Class Application/User UART", SYMLINK+="radar_config"
SUBSYSTEM=="tty", ATTRS{idVendor}=="0451", ATTRS{idProduct}=="bef3", \
    ATTRS{interface}=="XDS110 Class Auxiliary Data Port",   SYMLINK+="radar_data"
```

`full.launch.py` invokes `scripts/send_radar_config.sh` against `/dev/radar_config` before launching `radar_parser_node`, which reads `/dev/radar_data` until the node is shut down.

**Arducam AR0234.** A 1920×1080 global-shutter colour CMOS sensor. The Jetson exposes it as a standard V4L2 device at `/dev/video0`; we deliberately do not use a GStreamer capture path at runtime because that pulls in plugin discovery and dynamic linkage that the JSF AV deviation budget does not justify. Sensor-specific compensation (black level 2752, white-balance gains B=2.987 / G=1.000 / R=0.861, tone scale 0.026) lives in `constants.hpp` and is applied either inside the camera driver or by the optional `cuas_color_correct_node` depending on the launch flag. The lens calibration giving `CAMERA_FX = 1862.7`, `CAMERA_FY = 1877.7`, `CAMERA_CX = 1032.8`, `CAMERA_CY = 426.8` was performed on 2026-04-09 against a checkerboard target.

**Permanent device symlinks via udev.** `/dev/radar_data`, `/dev/radar_config`, and `/dev/video0` are guaranteed by udev rules so the launch file can hard-code paths without depending on USB enumeration order. This means the system boots into a known state — the Jetson can be power-cycled with both USB peripherals attached and `ros2 launch cuas_fusion full.launch.py` will come up green without operator intervention.
