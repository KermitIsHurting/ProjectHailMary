# ProjectHailMary — Interface Control Document (ICD)
# Version 0.8.0-alpha

## 1. Document Purpose

This Interface Control Document (ICD) specifies every software interface exposed between the ROS 2 nodes that compose the ProjectHailMary counter-unmanned-aerial-system (C-UAS) sensor fusion ground station, together with the one external interface (Cursor-on-Target / ATAK UDP multicast) that crosses the ground-station boundary. It records, for each interface, the topic name, message type, producing node, consuming nodes, publish rate, Quality-of-Service (QoS) profile, and `frame_id` contract, and it transcribes the field-by-field schema for every custom message type in the `cuas_msgs` package. The document follows defense program ICD conventions (single authoritative interface table, explicit QoS contracts, frozen message-field semantics, and a versioned change log) so that interfaces can be reviewed, regression-tested, and wire-compatible across release boundaries. All facts in this document were extracted directly from the source under `src/cuas_fusion/` and `msgs/cuas_msgs/`; nothing is paraphrased or inferred.

## 2. System Interface Overview

ProjectHailMary 0.8.0-alpha runs **twenty-one** ROS 2 nodes (twenty active in the standard hardware deployment plus one simulation substitute) that exchange data over **twenty-six** ROS topics typed against **nineteen** custom messages defined in `msgs/cuas_msgs/msg/` and a small number of stock ROS messages (`sensor_msgs/Image`, `sensor_msgs/PointCloud2`, `vision_msgs/Detection2DArray`, `visualization_msgs/MarkerArray`). The system has exactly one external interface: a UDP multicast Cursor-on-Target stream emitted by `cot_publisher_node` for ATAK-class clients.

The central data type is `cuas_msgs/msg/Track`, carried on the `/tracks` topic. It has exactly **one producer** (`imm_tracker_node`) and **ten consumers** (`fusion_node`, `kinematic_predictor_node`, `occlusion_predictor_node`, `threat_classifier_node`, `intent_classifier_node`, `geofence_node`, `reachability_node`, `cuas_visualizer_node`, `cuas_overlay_node`, `health_monitor_node`). Every downstream feature — fusion, prediction, threat classification, intent, geofencing, reachability, visualization, and health — derives from this single track stream, which makes `/tracks` the architectural narrow waist of the system.

No node sets a custom `rclcpp::QoS` profile. Every publisher and subscriber is constructed with a plain integer history depth, so every topic uses the ROS 2 default profile: **Reliable** reliability, **Volatile** durability, **Keep Last** history, with the depth declared at construction. Where a topic's nominal rate is governed by a YAML parameter rather than a hard-coded timer, this document records the launch-file value; where it is governed by upstream callbacks, the rate is the upstream rate.

## 3. Topic Interface Definitions

### /radar/detections

| Field        | Value                                               |
|--------------|-----------------------------------------------------|
| Message Type | sensor_msgs/msg/PointCloud2                         |
| Producer     | radar_parser_node (hardware) or sim_radar_node (sim)|
| Consumers    | clutter_map_node, imm_tracker_node (via remap), health_monitor_node |
| Rate         | 16 Hz                                               |
| QoS          | ROS 2 default (Reliable, Volatile, Keep Last 10)    |
| Frame ID     | radar_frame                                         |

**Description:** Carries clustered radar-return centroids from the IWR6843ISK radar (or the simulated radar in SIL mode) into the fusion pipeline.

**Message Fields:** Stock `sensor_msgs/PointCloud2`; `x`, `y`, `z` are the cluster centroid coordinates in `radar_frame` (X = azimuth, Y = range/depth, Z = elevation). `imm_tracker_node` consumes this topic via the launch-file remap `('/radar/detections', '/radar/filtered')`, so it actually reads the clutter-filtered stream.

### /radar/filtered

| Field        | Value                                            |
|--------------|--------------------------------------------------|
| Message Type | sensor_msgs/msg/PointCloud2                      |
| Producer     | clutter_map_node                                 |
| Consumers    | imm_tracker_node (via launch-file remap)         |
| Rate         | 16 Hz (driven by `/radar/detections`)            |
| QoS          | ROS 2 default (Reliable, Volatile, Keep Last 10) |
| Frame ID     | radar_frame                                      |

**Description:** Clutter-filtered radar centroids, with stationary occupancy-grid hits removed. The launch file remaps this topic onto the IMM tracker's `/radar/detections` subscription.

### /clutter/status

| Field        | Value                                            |
|--------------|--------------------------------------------------|
| Message Type | cuas_msgs/msg/ClutterStatus                      |
| Producer     | clutter_map_node                                 |
| Consumers    | reserved (operator console / health hook)        |
| Rate         | 1 Hz (1000 ms wall timer in `clutter_map_node`)  |
| QoS          | ROS 2 default (Reliable, Volatile, Keep Last 10) |
| Frame ID     | N/A (no `Header`)                                |

**Description:** Reports the learning state of the clutter-map occupancy grid (learning vs. live, frames learned, occupancy ratio).

**Message Fields:**

| Field             | Type                     | Description                                  |
|-------------------|--------------------------|----------------------------------------------|
| state             | uint8                    | Enumerated learning state                    |
| frames_learned    | uint32                   | Frames absorbed into the grid so far         |
| frames_required   | uint32                   | Frames required before the grid goes live    |
| occupancy_ratio   | float32                  | Fraction of cells classified as occupied     |
| stamp             | builtin_interfaces/Time  | Publish timestamp                            |

### /camera/image_raw

| Field        | Value                                            |
|--------------|--------------------------------------------------|
| Message Type | sensor_msgs/msg/Image                            |
| Producer     | camera_node                                      |
| Consumers    | cuas_color_correct_node, inference_node (when `color_correct:=false`), cuas_visualizer_node, health_monitor_node |
| Rate         | 30 Hz (`CAMERA_FPS` in `constants.hpp`)          |
| QoS          | ROS 2 default (Reliable, Volatile, Keep Last 10) |
| Frame ID     | camera                                           |

**Description:** Raw 1920×1080 `bgr8` frames captured from the AR0234 sensor over V4L2.

**Message Fields:** Stock `sensor_msgs/Image`; encoding `bgr8`, width 1920, height 1080.

### /camera/image_corrected

| Field        | Value                                                        |
|--------------|--------------------------------------------------------------|
| Message Type | sensor_msgs/msg/Image                                        |
| Producer     | cuas_color_correct_node                                      |
| Consumers    | inference_node (when `color_correct:=true`), cuas_visualizer_node, cuas_overlay_node (via remap when enabled) |
| Rate         | ~30 Hz (driven by `/camera/image_raw`)                       |
| QoS          | ROS 2 default (Reliable, Volatile, Keep Last 5)              |
| Frame ID     | camera (forwarded from input)                                |

**Description:** White-balance-corrected camera stream with per-channel gains B=2.987, G=1.000, R=0.861 applied to compensate for the AR0234's missing IR-cut filter. (Note: the prospective name `/camera/image_color_corrected` referenced in some planning notes is realized in source as `/camera/image_corrected`.)

### /inference/detections

| Field        | Value                                            |
|--------------|--------------------------------------------------|
| Message Type | vision_msgs/msg/Detection2DArray                 |
| Producer     | inference_node                                   |
| Consumers    | fusion_node                                      |
| Rate         | ~35 Hz (TensorRT INT8 throughput on Orin Nano)   |
| QoS          | ROS 2 default (Reliable, Volatile, Keep Last 5)  |
| Frame ID     | camera (forwarded from input image)              |

**Description:** YOLOv8s INT8 TensorRT detections; `hypothesis.class_id` is the COCO numeric ID as a string and `hypothesis.score` is the confidence.

### /fusion/detections

| Field        | Value                                                                                  |
|--------------|----------------------------------------------------------------------------------------|
| Message Type | cuas_msgs/msg/FusedDetectionArray                                                      |
| Producer     | fusion_node                                                                            |
| Consumers    | threat_classifier_node, cuas_visualizer_node, track_manager_node (only when launched)  |
| Rate         | ~20 Hz (gated by `/tracks` callback in `fusion_node`)                                  |
| QoS          | ROS 2 default (Reliable, Volatile, Keep Last 5)                                        |
| Frame ID     | radar_frame                                                                            |

**Description:** Camera-radar fused detections produced by projecting tracked radar centroids into the camera image plane and IoU-associating with YOLO bounding boxes.

**Message Fields:** see `cuas_msgs/msg/FusedDetectionArray` and `cuas_msgs/msg/FusedDetection` in section 5.

### /tracks

| Field        | Value                                                                                                                                       |
|--------------|---------------------------------------------------------------------------------------------------------------------------------------------|
| Message Type | cuas_msgs/msg/TrackArray                                                                                                                    |
| Producer     | imm_tracker_node                                                                                                                            |
| Consumers    | fusion_node, kinematic_predictor_node, occlusion_predictor_node, threat_classifier_node, intent_classifier_node, geofence_node, reachability_node, cuas_visualizer_node, cuas_overlay_node, health_monitor_node |
| Rate         | 20 Hz (50 ms wall timer in `imm_tracker_node`)                                                                                              |
| QoS          | ROS 2 default (Reliable, Volatile, Keep Last 10)                                                                                            |
| Frame ID     | base_link                                                                                                                                   |

**Description:** Authoritative confirmed-track stream; the central data type of the system.

### /tracks/confirmed

| Field        | Value                                            |
|--------------|--------------------------------------------------|
| Message Type | cuas_msgs/msg/TrackArray                         |
| Producer     | track_manager_node (named `tracker_node` in CMake; not enabled in `full.launch.py`) |
| Consumers    | none in standard deployment; reserved for offline replay and unit tests |
| Rate         | ~20 Hz (gated by `/fusion/detections` callback)  |
| QoS          | ROS 2 default (Reliable, Volatile, Keep Last 5)  |
| Frame ID     | (unset by the node)                              |

**Description:** Alternate confirmed-only track stream produced by the simpler `TrackManager` lifecycle when `track_manager_node` is launched as a debug aid.

### /threat/reports

| Field        | Value                                                                                                                            |
|--------------|----------------------------------------------------------------------------------------------------------------------------------|
| Message Type | cuas_msgs/msg/ThreatReportArray                                                                                                  |
| Producer     | threat_classifier_node                                                                                                           |
| Consumers    | cot_publisher_node, geofence_node, reachability_node, cuas_visualizer_node, cuas_overlay_node, imm_tracker_node (horizon feedback), health_monitor_node |
| Rate         | ~20 Hz (gated by `/tracks` callback in `threat_classifier_node`)                                                                 |
| QoS          | ROS 2 default (Reliable, Volatile, Keep Last 5)                                                                                  |
| Frame ID     | base_link (forwarded from input `Track`)                                                                                         |

**Description:** Per-track threat assessment with five-state escalation FSM, exclusion-zone violation list, predicted impact, and adaptive prediction horizon. The launch-file plan refers to this stream as `/threat_reports`; the actual topic string in `threat_classifier_node.cpp` is `/threat/reports`.

### /predicted_tracks/kinematic

| Field        | Value                                            |
|--------------|--------------------------------------------------|
| Message Type | cuas_msgs/msg/PredictedTrack                     |
| Producer     | kinematic_predictor_node                         |
| Consumers    | prediction_mux_node                              |
| Rate         | 20 Hz (`publish_rate_hz` in `predictor_params.yaml`) |
| QoS          | ROS 2 default (Reliable, Volatile, Keep Last 10) |
| Frame ID     | base_link                                        |

**Description:** Per-track forward state predicted by the constant-velocity / constant-acceleration kinematic predictor.

### /predicted_tracks/occlusion

| Field        | Value                                            |
|--------------|--------------------------------------------------|
| Message Type | cuas_msgs/msg/PredictedTrack                     |
| Producer     | occlusion_predictor_node                         |
| Consumers    | prediction_mux_node                              |
| Rate         | 20 Hz (`publish_rate_hz` in `predictor_params.yaml`) |
| QoS          | ROS 2 default (Reliable, Volatile, Keep Last 10) |
| Frame ID     | base_link                                        |

**Description:** Per-track forward state for occluded tracks, with Mahalanobis-gated reacquisition; bounded by `max_occlusion_sec = 4.0`.

### /predicted_tracks

| Field        | Value                                            |
|--------------|--------------------------------------------------|
| Message Type | cuas_msgs/msg/PredictedTrack                     |
| Producer     | prediction_mux_node                              |
| Consumers    | cuas_visualizer_node, health_monitor_node        |
| Rate         | 20 Hz (50 ms merge tick in `prediction_mux_node`) |
| QoS          | ROS 2 default (Reliable, Volatile, Keep Last 10) |
| Frame ID     | base_link                                        |

**Description:** Arbitrated per-track prediction; selects the occlusion stream when a track's `track_state` is `OCCLUDED`, otherwise the kinematic stream.

### /trajectory_waypoints/kinematic

| Field        | Value                                            |
|--------------|--------------------------------------------------|
| Message Type | cuas_msgs/msg/TrajectoryWaypoints                |
| Producer     | kinematic_predictor_node                         |
| Consumers    | prediction_mux_node                              |
| Rate         | 20 Hz                                            |
| QoS          | ROS 2 default (Reliable, Volatile, Keep Last 10) |
| Frame ID     | base_link                                        |

**Description:** Per-step trajectory polyline from the kinematic predictor, with per-step uncertainty radii.

### /trajectory_waypoints/occlusion

| Field        | Value                                            |
|--------------|--------------------------------------------------|
| Message Type | cuas_msgs/msg/TrajectoryWaypoints                |
| Producer     | occlusion_predictor_node                         |
| Consumers    | prediction_mux_node                              |
| Rate         | 20 Hz                                            |
| QoS          | ROS 2 default (Reliable, Volatile, Keep Last 10) |
| Frame ID     | base_link                                        |

**Description:** Trajectory polyline for tracks served by the occlusion predictor.

### /trajectory_waypoints

| Field        | Value                                            |
|--------------|--------------------------------------------------|
| Message Type | cuas_msgs/msg/TrajectoryWaypoints                |
| Producer     | prediction_mux_node                              |
| Consumers    | cuas_visualizer_node, cuas_overlay_node          |
| Rate         | 20 Hz                                            |
| QoS          | ROS 2 default (Reliable, Volatile, Keep Last 10) |
| Frame ID     | base_link                                        |

**Description:** Arbitrated trajectory polyline used by both the visualizer (RViz2 line strips) and the overlay node (downstream camera-image arc).

### /intent/reports

| Field        | Value                                            |
|--------------|--------------------------------------------------|
| Message Type | cuas_msgs/msg/IntentReportArray                  |
| Producer     | intent_classifier_node                           |
| Consumers    | reserved (operator console / future C2 link)     |
| Rate         | 10 Hz (`publish_rate_hz` parameter, default 10.0)|
| QoS          | ROS 2 default (Reliable, Volatile, Keep Last 10) |
| Frame ID     | N/A (no `Header`; uses `stamp` field instead)    |

**Description:** Behavioural intent classification (APPROACHING, LOITERING, ORBITING, DEPARTING, TRANSITING) per active track. Source topic string is `/intent/reports`.

### /geofence/violations

| Field        | Value                                            |
|--------------|--------------------------------------------------|
| Message Type | cuas_msgs/msg/GeofenceEventArray                 |
| Producer     | geofence_node                                    |
| Consumers    | reserved (operator console / CoT extension)     |
| Rate         | 10 Hz (`publish_rate_hz` parameter in launch)    |
| QoS          | ROS 2 default (Reliable, Volatile, Keep Last 10) |
| Frame ID     | N/A (no `Header`; uses `stamp` field instead)    |

**Description:** Entry/exit/dwell events for tracks crossing exclusion zones loaded from `config/geofence_zones.yaml`. Source topic string is `/geofence/violations`.

### /reachability/warnings

| Field        | Value                                            |
|--------------|--------------------------------------------------|
| Message Type | cuas_msgs/msg/InterceptReportArray               |
| Producer     | reachability_node                                |
| Consumers    | reserved (operator console / engagement gate)    |
| Rate         | 20 Hz (`publish_rate_hz` parameter in launch)    |
| QoS          | ROS 2 default (Reliable, Volatile, Keep Last 10) |
| Frame ID     | N/A (no `Header`; uses `stamp` field instead)    |

**Description:** Per-track time-to-intercept and forward covariance ellipse for tracks at or above `min_threat_level` (default SUSPECT). Source topic string is `/reachability/warnings`.

### /health/status

| Field        | Value                                            |
|--------------|--------------------------------------------------|
| Message Type | cuas_msgs/msg/SystemHealth                       |
| Producer     | health_monitor_node                              |
| Consumers    | reserved (operator console BIT panel)            |
| Rate         | 1 Hz (`publish_rate_hz` parameter, default 1.0)  |
| QoS          | ROS 2 default (Reliable, Volatile, Keep Last 10) |
| Frame ID     | N/A (no `Header`; uses `stamp` field instead)    |

**Description:** Per-subsystem and aggregate liveness with EMA-smoothed measured rates against the expected radar=16 Hz, camera=30 Hz, tracker=20 Hz, classifier=20 Hz, predictor=20 Hz contracts.

### /visualization/track_markers

| Field        | Value                                            |
|--------------|--------------------------------------------------|
| Message Type | visualization_msgs/msg/MarkerArray               |
| Producer     | cuas_visualizer_node                             |
| Consumers    | rviz2                                            |
| Rate         | 20 Hz (50 ms wall timer in visualizer)           |
| QoS          | ROS 2 default (Reliable, Volatile, Keep Last 10) |
| Frame ID     | base_link                                        |

**Description:** Threat-coloured `SPHERE` markers, one per active track, lifetime 100 ms.

### /visualization/trajectory_markers

| Field        | Value                                            |
|--------------|--------------------------------------------------|
| Message Type | visualization_msgs/msg/MarkerArray               |
| Producer     | cuas_visualizer_node                             |
| Consumers    | rviz2                                            |
| Rate         | 20 Hz                                            |
| QoS          | ROS 2 default (Reliable, Volatile, Keep Last 10) |
| Frame ID     | base_link                                        |

**Description:** `LINE_STRIP` markers tracing the predicted trajectory polyline per track.

### /visualization/uncertainty_markers

| Field        | Value                                            |
|--------------|--------------------------------------------------|
| Message Type | visualization_msgs/msg/MarkerArray               |
| Producer     | cuas_visualizer_node                             |
| Consumers    | rviz2                                            |
| Rate         | 20 Hz                                            |
| QoS          | ROS 2 default (Reliable, Volatile, Keep Last 10) |
| Frame ID     | base_link                                        |

**Description:** `SPHERE` markers sized to the per-step uncertainty radius, alpha-faded with horizon distance.

### /visualization/track_labels

| Field        | Value                                            |
|--------------|--------------------------------------------------|
| Message Type | visualization_msgs/msg/MarkerArray               |
| Producer     | cuas_visualizer_node                             |
| Consumers    | rviz2                                            |
| Rate         | 20 Hz                                            |
| QoS          | ROS 2 default (Reliable, Volatile, Keep Last 10) |
| Frame ID     | base_link                                        |

**Description:** `TEXT_VIEW_FACING` markers carrying track ID, state, bearing, and elevation.

### /camera/annotated

| Field        | Value                                            |
|--------------|--------------------------------------------------|
| Message Type | sensor_msgs/msg/Image                            |
| Producer     | cuas_visualizer_node                             |
| Consumers    | cuas_overlay_node                                |
| Rate         | ~30 Hz (driven by `/camera/image_raw` callback)  |
| QoS          | ROS 2 default (Reliable, Volatile, Keep Last 1)  |
| Frame ID     | camera (forwarded from input)                    |

**Description:** Camera image with bounding boxes, range/azimuth/velocity readouts, PPI sector, track table, and zone dwell timer drawn by the visualizer.

### /camera/annotated_enhanced

| Field        | Value                                            |
|--------------|--------------------------------------------------|
| Message Type | sensor_msgs/msg/Image                            |
| Producer     | cuas_overlay_node                                |
| Consumers    | rviz2                                            |
| Rate         | ~30 Hz (driven by `/camera/annotated` callback)  |
| QoS          | ROS 2 default (Reliable, Volatile, Keep Last 5)  |
| Frame ID     | camera (forwarded from input)                    |

**Description:** Visualizer output with the additional trajectory arc, prediction dots, and weighted track labels rendered by the downstream overlay engine.

> **Note on naming:** The internal planning shorthand `/threat_reports`, `/intent_reports`, `/geofence_events`, `/intercept_reports`, `/system_health`, `/clutter_status`, and `/camera/image_color_corrected` is realized in the source code as `/threat/reports`, `/intent/reports`, `/geofence/violations`, `/reachability/warnings`, `/health/status`, `/clutter/status`, and `/camera/image_corrected` respectively. The strings above are the on-the-wire topic names. There is no `/radar/frame` topic in the system; `cuas_msgs/msg/RadarFrame` is defined in the message package but is not currently published.

## 4. External Interfaces

The only software interface that crosses the ground-station boundary is the Cursor-on-Target multicast emitted by `cot_publisher_node`.

| Field    | Value                                                                                  |
|----------|----------------------------------------------------------------------------------------|
| Protocol | UDP multicast                                                                          |
| Format   | Cursor-on-Target (CoT) XML, schema version 2.0                                         |
| Address  | 239.2.3.1                                                                              |
| Port     | 6969                                                                                   |
| TTL      | 32 (LAN-scoped; does not cross the site router)                                        |
| Rate     | 1 Hz for THREATENING/ENGAGED escalation states; full sweep of all active reports every 5 s; both gated by `/threat/reports` arrival (~20 Hz) |
| Status   | Latitude / longitude stubbed at `0.0` / `0.0` pending GPS / `georef_node` integration; `ce`=10.0, `le`=10.0 |

The CoT event UID format is `CUAS-TRACK-<track_id>`; `type` is `a-u-G` (atom, unknown, ground); `how` is `m-g`. The `<detail>` block carries `<track speed=… course=…/>`, `<status readiness="true"/>`, and a `<remarks>` element with `ThreatLevel`, `Quality`, `Class`, and `Esc` (escalation state). Course is computed from `atan2(position_x_m, position_y_m)` of the threat report.

## 5. Message Type Reference

### cuas_msgs/msg/RadarDetection

**Purpose:** A single radar return with position, radial velocity, and timestamp.
**Used in topic:** Embedded inside `RadarFrame.detections[]`. Not published on its own; the radar parser publishes `sensor_msgs/PointCloud2` directly on `/radar/detections`.
**Fields:**

| Field        | Type    | Units | Description                              |
|--------------|---------|-------|------------------------------------------|
| x            | float32 | m     | Cluster centroid in `radar_frame` X axis |
| y            | float32 | m     | Cluster centroid in `radar_frame` Y axis |
| z            | float32 | m     | Cluster centroid in `radar_frame` Z axis |
| velocity     | float32 | m/s   | Radial velocity (Doppler)                |
| timestamp_ns | int64   | ns    | Sensor timestamp                         |

### cuas_msgs/msg/RadarFrame

**Purpose:** A frame of radar returns with a header.
**Used in topic:** Defined for completeness; not currently published as a topic.
**Fields:**

| Field      | Type                | Units | Description                  |
|------------|---------------------|-------|------------------------------|
| header     | std_msgs/Header     | —     | Frame stamp and `frame_id`   |
| detections | RadarDetection[]    | —     | Variable-length return array |

### cuas_msgs/msg/FusedDetection

**Purpose:** A camera-radar fused detection with 3D position, radial velocity, COCO class, and pixel projection.
**Used in topic:** Embedded inside `FusedDetectionArray.detections[]` on `/fusion/detections`.
**Fields:**

| Field           | Type    | Units   | Description                                    |
|-----------------|---------|---------|------------------------------------------------|
| position_x_m    | float32 | m       | Position in `radar_frame` X axis               |
| position_y_m    | float32 | m       | Position in `radar_frame` Y axis               |
| position_z_m    | float32 | m       | Position in `radar_frame` Z axis               |
| velocity_mps    | float32 | m/s     | Radial velocity                                |
| class_label     | string  | —       | COCO numeric ID encoded as string              |
| confidence      | float32 | [0,1]   | Detection confidence                           |
| pixel_u         | float32 | px      | Projected image-plane column                   |
| pixel_v         | float32 | px      | Projected image-plane row                      |
| timestamp_ns    | int64   | ns      | Source timestamp                               |
| range_m         | float32 | m       | Slant range                                    |
| azimuth_deg     | float32 | deg     | Azimuth angle from boresight                   |
| bbox_width_px   | float32 | px      | YOLO bounding-box width                        |
| bbox_height_px  | float32 | px      | YOLO bounding-box height                       |

### cuas_msgs/msg/FusedDetectionArray

**Purpose:** Header + array container for `FusedDetection`.
**Used in topic:** /fusion/detections
**Fields:**

| Field      | Type              | Units | Description                  |
|------------|-------------------|-------|------------------------------|
| header     | std_msgs/Header   | —     | Frame stamp and `frame_id`   |
| detections | FusedDetection[]  | —     | Variable-length detections   |

### cuas_msgs/msg/Track

**Purpose:** Confirmed track with kinematic state, IMM mode probabilities, and adaptive prediction horizon.
**Used in topic:** Embedded inside `TrackArray.tracks[]` on `/tracks` and `/tracks/confirmed`.
**Fields:**

| Field                  | Type    | Units | Description                                                         |
|------------------------|---------|-------|---------------------------------------------------------------------|
| track_id               | uint32  | —     | Stable per-track identifier                                         |
| position_x_m           | float32 | m     | Position in `base_link` X axis                                      |
| position_y_m           | float32 | m     | Position in `base_link` Y axis                                      |
| position_z_m           | float32 | m     | Position in `base_link` Z axis                                      |
| velocity_mps           | float32 | m/s   | Speed magnitude                                                     |
| doppler_mps            | float32 | m/s   | Reported Doppler (currently 0.0F; reserved)                         |
| class_label            | string  | —     | Class string ("unknown" until classifier joins)                     |
| confidence             | float32 | [0,1] | Track confidence; decays on miss, gains on hit                      |
| track_state            | string  | —     | Track state name (TENTATIVE/CONFIRMED/OCCLUDED/REACQUIRED/LOST)     |
| timestamp_ns           | int64   | ns    | Last update timestamp                                               |
| is_maneuvering         | bool    | —     | True when CT model dominant for 3+ frames                           |
| imm_ct_probability     | float32 | [0,1] | Current CT model probability                                        |
| prediction_horizon_s   | float32 | s     | Adaptive per-track prediction horizon (3.0–10.0)                    |
| track_state_id         | uint8   | —     | Enum equivalent of `track_state`                                    |
| vx_mps                 | float32 | m/s   | Velocity X component in `base_link`                                 |
| vy_mps                 | float32 | m/s   | Velocity Y component in `base_link`                                 |

### cuas_msgs/msg/TrackArray

**Purpose:** Header + array container for `Track`.
**Used in topic:** /tracks, /tracks/confirmed
**Fields:**

| Field  | Type            | Units | Description                |
|--------|-----------------|-------|----------------------------|
| header | std_msgs/Header | —     | Frame stamp and `frame_id` |
| tracks | Track[]         | —     | Variable-length track set  |

### cuas_msgs/msg/ThreatReport

**Purpose:** Per-track threat assessment with escalation state and predicted impact.
**Used in topic:** Embedded inside `ThreatReportArray.reports[]` on `/threat/reports`.
**Fields:**

| Field                       | Type                       | Units | Description                                       |
|-----------------------------|----------------------------|-------|---------------------------------------------------|
| track_id                    | uint32                     | —     | Reference to the `Track` being assessed           |
| threat_level                | string                     | —     | UNKNOWN / BENIGN / SUSPECT / THREAT               |
| position_x_m                | float32                    | m     | Snapshot of track X                               |
| position_y_m                | float32                    | m     | Snapshot of track Y                               |
| position_z_m                | float32                    | m     | Snapshot of track Z                               |
| velocity_mps                | float32                    | m/s   | Snapshot of track speed                           |
| class_label                 | string                     | —     | COCO numeric ID                                   |
| confidence                  | float32                    | [0,1] | Track confidence                                  |
| track_state                 | string                     | —     | Track-state string                                |
| timestamp_ns                | int64                      | ns    | Source timestamp                                  |
| quality_score               | float32                    | [0,1] | Composite quality score                           |
| dwell_time_s                | float32                    | s     | Time the track has been at or near threat range   |
| escalation_state            | string                     | —     | UNKNOWN/BENIGN/SUSPECT/THREAT/THREATENING/ENGAGED |
| predicted_impact_x_m        | float32                    | m     | Forward impact projection X                       |
| predicted_impact_y_m        | float32                    | m     | Forward impact projection Y                       |
| exclusion_zones_violated    | geometry_msgs/Point[]      | —     | Zone-centre points violated by this track         |
| prediction_horizon_s        | float32                    | s     | Adaptive horizon (echoed back to the tracker)     |
| threat_level_id             | uint8                      | —     | Enum equivalent of `threat_level`                 |

### cuas_msgs/msg/ThreatReportArray

**Purpose:** Header + array container for `ThreatReport`.
**Used in topic:** /threat/reports
**Fields:**

| Field   | Type            | Units | Description                |
|---------|-----------------|-------|----------------------------|
| header  | std_msgs/Header | —     | Frame stamp and `frame_id` |
| reports | ThreatReport[]  | —     | Variable-length reports    |

### cuas_msgs/msg/IntentReport

**Purpose:** Behavioural intent classification per track.
**Used in topic:** Embedded inside `IntentReportArray.reports[]` on `/intent/reports`.
**Fields:**

| Field             | Type                    | Units | Description                                        |
|-------------------|-------------------------|-------|----------------------------------------------------|
| track_id          | uint32                  | —     | Track reference                                    |
| intent            | uint8                   | —     | Enum: APPROACHING/LOITERING/ORBITING/DEPARTING/TRANSITING |
| confidence        | float32                 | [0,1] | Intent confidence                                  |
| loiter_radius_m   | float32                 | m     | Estimated loiter radius                            |
| approach_rate_mps | float32                 | m/s   | Rate of range closure (negative = approaching)     |
| stamp             | builtin_interfaces/Time | —     | Publish timestamp                                  |

### cuas_msgs/msg/IntentReportArray

**Purpose:** Array container for `IntentReport` with publish stamp.
**Used in topic:** /intent/reports
**Fields:**

| Field   | Type                    | Units | Description           |
|---------|-------------------------|-------|-----------------------|
| reports | IntentReport[]          | —     | Variable-length array |
| stamp   | builtin_interfaces/Time | —     | Publish timestamp     |

### cuas_msgs/msg/PredictedTrack

**Purpose:** Per-track forward state prediction with full covariance and IMM mode weights.
**Used in topic:** /predicted_tracks/kinematic, /predicted_tracks/occlusion, /predicted_tracks
**Fields:**

| Field                  | Type             | Units | Description                                  |
|------------------------|------------------|-------|----------------------------------------------|
| header                 | std_msgs/Header  | —     | Frame stamp and `frame_id`                   |
| track_id               | uint32           | —     | Track reference                              |
| pos_x_m                | float64          | m     | Predicted X                                  |
| pos_y_m                | float64          | m     | Predicted Y                                  |
| pos_z_m                | float64          | m     | Predicted Z                                  |
| vel_x_mps              | float64          | m/s   | Predicted velocity X                         |
| vel_y_mps              | float64          | m/s   | Predicted velocity Y                         |
| vel_z_mps              | float64          | m/s   | Predicted velocity Z                         |
| covariance             | float64[36]      | —     | 6×6 row-major covariance over `[pos,vel]`    |
| bearing_deg            | float64          | deg   | Bearing in `base_link`                       |
| elevation_deg          | float64          | deg   | Elevation in `base_link`                     |
| model_weight_cv        | float64          | [0,1] | IMM weight, constant-velocity model          |
| model_weight_ca        | float64          | [0,1] | IMM weight, constant-acceleration model      |
| model_weight_ct        | float64          | [0,1] | IMM weight, coordinated-turn model           |
| track_state            | string           | —     | Track-state string                           |
| prediction_horizon_sec | float64          | s     | Horizon used for this prediction             |
| track_state_id         | uint8            | —     | Enum equivalent of `track_state`             |

### cuas_msgs/msg/TrajectoryWaypoints

**Purpose:** Per-track polyline of forecast waypoints with per-step uncertainty radii.
**Used in topic:** /trajectory_waypoints/kinematic, /trajectory_waypoints/occlusion, /trajectory_waypoints
**Fields:**

| Field                | Type            | Units | Description                          |
|----------------------|-----------------|-------|--------------------------------------|
| header               | std_msgs/Header | —     | Frame stamp and `frame_id`           |
| track_id             | uint32          | —     | Track reference                      |
| waypoints_x_m        | float64[]       | m     | Per-step X positions                 |
| waypoints_y_m        | float64[]       | m     | Per-step Y positions                 |
| waypoints_z_m        | float64[]       | m     | Per-step Z positions                 |
| timestamps_sec       | float64[]       | s     | Per-step time-from-now               |
| uncertainty_radii_m  | float64[]       | m     | Per-step 1σ uncertainty radius       |
| bearing_deg          | float64[]       | deg   | Per-step bearing                     |
| elevation_deg        | float64[]       | deg   | Per-step elevation                   |

### cuas_msgs/msg/GeofenceEvent

**Purpose:** Single geofence violation (entry / exit / dwell).
**Used in topic:** Embedded inside `GeofenceEventArray.events[]` on `/geofence/violations`.
**Fields:**

| Field      | Type                    | Units | Description                          |
|------------|-------------------------|-------|--------------------------------------|
| zone_id    | string                  | —     | Zone identifier from YAML config     |
| track_id   | uint32                  | —     | Track reference                      |
| event_type | uint8                   | —     | Enum: ENTRY / EXIT / DWELL           |
| distance_m | float32                 | m     | Distance to zone boundary            |
| stamp      | builtin_interfaces/Time | —     | Publish timestamp                    |

### cuas_msgs/msg/GeofenceEventArray

**Purpose:** Array container for `GeofenceEvent`.
**Used in topic:** /geofence/violations
**Fields:**

| Field   | Type                    | Units | Description           |
|---------|-------------------------|-------|-----------------------|
| events  | GeofenceEvent[]         | —     | Variable-length array |
| stamp   | builtin_interfaces/Time | —     | Publish timestamp     |

### cuas_msgs/msg/InterceptReport

**Purpose:** Per-track intercept feasibility with time-to-intercept and covariance ellipse.
**Used in topic:** Embedded inside `InterceptReportArray.reports[]` on `/reachability/warnings`.
**Fields:**

| Field                            | Type                    | Units | Description                          |
|----------------------------------|-------------------------|-------|--------------------------------------|
| track_id                         | uint32                  | —     | Track reference                      |
| time_to_intercept_s              | float32                 | s     | Forward TTI                          |
| intercept_confidence             | float32                 | [0,1] | Confidence of the TTI estimate       |
| covariance_ellipse_major_m       | float32                 | m     | Major axis of impact ellipse         |
| covariance_ellipse_minor_m       | float32                 | m     | Minor axis of impact ellipse         |
| covariance_ellipse_heading_rad   | float32                 | rad   | Major-axis heading                   |
| intercept_possible               | bool                    | —     | Feasibility flag                     |
| stamp                            | builtin_interfaces/Time | —     | Publish timestamp                    |

### cuas_msgs/msg/InterceptReportArray

**Purpose:** Array container for `InterceptReport`.
**Used in topic:** /reachability/warnings
**Fields:**

| Field   | Type                    | Units | Description           |
|---------|-------------------------|-------|-----------------------|
| reports | InterceptReport[]       | —     | Variable-length array |
| stamp   | builtin_interfaces/Time | —     | Publish timestamp     |

### cuas_msgs/msg/SystemHealth

**Purpose:** Per-subsystem and aggregate liveness with measured rates.
**Used in topic:** /health/status
**Fields:**

| Field             | Type                    | Units | Description                                 |
|-------------------|-------------------------|-------|---------------------------------------------|
| status            | uint8                   | —     | Aggregate (worst-of) status enum            |
| radar_status      | uint8                   | —     | Radar liveness enum (OK/DEGRADED/FAULT)     |
| camera_status     | uint8                   | —     | Camera liveness enum                        |
| tracker_status    | uint8                   | —     | Tracker liveness enum                       |
| classifier_status | uint8                   | —     | Threat-classifier liveness enum             |
| predictor_status  | uint8                   | —     | Predictor liveness enum                     |
| radar_hz          | float32                 | Hz    | Measured radar topic rate                   |
| tracker_hz        | float32                 | Hz    | Measured tracker rate                       |
| classifier_hz     | float32                 | Hz    | Measured classifier rate                    |
| predictor_hz      | float32                 | Hz    | Measured predictor rate                     |
| stamp             | builtin_interfaces/Time | —     | Publish timestamp                           |

### cuas_msgs/msg/SystemHealthArray

**Purpose:** Array container for `SystemHealth` (reserved for multi-station aggregation).
**Used in topic:** Defined for completeness; not currently published as a topic.
**Fields:**

| Field   | Type                    | Units | Description           |
|---------|-------------------------|-------|-----------------------|
| reports | SystemHealth[]          | —     | Variable-length array |
| stamp   | builtin_interfaces/Time | —     | Publish timestamp     |

### cuas_msgs/msg/ClutterStatus

**Purpose:** Clutter-map learning state and current occupancy ratio.
**Used in topic:** /clutter/status
**Fields:**

| Field            | Type                    | Units | Description                              |
|------------------|-------------------------|-------|------------------------------------------|
| state            | uint8                   | —     | Enum: LEARNING / LIVE                    |
| frames_learned   | uint32                  | —     | Frames absorbed into the grid            |
| frames_required  | uint32                  | —     | Frames required to leave learning state  |
| occupancy_ratio  | float32                 | [0,1] | Fraction of cells classified as occupied |
| stamp            | builtin_interfaces/Time | —     | Publish timestamp                        |

## 6. QoS Policy Summary

No node sets a custom `rclcpp::QoS`. Every publisher and subscriber is constructed with a plain integer history depth, so the active profile on every topic is the ROS 2 default: **Reliable, Volatile, Keep Last `<depth>`**. The depths declared in source are tabulated below.

| Topic                                  | Reliability | Durability | History   | Depth |
|----------------------------------------|-------------|------------|-----------|-------|
| /radar/detections                      | Reliable    | Volatile   | Keep Last | 10    |
| /radar/filtered                        | Reliable    | Volatile   | Keep Last | 10    |
| /clutter/status                        | Reliable    | Volatile   | Keep Last | 10    |
| /camera/image_raw                      | Reliable    | Volatile   | Keep Last | 10    |
| /camera/image_corrected                | Reliable    | Volatile   | Keep Last | 5     |
| /inference/detections                  | Reliable    | Volatile   | Keep Last | 5     |
| /fusion/detections                     | Reliable    | Volatile   | Keep Last | 5     |
| /tracks                                | Reliable    | Volatile   | Keep Last | 10    |
| /tracks/confirmed                      | Reliable    | Volatile   | Keep Last | 5     |
| /threat/reports                        | Reliable    | Volatile   | Keep Last | 5     |
| /predicted_tracks/kinematic            | Reliable    | Volatile   | Keep Last | 10    |
| /predicted_tracks/occlusion            | Reliable    | Volatile   | Keep Last | 10    |
| /predicted_tracks                      | Reliable    | Volatile   | Keep Last | 10    |
| /trajectory_waypoints/kinematic        | Reliable    | Volatile   | Keep Last | 10    |
| /trajectory_waypoints/occlusion        | Reliable    | Volatile   | Keep Last | 10    |
| /trajectory_waypoints                  | Reliable    | Volatile   | Keep Last | 10    |
| /intent/reports                        | Reliable    | Volatile   | Keep Last | 10    |
| /geofence/violations                   | Reliable    | Volatile   | Keep Last | 10    |
| /reachability/warnings                 | Reliable    | Volatile   | Keep Last | 10    |
| /health/status                         | Reliable    | Volatile   | Keep Last | 10    |
| /visualization/track_markers           | Reliable    | Volatile   | Keep Last | 10    |
| /visualization/trajectory_markers      | Reliable    | Volatile   | Keep Last | 10    |
| /visualization/uncertainty_markers     | Reliable    | Volatile   | Keep Last | 10    |
| /visualization/track_labels            | Reliable    | Volatile   | Keep Last | 10    |
| /camera/annotated                      | Reliable    | Volatile   | Keep Last | 1     |
| /camera/annotated_enhanced             | Reliable    | Volatile   | Keep Last | 5     |

## 7. Coordinate Frame Contract

The `Header.frame_id` strings actually written by the producing nodes (verified by `grep frame_id` in `src/cuas_fusion/src/`) are tabulated below. Topics that are array containers without a `Header` field are listed as N/A; they carry a `stamp` field instead.

| Topic                              | frame_id      | Notes                                                            |
|------------------------------------|---------------|------------------------------------------------------------------|
| /radar/detections                  | radar_frame   | Set in `radar_parser_node.cpp` and `sim_radar_node.cpp`          |
| /radar/filtered                    | radar_frame   | Forwarded from input by `clutter_map_node`                       |
| /camera/image_raw                  | camera        | Set in `camera_node.cpp`                                         |
| /camera/image_corrected            | camera        | Forwarded from input                                             |
| /inference/detections              | camera        | Forwarded from input image                                       |
| /fusion/detections                 | radar_frame   | Set explicitly in `fusion_node.cpp`                              |
| /tracks                            | base_link     | Set explicitly in `imm_tracker_node.cpp`                         |
| /tracks/confirmed                  | (unset)       | `track_manager_node` does not set `frame_id`                     |
| /threat/reports                    | base_link     | Forwarded from input track                                       |
| /predicted_tracks/kinematic        | base_link     | Set explicitly in `kinematic_predictor_node.cpp`                 |
| /predicted_tracks/occlusion        | base_link     | Set explicitly in `occlusion_predictor_node.cpp`                 |
| /predicted_tracks                  | base_link     | Forwarded by `prediction_mux_node`                               |
| /trajectory_waypoints/kinematic    | base_link     | Forwarded from associated `PredictedTrack`                       |
| /trajectory_waypoints/occlusion    | base_link     | Forwarded from associated `PredictedTrack`                       |
| /trajectory_waypoints              | base_link     | Forwarded by `prediction_mux_node`                               |
| /visualization/track_markers       | base_link     | Set on every `Marker` in `cuas_visualizer_node.cpp`              |
| /visualization/trajectory_markers  | base_link     | Set on every `Marker` in `cuas_visualizer_node.cpp`              |
| /visualization/uncertainty_markers | base_link     | Set on every `Marker` in `cuas_visualizer_node.cpp`              |
| /visualization/track_labels        | base_link     | Set on every `Marker` in `cuas_visualizer_node.cpp`              |
| /camera/annotated                  | camera        | Forwarded from input image                                       |
| /camera/annotated_enhanced         | camera        | Forwarded from input image                                       |
| /clutter/status                    | N/A           | `ClutterStatus` has no `Header`; carries `stamp` only            |
| /intent/reports                    | N/A           | `IntentReportArray` has no `Header`; carries `stamp` only        |
| /geofence/violations               | N/A           | `GeofenceEventArray` has no `Header`; carries `stamp` only       |
| /reachability/warnings             | N/A           | `InterceptReportArray` has no `Header`; carries `stamp` only     |
| /health/status                     | N/A           | `SystemHealth` has no `Header`; carries `stamp` only             |

The static frame tree is rooted at `base_link`. The launch file publishes a static `base_link → radar_frame` identity transform via `tf2_ros::static_transform_publisher`. `fusion_node` publishes a static `radar_frame → camera_frame` transform with translation `(x = -0.0075 m, y = 0.017 m, z = -0.079 m)` and an identity rotation. The `frame_id` string `camera` set by `camera_node` is the legacy name used by sensor-side messages and is treated as an alias of `camera_frame` for projection purposes.

## 8. Interface Change Log

The following interface-affecting changes are recorded in `CHANGELOG.md`. Only entries that introduced, removed, or modified a topic name, message field, or external interface are reproduced here.

### 0.8.0-alpha — current

- Added topic `/intent/reports` carrying `cuas_msgs/IntentReportArray` (intent classifier node).
- Added topic `/clutter/status` carrying `cuas_msgs/ClutterStatus`, replacing the prior string-typed clutter publisher (see 0.5.0 entry below).

### 0.7.0-alpha

- Added field `track_state_id` (uint8) to `cuas_msgs/Track` and `cuas_msgs/PredictedTrack`. The pre-existing `track_state` string field was retained at the message boundary but is no longer authoritative; downstream consumers must compare `track_state_id` against the `cuas::track_state::*` enum.
- Added field `threat_level_id` (uint8) to `cuas_msgs/ThreatReport`. The string-valued `threat_level` field was retained but downgraded in the same way.

### 0.6.0-alpha

- Added topics `/geofence/violations` (`cuas_msgs/GeofenceEventArray`) and `/reachability/warnings` (`cuas_msgs/InterceptReportArray`).
- Added field `prediction_horizon_s` to `cuas_msgs/Track` and to `cuas_msgs/ThreatReport`. The IMM tracker reads the per-track horizon from the latest `ThreatReportArray` and stamps it onto every published `Track`, removing the predictor's prior need to subscribe to threat state directly.
- Added field `exclusion_zones_violated` (`geometry_msgs/Point[]`) to `cuas_msgs/ThreatReport`.

### 0.5.0-alpha

- Added topic `/health/status` carrying `cuas_msgs/SystemHealth` (health monitor node).
- Replaced the prior string-typed clutter status publisher with the typed `cuas_msgs/ClutterStatus` message on `/clutter/status`.
- Sim-radar SIL substitute introduced; same `/radar/detections` topic and `radar_frame` `frame_id` so downstream contracts are unchanged.

### 0.4.0-alpha

- Added fields `is_maneuvering` (bool) and `imm_ct_probability` (float32) to `cuas_msgs/Track` to expose IMM-derived maneuver state.
- Added fields `vx_mps` and `vy_mps` (float32, in `base_link`) to `cuas_msgs/Track` for downstream consumers that need the velocity vector rather than just the speed magnitude.
- Visualizer label rendering was split into two tiers (CONFIRMED full-weight, TENTATIVE reduced-alpha) with no message-boundary impact.

### 0.3.0-alpha

- Introduced topic `/inference/detections` (`vision_msgs/Detection2DArray`) from the YOLOv8s INT8 TensorRT inference node.
- Introduced topic `/fusion/detections` (`cuas_msgs/FusedDetectionArray`) from the camera-radar pinhole-projection fusion node.
- Introduced the threat classifier escalation FSM and the `escalation_state` field on `cuas_msgs/ThreatReport`.
- Introduced the Cursor-on-Target UDP multicast external interface to `239.2.3.1:6969` from `cot_publisher_node`. CoT events are emitted at 1 Hz for THREATENING/ENGAGED escalation states with a full sweep every 5 s. Latitude / longitude remain stubbed at `0.0` / `0.0` until a `georef_node` is fielded.

### 0.2.0-alpha

- Introduced visualization topics `/visualization/track_markers`, `/visualization/trajectory_markers`, `/visualization/uncertainty_markers`, and `/visualization/track_labels`, all `visualization_msgs/MarkerArray`, all in `frame_id = "base_link"`.
- Introduced `/predicted_tracks/{kinematic,occlusion}` and `/trajectory_waypoints/{kinematic,occlusion}` from the kinematic and occlusion predictors, plus the arbitrated `/predicted_tracks` and `/trajectory_waypoints` from `prediction_mux_node`.
- Introduced the downstream `cuas_overlay_node` that consumes `/camera/annotated` and republishes `/camera/annotated_enhanced` with the trajectory arc.

### 0.1.0-alpha

- Initial release. Topics: `/radar/detections` (`sensor_msgs/PointCloud2`, `frame_id = "radar_frame"`), `/tracks` (`cuas_msgs/TrackArray`, `frame_id = "base_link"`). Track-state machine, Hungarian assignment, and the ROS 2 Humble workspace established.
