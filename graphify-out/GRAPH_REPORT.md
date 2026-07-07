# Graph Report - /home/zork/ProjectHailMarry  (2026-04-27)

## Corpus Check
- 124 files · ~386,216 words
- Verdict: corpus is large enough that graph structure adds value.

## Summary
- 775 nodes · 1054 edges · 56 communities detected
- Extraction: 78% EXTRACTED · 22% INFERRED · 0% AMBIGUOUS · INFERRED: 229 edges (avg confidence: 0.8)
- Token cost: 0 input · 0 output

## Community Hubs (Navigation)
- [[_COMMUNITY_Release History (CHANGELOG)|Release History (CHANGELOG)]]
- [[_COMMUNITY_Threat Classifier Internals|Threat Classifier Internals]]
- [[_COMMUNITY_Kalman Transition Matrices|Kalman Transition Matrices]]
- [[_COMMUNITY_Drivers & Sensor Nodes|Drivers & Sensor Nodes]]
- [[_COMMUNITY_Camera Calibration Scripts|Camera Calibration Scripts]]
- [[_COMMUNITY_ROS Node Executables|ROS Node Executables]]
- [[_COMMUNITY_Guided Camera Calibration|Guided Camera Calibration]]
- [[_COMMUNITY_Geometric & Threat Helpers|Geometric & Threat Helpers]]
- [[_COMMUNITY_Fusion + Track Association|Fusion + Track Association]]
- [[_COMMUNITY_Clutter Map Internals|Clutter Map Internals]]
- [[_COMMUNITY_Common Types & Containers|Common Types & Containers]]
- [[_COMMUNITY_Geofence Engine + Tests|Geofence Engine + Tests]]
- [[_COMMUNITY_ROS2 Launch Files|ROS2 Launch Files]]
- [[_COMMUNITY_Visualizer & Image Adapter|Visualizer & Image Adapter]]
- [[_COMMUNITY_IMM Tracker & Motion Models|IMM Tracker & Motion Models]]
- [[_COMMUNITY_Health Monitor Module|Health Monitor Module]]
- [[_COMMUNITY_6D Kalman Math (CA)|6D Kalman Math (CA)]]
- [[_COMMUNITY_Color Correction Engine|Color Correction Engine]]
- [[_COMMUNITY_Overlay Drawing Primitives|Overlay Drawing Primitives]]
- [[_COMMUNITY_TRT Detector & Helpers|TRT Detector & Helpers]]
- [[_COMMUNITY_Radar Test Script|Radar Test Script]]
- [[_COMMUNITY_Intent Classification Types|Intent Classification Types]]
- [[_COMMUNITY_Sim Radar Targets|Sim Radar Targets]]
- [[_COMMUNITY_Timestamp Associator|Timestamp Associator]]
- [[_COMMUNITY_Radar Sim Entry|Radar Sim Entry]]
- [[_COMMUNITY_Reachability Engine|Reachability Engine]]
- [[_COMMUNITY_Threat Classifier Types|Threat Classifier Types]]
- [[_COMMUNITY_Geofence Engine Header|Geofence Engine Header]]
- [[_COMMUNITY_Jitter Logger Pipeline|Jitter Logger Pipeline]]
- [[_COMMUNITY_ReachabilityEngine triple|ReachabilityEngine triple]]
- [[_COMMUNITY_IntentClassifier triple|IntentClassifier triple]]
- [[_COMMUNITY_RadarDriver triple|RadarDriver triple]]
- [[_COMMUNITY_FusionEngine triple|FusionEngine triple]]
- [[_COMMUNITY_TimestampAssociator triple|TimestampAssociator triple]]
- [[_COMMUNITY_KalmanCV triple|KalmanCV triple]]
- [[_COMMUNITY_KalmanCA triple|KalmanCA triple]]
- [[_COMMUNITY_KalmanCT triple|KalmanCT triple]]
- [[_COMMUNITY_Track triple|Track triple]]
- [[_COMMUNITY_HungarianSolver triple|HungarianSolver triple]]
- [[_COMMUNITY_Wgs84Transform triple|Wgs84Transform triple]]
- [[_COMMUNITY_KinematicPredictor triple|KinematicPredictor triple]]
- [[_COMMUNITY_OcclusionPredictor triple|OcclusionPredictor triple]]
- [[_COMMUNITY_JitterLogger Callbacks|JitterLogger Callbacks]]
- [[_COMMUNITY_ColorCorrectEngine pair|ColorCorrectEngine pair]]
- [[_COMMUNITY_RadarSim pair|RadarSim pair]]
- [[_COMMUNITY_HealthMonitor pair|HealthMonitor pair]]
- [[_COMMUNITY_ClutterMap pair|ClutterMap pair]]
- [[_COMMUNITY_CuasVisualizer pair|CuasVisualizer pair]]
- [[_COMMUNITY_Common TUs pair|Common TUs pair]]
- [[_COMMUNITY_Georef placeholder pair|Georef placeholder pair]]
- [[_COMMUNITY_now_ns|now_ns]]
- [[_COMMUNITY_Threat class enums|Threat class enums]]
- [[_COMMUNITY_Wgs84Transform isolated|Wgs84Transform isolated]]
- [[_COMMUNITY_radar_driver TU|radar_driver TU]]
- [[_COMMUNITY_Hungarian (Jonker-Volgenant)|Hungarian (Jonker-Volgenant)]]
- [[_COMMUNITY_CUAS_STRICT_FLAGS|CUAS_STRICT_FLAGS]]

## God Nodes (most connected - your core abstractions)
1. `shutdown()` - 34 edges
2. `cuas_msgs rosidl interface package` - 33 edges
3. `init()` - 32 edges
4. `cuas_fusion ament_cmake package` - 24 edges
5. `ProjectHailMary Coding Standard (JSF AV C++ + MISRA C++:2023 + DO-178C awareness)` - 18 edges
6. `Track` - 14 edges
7. `/tracks` - 11 edges
8. `Rule: Math classes ROS-free; nodes are thin sub/call/pub wrappers` - 11 edges
9. `close()` - 10 edges
10. `HealthMonitorNode` - 9 edges

## Surprising Connections (you probably didn't know these)
- `Color Balance Verification` --references--> `ColorCorrectEngine`  [INFERRED]
  test/camera_calibration/verify_final.png → src/cuas_fusion/include/cuas_fusion/color_correct_engine.hpp
- `Warm White Balance` --conceptually_related_to--> `ColorCorrectEngine`  [INFERRED]
  test/camera_calibration/final_warm.png → src/cuas_fusion/include/cuas_fusion/color_correct_engine.hpp
- `Proper White Balance Calibration` --conceptually_related_to--> `ColorCorrectEngine`  [INFERRED]
  test/camera_calibration/final_proper.png → src/cuas_fusion/include/cuas_fusion/color_correct_engine.hpp
- `Low-Light Indoor Capture` --references--> `CameraDriver`  [INFERRED]
  test/camera_calibration/verify_final.png → src/cuas_fusion/include/cuas_fusion/drivers/camera_driver.hpp
- `JitterLogger.yolo_cb` --semantically_similar_to--> `BoundingBox`  [INFERRED] [semantically similar]
  scripts/jitter_log.py → src/cuas_fusion/include/cuas_fusion/common/types.hpp

## Hyperedges (group relationships)
- **JSF AV / IR-3 typed numeric state IDs replacing strings** — intent_ids_intent_class, track_state_ids_track_state, track_state_ids_threat_level [EXTRACTED 0.95]
- **Radar simulation classes and supporting types** — radar_sim_radarsim, sim_radar_simradar, clutter_map_cluttermap, radar_driver_radardriver [INFERRED 0.75]
- **Per-track threat/intent/reachability assessment pipeline** — threat_classifier_threatclassifier, intent_classifier_intentclassifier, reachability_engine_reachabilityengine, health_monitor_healthmonitor [INFERRED 0.70]
- **IMM mixes CV/CA/CT Kalman models** — imm_filter_immfilter, kalman_cv_kalmancv, kalman_ca_kalmanca, kalman_ct_kalmanct [EXTRACTED 1.00]
- **Kinematic-only launch nodes** — exec_radar_parser_node, exec_imm_tracker_node, exec_kinematic_predictor_node [EXTRACTED 1.00]
- **Full launch perception+tracking+prediction chain** — exec_radar_parser_node, exec_imm_tracker_node, exec_fusion_node, exec_inference_node, exec_cuas_visualizer_node [EXTRACTED 1.00]
- **Tracks topic feeds geofence/reachability/intent/threat** — geofence_node_class, reachability_node_class, intent_classifier_node_class [EXTRACTED 1.00]
- **ThreatReports prioritize geofence and reachability** — threat_classifier_node_class, geofence_node_class, reachability_node_class [EXTRACTED 0.95]
- **Sim radar -> clutter map -> downstream pipeline** — sim_radar_node_class, clutter_map_node_class, health_monitor_node_class [INFERRED 0.85]
- **IMM filter combines CV/CA/CT motion models** — imm_filter_class, kalman_cv_class, kalman_ca_class, kalman_ct_class [EXTRACTED 1.00]
- **Prediction mux fuses kinematic and occlusion predictor outputs** — prediction_mux_node_class, kinematic_predictor_node_class, occlusion_predictor_node_class [EXTRACTED 1.00]
- **Radar+camera fusion pipeline: parser, inference, fusion engine** — radar_parser_node_class, inference_node_class, fusion_node_class, fusion_engine_class [INFERRED 0.85]
- **test/unit GoogleTest suite** — test_imm_filter, test_kalman_cv, test_threat_classifier, test_timestamp_associator, test_wgs84_transform [INFERRED 0.85]
- **cuas_fusion engine unit tests** — test_clutter_map, test_color_correct_engine, test_geofence_engine, test_health_monitor, test_intent_classifier, test_reachability_engine [INFERRED 0.85]
- **Visualization nodes consuming Track/Threat/Trajectory streams** — cuas_overlay_node, cuas_visualizer_node, capture_node, overlay_engine [INFERRED 0.80]
- **ProjectHailMary alpha release history (v0.1 to v0.8)** — changelog_v0_1_0_alpha, changelog_v0_3_0_alpha, changelog_v0_8_0_alpha [EXTRACTED 0.90]
- **JSF AV / MISRA C++:2023 enforcement (flags + rules + deviations)** — coding_standard_root, cuas_safety_flags, deviation_dev_001 [EXTRACTED 0.90]
- **Jitter telemetry corpus (3 runs, shared schema)** — jitter_log_main, jitter_log_centroid, jitter_log_centroid_run2 [INFERRED 0.90]

## Communities

### Community 0 - "Release History (CHANGELOG)"
Cohesion: 0.04
Nodes (91): ProjectHailMary CHANGELOG, v0.1.0-alpha: Radar serial driver + TLV/DBSCAN + IMM + ROS 2 Humble workspace, v0.2.0-alpha: RViz2 viz + KinematicPredictor + OcclusionPredictor + PredictionMux + Overlay, v0.3.0-alpha: YOLOv8s INT8 TRT 35Hz + camera-radar fusion + threat FSM + CoT/ATAK, v0.4.0-alpha: Hungarian + Mahalanobis + IMM CV+CA+CT + Doppler centroid, v0.5.0-alpha: Health Monitor + Sim Radar + Clutter Map + ClutterStatus msg, v0.6.0-alpha: Geofence + Reachability + Adaptive Horizon, v0.7.0-alpha: JSF AV / MISRA C++:2023 compliance pass (144 violations across 20 files) (+83 more)

### Community 1 - "Threat Classifier Internals"
Cohesion: 0.03
Nodes (41): init(), ClassifierNode, main(), ThreatClassifier, main(), SimRadarNode, FusionNode, main() (+33 more)

### Community 2 - "Kalman Transition Matrices"
Cohesion: 0.05
Nodes (33): getCaTransitionMatrix(), getCovariance(), getCtTransitionMatrix(), getCvTransitionMatrix(), getMixedF(), getMixedQ(), getModelState(), getState() (+25 more)

### Community 3 - "Drivers & Sensor Nodes"
Cohesion: 0.07
Nodes (51): CameraDriver, CameraNode, CaptureNode, ClutterMap, ClutterMapNode, ColorCorrectEngine, CotPublisherNode (Cursor-on-Target XML over UDP multicast 239.2.3.1:6969), CuasColorCorrectNode (+43 more)

### Community 4 - "Camera Calibration Scripts"
Cohesion: 0.06
Nodes (30): calibrate_camera (scripts), calibrate_camera (test), capture_raw(), main(), process_frame(), Replicate the C++ camera_driver.cpp pipeline exactly., camera_driver, CameraDriver (+22 more)

### Community 5 - "ROS Node Executables"
Cohesion: 0.07
Nodes (38): CuasVisualizerNode, camera_node, classifier_node, clutter_map_node, cot_publisher_node, cuas_color_correct_node, cuas_overlay_node, cuas_visualizer_node (+30 more)

### Community 6 - "Guided Camera Calibration"
Cohesion: 0.09
Nodes (28): camera_calibration_result.yaml output, GuidedCalibrationNode._compute_spread, GuidedCalibrationNode, GuidedCalibrationNode.image_cb, GuidedCalibrationNode.run_calibration, STEPS calibration plan, CameraDriver, ColorCorrectEngine (+20 more)

### Community 7 - "Geometric & Threat Helpers"
Cohesion: 0.09
Nodes (14): classify(), predicted_impact(), predicted_range(), pruneStale(), erase_if(), FixedMap, FixedVector, IntentClassifier (+6 more)

### Community 8 - "Fusion + Track Association"
Cohesion: 0.11
Nodes (14): FusionEngine, HungarianSolver, FusionPipelineIntegrationTest, /tracks/confirmed, TrackManager, TrackerNode, solve(), allocateSlot() (+6 more)

### Community 9 - "Clutter Map Internals"
Cohesion: 0.16
Nodes (16): ClutterMap, add_frame(), cell_index(), ClutterMap(), is_clutter(), is_learned(), ClutterMapNode, main() (+8 more)

### Community 10 - "Common Types & Containers"
Cohesion: 0.12
Nodes (20): ClutterMap, FixedVector, fixed-width type aliases (int32_t/float32_t etc.), GeofenceEngine, GeofenceEventType, GeofenceResult, ZoneConfig, ZoneShape (+12 more)

### Community 11 - "Geofence Engine + Tests"
Cohesion: 0.17
Nodes (13): GeofenceEngine, evaluate(), load_zones(), min_distance_to_polygon_edges(), point_in_circle(), point_in_polygon(), zone_count(), GeofenceNode (+5 more)

### Community 12 - "ROS2 Launch Files"
Cohesion: 0.12
Nodes (9): generate_launch_description(), generate_launch_description(), generate_launch_description(), generate_launch_description(), generate_launch_description(), generate_launch_description(), generate_launch_description(), # WHY: replaces hardware radar with SimRadar for software-in-the-loop testing (+1 more)

### Community 13 - "Visualizer & Image Adapter"
Cohesion: 0.16
Nodes (8): rosImageToBgr(), formatLabeled(), imageCallback(), main(), parseClassId(), publishMarkers(), threatColor(), trackPaletteColor()

### Community 14 - "IMM Tracker & Motion Models"
Cohesion: 0.21
Nodes (16): ImmFilter (Interacting Multiple Model with CV/CA/CT mixing), IMMTracker (per-track state machine: TENTATIVE/CONFIRMED/OCCLUDED with confidence decay, CT-driven maneuver), KalmanCA (constant acceleration Kalman filter), KalmanCT (coordinated turn Kalman filter with omega-rate state), KalmanCV (constant velocity Kalman filter), KinematicPredictor (forward-propagation trajectory with IMM blending), KinematicPredictorNode (publishes /predicted_tracks/kinematic and waypoints), OcclusionPredictor (ghost-track propagation with covariance inflation, Mahalanobis reacquire) (+8 more)

### Community 15 - "Health Monitor Module"
Cohesion: 0.17
Nodes (9): HealthMonitor, HealthMonitorNode, main(), overall_status(), query(), refresh_status(), set_expected_hz(), HealthMonitorTest (+1 more)

### Community 16 - "6D Kalman Math (CA)"
Cohesion: 0.16
Nodes (7): predictCaStep(), predictCtStep(), predictCvStep(), propagateForward(), configure(), OcclusionPredictorNode, propagateGhost()

### Community 17 - "Color Correction Engine"
Cohesion: 0.2
Nodes (10): ColorCorrectEngine, apply_bgr(), build_lut(), ColorCorrectEngine(), scale_clamp(), CuasColorCorrectNode, main(), ColorCorrectEngineTest (+2 more)

### Community 18 - "Overlay Drawing Primitives"
Cohesion: 0.29
Nodes (9): draw_scaled_label(), draw_trajectory_arc(), is_in_bounds(), OverlayEngine, project_to_image(), render(), threat_color(), threat_level_from_string() (+1 more)

### Community 19 - "TRT Detector & Helpers"
Cohesion: 0.29
Nodes (7): now_ns(), infer(), init(), iou(), nms(), postprocess(), preprocess()

### Community 20 - "Radar Test Script"
Cohesion: 0.24
Nodes (10): test_radar.check_ports, test_radar.diagnose_failure, test_radar.drain_response, test_radar.listen_data_port, test_radar.load_config, test_radar.main, test_radar.probe_firmware, radar_profile.cfg input (+2 more)

### Community 22 - "Intent Classification Types"
Cohesion: 0.22
Nodes (9): IntentClassifier, IntentInput, IntentResult, intent_class numeric IDs, InterceptResult, ReachabilityEngine, ReachabilityTrackState, track_state numeric IDs (+1 more)

### Community 23 - "Sim Radar Targets"
Cohesion: 0.29
Nodes (3): gaussian_sample(), getPoints(), SimRadar

### Community 24 - "Timestamp Associator"
Cohesion: 0.36
Nodes (6): addCameraFrame(), findBestMatch(), TimestampAssociatorTest, TimestampAssociator, make_dummy_frame(), TEST()

### Community 25 - "Radar Sim Entry"
Cohesion: 0.33
Nodes (3): gaussian_sample(), generate(), set_target()

### Community 26 - "Reachability Engine"
Cohesion: 0.38
Nodes (5): ReachabilityEngine, compute(), ReachabilityEngineTest, make_state(), TEST()

### Community 27 - "Threat Classifier Types"
Cohesion: 0.29
Nodes (7): TRACK_MAX_TRACKS, FixedMap, ClassificationResult, EscalationState, ThreatClassifier, threat_level numeric IDs, ThreatLevel enum

### Community 28 - "Geofence Engine Header"
Cohesion: 0.5
Nodes (1): GeofenceEngine

### Community 30 - "Jitter Logger Pipeline"
Cohesion: 0.5
Nodes (4): JitterLogger, /fusion/detections topic, /inference/detections topic, FusedDetection

### Community 31 - "ReachabilityEngine triple"
Cohesion: 0.67
Nodes (1): ReachabilityEngine

### Community 32 - "IntentClassifier triple"
Cohesion: 0.67
Nodes (1): IntentClassifier

### Community 33 - "RadarDriver triple"
Cohesion: 0.67
Nodes (1): RadarDriver

### Community 34 - "FusionEngine triple"
Cohesion: 0.67
Nodes (1): FusionEngine

### Community 35 - "TimestampAssociator triple"
Cohesion: 0.67
Nodes (1): TimestampAssociator

### Community 36 - "KalmanCV triple"
Cohesion: 0.67
Nodes (1): KalmanCV

### Community 37 - "KalmanCA triple"
Cohesion: 0.67
Nodes (1): KalmanCA

### Community 38 - "KalmanCT triple"
Cohesion: 0.67
Nodes (1): KalmanCT

### Community 39 - "Track triple"
Cohesion: 0.67
Nodes (1): Track

### Community 40 - "HungarianSolver triple"
Cohesion: 0.67
Nodes (1): HungarianSolver

### Community 41 - "Wgs84Transform triple"
Cohesion: 0.67
Nodes (1): Wgs84Transform

### Community 42 - "KinematicPredictor triple"
Cohesion: 0.67
Nodes (1): KinematicPredictor

### Community 43 - "OcclusionPredictor triple"
Cohesion: 0.67
Nodes (1): OcclusionPredictor

### Community 44 - "JitterLogger Callbacks"
Cohesion: 0.67
Nodes (3): JitterLogger.fused_cb, JitterLogger.yolo_cb, BoundingBox

### Community 45 - "ColorCorrectEngine pair"
Cohesion: 1.0
Nodes (1): ColorCorrectEngine

### Community 46 - "RadarSim pair"
Cohesion: 1.0
Nodes (1): RadarSim

### Community 47 - "HealthMonitor pair"
Cohesion: 1.0
Nodes (1): HealthMonitor

### Community 48 - "ClutterMap pair"
Cohesion: 1.0
Nodes (1): ClutterMap

### Community 49 - "CuasVisualizer pair"
Cohesion: 1.0
Nodes (1): CuasVisualizerNode

### Community 50 - "Common TUs pair"
Cohesion: 1.0
Nodes (2): clock translation unit, types translation unit

### Community 51 - "Georef placeholder pair"
Cohesion: 1.0
Nodes (2): georef_node (placeholder executable), wgs84_transform translation unit (header-only impl)

### Community 59 - "now_ns"
Cohesion: 1.0
Nodes (1): now_ns

### Community 60 - "Threat class enums"
Cohesion: 1.0
Nodes (1): THREAT_DRONE_CLASSES / THREAT_BENIGN_CLASSES

### Community 61 - "Wgs84Transform isolated"
Cohesion: 1.0
Nodes (1): Wgs84Transform

### Community 62 - "radar_driver TU"
Cohesion: 1.0
Nodes (1): radar_driver translation unit

### Community 63 - "Hungarian (Jonker-Volgenant)"
Cohesion: 1.0
Nodes (1): HungarianSolver (Jonker-Volgenant O(n^3) assignment over square-padded buffer)

### Community 64 - "CUAS_STRICT_FLAGS"
Cohesion: 1.0
Nodes (1): CUAS_STRICT_FLAGS (adds -Wsign-conversion/-Wconversion for non-ROS targets)

## Ambiguous Edges - Review These
- `cuas_fusion ament_cmake package` → `latency_budget.md (empty placeholder)`  [AMBIGUOUS]
  docs/latency_budget.md · relation: references
- `cuas_fusion ament_cmake package` → `architecture_diagram.md (empty placeholder)`  [AMBIGUOUS]
  docs/architecture_diagram.md · relation: references
- `cuas_msgs rosidl interface package` → `ICD.md (Interface Control Document - empty placeholder)`  [AMBIGUOUS]
  docs/ICD.md · relation: references

## Knowledge Gaps
- **127 isolated node(s):** `# WHY: replaces hardware radar with SimRadar for software-in-the-loop testing`, `ColorCorrectEngine`, `GeofenceEngine`, `RadarSim`, `HealthMonitor` (+122 more)
  These have ≤1 connection - possible missing edges or undocumented components.
- **Thin community `Geofence Engine Header`** (4 nodes): `GeofenceEngine`, `GeofenceResult()`, `ZoneConfig()`, `geofence_engine.hpp`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `ReachabilityEngine triple`** (3 nodes): `ReachabilityEngine`, `.ReachabilityEngine()`, `reachability_engine.hpp`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `IntentClassifier triple`** (3 nodes): `IntentClassifier`, `.IntentClassifier()`, `intent_classifier.hpp`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `RadarDriver triple`** (3 nodes): `RadarDriver`, `.RadarDriver()`, `radar_driver.hpp`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `FusionEngine triple`** (3 nodes): `FusionEngine`, `.FusionEngine()`, `fusion_engine.hpp`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `TimestampAssociator triple`** (3 nodes): `TimestampAssociator`, `.TimestampAssociator()`, `timestamp_associator.hpp`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `KalmanCV triple`** (3 nodes): `KalmanCV`, `.KalmanCV()`, `kalman_cv.hpp`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `KalmanCA triple`** (3 nodes): `KalmanCA`, `.KalmanCA()`, `kalman_ca.hpp`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `KalmanCT triple`** (3 nodes): `KalmanCT`, `.KalmanCT()`, `kalman_ct.hpp`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Track triple`** (3 nodes): `track.hpp`, `Track`, `.Track()`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `HungarianSolver triple`** (3 nodes): `hungarian_solver.hpp`, `HungarianSolver`, `.HungarianSolver()`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Wgs84Transform triple`** (3 nodes): `Wgs84Transform`, `.Wgs84Transform()`, `wgs84_transform.hpp`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `KinematicPredictor triple`** (3 nodes): `KinematicPredictor`, `.KinematicPredictor()`, `kinematic_predictor.hpp`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `OcclusionPredictor triple`** (3 nodes): `OcclusionPredictor`, `.OcclusionPredictor()`, `occlusion_predictor.hpp`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `ColorCorrectEngine pair`** (2 nodes): `ColorCorrectEngine`, `color_correct_engine.hpp`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `RadarSim pair`** (2 nodes): `RadarSim`, `radar_sim.hpp`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `HealthMonitor pair`** (2 nodes): `HealthMonitor`, `health_monitor.hpp`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `ClutterMap pair`** (2 nodes): `ClutterMap`, `clutter_map.hpp`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `CuasVisualizer pair`** (2 nodes): `cuas_visualizer.hpp`, `CuasVisualizerNode`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Common TUs pair`** (2 nodes): `clock translation unit`, `types translation unit`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Georef placeholder pair`** (2 nodes): `georef_node (placeholder executable)`, `wgs84_transform translation unit (header-only impl)`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `now_ns`** (1 nodes): `now_ns`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Threat class enums`** (1 nodes): `THREAT_DRONE_CLASSES / THREAT_BENIGN_CLASSES`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Wgs84Transform isolated`** (1 nodes): `Wgs84Transform`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `radar_driver TU`** (1 nodes): `radar_driver translation unit`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Hungarian (Jonker-Volgenant)`** (1 nodes): `HungarianSolver (Jonker-Volgenant O(n^3) assignment over square-padded buffer)`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `CUAS_STRICT_FLAGS`** (1 nodes): `CUAS_STRICT_FLAGS (adds -Wsign-conversion/-Wconversion for non-ROS targets)`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **What is the exact relationship between `cuas_fusion ament_cmake package` and `latency_budget.md (empty placeholder)`?**
  _Edge tagged AMBIGUOUS (relation: references) - confidence is low._
- **What is the exact relationship between `cuas_fusion ament_cmake package` and `architecture_diagram.md (empty placeholder)`?**
  _Edge tagged AMBIGUOUS (relation: references) - confidence is low._
- **What is the exact relationship between `cuas_msgs rosidl interface package` and `ICD.md (Interface Control Document - empty placeholder)`?**
  _Edge tagged AMBIGUOUS (relation: references) - confidence is low._
- **Why does `init()` connect `Threat Classifier Internals` to `Kalman Transition Matrices`, `Camera Calibration Scripts`, `Geometric & Threat Helpers`, `Clutter Map Internals`, `Geofence Engine + Tests`, `Visualizer & Image Adapter`, `Health Monitor Module`, `Color Correction Engine`?**
  _High betweenness centrality (0.170) - this node is a cross-community bridge._
- **Why does `shutdown()` connect `Threat Classifier Internals` to `Camera Calibration Scripts`, `Clutter Map Internals`, `Geofence Engine + Tests`, `Visualizer & Image Adapter`, `Health Monitor Module`, `Color Correction Engine`, `TRT Detector & Helpers`?**
  _High betweenness centrality (0.144) - this node is a cross-community bridge._
- **Why does `Track` connect `Threat Classifier Internals` to `Fusion + Track Association`, `Release History (CHANGELOG)`, `Overlay Drawing Primitives`, `Geometric & Threat Helpers`?**
  _High betweenness centrality (0.113) - this node is a cross-community bridge._
- **Are the 33 inferred relationships involving `shutdown()` (e.g. with `main()` and `.GeofenceNode()`) actually correct?**
  _`shutdown()` has 33 INFERRED edges - model-reasoned connections that need verification._