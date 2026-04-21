# ProjectHailMary Changelog

## [0.8.0-alpha] — current
- Intent classifier node: APPROACHING/LOITERING/ORBITING/DEPARTING/TRANSITING behavioral classification
- Unit tests wired into colcon for all pure math classes
- Config management: VERSION, setup_env.sh, CHANGELOG.md

## [0.7.0-alpha]
- JSF AV C++ / MISRA C++:2023 full compliance pass
- 144 violations resolved across 20 files
- Typed track_state_id and threat_level_id fields replacing runtime string comparisons
- Math logic moved from node files into math classes

## [0.6.0-alpha]
- Geofence exclusion zone manager with circle and polygon zones
- Reachability engine: time-to-intercept and covariance ellipse
- Adaptive prediction horizon tied to threat level (3s-10s)
- Horizon ownership refactored to imm_tracker (3 iterations)

## [0.5.0-alpha]
- Health monitor / BIT node with EMA rate measurement
- Sim radar node for SIL testing (sim.launch.py)
- Static clutter map with occupancy grid learning
- ClutterStatus typed message replacing string publisher

## [0.4.0-alpha]
- Hungarian algorithm track association
- Mahalanobis gating replacing Euclidean distance
- IMM CV+CA+CT Kalman filter with Bayesian mode mixing
- Doppler-weighted centroid in radar parser
- Two-tier label visual weight in RViz2 visualizer

## [0.3.0-alpha]
- YOLOv8s INT8 TensorRT inference at 35Hz
- Camera-radar fusion via pinhole projection
- Threat classifier 5-state escalation FSM
- CoT/ATAK UDP multicast publisher

## [0.2.0-alpha]
- RViz2 visualizer with MarkerArrays and operator HUD
- Trajectory prediction: KinematicPredictor + OcclusionPredictor
- PredictionMuxNode stream arbitration
- Overlay engine downstream arc overlay

## [0.1.0-alpha]
- Radar serial driver with udev symlinks
- TLV parser with DBSCAN clustering
- IMM tracker with Hungarian assignment
- ROS 2 Humble workspace established
