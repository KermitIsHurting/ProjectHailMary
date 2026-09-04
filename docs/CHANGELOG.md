# ProjectHailMary Changelog

## [0.10.0-alpha] — 2026-09-02 whole-project audit (branch `polish`)
Playbook §15 audit: 7 review agents, 80 code findings + 51 doc rows → 37 root causes (+1 found
during the fixes); 30 fixed in code, 1 partial, 2 doc-only, 5 left as owner decisions; seven gated
batches (build −Werror 0 warnings → 160 GoogleTest cases → cppcheck baseline → sim smoke), three
bug-finder passes (cumulative diff, fixes, fixes again), shape runs on the final tree. Record: `docs/audit-plan.md`,
`docs/audit-log.md`, `docs/claims-evidence.md`, `docs/polish-retro.md`.
- Tracker: `doppler_mps` from the estimate (THREATENING was unreachable); covariance-scaled
  velocity and association gates (fast targets were pinned near 0 m/s); one clock family
  (CLOCK_MONOTONIC) end to end; layout/empty/NaN guards; duplicate-return guard
- IMM: block-diagonal mixing injection (P went indefinite within 5 s), mode update from the mixed
  prior (μ_CT underflowed), Cholesky-guarded Joseph-form updates
- Parser: filter-then-cap at 256 raw points, poll() before read(), reopen with backoff, publish
  every frame, singletons pass, CP210x interface 01 = data
- Fusion/inference: empty arrays published, z extrapolated, Euclidean range, top-128 by score
  before NMS, Image buffer guard
- Classifiers: fused labels ≤ 250 ms old joined by position + bearing, id-set pruning, parameter
  clamps; geofence reports every containing zone and fails loud on bad YAML; per-track prediction
  horizon; one bearing convention (boresight = +Y)
- Health idle-aware (no more DEAD on an empty scene), overlay health/geofence strip, inference
  exits 1 without an engine, launch starts the parser only after a successful radar config,
  auto-exposure off by default, sim scenes in the ICD frame + crossing/clutter/max_range/hover
- Docs: README, facts sheet, claims reworded (no MISRA checker; 22–25 Hz inference; 20 Hz radar;
  160 GoogleTest cases; Hungarian tracker not launched; nominal extrinsics)
- Owner decisions left open: network exposure, threat class policy, de-escalation, clutter
  relearn, extrinsics sign (see `docs/claims-evidence.md`)

## [0.9.0-alpha]
- OVERHAUL P3.1 interface growth: `Track` gains vz / acceleration / 6×6-covariance-triangle / source_mask; `FusedDetection` gains source_mask (all additive — earlier bags replay with zeros)
- Every cuas_msgs sequence bounded to its producer capacity (DEV-011 narrowed to strings only)
- Shared covariance wire packer `cuas::packUpperTriangle6` with layout-anchor unit tests

## [0.8.0-alpha]
- Intent classifier node: APPROACHING/LOITERING/ORBITING/DEPARTING/TRANSITING behavioral classification
- Unit tests wired into colcon for all pure math classes
- Config management: VERSION, setup_env.sh, CHANGELOG.md

## [0.7.0-alpha]
- Manual pass guided by JSF AV C++ / MISRA C++:2023 (fixed-width types, typed ids, math out of node files). No checker exists that produced a violation count; the "144 violations across 20 files" figure has no artefact (audit 2026-09, F-7)
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
- Hungarian algorithm track association with Mahalanobis gating in `track_manager` (not in the launched pipeline)
- IMM CV+CA+CT Kalman filter with mode mixing (mixing was CV-only until b8a00c2; corrected mode-probability update in 0.10.0)
- Cluster centroid in the radar parser (position mean; velocity = the max-|Doppler| member, not Doppler-weighted)
- Two-tier label visual weight in RViz2 visualizer

## [0.3.0-alpha]
- YOLOv8s INT8 TensorRT inference (35 Hz was never measured in-pipeline; 22–25 Hz on the Orin Nano in the full graph, 2026-09-02)
- Camera-radar fusion via pinhole projection into 25 %-padded boxes (no IoU)
- Threat classifier escalation FSM UNKNOWN→TRACKED→IDENTIFIED→THREATENING→ENGAGED (separate 4-value ThreatLevel)
- CoT/ATAK UDP multicast publisher

## [0.2.0-alpha]
- RViz2 visualizer with MarkerArrays and operator HUD
- Trajectory prediction: KinematicPredictor + OcclusionPredictor
- PredictionMuxNode stream arbitration
- Overlay engine downstream arc overlay

## [0.1.0-alpha]
- Radar serial driver with udev symlinks
- TLV parser with DBSCAN clustering
- IMM tracker (nearest-neighbour association; the Hungarian/Mahalanobis `track_manager` of 0.4.0 is unit-tested but not launched)
- ROS 2 Humble workspace established
