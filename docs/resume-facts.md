# ProjectHailMary — facts sheet (claim → evidence → confidence)

Every claim you might put on a résumé or say in an interview, with the artefact that backs it.
Confidence: **measured** (a command and its output on the Jetson, dated) · **implemented** (code +
unit test, not measured live) · **unmeasured** (claim exists, no artefact) · **planned**.
Conditions for every measured number: Jetson Orin Nano 8 GB (Super mode, 6× A78AE @ 1728 MHz),
JetPack 6.x / L4T 36.5, ROS 2 Humble, TensorRT 10.3, `sim.launch.py` (simulated radar at 20 Hz,
REAL AR0234 camera, YOLOv8s INT8 engine), 2026-09-02, branch `polish`.

## What it is

| Claim | Evidence | Confidence |
|---|---|---|
| Counter-UAS radar + camera fusion stack, ROS 2 Humble, C++17, 22 nodes | `src/cuas_fusion/CMakeLists.txt` (22 `add_executable`), `docs/ICD.md` §2; 19 nodes launched by `full.launch.py` | measured (build) |
| Sensors: TI IWR6843ISK 60 GHz mmWave radar over CP2105 USB serial, AR0234 global-shutter camera over V4L2 | `src/cuas_fusion/src/drivers/radar_parser_node.cpp` (TI TLV parser), `camera_driver.cpp`, `scripts/99-iwr6843.rules` | implemented; hardware attached on the dev box |
| ~12.9 k lines of owned C++ (src + include) + 3.0 k lines of tests at `12c19fb` (11.8 k / 2.3 k at the audit baseline `ca48665`) | `git ls-files src/cuas_fusion/src src/cuas_fusion/include | xargs cat | wc -l` = 12,852; tests 2,992 | measured |
| 160 GoogleTest cases in 22 binaries, 0 failures (`colcon test-result` reports 182 = 160 cases + one ctest row per binary) | `grep -cE '^\s*TEST(_F)?\(' test/unit/*.cpp src/cuas_fusion/test/*.cpp` = 160 on `12c19fb`; `colcon test-result` 182/0 failures | measured |
| Builds with `-Werror` at 0 warnings | every audit-log gate line; `CUAS_WERROR` in CMake | measured |

## The pipeline, in order (say it this way)

serial TLV bytes → per-return validity filter → DBSCAN clustering → IMM tracker (CV/CA/CT,
covariance-scaled association) → camera fusion (pinhole projection into YOLOv8s INT8 boxes) →
threat + intent classifiers → geofence / reachability → RViz2 + camera overlay + CoT/ATAK UDP.

| Stage | File | Decision you made and why (one line) |
|---|---|---|
| TLV parser | `drivers/radar_parser_node.cpp` | `poll()` before `read()` and thread-owned reopen with backoff: a silent or unplugged radar must never park the process or leave a zombie node (RC-11) |
| Raw-point filter then cap | same, `pointPasses()` | filter (finite, range, Doppler) BEFORE the 256-point cap so a dense scene keeps the distant target (RC-10) |
| DBSCAN | same, `dbscanCluster()` | eps 0.7 m / minPts 2; unclustered singletons always pass as single-point detections so a far target with one return is not dropped (RC-35) |
| IMM tracker | `tracking/imm_tracker.cpp` | born with sensor-sigma position and 10 m/s velocity sigma; velocity-jump gate = max(3 m/s²·dt, 3σ_v); association gate = 0.8 m + 3(σ_pos + σ_v·dt) capped 5 m — the fixed 0.8 m / 0.15 m/s gates pinned every fast target near 0 m/s (RC-2/3) |
| IMM mixing | `estimation/imm_filter.cpp`, `kalman_ca/ct.hpp` | block-diagonal injection of the mixed 6×6, cross terms zeroed: PSD by construction; mode weights from the mixed prior with a 1e-6 floor (RC-34, RC-7) |
| Kalman update | `estimation/position_update.hpp` | Cholesky with `info()` check, Joseph form; a non-SPD innovation covariance skips the update instead of applying a garbage gain |
| Fusion | `fusion/fusion_node.cpp`, `fusion_engine.cpp` | per-measurement temporal alignment to the nearest camera set (150 ms gate), track extrapolated on all three axes to the camera instant; range = Euclidean |
| Inference | `inference/trt_detector.cpp`, `topk_boxes.hpp` | YOLOv8s INT8 TensorRT engine; the 128 best anchors by score go to NMS, not the first 128 (RC-19) |
| Threat classifier | `classification/threat_classifier*.cpp` | label join requires position ≤ 1 m AND bearing ≤ 15°, fused set ≤ 250 ms old; escalation UNKNOWN→TRACKED→IDENTIFIED→THREATENING→ENGAGED on the steady clock |
| Geofence | `geofence_engine.cpp`, `geofence_node.cpp` | every containing zone reported (bitmask per track); YAML fails loud on any malformed zone (RC-16, RC-13) |
| Prediction | `prediction/kinematic_predictor*.cpp` | CV forward propagation over the per-track horizon (3–10 s by threat level), step coarsened past the 64-step buffer (RC-28) |
| Health | `health_monitor*.cpp` | EMA rate per topic; a producer with nothing to publish is idle, not dead; dead reports 0 Hz (RC-12) |
| Clocks | `common/clock.hpp` | one clock family end-to-end (CLOCK_MONOTONIC) — the tracker's RCL_STEADY_TIME drifted 415 ms from the drivers under NTP slew and broke the fusion gate after ~75 min (RC-6) |
| CoT | `output/cot_publisher_node.cpp` | CoT 2.0 XML over UDP multicast 239.2.3.1:6969 at 1 Hz; lat/lon are 0.0 (no GNSS) — never received by an ATAK client yet |

## Measured numbers (2026-09-02, conditions above)

| Number | Value | Artefact |
|---|---|---|
| Radar frame rate (sim and profile) | 20 Hz | `radar_profile.cfg` frameCfg 50 ms; readout `/radar/detections` 20.00 Hz |
| `/tracks`, `/threat/reports`, `/fusion/detections` | 20 Hz each | `~/backups/b7-sim-smoke-*/readout.txt` |
| Camera raw | 46–54 Hz across 27 readouts, ≈49 typical (the driver's 30 Hz request is best-effort) | all `readout.txt` under `~/backups/` |
| YOLOv8s INT8 in-pipeline | 22–27 Hz across 27 readouts (≈24 typical); GPU load (GR3D) spans 10–90 % within a run, so "GPU-bound" is inferred, not isolated | all `readout.txt`; `tegrastats.txt` per run |
| `/tracks` stamp age at the subscriber | 2.7 ms p50 / 15.2 ms p95, n = 39 over 10 s (was 415.7 ms p50 in A1) | `~/backups/b2-stamp-probe-20260902T0310Z/probe.txt` |
| Shape runs (A6 pass 2): approach / lateral / max_range / hover / circle → exactly 1 track id per target; two_targets → 2; crossing → 2 with no new ids | audit-plan §9 | `~/backups/a6-shapes-pass2-20260902T0409Z/*/readout.txt` |
| Geofence events on the approach: perimeter ENTERED 1 / EXITED 1; lateral: no_fly_left + perimeter | same | same |
| Max range seen in the max_range window | 14.2 m (scene starts at 14.9 m under the 15 m clip; the first-detection instant itself is not captured) | same |
| Clutter map: 3 learned static reflectors (occupancy 0.002) still sustain 3 CONFIRMED tracks; the ~38 % leak rate is a derived estimate (cell-hit probability 0.62 for 0.1 m noise in 0.25 m cells), not measured | D-4 for the owner | `…/clutter/readout.txt` |
| Sim smoke on the fix tree (67906aa + the uncommitted A6/R6 fixes, committed unchanged as `12c19fb` nine minutes later) | exit 0, 0 error lines, health NOMINAL, 1 track | `~/backups/fix-sim-smoke-20260902T0419Z` |

## Pinned by unit test (implemented, not measured live)

| Behaviour | Test |
|---|---|
| Straight targets at 1/5/10/15 m/s stay one track and converge within 10 % (floor 0.3 m/s) after 40 updates | `test/unit/test_imm_tracker.cpp` |
| IMM covariance over 2000 steps (100 s): symmetric, min eigenvalue ≥ −1e-9, every mode weight ≥ 1e-6; CT weight recovers > 0.5 after 50 s straight | `test/unit/test_imm_filter.cpp` |
| Label join extrapolates the track to the fused instant (±dt) and rejects by distance and bearing | `test/unit/test_threat_classifier.cpp` |
| Overlapping zones both reported; per-track step plan follows the horizon | `test_geofence_engine.cpp`, `test_kinematic_predictor.cpp` |

## Claims to drop or reword (they could not be proven)

| Old claim | Say instead |
|---|---|
| "MISRA C++:2023 / JSF AV full compliance, 144 violations resolved" | "written to MISRA C++:2023 / JPL Power-of-10 discipline (fixed-width types, no heap on hot paths, bounded loops, single exception boundary); no compliance checker was run — cppcheck at 6 findings, clang-tidy 710 warnings recorded in the audit" |
| "35 Hz YOLOv8s INT8" | "22–25 Hz in the full pipeline on an Orin Nano, GPU-bound" |
| "16 Hz radar" | "20 Hz (profile frameCfg 50 ms)" |
| "108 tests" / "43 GoogleTest classes" | "160 GoogleTest cases in 22 binaries" |
| "Hungarian + Mahalanobis tracker" (as the live tracker) | "implemented and unit-tested (`track_manager`); the launched tracker is the IMM node with covariance-scaled nearest-neighbour association" |
| "Calibrated SE(3) extrinsics" | "nominal mount extrinsics; the translation sign is unverified (Needs a reflector calibration)" |
| "Pinhole + IoU association" | "pinhole projection into 25 %-padded boxes" |

## Decisions still yours (Needs John)

| D | Question | Default in the tree |
|---|---|---|
| D-1 | `ROS_LOCALHOST_ONLY=1` / fixed domain id (RC-15) | not set; any LAN host can inject `/tracks` |
| D-2 | Threat class policy (RC-17): any non-person COCO class > 0.5 is THREAT | unchanged |
| D-3 | De-escalation hysteresis (RC-18) | none; THREATENING/ENGAGED exit only by prune |
| D-4 | Clutter-map relearn/decay (RC-27): a hovering target in the first 10 s is learned as clutter | unchanged (see the `hover` scene) |
| D-5 | Extrinsics translation sign (RC-31) | nominal |
| D-13 | Geofence hysteresis (A6-4): a target on a zone edge chatters ENTERED/EXITED at 10 Hz | none |
| D-0 | Keep `78d613d` / `ca48665` | kept |
