# RESUME_PLAN — remaining audit fixes (stopped early at owner request)

State when paused: branch `misra-audit-fixes`, 17 commits, build warning-clean,
97/97 tests green. All findings below are already documented with file:line and
fixes in **AUDIT_REPORT.md** (IDs referenced here). Workflow per commit:
edit → `colcon build --packages-select cuas_fusion` (must stay at **0 warnings**)
→ `colcon test --packages-select cuas_fusion` (must stay green) → commit with
the audit ID in the message. To resume with the executor: *"continue RESUME_PLAN.md
on branch misra-audit-fixes, same rules as before (no push, no camera files)"*.

Ordered by value-per-effort:

## R1. Radar parser robustness batch (A2.4, A1.10, A1.11, A2.9) — HIGH, ~4 commits
`src/drivers/radar_parser_node.cpp`
1. Replace the three `reinterpret_cast` type-puns (lines ~410, ~442, ~452) with
   `memcpy` into locals (`FrameHeader hdr; std::memcpy(&hdr, hdr_buf, sizeof hdr);`)
   — strict-aliasing UB, MISRA 8.2.5; a deviation is NOT defensible (memcpy is free).
2. In `read_exact` (~line 344): handle `r == 0` (serial EOF/unplug) — currently
   busy-spins at 100% CPU. Log + return false.
3. TLV loop (~446): validate `offset + tlv.length <= payload_len` before parsing
   points, and advance `offset` by exactly `tlv.length` (not `floor(len/16)*16`)
   — corrupt length currently desyncs the frame and publishes garbage.
4. `strerror(errno)` → `strerror_r` buffer (4 call sites) — thread-safety on a
   Mandatory-family rule (25.5.3).
5. Optional (NEEDS-HARDWARE to verify): `VMIN=0/VTIME=5` or poll() so a silent
   radar can't block shutdown; reopen-with-backoff instead of `break` in
   parse_loop (mirror camera_node's retry); new `RADAR_MAX_POINTS_PER_FRAME=256`
   constant instead of borrowing TRACK_MAX_TRACKS(=32) for raw points (currently
   drops points 33+ before DBSCAN).

## R2. Fusion correctness (A1.7, A1.8) — HIGH, 2 commits
- `src/fusion/fusion_node.cpp:162-169`: stale-YOLO gate — use the written-but-
  never-read `latest_yolo_ts_`: skip label fusion when newest YOLO data older
  than a named window; treat empty detection arrays as valid (clear boxes).
- `src/fusion/fusion_engine.cpp:122-143`: EMA is keyed by class_id, so two
  same-class targets smear into each other and entries never evict. Key by
  nearest-EMA-within-gate (or track id) + timeout eviction. Add a two-targets-
  same-class unit test.

## R3. TrackManager estimator (A1.9, A3.2, A3.3) — HIGH, 2-3 commits
`src/tracking/track_manager.cpp`
- 80-86: gain K computed but never applied to state (state jumps to raw
  measurement while P shrinks) — apply `pos += K*innovation` or drop the P
  update; add convergence unit test.
- 101-113: single miss demotes CONFIRMED→COASTED and zeroes hit_count → track
  flicker. M-of-N: only demote after TRACK_MAX_MISSES; don't reset hits for
  confirmed tracks. Unit test: confirm → miss once → hit → still published.
- 137: `cost_matrix_.resize(N,M)` reallocates per scan → fixed
  `Eigen::Matrix<double,32,32>` member + `.topLeftCorner(N,M)`.
- 168: hoist `S.inverse()` (computed 2× per pair; use `llt().solve(I)` once per
  track, check `info()==Success` — this also feeds the Hungarian NaN guard).

## R4. Kinematic predictor (A1.13) — MED, 1-2 commits
`src/prediction/kinematic_predictor_node.cpp:38` + `kinematic_predictor.cpp`
- Validate `step_dt > 0`; clamp `n_steps_` to kMaxTrajectorySteps; reconcile
  PREDICTION_MAX_STEPS(128) vs kMaxTrajectorySteps(64) into one constant.
- The three "model" step functions are byte-identical CV and `blended_vel` is
  discarded — either implement real CA/CT steps or delete the pretense and
  document CV-only (published model_weight_ca/ct currently misrepresent).

## R5. CoT publisher (A1.6) — MED, 1 commit
`src/output/cot_publisher_node.cpp`
- Check `setsockopt`/`inet_aton`/`sendto` returns (failed inet_aton = events to
  0.0.0.0); `strftime` return check (line ~67); normalize course to 0..360;
  named constants for 239.2.3.1:6969/TTL/rates; the lat/lon 0.0 placeholder
  needs georeferencing or a loud comment + ICD note.

## R6. Hot-path allocation batch (A3.1, A3.4-A3.8) — MED (big but mechanical)
- A3.1: estimation stack `VectorXd/MatrixXd` → fixed-size `Eigen::Matrix<double,N,M>`
  (dims are compile-time: 6/7/9/3). The 20 math-core tests protect this refactor.
- A3.4: trt_detector preprocess: member cv::Mats created in init().
- A3.5: timestamp_associator `frame.clone()` → preallocated ring + `copyTo`.
- A3.6: `latest_tracks_ = *msg` deep copies → store ConstSharedPtr (pattern:
  intent_classifier_node.cpp:51) in geofence/reachability/kinematic/imm_tracker.
- A3.7: `reserve()` before message-building loops (fusion_node:152,
  track_manager_node:78, imm_tracker_node:183, inference_node:66).
- A3.8: `FusedDetection::class_label`/`Track::class_label_` std::string → int32
  class_id, stringify at publish boundary (DEV-005 chokepoint pattern).

## R7. Small fixes — MED, batch commits
- Health monitor float32 seconds → int64 ns (A6.8/watchdog rot at ~194 days):
  `health_monitor_node.cpp:70-73`, `health_monitor.hpp:30`, subtract-then-narrow.
- Geofence fail-loud on over-capacity YAML (A2.6): >32 verts / >16 zones must
  FATAL, not silently truncate the enforced boundary.
- Clutter callback: validate PointCloud2 fields before iterators (A2.5-part) —
  a foreign publisher currently throws into the (now-caught) main handler;
  per-frame drop-with-warn is the right graceful degradation.
- `load_scenario` recursion (A2.10, drivers/sim_radar_node.cpp:110) — normalize
  the name first, single dispatch. Only recursion in the codebase.
- `sim_radar.cpp getTarget()` unchecked index (A2.9-part).
- IMM tracker: dt=0 velocity freeze (A1.14, imm_tracker.cpp:63) — floor dt or
  composite same-stamp updates; unify R (1.0 vs kRadarDetectionSigmaM²=0.0225).
- TRT engine shape validation vs compile-time buffers (A2.8) + logger latching
  errors + functor deleters (audit findings 2-4 of inference agent).
- `trackStateFromString` silent TENTATIVE default (A6.6) + missing
  COASTED/DELETED ids in track_state_ids.hpp.
- Overlay waypoint cache never evicts dead ids (32 lifetime ids → arcs stop;
  visualization agent finding 18) — erase_if against live track set.

## R8. Dead code removal (A6.1-A6.5) — EASY, 1 commit
`git rm src/cuas_fusion/src/sim_radar_node.cpp src/cuas_fusion/src/radar_sim.cpp
src/cuas_fusion/include/cuas_fusion/radar_sim.hpp src/cuas_fusion/src/georef/georef_node.cpp
src/cuas_fusion/src/georef/wgs84_transform.cpp test/unit/test_wgs84_transform.cpp
test/integration/test_fusion_pipeline.cpp` (last two are empty stubs; keep if
you plan to implement); `git rm -r src/cuas_msgs` (empty leftover); delete the
dead remap line full.launch.py:192; delete dead `matched_fd = nullptr` else-arm,
vacuous switch + unused kOccludedTimeout/kLostTimeout in imm_tracker.cpp
(or implement the OCCLUDED/LOST lifecycle they imply).

## R9. Build enforcement (A4.2-A4.10) — do LAST, 2-3 commits
1. Promote `trt_detector, fusion_engine, tracking_lib, classification_lib,
   geofence_engine, reachability_engine` to CUAS_STRICT_FLAGS (the stated
   policy for no-ROS libs); burn down resulting -Wconversion warnings.
2. Add `-Wshadow` to CUAS_SAFETY_FLAGS; apply SAFETY flags to test targets;
   `if(NOT CMAKE_BUILD_TYPE) set(CMAKE_BUILD_TYPE RelWithDebInfo)`; guard
   find_library results with FATAL_ERROR; find_package(CUDAToolkit) instead of
   hardcoded cuda-12.6 path; fix the header comment lie at CMakeLists.txt:38
   (exceptions are NOT disabled; "18-4-1" is 2008 numbering for heap, not
   exceptions).
3. `-Werror` (or CUAS_WERROR option ON) — only after 1+2 are warning-free.
4. package.xml: add eigen/libopencv-dev/gstreamer deps, rviz2 → exec_depend;
   launch files: engine_path → DeclareLaunchArgument (currently /home/zork/...).

## R10. Paperwork (B-section) — 1 commit
Rewrite docs/CODING_STANDARD.md deviations per AUDIT_REPORT.md §B: upgrade
DEV-001..005 to full Compliance:2020 §4.2 field sets (reason category, risk,
locations, signatory) and add DEV-006..012. Note DEV-002's EIGEN_NO_MALLOC
claim is currently FALSE — after R6/A3.1, add
`$<$<CONFIG:Debug>:EIGEN_NO_MALLOC>` to the strict math libs to make it true.

## R11. Nice-to-have (interview polish)
- CUAS_CHECK(cond) log-and-return macro + assertion density pass (P5).
- Magic-number batch (visualizer layout, threat ENGAGED gate, sim RCS model,
  kConfirmHits 5 vs TRACK_CONFIRM_HITS 3, kPi in constants.hpp).
- std::bind → explicit-capture lambdas; `final` on all node classes (D4).
- Visualizer: split 562-line imageCallback into draw helpers; shrink the mutex
  to a snapshot (blocks all data callbacks for the whole render today);
  ROI-vs-frame-size guards (frame <320px wide currently throws).
- Camera PROTECTED findings (report-only in AUDIT_REPORT §B PROT): if you ever
  approve changes, follow the backup/side-file/ROLLBACK.md protocol.

## Step 5 — interview-impact improvements (not yet started; suggest-only)
Ranked impact/effort: (1) GitHub Actions CI: build + colcon test + cppcheck +
clang-tidy gates — biggest signal, half a day. (2) DEVIATIONS.md at repo root
(R10 makes it). (3) Requirements traceability matrix: REQ-ID → code → gtest
(docs/ICD.md anchors it; DO-178C flavor). (4) Coverage report (gcovr) with a
stated statement/decision target (DAL-B imitation). (5) Fault-injection tests
(corrupt TLV frames, NaN measurements — some already exist after this branch).
(6) Sensor-degradation modes doc (what the system does when radar/camera dies —
partially implemented via health monitor).
