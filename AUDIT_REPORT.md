# AUDIT_REPORT — MISRA C++:2023 / JPL Power of 10 / DO-178C-inspired Compliance Audit

**Scope**: all C++ sources, headers, CMake, launch, config, and message definitions of `cuas_fusion` +
`cuas_msgs` (12,852 lines of owned C++ src+include at `03bbe26`; ~11,150 at the audit baseline). Basis: `STANDARDS_CHECKLIST.md` (rule IDs M1–M30 = MISRA C++:2023,
P1–P10 = JPL Power of 10, D1–D10 = DO-178C/DO-332-inspired). Method: manual file-by-file audit by
six parallel reviewers against the verified rule set, cross-checked with g++ 11.4 (`-Wall -Wextra
-Wpedantic` + project flags), cppcheck 2.7 (native checks), and clang-tidy 14
(clang-analyzer/bugprone/cert/cppcoreguidelines). *Honesty note*: no free tool checks MISRA
C++:2023 (cppcheck's misra addon is MISRA C:2012, C-only, and is excluded as evidence); rule
citations below were verified against the complete published rule table.

**Baseline** (clean rebuild, this Jetson, branch `misra-audit-fixes`):
build OK · **90 warnings** (88 from Eigen via non-SYSTEM includes, 2 in own code) ·
**3 failing test cases** (all three are *test* defects, not code defects — see A5.1/A5.2) ·
cppcheck native findings: 68 · clang-tidy: 771 warnings (504 magic-number, 13 narrowing,
12 exception-escape, 8 member-init, …) · 1 clang-tidy error (dead file not in compile DB).

## Severity-ranked summary

| Rank | Class | Count | Examples |
|---|---|---|---|
| 1 | Mandatory-rule exposure (MISRA 11.6.2 — no deviation possible) | 3 latent paths | unchecked `clock_gettime` feeding every timestamp; `FixedVector::back()` on empty wraps to idx 4×10⁹; `FixedVector::resize()` can grow over never-written slots |
| 2 | Functional defects found by rule-driven review | 12 | Hungarian solver infinite loop on NaN cost; IMM mixing zeroes model-private states (IMM degenerates to CV); reachability fabricates velocity; bearing no-wrap at ±180°; classifier YAML params silently ignored; CoT events geolocate to (0,0); stale-YOLO fusion; EMA cross-target smearing; TLV desync; radar EOF busy-spin; track flicker on single miss; clutter map ignores runtime params |
| 3 | Required-rule violations, fixable | ~25 | strict-aliasing `reinterpret_cast` in TLV parse (8.2.5); recursion in `load_scenario` (8.2.10); missing catch-all mains (18.3.1/D3); unchecked returns (0.1.2/P7) |
| 4 | Hot-path allocation (P3 / JSF AV 206) | ~15 sites | dynamic Eigen in estimation stack; per-cycle `cost_matrix_.resize`; per-frame `cv::Mat` in TRT preprocess; `frame.clone()` at 30 Hz; `latest_* = *msg` deep copies |
| 5 | Enforcement gaps (P10) | 8 | no `-Werror`; no `-Wshadow`; 6 non-ROS libs missing `-Wconversion` against stated policy; Eigen/OpenCV not SYSTEM; no build-type default; no CI; empty test stubs claiming coverage |
| 6 | Deviations to formalize (§B) | 10 records | rclcpp/message allocation; sockets ABI cast; OpenCV `const_cast`; executor threading contract; unbounded .msg types |

**Compliance stats** (MISRA C++:2023 posture): 0 confirmed Mandatory violations in executed paths;
3 latent Mandatory (11.6.2) paths — must-fix, no deviation permitted. Required rules: ~25 violation
groups fixable + 10 properly deviable with records. The 5 Mandatory rules (6.8.2, 8.18.1, 11.6.2,
25.5.2, 25.5.3): 6.8.2/8.18.1 clean; 25.5.2/25.5.3 compliant (getenv/strerror use verified; one
thread-safety hardening noted A2.9). JPL P1/P2/P8: clean except items below. Recursion (M18): one
instance. goto/setjmp/unions/C-style casts in owned code: **zero**.

---

# A) VIOLATIONS — FIXABLE

Legend: severity = [M]andatory-exposure / [R]equired / [A]dvisory / [F]unctional-defect.
Verify: VH = verifiable here (build/tests on this Jetson), NH = needs live hardware.

## A1 — Functional defects (rule-driven review catches)

**A1.1 [F] Hungarian solver: provable infinite loop on non-finite cost** — P2/Dir 0.3.1 ·
`src/tracking/hungarian_solver.cpp:69-102` · If any cost entry is NaN (reachable via broken
`S.inverse()` in track_manager), all comparisons go false, `j0` never advances, the augmenting-path
`do/while` never exits — tracker thread hangs mid-scan. **Fix**: reject non-finite matrices
(`cost.allFinite()`) + bounded guard counter with fault return on both loops (also back-path
`while (j0 != 0)`). Tradeoff: one pass over ≤32×32 doubles. VH (unit test with NaN entry).

**A1.2 [F] IMM mixing destroys model-private states — IMM is effectively CV-only** — Dir 0.3.2 ·
`src/estimation/imm_filter.cpp:55-83` + `kalman_ca.cpp:11-21`, `kalman_ct.cpp:11-21` · Mixing
extracts the common 6-dim substate then calls `init()`, which zeroes CA's acceleration and CT's
turn-rate ω and resets their covariance rows **every cycle**. CT can never develop ω≠0, CA never
holds accel; maneuver detection sees only process-noise differences. **Fix**: add
`setMixedState(x6, P6)` to each model that overwrites only the shared block, preserving
model-private states/covariance. VH (unit test: circular trajectory → CT weight must dominate;
currently fails).

**A1.3 [F] Reachability velocity is fabricated** — P7/D6 · `src/reachability_node.cpp:37-42` ·
Velocity is reconstructed radially outward along position bearing instead of using the real
`Track.vx_mps/vy_mps` (which intent_classifier uses correctly). `speed_toward` then reduces to
`-velocity_mps`: intercept warnings are geometry-free. **Fix**: `rstate.vx_mps = tr.vx_mps;` etc.
Output changes materially — that is the correction. VH (existing reachability gtest extendable).

**A1.4 [F] Bearing difference doesn't wrap at ±180°** — Dir 0.3.1 ·
`src/classification/threat_classifier_node.cpp:134` · +179° vs −179° yields diff 358° not 2°;
camera-label fusion fails exactly when a target crosses the seam behind the sensor. **Fix**:
wrap: `diff = fmod(|Δ|,360); if (diff>180) diff = 360−diff;`. VH.

**A1.5 [F] Threat classifier tuning params silently ignored** — D6/config ·
`config/system_params.yaml:6` says `threat_classifier_node:` but the node is `Node("classifier_node")`
(all launch files agree). All five parameters (`threatening_range_m`, …) never reach the node;
today's defaults happen to match, so the first YAML tune will silently change nothing. **Fix**:
rename YAML key to `classifier_node:`. VH (`ros2 param get` in sim to demo: NH).

**A1.6 [F] CoT output geolocates every event to (0,0) + unchecked sends** — P7/ICD ·
`src/output/cot_publisher_node.cpp:75` (`<point lat="0.0" lon="0.0" …>` hardcoded), `:107-108`
(`sendto`/`inet_aton`/`setsockopt` returns dropped — failed `inet_aton` ⇒ events to 0.0.0.0),
`:86` (course emitted −180..180; CoT expects 0..360). **Fix**: check returns (disable socket on
ctor failure, count send failures), normalize course, wire georeferenced lat/lon or document
placeholder loudly. Partially NH (ATAK display), rest VH.

**A1.7 [F] Fusion uses stale YOLO boxes forever** — D9 · `src/fusion/fusion_node.cpp:162-169` ·
Empty detection sets return early and dead inference keeps last non-empty boxes; live radar tracks
keep getting fused against arbitrarily old labels. `latest_yolo_ts_` is written, never read — the
staleness gate was planned and never built. **Fix**: staleness window on `latest_yolo_ts_`; treat
empty detections as valid information (clear). VH.

**A1.8 [F] Fusion EMA keyed by class smears distinct targets** — Dir 0.3.2 ·
`src/fusion/fusion_engine.cpp:122-143` · Two same-class targets share one EMA state — positions
blend toward the midpoint; entries never evict, so a returning target blends with its minutes-old
ghost. **Fix**: key by association identity with timeout eviction. VH (unit test: two same-class
boxes must not converge).

**A1.9 [F] TrackManager: covariance-state inconsistency + track flicker** — Dir 0.3.2/D9 ·
`src/tracking/track_manager.cpp:80-86` computes Kalman gain K, shrinks P as if blending, then
overwrites state with the raw measurement — overconfident P tightens the Mahalanobis gate on a
full-noise state. `:101-113` one missed scan zeroes `hit_count` and demotes CONFIRMED→COASTED;
since only CONFIRMED tracks publish, a single dropout blanks the track for 3 scans (downstream
threat/CoT flicker). **Fix**: apply K to state (or declared alpha-filter with matching covariance);
M-of-N demotion — don't reset hits for confirmed tracks. VH (convergence + miss/hit unit tests).

**A1.10 [F] Radar TLV desync on corrupt length** — P2/P7 ·
`src/drivers/radar_parser_node.cpp:446-456` · Detected-points branch never validates `tlv.length`
against remaining payload and advances by `floor(len/16)*16`, not `len`; corrupt length ⇒ trailing
bytes parsed as next TlvHeader ⇒ garbage points published (bounded, no overrun). **Fix**: bounds
check before loop; advance by exactly `tlv.length`. VH (crafted frame test).

**A1.11 [F] Radar serial EOF busy-spin + zombie node** — P2/D9/D10 ·
`radar_parser_node.cpp:344-353` — `read()==0` (USB unplug) not handled: `total` stops advancing,
loop spins at 100% CPU. `:237-242,397-431` — ctor open failure logs FATAL then idles; parse-thread
errors `break` and the node keeps looking alive with no retry/heartbeat (camera node has a retry
loop; radar has none). **Fix**: handle r==0; reopen-with-backoff mirroring camera_node; publish
health. Busy-spin repro NH; the missing branch VH.

**A1.12 [F] Clutter map ignores its runtime parameters** — P7 · `src/clutter_map.cpp:134` uses
compile-time `kThreshold` instead of member `threshold_`; `clutter_map_node.cpp:152` publishes
`kLearnFrames` instead of `learn_frames()` — status output lies whenever params override defaults.
**Fix**: use the members. VH (existing clutter gtest extendable).

**A1.13 [F] Kinematic predictor: unbounded n_steps + no-op "IMM blend"** — P2/M23 ·
`prediction/kinematic_predictor_node.cpp:38` `n_steps = horizon/step_dt` unvalidated (`step_dt=0`
⇒ inf ⇒ UB cast; 10000 steps of 6×6 math per track while FixedVector discards past 64 — CPU burned
for discarded data; `PREDICTION_MAX_STEPS`(128) vs `kMaxTrajectorySteps`(64) never reconciled).
`kinematic_predictor.cpp:81-109` — the three "model" step functions are byte-identical CV
integrators and `blended_vel` is computed then `(void)`-discarded: the published
`model_weight_ca/ct` misrepresent a blend that doesn't exist. **Fix**: validate params, clamp to
one authoritative cap; either implement CA/CT steps or delete the pretense and document CV-only. VH.

**A1.14 [F] IMM tracker: dt=0 velocity freeze + 44× sensor-model inconsistency** — Dir 0.3.1 ·
`src/tracking/imm_tracker.cpp:63,74-78` — several points of one cloud update with identical `now`
⇒ `max_delta_v = 0` ⇒ every velocity innovation rejected precisely when data is densest.
`imm_tracker.cpp:57` + `kalman_*.cpp` — measurement noise R = 1 m²/axis vs TrackManager's
0.0225 m² (`kRadarDetectionSigmaM²`) with no rationale. Also CT Jacobian divides by ω² guarded
only at 1e-6 (1e12 amplification at the boundary — currently masked by A1.2). **Fix**: floor dt
for the gate or composite same-stamp updates; single R constant; raise ω guard/Taylor limit. VH.

## A2 — Mandatory-exposure & UB paths

**A2.1 [M] `clock_gettime` return unchecked → uninitialized timespec read** — MISRA 11.6.2 + 0.1.2 ·
`include/cuas_fusion/common/clock.hpp:13-16` · Failure path reads indeterminate `ts`; every
timestamp flows through here. Mandatory rules admit no deviation. **Fix**: `timespec ts{};` +
check return, return last-good/0. VH.

**A2.2 [M] `FixedVector::back()` on empty = idx 0xFFFFFFFF; `resize()` can grow over
never-written slots; `operator[]` unchecked** — MISRA 11.6.2 latent + P5 ·
`common/fixed_containers.hpp:31-41,52` · Also `data_` not value-initialized (contrast: FixedMap
zero-inits). **Fix**: value-init `data_{}`; guard `back()`; make `resize` bool + value-fill grown
region (or shrink-only); `[[nodiscard]]` on `push_back`/`insert_or_assign`/`erase` (every
truncation-safety argument in the parser rests on callers noticing the bool). VH (unit tests).

**A2.3 [F/UB] `static_cast<int32_t>(inf)` from unvalidated rate/step params** — P7/[conv.fpint] ·
`geofence_node.cpp:64-65`, `reachability_node.cpp:94-95`, `kinematic/occlusion_predictor_node.cpp`,
`health_monitor_node.cpp:59-60`, `sim_radar_node.cpp:29`, `capture_node.cpp:44-47` ·
`publish_rate_hz: 0.0` ⇒ UB; >1000 ⇒ 0 ms busy-spin timer. `intent_classifier_node.cpp:37` is the
in-repo compliant pattern. **Fix**: clamp/validate at declaration (rcl ParameterDescriptor ranges
or explicit clamp+FATAL). VH.

**A2.4 [R] Strict-aliasing UB: `reinterpret_cast` type-punning in TLV parse** — MISRA 8.2.5/8.7.1 ·
`radar_parser_node.cpp:410,442-443,452-454` · No FrameHeader/TlvHeader/DetectedPoint object exists
in that storage; every member read is UB. A deviation is NOT defensible — `memcpy` into a local is
free (compiles to identical loads at -O2). **Fix**: `memcpy` pattern; keep the `static_assert`
layout proof. VH.

**A2.5 [R] YAML/pointcloud exceptions terminate nodes** — MISRA 18.3.1/D3 ·
`geofence_node.cpp:82-138` (`YAML::LoadFile`/`.as<T>()` throw on malformed config →
`std::terminate` from ctor), `clutter_map_node.cpp:66-67` (`PointCloud2ConstIterator` throws if a
field is missing → a foreign publisher on `/radar/detections` crashes the clutter filter).
**Fix**: catch-all in every `main()` (the sanctioned single handler under DEV-001) + per-frame
field validation in clutter callback (graceful-degradation is the right D9 behavior). Applies to
all 21 node mains: none has a catch-all today. VH.

**A2.6 [F] Geofence config truncation changes the enforced boundary** — P2/safety ·
`geofence_node.cpp:91-143` · Polygons with >32 vertices silently lose vertices 33+ (`(void)push_back`);
zones >16 silently dropped — the engine enforces a *different fence than configured*. **Fix**:
over-capacity config = fatal startup error (fail-loud beats fence-shrinking). VH.

**A2.7 [R] `NaN < 1e-12` defeats the likelihood guard** — Dir 0.3.1 · `imm_filter.cpp:113-116`,
`kalman_cv.cpp:75-77` (+ca/ct) · NaN det passes `det < 1e-12` false-branch, `exp(NaN)` propagates,
`mu_` goes NaN permanently, feeding A1.1's hang. **Fix**: negate comparisons (`!(det > eps)`) /
`std::isfinite`, guard `sum` like `c_sum`, explicit reset on non-finite. VH.

**A2.8 [R] TRT engine shapes never validated against compile-time buffer sizes** — P7 ·
`inference/trt_detector.cpp:51-75,102-105` · A different YOLO variant engine ⇒ `enqueueV3` writes
past `output_dev_` (device-side overflow, no diagnostic); multi-tensor engines silently keep last
input/output. **Fix**: validate `getTensorShape()` against expected dims at init; fail on
mismatch/extra tensors. VH (mismatched engine); overflow repro NH.

**A2.9 [A] Hardening batch** — `strerror`→`strerror_r` (25.5.3-family thread-safety,
`radar_parser_node.cpp:301+`); `strftime` return checked (`cot_publisher_node.cpp:67-69` —
unterminated-buffer string ctor path); visualizer ROI geometry vs frame size
(`cuas_visualizer_node.cpp:514,587` — <320 px frame throws `cv::Exception` out of the callback);
`getTarget()` unchecked index (`sim_radar.cpp:105`); V4L2 `buf.index` trust boundary [PROT —
report only]. All VH.

**A2.10 [R] Recursion: `load_scenario` calls itself** — MISRA 8.2.10 ·
`src/drivers/sim_radar_node.cpp:110-114` · The only recursion in the codebase; breaks the acyclic
call-graph/static-stack argument. **Fix**: normalize name first, single dispatch. VH.

**A2.11 [F] Format-string UB: `%zu` fed `uint32_t`** — P10/-Wformat ·
`radar_parser_node.cpp:285-287` — one of the two real warnings in the baseline build. **Fix**:
`%u` or cast to `size_t`. VH.

## A3 — Hot-path allocation (P3 / JSF AV 206 / M1) — fixable sites

| # | Site | Fix |
|---|---|---|
| A3.1 | Estimation stack: all state/covariance/temporaries are `Eigen::VectorXd/MatrixXd` though dims are compile-time (3/6/7/9) — dozens of mallocs per track per tick (`kalman_*.hpp/cpp`, `imm_filter.cpp`, `imm_tracker.cpp`) | fixed-size `Eigen::Matrix<double,N,M>` throughout; also enables vectorization |
| A3.2 | `track_manager.cpp:137` `cost_matrix_.resize(N,M)` reallocates whenever scan size changes, defeating the init-time presize | fixed `Matrix<double,32,32>` member + `topLeftCorner(N,M)` view |
| A3.3 | `track_manager.cpp:168` `S.inverse()` computed twice per pair (≤2048 3×3 inversions/scan), no conditioning check | hoist per-track `llt().solve(I)` with `info()` check; reuse Mahalanobis value as cost |
| A3.4 | `trt_detector.cpp:124-148` preprocess allocates 4 full-frame Mats per frame beside its preallocated pinned buffers | member Mats created at init; resize/convertTo/split reuse matching dst |
| A3.5 | `timestamp_associator.cpp:13` `frame.clone()` ≈6 MB @30 Hz | preallocate ring Mats at CAMERA_IMAGE_W/H; `copyTo` reuses buffer |
| A3.6 | `latest_tracks_ = *msg` deep copies in geofence/reachability/kinematic nodes; `latest_threats_ = *msg` in imm_tracker_node | store `ConstSharedPtr` (intent node is the in-repo pattern) |
| A3.7 | Missing `reserve()` on outgoing message vectors (fusion_node:152, track_manager_node:78, imm_tracker_node:183, inference_node:66-84) | reserve from known caps; per-detection `std::to_string` → init-time label table |
| A3.8 | `FusedDetection::class_label`/`Track::class_label_` are `std::string` in hot-path value types (SSO is an accident, not a guarantee; defeats FixedVector trivial-copy) | carry `int32_t class_id`; stringify at ROS boundary (DEV-005 chokepoint pattern already exists) |
| A3.9 | `imm_tracker_node.cpp:90-125` unbounded per-point loop; each unmatched point constructs a full IMMTracker (~10 Eigen allocs via A3.1) | cap points/scan with named constant + overflow counter |
| A3.10 | 64 KiB payload buffer zero-filled `{}` per frame (`radar_parser_node.cpp:425`) — `read_exact` overwrites the used region | drop the `{}` (free WCET win; keep the DEV-005 comment) |

Residual allocation inside rclcpp/rosidl publish machinery: covered by deviation DEV-006 (§B).
Display-path allocations (visualizer clone-per-frame, ostringstream label churn, overlay/capture
clones): DEV-012 (§B) — except `overlay_engine.cpp:308` "T<n>" via ostringstream, which has a free
`to_chars` replacement and per Compliance:2020 cannot be deviated (A-fix).

## A4 — Build & enforcement (P10)

**A4.1** Eigen/OpenCV/CUDA/GST includes not `SYSTEM` — the mechanism behind 88/90 baseline warnings
(`CMakeLists.txt:33,90,98-103,196-228,263-289,461-464`; note trt_detector's PUBLIC include
propagates non-SYSTEM to inference_node). Fix: `SYSTEM` keyword per target; drop dir-scope l.33. VH.
**A4.2** No `-Werror` anywhere. Add after A4.1, ideally as `CUAS_WERROR` option ON in CI. VH.
**A4.3** No `-Wshadow` project-wide. Add to `CUAS_SAFETY_FLAGS`. VH.
**A4.4** Six non-ROS libs violate the file's own stated policy (comment l.62) by getting SAFETY not
STRICT flags: `trt_detector, fusion_engine, tracking_lib, classification_lib, geofence_engine,
reachability_engine` — the hot-path math where narrowing matters most (M11 proxies
`-Wconversion -Wsign-conversion` never run there). Promote + burn down resulting warnings. VH.
**A4.5** Test targets compiled with no warning flags (`CMakeLists.txt:454-505`). Apply SAFETY flags. VH.
**A4.6** No `CMAKE_BUILD_TYPE` default ⇒ colcon builds -O0: the latency budget is measured on an
unrepresentative binary. Default `RelWithDebInfo`. VH.
**A4.7** Hardcoded machine paths (`OpenCV_DIR`, `/usr/local/cuda-12.6/...`) + unchecked
`find_library` results (`NVINFER_LIB-NOTFOUND` ⇒ cryptic link error). Use `find_package(CUDAToolkit)`,
guard with FATAL_ERROR. VH.
**A4.8** `package.xml` missing eigen/libopencv-dev/gstreamer deps; TensorRT/CUDA documented as
manual prerequisites; `rviz2` should be exec_depend. VH.
**A4.9** Launch files hardcode `/home/zork/...` engine path (full.launch.py:107,117; sim.launch.py:76)
and capture dir default. `DeclareLaunchArgument`. VH.
**A4.10** CMakeLists header comment (l.38) states exceptions/RTTI are disabled (they are not — that
is the deviation) and cites MISRA C++:2008 numbering ("18-4-1", which is the *heap* rule there).
Reword; cite 2023 IDs. VH.

## A5 — Tests & traceability (D6/D7)

**A5.1 [F] TimestampAssociator failures = stale tests, not code**: commit 8652f22 deliberately
widened the association window to 150 ms (`constants.hpp:41`, argued in docs/latency_budget.md
§jitter) but `test_timestamp_associator.cpp:45,76` still hard-code the superseded 50 ms
requirement. `findBestMatch`'s strict-greater rejection is correct. **Fix**: derive test bounds
from `MAX_TIMESTAMP_DELTA_NS ± 1`, rename tests; if the ICD contractually requires 50 ms, revert
the constant instead — either way, kill the duplicated literal. VH.
**A5.2 [F] ThreatClassifier failure = test fixture bug**: `make_track` helper defaults its 5th
parameter to `TrackState::CONFIRMED`; the "radar-only" fixture therefore earns the +0.2 CONFIRMED
term (0.3+0.2 = 0.5 observed). Formula is pinned by the *passing* QualityScoreFull test and
unchanged since d2becbd. **Fix**: pass TENTATIVE explicitly at `test_threat_classifier.cpp:79`;
remove the default so every test states track state. VH.
**A5.3** Four orphaned test files are **empty stubs whose header comments claim verification that
does not exist** (`test_kalman_cv.cpp`, `test_imm_filter.cpp`, `test_wgs84_transform.cpp`,
`test_fusion_pipeline.cpp` — include + empty namespace only). The estimation stack — the
mathematical core — has zero coverage. In a DO-178C frame these are false artifacts. **Fix**:
implement real tests for kalman_cv/imm_filter/hungarian (pure math, no hardware) + register;
wgs84 blocks on A6.2 (file is an empty placeholder). VH.
**A5.4** No structural-coverage measurement, no CI gate (see Step 5 recommendations).

## A6 — Dead code & hygiene

**A6.1** Duplicate simulator stack not in any CMake target: `src/sim_radar_node.cpp` +
`src/radar_sim.cpp` + `radar_sim.hpp` — same class/node names as the built one but a *different
parameter set*, plus its own latent bugs (mutable function-local static RNG shared across
instances; float32 absolute-seconds dt corruption). Delete (git preserves). VH.
**A6.2** `georef/wgs84_transform.cpp` + `georef_node.cpp`: empty placeholders, not built (the one
clang-tidy "error" is this file missing from the compile DB). Delete or implement. VH.
**A6.3** Empty leftover `src/cuas_msgs/` directory. Remove. VH.
**A6.4** Dead remap `('/camera/image_raw','/camera/image_corrected')` on cuas_overlay_node
(full.launch.py:192) — node has no such subscription. Delete line. VH.
**A6.5** Dead code batch: vacuous `switch` in `imm_tracker.cpp:39-50`; unreferenced
`kOccludedTimeout/kLostTimeout` (the OCCLUDED/LOST lifecycle they imply is unimplemented — the
`case OCCLUDED` in update() is unreachable, and node-level deletion duplicates a magic `5.0`);
`initialized_` flags written never read in all three Kalman classes (check-or-remove);
dead store `matched_fd = nullptr` (`threat_classifier_node.cpp:144-146`). VH.
**A6.6** `trackStateFromString` silently maps unknown strings to TENTATIVE
(`common/types.hpp:42-52`) — masks producer bugs, and `track_state_ids.hpp` lacks COASTED/DELETED
so those states can't round-trip the message boundary. Sentinel return + add/delete IDs. VH.
**A6.7** Magic-number batch (JSF AV 151 hygiene): visualizer layout/thresholds
(cache bin 50 px, zone radius 3 m, bearing gate 15°, lookahead 3 s, table geometry…), CoT endpoint
(`239.2.3.1:6969`, TTL 32, stale 30 s, rates 1/5 s), threat ENGAGED gate (`range<2 && dwell>2`),
sim RCS model (5.0/2/8), IMM node horizon/delete `5.0`s, `kConfirmHits=5` vs
`TRACK_CONFIRM_HITS=3` (undocumented divergence), duplicated `kPpiFovHalfRad` hand-derivation,
π locals → `kPi` in constants.hpp. VH.
**A6.8** `std::bind` → explicit-capture lambdas at every subscription/timer (M22-friendlier,
analyzer-visible; the underlying std::function stays under DEV-009). Leaf node classes → `final`
(D4 sealed hierarchies). TRT deleters: function-pointer deleter type + twice-spelled lambda →
stateless functor like the CUDA trio. TRT logger discards kERROR (latch into atomic + fixed
buffer for diagnostics). Waypoint cache never evicts dead track IDs — after 32 lifetime IDs new
tracks silently lose trajectory arcs (`cuas_overlay_node.cpp:61-72`; use `erase_if` against live
set). Health monitor float32 seconds quantization breaks staleness math at ~194 days uptime
(`health_monitor_node.cpp:70-73` — store int64 ns, subtract before narrowing). Mixed time bases in
one message (`imm_tracker_node.cpp:68,152,176` — steady vs ROS clock; ICD one base). P4
function-length: `imageCallback` 562 lines, `publishMarkers` 166, `dbscanCluster` 106,
`TrtDetector::init` 99, `parse_loop` 86, `render` 82, `track_callback` 113 → extract helpers,
behavior-preserving. Assertion density ≈0 project-wide → introduce `CUAS_CHECK` (log + error
return, never abort, per P5/Holzmann) + `static_assert`s (threshold ordering, EMA alpha range). VH.
**A6.9** Visualizer mutex held across the entire ~560-line render — every data callback blocks
behind OpenCV drawing (`cuas_visualizer_node.cpp:188`). Snapshot-under-lock, render-outside. VH.
**A6.10** Threat impact point computed with hardcoded 5 s horizon while the same report stamps a
per-level horizon (10/7/5/3 s) — inconsistent output (`threat_classifier_node.cpp:168 vs 201`). VH.
**A6.11** Reachability float-equality branch `if (b != 0.0F)` — catastrophic cancellation for tiny
|b| (`reachability_engine.cpp:70`); epsilon guard. Clutter/candidates/waypoints silent truncation →
overflow counters into health/ClutterStatus (D9 telemetry). VH.

---

# B) VIOLATIONS — LEAVE ALONE (formal deviation records)

Per MISRA Compliance:2020 §4.2 every record states: guideline; circumstances; **Reason** (one of:
R1 code quality, R2 access to hardware, R3 adopted-code integration, R4 non-compliant adopted
code); background; risk assessment + precautions; locations. Sign-off: project owner (J. Lopez) —
records below are drafted for that signature. Existing DEV-001..005 (docs/CODING_STANDARD.md) are
upgraded in place per A-item below.

**DEV-001 (existing, upgraded) — Exceptions/RTTI not disabled** · MISRA 18.x family / JSF AV 208 ·
R3 adopted code: rclcpp/rosidl require exceptions & RTTI; `-fno-exceptions` will not link. Owned
code contains zero throw/try/catch (verified by audit). *Risk*: a library throw in a callback
propagates to terminate with no fault record. *Precaution being added (A2.5)*: catch-all handler
in every `main()` logging FATAL with defined exit code — making the fault path deterministic and
observable. *Interview line*: "I can't remove exceptions from ROS 2, so I own the boundary: no
throws in my code, one sanctioned handler per process, fault visible to the health monitor."

**DEV-002 (existing, corrected) — Eigen internal allocation** · P3/JSF AV 206 · R3 ·
The existing record claimed "all matrices use fixed dimensions" and "EIGEN_NO_MALLOC can be defined
in debug builds" — **both false today** (dynamic MatrixXd throughout estimation/prediction; the
define appears nowhere). After A3.1/A3.2 land, the record becomes true; precaution:
`target_compile_definitions(... $<$<CONFIG:Debug>:EIGEN_NO_MALLOC>)` on the strict math libs so
any dynamic Eigen temp aborts in debug — an enforced precaution instead of an aspiration.

**DEV-003..005 (existing)** — retained; add the missing §4.2 fields (reason category, residual
risk, location tags at use sites, signatory/date). DEV-005's `class_label` location disappears
when A3.8 lands (record shrinks — good).

**DEV-006 (new) — rclcpp/rosidl allocation in the publish/subscribe machinery** · MISRA 21.6.1
(Advisory)/P3/D2 · R3 · Circumstance: message types are rosidl-generated std::vector/std::string;
executor and DDS allocate internally. Owned code minimizes traffic (FixedVector state, reserve(),
ConstSharedPtr snapshots, member-message reuse); the residue is middleware-internal. *Risk*:
allocator jitter in callbacks; bounded in practice by small message sizes; full cure (TLSF
executor allocator, loaned messages, bounded IDL — see Step 5) is a roadmap item, not a blocker.
*Locations*: every `publish()`/subscription in node files (tagged `// DEV-006` at publish sites).
*Interview line*: cite design.ros2.org three-phase pattern — rules apply to code you own.

**DEV-007 (new) — `const_cast` in zero-copy image wrap** · MISRA 8.2.3 (Required) · R3 ·
`common/ros_image_adapter.hpp:25,32`: OpenCV's external-buffer `cv::Mat` ctor takes non-const
`void*`. Precaution: return path treated as read-only (const Mat contract, documented); consumers
verified read-only (cv::resize src). Alternative full compliance = ~6 MB/frame copy — rejected as
worse engineering (R1 also applies).

**DEV-008 (new) — `reinterpret_cast` to `sockaddr*`** · MISRA 8.2.5 (Required) · R2 access to
system interface: BSD sockets ABI requires it; no conforming alternative exists. Precaution:
confined to one tagged helper (`as_sockaddr`), single location (`cot_publisher_node.cpp:108`).

**DEV-009 (new) — type-erased callbacks (std::function) in rclcpp API** · P9 (no function
pointers) · R3 · Subscriptions/timers are std::function by API. Precaution: explicit-capture
lambdas (analyzable capture set, A6.8), node classes `final`, no *owned* function-pointer tables.
D-60411's "const function-pointer table" relaxation is the honest frame.

**DEV-010 (new) — unsynchronized shared state under single-threaded executor** · D1/concurrency ·
R1 · `latest_*` members written in subscription callbacks, read in timers without locks — safe
iff the node runs a single-threaded executor (all launches do). threat_classifier already
mutexes; A3.6's ConstSharedPtr swap shrinks the window. *Precaution*: constraint documented at
each node + launch; if MultiThreadedExecutor is ever adopted, this record is void and mutexes are
mandatory. (Alternative: just add the mutex — trivial cost; recommended eventually.)

**DEV-011 (new) — unbounded sequences/strings in cuas_msgs** · D1/P2 at the interface · R1
(deferred interface change) · 10 unbounded sequences + 9 unbounded strings enumerated in audit;
bounds exist as constants (32/64/128). Bounding is an interface-breaking change (regen, downstream,
rosbag compat) — deferred deliberately; the uint8 `*_id` fields already present are the migration
path. *Risk*: middleware-side allocation per publish (see DEV-006).

**DEV-012 (new) — display/test-utility path allocations** · P3/M1 · R1 code quality ·
Visualizer/overlay/capture per-frame clones, ostringstream label text, marker vector growth:
operator-display paths tolerate allocator jitter; hot tracking path unaffected. Locations listed
in audit. Exclusion: `overlay_engine.cpp:308` (to_chars alternative exists → must fix, not
deviate).

**PROT — camera files (report-only, HARD RULE: no edits)**
- `camera_driver.cpp:128-143` + `camera_node.cpp:75-87`: ~5 full-frame Mats + fresh 6 MB Image
  msg per frame at 30 Hz — the single largest steady-state allocation source in the system.
  Proposed (requires owner approval + backup/side-file/ROLLBACK protocol): preallocated member
  Mats + convert directly into msg.data.
- `camera_driver.cpp:118,128-129`: kernel-supplied `buf.index` indexes 4-slot array unchecked;
  `req.count` may be < V4L2_BUF_COUNT leaving nullptr slots.
- `camera_driver.cpp:57-171`: five unchecked ioctl/munmap/close returns (make `(void)` explicit /
  log).
- `camera_driver.cpp:118` + `camera_node.cpp:31-38`: blocking DQBUF defeats shutdown (same class
  as radar A1.11) — poll-with-timeout proposal.
- `cuas_color_correct_node.cpp:59`: `make_unique<Image>(*msg)` deep-copies 6 MB @30 Hz (~180 MB/s
  allocator traffic).
- `color_correct_engine.cpp:11-21`: NaN gain would be UB in cast — unreachable today (constexpr
  gains), document/validate precondition.
- `constants.hpp:14-15 vs 49-50`: CAMERA_WIDTH/HEIGHT duplicated as CAMERA_IMAGE_W/H — derive one
  from the other.

---

# C) ALREADY COMPLIANT — say these proactively in interviews

1. **CUDA/TensorRT resource management is exemplary** (M2/P7): every resource in `unique_ptr` with
   null-checked deleters; `shutdown()` releases in strict reverse dependency order after stream
   sync; **all five CUDA/TRT status returns on the per-frame path are checked**; zero
   `reinterpret_cast` — GPU buffers round-trip via legal `void*` `static_cast`; copy deleted +
   user dtor suppresses moves so double-free is structurally impossible; the one init-time vector
   carries an in-code DEV-004 tag — textbook Compliance:2020 traceability.
2. **Fixed-capacity container discipline** (P3): `FixedVector`/`FixedMap` for all owned steady
   state across the pipeline; `push_back` reports failure and call sites check or explicitly
   `(void)`-discard (M24 idiom); Hungarian solver owns all working storage as fixed members —
   zero per-solve allocation — and *rejects* oversized input rather than truncating, with a
   rationale comment.
3. **Bounded parsing under P2**: TLV loop offset-bounded; `sync_to_magic` resync provably correct
   (0x02 only at magic position 0); `totalPacketLen` plausibility gate; compile-time
   `static_assert(sizeof(FrameHeader) == HEADER_SIZE)` layout proof against the documented TI SDK
   format; DBSCAN frontier capped by container capacity. Every loop in inference/overlay/health
   audited to a static bound (PPI dash loop strictly decreasing — provably terminates).
4. **Zero** goto, setjmp, unions (owned code), C-style/functional casts, `printf`-family, naked
   new/delete, non-const globals; `nullptr` exclusively; recursion: one instance (A2.10), now
   known. All conversions are visible `static_cast` to explicit fixed-width types
   (`float32_t/float64_t` JSF-AV-style aliases project-wide).
5. **Monotonic time discipline** (D10): RCL_STEADY_TIME / steady_clock / CLOCK_MONOTONIC in every
   liveness, dwell, and rate-limit computation — wall-clock jumps cannot break the watchdog.
   prediction_mux deliberately avoids rclcpp::Time's throwing subtraction with raw ns arithmetic
   and documents why.
6. **Three-phase ROS 2 real-time pattern** respected: all pubs/subs/timers/params constructed in
   ctors (init phase); steady-state callbacks audited separately — which is what makes the P3
   findings above a finite, fixable list.
7. **Per-target warning-flag architecture with documented two-tier policy** and deviations recorded
   *at the point of decision* in the build file, cross-referenced to docs/CODING_STANDARD.md — the
   practice is right; the audit tightens its execution (A4.x).
8. **Named-constant culture**: gating thresholds with engineering rationale and units in names
   (`kConfirmedGateChi2`, `kMaxPhysicalAcceleration`, `kSimRadarNoiseSigmaM`); overlay_engine is
   fully literal-free including a compile-time-derived constant with WHY comment; message-field
   ids typed `uint8_t` matching IDL.
9. **Defensive patterns already in place**: `std::from_chars` (no-exception parsing) in two nodes;
   health monitor bounds-checks every topic index; clutter engine bounds-checks every grid access
   and coerces `learn_frames(0)`→1; geofence guards ray-cast division + clamps edge-distance t;
   reachability guards radicand and distance>0; occlusion adds identity R before inverse
   (guaranteed invertible); intent node validates rate before division; geofence FATALs on
   unloadable config rather than limping (the fail-loud pattern A2.6 extends).
10. **Switch-enum completeness** (M26): `-Wswitch-enum` on all targets and every audited switch
    enumerates all cases with returns on all paths; getenv Mandatory rules (25.5.2/3) satisfied
    where used; atomic-guarded thread shutdown with join-in-dtor in both threaded nodes.
