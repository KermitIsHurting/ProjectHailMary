# PICK UP HERE — 2026-09-02 polish audit (read this block first; the 2026-07-07 dump follows)

**Branch:** `polish` (from `misra-audit-fixes` at `ca48665`), NOT pushed. Tags: `pre-polish-2026-09-01`,
`pre-review-20260902T0144Z` (the audit baseline; `git diff pre-review-20260902T0144Z..HEAD` is the
whole change). Backups: `~/backups/` (tree tgz + bundle + rosbags, sha256 in `docs/audit-plan.md` §0).

**What happened:** playbook §15 audit. A0 backup → A1 ground truth → A2 map → A3 seven review
agents (80 code findings + 51 doc rows → 37 root causes, +RC-38 found in B6) → A4 triage → A5 seven fix batches, one checkpoint commit
each, gates per batch → role-6 pass on the cumulative diff → A6 shape runs → this record.
The running record is `docs/audit-log.md` (append-only, UTC). The plan with every table is
`docs/audit-plan.md`. Facts sheet: `docs/resume-facts.md`. Retro: `docs/polish-retro.md`.

**State of the tree (`12c19fb` + the A7 docs commit):** build 0 warnings (-Werror), 160 GoogleTest cases green (colcon test-result 182 incl. suite rows),
cppcheck 6 (baseline), sim smoke exit 0 with health NOMINAL. Checkpoints: B1 `94508df` · B2 `0359168` ·
B3 `20ea851` · B4 `368d770` · B5 `8c6d7a4` · B6 `b742468` · B7 `67906aa` · A6/R6 fixes `12c19fb`. Session died once mid-audit (Jetson CPU-complex watchdog,
see audit-log 02:44Z); everything after B1 was rebuilt by the next session from the on-disk diff.

**Owner decisions still open (do not build them silently):** D-0 keep `78d613d`/`ca48665`;
D-1 `ROS_LOCALHOST_ONLY`; D-2 threat class policy (any non-person COCO class > 0.5 = THREAT;
IDENTIFIED requires a *person* label, so a drone never escalates — A6 showed every scene stuck at
TRACKED); D-3 de-escalation; D-4 clutter relearn and the cell-size/threshold number (`clutter` scene: 3 learned reflectors still keep 3 CONFIRMED tracks); D-5 extrinsics translation sign; D-13 geofence hysteresis (edge chatter at 10 Hz).

**Hardware shapes for John (watched, readout open):** S-7 camera-only / radar-only, S-9 unplug the
radar and the camera, S-12 `full.launch.py` production defaults. `scripts/_smoke_sim.sh <outdir>`
and `scripts/_a6_shape.sh <scene> <outdir>` are the harnesses (gitignored `_` prefix).

**Traps added this audit:**
- Build with `--parallel-workers 1 MAKEFLAGS=-j3` after touching a shared header; a 6-way full
  rebuild is the load that preceded the watchdog reset.
- Never put a process pattern (`install/cuas_fusion`, `rviz2`, `ros2 launch cuas`) on your own
  shell's command line while a cleanup `pkill -f` can run — it kills you. Use the script files.
- After any launch: `bash scripts/_a6_shape.sh`-style cleanup, then `pgrep -af install/cuas_fusion`
  must be empty; a surviving graph makes the next launch fail (owner-reported).
- The sim publishes ONE centroid per target (ICD: cluster centroids). Raw multi-return scenes
  spawn twin tracks in the one-to-one associator unless the duplicate-return guard catches them.
- `radar_frame` is X = azimuth, Y = range; every bearing goes through `common/bearing.hpp`.
- Two real targets closer than 0.8 m merge into one track (duplicate-return guard, R6b-4); they
  split again once apart. Hard limit, by design.
- Bag replay: the tracker anchors its clock to the newest cloud stamp; a >1 s backward jump
  (replaying again, `--loop`) resets the track table with one WARN. The April bags replay only
  `/radar/detections` (their cuas_msgs topics no longer deserialize).
- Radar replug: the parser reopens the port but the sensor comes back idle — re-run
  `scripts/send_radar_config.sh` (unverified on hardware, R6 F5).

---

# HANDOFF — session knowledge dump (2026-07-07)

Written at the end of the session that completed RESUME_PLAN items R1–R10 and
R12 (44 commits, ~00:10–07:25). Everything here is what a fresh session would
NOT learn from the other documents: decisions, rationale, environment quirks,
and traps. Facts you can derive from the code or AUDIT_REPORT.md are not
repeated.

## 1. State of the branch

- `misra-audit-fixes`, 63 commits ahead of `main`, **not pushed** (rule).
- From-scratch build of both packages (`cuas_msgs`, `cuas_fusion`) is clean:
  **0 compiler warnings with `-Werror` ON** (`CUAS_WERROR` option), and
  **108/108 tests green**. The only stderr in a clean build is colcon's
  benign `CATKIN_INSTALL_INTO_PREFIX_ROOT` CMake notice — not ours.
- `build.log` at repo root is stale; trust `log/latest_build/`.
- Run `colcon test` only after `source install/setup.bash`.

## 2. Environment quirks (will bite you if unknown)

- **This Jetson has no `nvcc`** (runtime-only CUDA image). CMake's
  `find_package(CUDAToolkit)` hard-fails ("Could not find nvcc") even with
  `CUDAToolkit_ROOT` set. That is why CMakeLists uses guarded
  `find_path(CUDA_INCLUDE_DIR …)` / `find_library(CUDART_LIB …)` against the
  versionless `/usr/local/cuda` symlink. It looks less idiomatic; it is the
  only thing that works here. Do not "modernize" it back.
- **`CMAKE_BUILD_TYPE` now defaults to RelWithDebInfo.** Before R9.2 it was
  unset (-O0). Turning on -O2 surfaced GCC `-Wnull-dereference` false-ish
  positives inside dynamic-size Eigen code — the fix was moving that code to
  fixed-size types, not suppressing the warning. If a new dynamic
  `MatrixXd`/`VectorXd` local in an -O2 TU warns, that is the same disease.
- Full clean rebuild of cuas_fusion takes ~5.5 min on this box; incremental
  single-TU cycles are 3 s–2 min. Tests add ~3.5 s.
- Camera/radar hardware may be attached but was **not** exercised this
  session; all camera verification was build-and-inspection only.

## 3. Clock domains (the single most important system fact)

There are TWO time bases in flight and they are NOT comparable:

- Camera frames and radar frames are stamped **CLOCK_MONOTONIC**
  (`camera_driver.cpp` `grabFrame`, radar `monotonic_stamp()`); inference
  copies the camera stamp.
- `imm_tracker_node` stamps `/tracks` with **`this->now()`** (ROS system
  clock).

Consequence: fusion_node's YOLO staleness gate (A1.7) deliberately compares
against **local monotonic arrival time** (`latest_yolo_rx_ns_`,
`kYoloMaxAgeNs` = 500 ms), never message stamps across streams. Any future
"simplification" to stamp math across these streams is wrong until the
whole pipeline is moved to one clock.

## 4. Design decisions made this session, with the why

- **Fusion EMA (A1.8)**: pool of `TRACK_MAX_TRACKS` slots keyed by *nearest
  valid same-class state within `kEmaGateM` = 2.0 m*; eviction after
  `kEmaTimeoutNs` = 1 s of **measurement time** (radar stamps), not wall
  time. When no slot is free, the longest-unrefreshed slot is recycled.
  Gate chosen as ~one inter-scan displacement at 20 m/s / 10 Hz.
- **TrackManager (A1.9)**: Kalman gain now actually applied
  (`pos += K*innov`) via `LLT` with `info()` check; a failed factorization
  (NaN covariance) re-anchors on the measurement with reset P. M-of-N:
  `hit_count` **saturates** at `TRACK_CONFIRM_HITS` and is never reset by
  misses; CONFIRMED survives misses 1..`TRACK_COAST_MISSES`-1 still
  publishing, becomes COASTED (unpublished) at 2, is deleted at
  `TRACK_MAX_MISSES` = 5; one hit re-confirms instantly from COASTED.
- **Hungarian solver**: `solve()` takes
  `Eigen::Ref<const MatrixXd, 0, OuterStride<>>` (`CostMatrixRef`) so the
  fixed 32×32 member's `topLeftCorner(N,M)` binds **without a copy**. A
  plain `Ref<const MatrixXd>` would silently heap-copy blocks (outer stride
  mismatch) — that stride parameter is load-bearing.
- **Predictor is CV-only (A1.13)**: the CV/CA/CT "models" were byte-identical
  integrators and the blend was discarded; more importantly Track.msg
  carries no acceleration/turn-rate, so CA/CT would have nothing to
  integrate. Publishers now report `model_weight_cv=1.0, ca=ct=0.0`.
  `GhostTrack.model_weights` is gone. If someone wants real CA/CT, the
  message interface has to grow first.
- **`clamp_prediction_steps()`** (param_utils.hpp) is shared by kinematic and
  occlusion predictor nodes; `kMaxTrajectorySteps` = 64 is the one cap
  (`PREDICTION_MAX_STEPS` was deleted, it was 128 and never used).
- **Class ids (A3.8)**: hot-path types carry `int32_t class_id`, `-1` =
  unlabeled. ROS boundary chokepoints live in `common/types.hpp`:
  `parseClassId` (empty/unparseable → -1) and `classIdToLabel` (-1 → empty
  string). The empty-string ↔ -1 round-trip is what preserves "unlabeled"
  semantics across the wire — don't publish "-1".
  `imm_tracker_node` still publishes the literal `"unknown"`, which parses
  to -1 downstream; that is intentional, not a bug.
- **Estimation stack (A3.1)**: all fixed-size via `common/eigen_types.hpp`
  (Vector6d/Matrix6d, 7, 9; measurements Vector3d/Matrix3d). Public APIs take
  the fixed types; the existing tests still pass `VectorXd`/`MatrixXd` and
  bind through Eigen's converting constructors — no test changes were
  needed, which was the point. `EIGEN_NO_MALLOC` is defined on
  `tracking_lib` in Debug builds (DEV-002 enforcement). Do not add that
  define to test targets: test fixtures themselves construct `MatrixXd`.
- **Health monitor (A6.8)**: all timestamps are int64 **nanoseconds**;
  float32 epoch-seconds quantized after ~2^24 s (~194 days). Pattern is
  subtract-then-narrow: diff in int64, convert only the small difference.
- **IMM tracker (A1.14)**: measurement R unified to
  `kRadarDetectionSigmaM²` (0.0225) matching TrackManager; the velocity
  gate dt is floored at `kMinGateDtS` = 0.05 s because points of one radar
  cloud share a stamp (dt=0 rejected everything exactly when data was
  densest).
- **TRT (A2.8)**: init requires exactly 1 input + 1 output tensor and
  validates shapes 1×3×640×640 / 1×84×8400 against the compile-time
  buffers; the ILogger latches kERROR+ to stderr (it used to swallow
  everything, making load failures anonymous nullptrs).
- **`trackStateFromString`** has an optional `bool* recognized` out-param;
  unknown input still maps to TENTATIVE but the caller (threat classifier)
  warns. `track_state::kCoasted` = 6 and `kDeleted` = 7 were appended —
  wire-compatible because they're new values.
- **NaN discipline** used everywhere this session: comparisons are written
  negated (`!(x > eps)`) so NaN takes the safe branch; `md2 < gate` is
  NaN-safe by construction. Keep the idiom.

## 5. Camera work (R12) — what is and isn't verified

All seven fixes (a–g) are committed under the protocol in **ROLLBACK.md**
(backups in `camera_backups/`, one fix per commit, register/format values
byte-identical). Three commits are **unverified on hardware**:

- `d71de5f` (R12d) color-correct double-buffer,
- `6774651` (R12e) preallocated grabFrame Mats + hoisted Image msg,
- `7d9cd7b` (R12f) 500 ms poll() bound on DQBUF.

Verification procedure and per-commit rollback table are in ROLLBACK.md.
Until someone runs `full.launch.py` and checks image quality + ~30 Hz +
inference + prompt Ctrl-C exit, treat these as provisional. R12e detail
worth remembering: in-place `convertTo` with a depth change reallocates
every call — that's why there are separate 16-bit and 8-bit channel plane
members.

HARDWARE_TEST_PLAN.md §7 (PROTECTED-diff-must-be-empty) is intentionally
obsolete; §7b (register-values invariant) replaces it.

## 6. Known-open items (beyond RESUME_PLAN's banner)

### 6.1 Radar hardening, R1 item 5 — NEEDS-HARDWARE, three distinct fixes

All in `src/drivers/radar_parser_node.cpp`. They were deliberately skipped
because none can be *observed* without the live IWR6843ISK; the software
changes themselves are straightforward.

**(a) Blocking serial read defeats shutdown.** `open_port()` sets termios
`VMIN=1/VTIME=0`, so `::read()` in `read_exact()` blocks until at least one
byte arrives. A radar that goes silent (powered but not streaming, or config
port never sent the profile) parks the parse thread inside `read()`, where it
cannot check `running_` — the destructor's `join()` then hangs and Ctrl-C
doesn't exit. This is the exact same failure class as the camera DQBUF hang
that R12f fixed (commit `7d9cd7b`); mirror that fix: `poll()` on `fd_` with a
timeout before each read (preferred — keeps termios untouched), or switch to
`VMIN=0/VTIME=5`. Verify on hardware: start with the radar silent, and
unplug it mid-run — Ctrl-C must exit promptly both times.

**(b) Parse-thread errors leave a zombie node.** Every failure path in
`parse_loop()` is `break` — the thread exits, but the node keeps spinning and
looks alive to ROS while publishing nothing. Contrast `camera_node.cpp`'s
`capture_loop()`, which closes, then retries `open()` every
`CAMERA_RETRY_INTERVAL_MS` forever — that loop is the in-repo pattern to
mirror (reopen with backoff instead of `break`). While there, consider
publishing a health signal so the health monitor sees the outage rather than
inferring it from silence.

**(c) Raw radar points are capped at 32 by the wrong constant.**
`filterPoints`/`dbscanCluster` use
`FixedVector<DetectedPoint, TRACK_MAX_TRACKS>` — but `TRACK_MAX_TRACKS` (32)
is the *post-clustering track* cap, borrowed for *pre-clustering raw points*,
which is a different concept. The IWR6843 can emit far more than 32 points
per frame in a dense scene; points 33+ are silently dropped **before**
filtering and DBSCAN, so exactly the frames with the most information lose
data. Fix: a dedicated `RADAR_MAX_POINTS_PER_FRAME` (plan says 256) threaded
through `filterPoints`, `dbscanCluster`, and its `label`/`neighbors` arrays.
Two cautions: DBSCAN here is O(n²) in distance checks (256 points ≈ 65k
pairs/frame — measure CPU on target), and the *output* (centroids) can stay
capped at `TRACK_MAX_TRACKS` since downstream is sized for that. It's
buildable and testable in software; it's tagged NEEDS-HARDWARE because the
behavior change should be observed against real dense returns.

### 6.2 Dead constants — now VERIFIED dead (grep-confirmed 2026-07-07)

`THREAT_DRONE_CLASSES`, `THREAT_BENIGN_CLASSES`, and `FUSION_MAX_CLASSES` in
`constants.hpp` have **zero references outside their own definitions**.

- The two THREAT arrays (string_view lists of COCO numeric-id *strings*,
  e.g. "14"/"4"/"33") look like an aspirational class-based threat policy
  that was never wired in: `threat_classifier.cpp` classifies on
  `class_id == 0` (person) and confidence only, and never consulted these
  lists even before A3.8. Two honest options: delete them (safe, one
  dead-code commit in the A6 style), or actually implement the drone/benign
  class policy they imply inside `ThreatClassifier::classify` — in which
  case convert them to `int32_t` arrays first (the string forms are
  incompatible with the post-A3.8 `class_id` world).
- `FUSION_MAX_CLASSES` (80) sized the old class-keyed EMA `FixedMap` that
  A1.8 (commit `fd691cc`) replaced with the identity-keyed slot pool sized
  by `TRACK_MAX_TRACKS`. Nothing uses it. Delete.

### 6.3 Duplicate `parseClassId` in the visualizer

`cuas_visualizer_node.cpp:71` has a file-`static` `parseClassId` that is
byte-for-byte the same logic as the shared `cuas::parseClassId` added to
`common/types.hpp` by A3.8 (commit `6f8c6ca`). It compiles cleanly because
the file-static shadows the namespace-level inline. Harmless, but it's the
kind of drift that bites when one copy gets a fix. Cleanup: delete the local
one and rely on the types.hpp include (verify the include is present; the
call site is line ~315). Left alone this session only because the visualizer
wasn't otherwise being touched and R11 owns visualizer cleanup wholesale.

### 6.4 R11 "interview polish" list — untouched, in rough value order

- **CUAS_CHECK macro + assertion density (JPL P5).** P5 wants ~2 assertions
  per function; the codebase relies on early-return guards instead. A
  `CUAS_CHECK(cond)` log-and-return macro would make preconditions visible
  and countable. Design decision needed first: what does a failed check do
  in a node callback vs. a pure math class (log+return vs. return false)?
- **The two different confirmation thresholds.** `IMMTracker::kConfirmHits`
  = 5 (imm_tracker.hpp:63) vs `TRACK_CONFIRM_HITS` = 3 (constants.hpp:56,
  used by TrackManager). Two trackers, two confirm policies, no comment
  saying whether that's deliberate. Either unify or document why they
  differ — silently different is the only wrong state.
- **Magic-number batch**: visualizer layout literals, the threat ENGAGED
  gate, the sim RCS model, and a local pi instead of a shared `kPi`.
- **`std::bind` → explicit-capture lambdas** across all node constructors,
  and `final` on all node classes (enables devirtualization, documents
  no-inheritance intent).
- **Visualizer structural work** (`cuas_visualizer_node.cpp`, 956 lines):
  split the ~562-line `imageCallback` into draw helpers; the mutex is held
  for the entire render, which blocks every data callback for the frame
  duration — take a snapshot of the shared state under the lock, render
  outside it; add ROI-vs-frame-size guards (a frame narrower than ~320 px
  currently throws `cv::Exception` out of the callback — it's caught by the
  DEV-001 boundary now, but that kills the node instead of degrading).

### 6.5 Step 5 — suggested improvements, none started (suggest-only)

Ranked by impact/effort in the original plan; the ranking still holds:
(1) **GitHub Actions CI** — build + `colcon test` + cppcheck + clang-tidy
gates; biggest external signal, ~half a day; note the runner needs a
non-Jetson build path or a self-hosted runner because of the CUDA/TRT and
aarch64 dependencies — stubbing the TRT lib or gating those targets behind
an option is the pragmatic first step. (2) DEVIATIONS.md at repo root
(R10's records could be lifted out of docs/CODING_STANDARD.md). (3)
Requirements traceability matrix REQ-ID → code → gtest anchored on
docs/ICD.md. (4) gcovr coverage with a stated statement/decision target.
(5) Fault-injection tests (corrupt TLV frames, NaN measurements — several
already exist on this branch: the TLV bounds test gap is the notable
missing one). (6) Sensor-degradation-modes doc (what the system does when
radar/camera dies; the health monitor + camera/radar retry work is most of
the substance already).

## 7. Process rules that were in force (keep them)

- Edit → build (0 warnings) → test (all green) → commit with the audit ID.
  Never batch unrelated findings into one commit.
- Never push; owner merges.
- When a fix needs a regression test, the test asserts the *old failure
  mode* is impossible (e.g. two-same-class-targets non-convergence), not
  just that the new code runs.
- FixedVector's `operator[]` clamps out-of-range instead of UB; the
  `idx < size()` contract is still the caller's. `resize()` shrink just
  drops size; growth value-initializes with `T()` (not `T{}` — rosidl types
  have an explicit-init ctor that braces would select).
- The live radar sim is `src/drivers/sim_radar*`; the near-duplicate that
  used to sit at `src/sim_radar_node.cpp` was dead and is deleted — don't
  resurrect it from git history by accident.

## 8. OVERHAUL progress (2026-07-07 evening session, post-freeze recovery)

Written by the session that resumed after the machine froze mid-P3.1 build.
Read OVERHAUL_PLAN.md for the roadmap; this section is only what a fresh
session cannot get from it or from git log.

- **State**: P1.1, P1.2, P2.1, P2.2 committed by the earlier session; the
  freeze (19:40 build, died at 49%) left P3.1 as uncommitted WIP. It was
  recovered, completed, and committed as `a4f8e42` (P3.1: Track/
  FusedDetection growth + every cuas_msgs sequence bounded, DEV-011
  narrowed to strings, VERSION 0.9.0-alpha). `0a06f1c` (P3.2) added
  `tracking/measurement_models` — radar [pos, r_dot] and camera bearing
  models with finite-difference-verified Jacobians. Test count is now
  **136** by colcon's accounting (was 108 at §1's writing; the P1/P2
  session and P3.1/P3.2 added suites).
- **Bounded-sequence trap**: message arrays are now
  `rosidl_runtime_cpp::BoundedVector`. `push_back` past the bound THROWS
  `std::length_error` (caught by the DEV-001 boundary = node death, not
  drop). Every current producer is structurally capped below its bound —
  keep it that way when adding producers; the bounds are commented in the
  .msg files.
- **NEXT = P3.3 (association / central tracker core). Open design fork,
  decide before coding**: the KEPT kalman filters only expose
  `update(Vector3d z, Matrix3d R)` (position-only). The central tracker
  needs generalized `(z, H, R)` updates in each sensor's measurement
  space (radar 4-dim with Doppler, camera 2-dim bearing). Options:
  (a) extend KalmanCV/CA/CT with an update taking H over the shared
  6-block (internally lifted to the 9/7-dim private states) — touches
  tested estimation code, do it behind the existing tests; or (b) give
  the central tracker its own per-track 6-state EKF update and use the
  filters only for predict/mixing — less invasive, but splits the update
  math into a second place. Either way the P3.2 models supply h/H/R and
  the P3.1 wire format is already sufficient — Track.msg needs NO further
  change for P3.3–P3.6.
- **P0 caveat still stands**: zero live hardware runs on this branch.
  Everything in Phase 3 is built behind unit tests; gate sizes and noise
  constants (`kRadarDopplerSigmaMps`, `kCameraPixelSigmaPx`) are
  estimates to be refined against the Phase-0 bag corpus when hardware
  time happens.
