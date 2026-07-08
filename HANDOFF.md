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

- Radar R1.5 (NEEDS-HARDWARE): VMIN=0/VTIME or poll() on the serial fd,
  reopen-with-backoff in `parse_loop` (mirror camera_node's retry), and a
  dedicated `RADAR_MAX_POINTS_PER_FRAME` (raw points currently borrow
  `TRACK_MAX_TRACKS` = 32, dropping points 33+ before DBSCAN).
- `THREAT_DRONE_CLASSES` / `THREAT_BENIGN_CLASSES` in constants.hpp are
  string_view arrays of numeric-id strings; after A3.8 they may be unused —
  verify before deleting (was not checked this session).
- `cuas_visualizer_node.cpp` still has its own file-local `parseClassId`
  duplicating the shared one in types.hpp — harmless, tidy someday (R11).
- `FUSION_MAX_CLASSES` may be orphaned after the EMA rework — verify.
- R11 list (visualizer split, magic numbers, std::bind→lambdas, `final`,
  CUAS_CHECK macro) and Step 5 (CI first — biggest signal) are untouched.

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
