# RESUME_BULLETS — evidence-backed candidate bullets for ProjectHailMary

Generated 2026-09-01 against branch `misra-audit-fixes` @ `d8a5fa7` (102 commits,
78 ahead of `main`). Every number below was traced to a file, a commit, a config
line, a CSV, or a command run on this Jetson. Where a number could not be traced,
it is called out as such and not used.

Conventions used for the counts (so the numbers can be reproduced on demand):

| Quantity | Command / source | Value |
|---|---|---|
| C++ LOC, owned code (no tests) | `git ls-files 'src/cuas_fusion/src/*' 'src/cuas_fusion/include/*' \| grep -E '\.(cpp\|hpp)$' \| xargs cat \| wc -l` | 11,827 |
| C++ LOC, tests | same over `test/unit/*.cpp src/cuas_fusion/test/*.cpp` | 2,283 |
| C++ non-blank, non-comment (owned) | above, `grep -vE '^\s*$\|^\s*//'` | 9,297 |
| Node executables | `grep -c add_executable src/cuas_fusion/CMakeLists.txt` | 22 |
| Nodes launched in `full.launch.py` | 19 cuas_fusion nodes + tf2 static + rviz2 = 21 processes | 19 |
| Static libraries | `grep -c add_library src/cuas_fusion/CMakeLists.txt` | 11 |
| Messages | `ls msgs/cuas_msgs/msg \| wc -l` | 19 (136 fields) |
| Topics | `docs/ICD.md` §3 headers | 26 |
| Launch files | `ls src/cuas_fusion/launch` | 7 |
| GoogleTest cases | sum of `tests=` in `build/cuas_fusion/test_results/cuas_fusion/*.gtest.xml` | 129 in 18 binaries, 0 failures (run 2026-09-01 15:44) |
| `colcon test-result` | counts the 18 binaries as tests too | 147 |
| Commits on branch citing an audit ID | `git log --format=%s main..HEAD \| grep -cE '\(A[1-6]\.\|R1[0-9]\|R9\.'` | 57 |
| Unique audit findings fixed | `git log --format=%s main..HEAD \| grep -oE 'A[1-6]\.[0-9]+' \| sort -u \| wc -l` | 47 of 60 |
| Code-change commits on branch | `fix:` 35, `perf:` 10, `build:` 5, `test:` 2 | 52 |

Hardware (checked 2026-09-01): `/proc/device-tree/model` = "NVIDIA Jetson Orin Nano
Engineering Reference Developer Kit Super"; six Cortex-A78AE (CPU part 0xd42), all
with `cpuinfo_max_freq` 1728000; nvpmodel MAXN_SUPER; 8 GB; L4T R36.5.0 (JetPack 6.2);
CUDA 12.6 runtime, no nvcc; TensorRT 10.3.0.30; ROS 2 Humble; OpenCV 4.5.4; Eigen 3.4.

Timeline from `git log`: original build 2026-03-30 → 2026-04-21 (21 commits, five
"Day N" milestone commits with live-hardware confirmations in their subjects);
docs written ~2026-04-27; audit + fixes 2026-07-06 22:58 → 2026-07-07 07:18
(≈60 commits); overhaul phases P1–P3.2 and auto-exposure 2026-07-07 17:55 → 22:16.

---

## Section 1 — Ranked candidate bullets

Ranking criteria: (a) a real defect with a before/after that can be walked through,
(b) a number that traces to a file, (c) from-scratch implementation over library
call. Bullet text is ≤ ~200 characters so it fits two lines at 10 pt Arial; terms
marked `**…**` are the bolding candidates. "Defense" is the two-sentence
walk-through; "Risk" is what a sharp interviewer will push on and the honest answer.

### 1. IMM mixing bug + CT Jacobian sign errors

**Bullet.** Fixed an **IMM CV/CA/CT** filter whose mixing step re-initialized each
model every cycle, zeroing CA acceleration and CT turn-rate (the IMM had collapsed
to CV); corrected 2 sign errors in the **CT Jacobian** ∂p/∂ω.

**Evidence.** Commit `7430a57` (A1.2, A1.14-CT). Before: `ImmFilter::predict`
called `cv_.init/ca_.init/ct_.init` with the mixed 6-state, and `KalmanCA::init` /
`KalmanCT::init` zero the full 9/7-state before copying the 6-block
(`src/cuas_fusion/src/estimation/kalman_ca.cpp:16-24`, `kalman_ct.cpp:16-24`).
After: `setMixedState` writes only `head<6>()` / `topLeftCorner<6,6>()`
(`include/cuas_fusion/estimation/kalman_ca.hpp:21-24`, `kalman_ct.hpp:21-24`),
injected at `imm_filter.cpp:80-86`. Jacobian: `kalman_ct.cpp:88-89, 93-94` (vy
terms had flipped signs; verified by the ω→0 Taylor limit against the guard branch
at `:70-73`); ω guard raised 1e-6 → 1e-3 (`kalman_ct.hpp:47`). Regression test
`test/unit/test_imm_filter.cpp:102-134` `TurningTargetShiftsWeightTowardCT`
(ω = 0.5 rad/s, r = 20 m, 6 s: CT weight must dominate; fails on old code).
State dims: CV 6, CT 7, CA 9 (`kalman_*.hpp`).

**Defense.** "The mixing step called `init()` on each sub-filter, which zeroes the
9-state CA model's acceleration and the 7-state CT model's turn rate before copying
in the shared 6-state block, so those states never survived a cycle and every model
behaved like constant-velocity. I replaced it with `setMixedState` that overwrites
only the shared block, and wrote a test that drives a circle at 0.5 rad/s and
asserts the CT weight wins, which the old code fails."

**Risk.** "Is the mode-probability update textbook?" No: `imm_filter.cpp:113-115`
multiplies by the prior μ_j, not the Markov-predicted c̄_j (that value is computed
at `:40-46` and discarded). The mixing/interaction step is textbook
Blom/Bar-Shalom; say "likelihood-weighted mode probabilities", not "full
Bar-Shalom IMM". Also: sub-filter updates use `S.inverse()` and `P = (I−KH)P`, not
LLT or Joseph form (`kalman_cv.cpp:64-66`).

### 2. Kalman gain computed but never applied; single-miss track flicker

**Bullet.** Found a tracker computing the **Kalman gain** and shrinking P, then
overwriting state with the raw measurement; replaced it with an **LLT**-factored
update and **M-of-N** lifecycle (3 hits / 2-miss coast / 5-miss delete), 5
tests.

**Evidence.** Commit `41581f0` (A1.9): old `track_manager.cpp:80-86` did
`K = P·S⁻¹; P = (I−K)·P; position = det.position` (state jumps to measurement,
covariance tightens as if blended). New `src/cuas_fusion/src/tracking/track_manager.cpp:80-107`:
`LLT<Matrix3d>` with `info()==Success` check, `pos += K·(z − pos)`, failed
factorization re-anchors with `P = I·kInitialPosVar`. Commit `ba822cc`: old
`applyMiss` zeroed `hit_count` and demoted CONFIRMED→COASTED on any miss; new
`:114-144` saturates hits at `TRACK_CONFIRM_HITS = 3`, coasts at
`TRACK_COAST_MISSES = 2`, deletes at `TRACK_MAX_MISSES = 5`
(`include/cuas_fusion/common/constants.hpp:54-58`). Tests
`test/unit/test_track_manager.cpp:62` (gain blends, old code jumped to 1.5),
`:83` (survives one miss), `:104` (re-confirms on first hit), `:126` (deleted
after max misses), `:47`.

**Defense.** "The update computed K and shrank P as if a blend had happened, then
assigned the raw measurement to the state, so the covariance was overconfident
around a full-noise position and the Mahalanobis gate started rejecting valid
returns. Now it is x += K(z − x) with an LLT factorization and an `info()` check,
and a NaN covariance re-anchors on the measurement instead of poisoning the cost
matrix."

**Risk.** This `TrackManager` (`tracker_node`) is not in `full.launch.py` or
`sim.launch.py`; the launched tracker is `imm_tracker_node`, which uses greedy
nearest-neighbour association with a 0.8 m Euclidean gate
(`imm_tracker_node.cpp:81,109-121`) and a separate 5-consecutive-hit confirm
(`imm_tracker.hpp:63`). `TrackManager` also has no predict step: P inflates only
on a miss (`track_manager.cpp:130`). Say "implemented and unit-tested; the offline
association path", not "the live tracker".

### 3. Hot-path allocation removal (camera + estimation)

**Bullet.** Cut per-frame allocation on the 30 Hz camera path (35 MB/frame of
scratch **cv::Mat** + a 6.2 MB image copy ≈ 1.2 GB/s) to zero at steady state;
estimation stack moved to fixed-size **Eigen** (≈12 mallocs/filter/scan → 0).

**Evidence.** R12e `1973322`: `src/cuas_fusion/src/drivers/camera_driver.cpp:114-121`
allocates `raw_sub_` (16UC1), `bgr16_` (16UC3), 3× `channels16_`, 3× `channels8_`
once in `open()`; per frame that is 1920×1080×(2 + 6 + 3×2 + 3×1) = 35,251,200 B.
R12d `f7ac045`: `cuas_color_correct_node.cpp:62-76` replaces
`make_unique<Image>(*msg)` (6,220,800 B) with two preallocated messages.
(35.25 + 6.22) MB × 30 fps = 1.24 GB/s. A3.1 `81d1cca`: `common/eigen_types.hpp:18-23`
(Vector6d/7d/9d, Matrix6d/7d/9d); commit message "roughly a dozen allocations per
filter per scan". A3.2 `2eef542`: `track_manager.hpp:44-51` fixed 32×32 cost matrix,
bound through `Eigen::Ref<const MatrixXd, 0, OuterStride<>>`
(`hungarian_solver.hpp:18-22`) so `topLeftCorner(N,M)` binds without a copy;
A3.3: one LLT per track instead of two 3×3 inversions per pair (≤ 2·32·32 = 2048
per scan → ≤ 32). A3.4 `4fc5ffc` (TRT preprocess Mats), A3.6 `6995956`
(`ConstSharedPtr` instead of deep-copying track arrays), A3.10 `457fc26`
(64 KiB `std::array{}` zero-fill per radar frame dropped; `radar_parser_node.cpp:463-467`).

**Defense.** "Per frame the driver allocated a 16-bit subtract buffer, a 16-bit
BGR, three 16-bit planes and three 8-bit planes, about 35 MB, and the color-correct
node deep-copied the 6.2 MB message, so the allocator saw roughly 1.2 GB/s. I made
them members allocated once in `open()`, kept separate 16-bit and 8-bit planes
because an in-place `convertTo` with a depth change reallocates every call, and did
the same in the tracker with fixed-size Eigen types and a stride-flexible `Ref` so
the 32×32 cost matrix block never copies."

**Risk.** The three camera commits are tagged NEEDS-HARDWARE and have never run
on the live AR0234 (HANDOFF.md §5); there is no before/after CPU or latency
measurement. `EIGEN_NO_MALLOC` is enforced only on `tracking_lib` in Debug builds
(`CMakeLists.txt:209`), and the default build is RelWithDebInfo. Honest phrasing:
"fixed-size by construction, Debug-build malloc trap on the tracking library;
measurement is next."

### 4. Hungarian solver NaN hang and NaN-safe discipline

**Bullet.** Proved a hand-written O(n³) **Hungarian** solver could spin forever
on a NaN cost (every comparison false); added `allFinite()` rejection, bounded
loop guards, and NaN-safe negated comparisons in all 3 **Kalman** likelihoods.

**Evidence.** `94cabb1` (A1.1): `src/cuas_fusion/src/tracking/hungarian_solver.cpp:30-32`
(`allFinite`), `:82-87` (`path_guard > k`), `:124-129` (`back_guard`);
shortest-augmenting-path with dual potentials `:68-133`, all working storage
`std::array` sized 32+1 (`hungarian_solver.hpp:38-48`). `33089ca` (A2.7):
`kalman_cv.cpp:84`, `kalman_ca.cpp:118`, `kalman_ct.cpp:193` (`!(det > 1e-12)`),
`imm_filter.cpp:122-125` (weight-sum recovery to {0.33,0.33,0.34}). Test
`test/unit/test_hungarian_solver.cpp:91-105` `NonFiniteCostRejectedNotHung`.
NaN source: a singular S in the gate (`track_manager.cpp:184-189` now prices the
row out on LLT failure).

**Defense.** "In the augmenting-path loop `cur < minv[j]` is false for NaN, so the
column index never advances and `p[j0] != 0` never clears; the tracker thread hangs
mid-scan. A NaN can reach it from a singular innovation covariance, so I reject
non-finite matrices up front, bound both loops, and write every float guard as
`!(x > eps)` so NaN takes the safe branch."

**Risk.** The header calls it "Jonker-Volgenant" (`hungarian_solver.hpp:2`); it is
Kuhn–Munkres shortest augmenting path (cp-algorithms form) without JV's reduction
passes. Say "Hungarian".

### 5. The MISRA / JPL / DO-178C-style audit itself

**Bullet.** **MISRA C++:2023** / **JPL Power-of-10** / DO-178C-style audit:
60 findings logged with file:line, 47 fixed one-per-commit (52 commits),
12 Compliance:2020 §4.2 deviation records, 3 latent Mandatory-rule paths closed.

**Evidence.** `AUDIT_REPORT.md` A1.1–A1.14, A2.1–A2.11, A3.1–A3.10, A4.1–A4.10,
A5.1–A5.4, A6.1–A6.11 = 60 IDs; severity table lists 12 functional defects,
~25 Required violations, ~15 allocation sites, 8 enforcement gaps. Fixed IDs from
commit subjects: 47 (command in the table above). `docs/CODING_STANDARD.md`
DEV-001..012 (commit `6ca9c50`, each with reason category, risk, locations,
signatory). Mandatory 11.6.2 paths: `4e37a08` (`clock_gettime` unchecked,
`common/clock.hpp:16-24`), `0ead798` (`FixedVector::back()` on empty, `resize()`
over never-written slots). `STANDARDS_CHECKLIST.md` §1: 179 guidelines, 5 Mandatory.
Baseline: 90 warnings, 3 failing tests → 0 warnings under `-Werror`, 129/129.

**Defense.** "I built the checklist from the published rule table, audited every
file against it and gave each finding an ID, rule, severity, file:line and fix,
then fixed them one per commit with the ID in the message. The three Mandatory
items were an unchecked `clock_gettime` feeding every timestamp and two
`FixedVector` paths that could read never-written storage; Mandatory rules admit no
deviation, so those went first."

**Risk.** No licensed MISRA C++ checker exists for free; `STANDARDS_CHECKLIST.md`
§4 says so explicitly (cppcheck's MISRA addon is C-only and was excluded as
evidence). Never say "MISRA compliant" or "DO-178C compliant"; say "audited
against" and "DO-178C-inspired". The older "144 violations across 20 files"
(CHANGELOG 0.7.0) has no list anywhere in the repo; do not cite it. The audit and
fixes landed in one ~8 h window on 2026-07-06/07 (see Section 3, item 12).

### 6. Radar TLV serial parser hardening

**Bullet.** Hardened a from-scratch **TI mmWave TLV** serial parser (921.6
kbaud): magic-word resync, `static_assert` layout, TLV length bounds (fixed a
frame desync), EOF handling (was a 100 % CPU spin), `memcpy` over 3 type-puns.

**Evidence.** `src/cuas_fusion/src/drivers/radar_parser_node.cpp`: magic word
`:36-38`; packed `FrameHeader` (40 B) / `TlvHeader` (8 B) / `DetectedPoint` (16 B)
with `static_assert` `:61-91`; `sync_to_magic` `:393-417`; packet-length gate
[40, 65536] `:452-459`; termios `B921600`, raw 8N1 `:336-349`. `86a411f` (A1.10):
old code advanced `floor(len/16)*16` and never checked `tlv.length` against the
remaining payload; new `:488-495` bounds every TLV, `:510` advances by exactly
`length`. `5ef7629` (A1.11): `read()==0` on USB unplug left `total` unchanged →
busy-spin; new `:381-387`. `2df7b57` (A2.4): three `reinterpret_cast` puns →
`memcpy` `:449-450, :485, :502`. `2bd7cbd` (A2.9): `strerror_r` `:46-55`.
Only TLV type 1 (Cartesian points) is decoded `:58, :497-506`.

**Defense.** "A corrupt TLV length used to advance the offset by
floor(len/16)·16, so trailing bytes parsed as the next header and garbage points
were published; now every TLV is checked against the remaining payload and the
offset advances by exactly the declared length. On unplug `read()` returns 0, which
left the byte count unchanged so the loop spun at 100 % CPU; it now returns false."

**Risk.** Three items are still open and tagged NEEDS-HARDWARE (HANDOFF.md §6.1):
the `VMIN=1` blocking read still defeats shutdown, parse-thread errors still `break`
to a zombie node, and raw points are capped at 32 per frame by a borrowed constant
(`:477` uses `TRACK_MAX_TRACKS`) before clustering. On the wire at 921.6 kbaud a
50 ms frame can carry ≈285 points; the parser keeps 32.

### 7. Fusion EMA keyed by class; stale boxes; stamp-based alignment

**Bullet.** Fixed fusion that keyed its smoothing **EMA** by class (same-class
targets blended to a midpoint, never evicted) and kept stale **YOLO** boxes
forever; now identity-keyed, 2 m gate, 1 s eviction, stamp-nearest boxes within
150 ms.

**Evidence.** `caca54f` (A1.8): old `FixedMap<int32_t, EmaState, FUSION_MAX_CLASSES>`
keyed by `class_id`; new pool of 32 with `associateEma` nearest same-class within
`kEmaGateM = 2.0` m and `kEmaTimeoutNs = 1 s` in measurement time
(`src/cuas_fusion/src/fusion/fusion_engine.cpp:177-218`, `fusion_engine.hpp:26-32`).
`e8bf689` (A1.7): empty detection arrays used to `return` and keep old boxes;
`latest_yolo_ts_` was written and never read. `be29e6c` (P2.2): `DetectionSetBuffer`
ring of 6 box sets, nearest within `MAX_TIMESTAMP_DELTA_NS = 150 ms` inclusive
(`include/cuas_fusion/fusion/detection_set_buffer.hpp:40-63`), tracks extrapolated
`x += vx·dt` to the camera instant (`fusion_node.cpp:150-162`). Tests
`test/unit/test_fusion_engine.cpp:74` `TwoSameClassTargetsDoNotConverge`
(two class-0 targets 4 m apart, 20 iterations), `:103` stale-state eviction;
`test_detection_set_buffer.cpp:54` (150 ms inclusive, +1 ns rejects), `:67`
(empty set is a valid observation).

**Defense.** "The EMA state was a map keyed by class id, so two people shared one
smoothed position that drifted to their midpoint, and an entry never timed out, so
a returning target blended with its minutes-old ghost. I keyed it by nearest
same-class state within one inter-scan displacement, 2 m at 20 m/s and 10 Hz, with
1 s eviction in measurement time, and wrote the two-targets test that fails on the
old code."

**Risk.** Association is projected-point-in-box with 25 % padding
(`fusion_engine.cpp:84-88`), not IoU; `FUSION_IOU_THRESHOLD` is a dead constant.
Fusion consumes `/tracks`, not raw radar points (`fusion_node.cpp:114-116`), and the
camera contributes only a label, confidence and box; no camera geometry enters any
estimate yet. Fused `range_m` is the forward coordinate, not Euclidean range
(`:167`). Say "label-level fusion".

### 8. Radar waveform: decoding the IWR6843ISK profile

**Bullet.** **TI IWR6843ISK** 60 GHz radar profile: 3TX×4RX **TDM-MIMO**, 737
MHz sweep → 0.20 m range bins, 52 m max range, 20 Hz frames, ±4.7 m/s
unambiguous Doppler in 0.59 m/s bins (quantization visible in logged
velocities).

**Evidence.** `src/cuas_fusion/config/radar_profile.cfg`: `channelCfg 15 7 0`
(4 RX, 3 TX); `profileCfg 0 60.75 30 7 57.14 0 0 15 1 256 5209 0 0 158`
(60.75 GHz start, idle 30 µs, ramp 57.14 µs, slope 15 MHz/µs, 256 samples,
5209 ksps); `chirpCfg` 0/1/2 → TX1/TX2/TX3; `frameCfg 0 2 16 0 50 1 0` (16 loops,
50 ms period); `cfarFovCfg` ±4.7 m/s, 0.25–52 m. Arithmetic: B = 15 MHz/µs ×
256/5.209 MHz = 737 MHz → ΔR = c/2B = 0.203 m; R_max = Fs·c/2S = 52.1 m
(= 256 × 0.203); per-TX repeat T = 3 × (30 + 57.14) µs = 261.4 µs → v_max =
λ/4T = 4.7 m/s at λ = 4.9 mm; v_res = 2·v_max/16 = 0.59 m/s. Cross-check:
`logs/jitter_log_centroid.csv` and `_run2.csv` `vel_mps` cluster at exactly ±0.59
(counts 20/15 and 22/17). Frame rate 20 Hz from `frameCfg`, unchanged since
`0b5539f` (2026-03-30).

**Defense.** "Range resolution is c over twice the swept bandwidth, and the
bandwidth is slope times the ADC window, 15 MHz/µs × 49 µs ≈ 737 MHz, so about
20 cm. With three transmitters time-multiplexed the per-antenna chirp repeat is
261 µs, which caps unambiguous Doppler at λ/4T ≈ 4.7 m/s, and the logged cluster
velocities pile up at exactly ±0.59 m/s, which is that span divided into the 16
Doppler bins."

**Risk.** The profile is TI's stock demo profile with light edits; say "selected
and decoded", not "designed". ±4.7 m/s aliases a real drone; `OVERHAUL_PLAN.md`
names this as the first hard limit. 52 m is the sampling-limited geometric range,
not a detection range; no log shows a target beyond 4.7 m. The docs say 16 Hz
(see Section 2, bullet 7).

### 9. Test suite

**Bullet.** 129 **GoogleTest** cases in 18 suites against pure math classes (no
ROS in tests), 100 % green under `-Werror`; ~15 pin a previously shipped failure
mode; analytic **Jacobians** checked against central finite differences.

**Evidence.** `build/cuas_fusion/test_results/cuas_fusion/*.gtest.xml` sum
`tests="129" failures="0"` (2026-09-01 15:44; identical on 2026-07-07 21:28).
`src/cuas_fusion/CMakeLists.txt:514-623` registers 18 `ament_add_gtest` targets,
each linking the math `.cpp` directly; no test file includes `rclcpp`. Finite
differences: `test/unit/test_measurement_models.cpp:52-74, 193-230`. Regression
tests that assert an old failure is impossible: `test_hungarian_solver.cpp:91`,
`test_imm_filter.cpp:102`, `test_track_manager.cpp:62/83/104`,
`test_fixed_containers.cpp:24-63` (5), `test_fusion_engine.cpp:74/103`,
`test_detection_set_buffer.cpp:54/67`, `test_timestamp_associator.cpp:77`,
`test_threat_classifier.cpp:81` → 15. `6b5e929` (A5.3) replaced four empty stub
files whose headers claimed verification that did not exist. Suite wall time
3.9 s.

**Defense.** "The math lives in classes that don't include rclcpp, so each suite
links the .cpp directly and the whole run takes four seconds. When I fixed a bug I
first wrote the test that fails on the old code, for example two same-class targets
4 m apart must still be 4 m apart after 20 EMA iterations."

**Risk.** The repo's own docs disagree on the count (97 / 108 / 136); 108 counts
only `TEST(` and omits 21 `TEST_F(`. Always say 129 and how it was counted. No
tests exist for the kinematic/occlusion predictors, the `IMMTracker` node class,
overlay/visualizer, CoT, or `TrtDetector`; no coverage measurement; no CI.

### 10. Per-sensor measurement models with verified Jacobians

**Bullet.** Per-sensor measurement models for a central tracker: radar [x, y, z,
ṙ] with a 4×6 **Jacobian** and camera bearing-only [az, el] with a 2×6 Jacobian
through **SE(3)** extrinsics; 14 tests incl. finite-difference checks.

**Evidence.** Commit `db2f87a` (P3.2). `src/cuas_fusion/src/tracking/measurement_models.cpp:25`
(ṙ = p·v/|p|), `:46-47` (∂ṙ/∂p = (v − ṙ·p/r)/r, ∂ṙ/∂v = p/r), `:113-148`
(p_cam = R·p + t, az = atan2(x_c, z_c), el = atan2(−y_c, ρ), H = [G·R | 0]);
`measurement_models.hpp:29-51`; NaN-safe range/depth guards `:21, :37, :114, :133`.
Noise: `kRadarDopplerSigmaMps = 0.2`, `kCameraPixelSigmaPx = 4.0`
(`constants.hpp:63-65`). `test/unit/test_measurement_models.cpp` (14 tests).

**Defense.** "Range-rate is velocity projected on the line of sight, so its
Jacobian is (v − ṙ·p/r)/r with respect to position and p/r with respect to
velocity. The camera model rotates and translates into the camera frame and takes
atan2 for azimuth and elevation; I checked both Jacobians against central
differences to 1e-5."

**Risk.** Not yet wired into any filter update; HANDOFF.md §8 records the open
design fork for the central tracker (P3.3), and the Kalman classes only expose a
position-only `update(z3, R3)`. The noise constants are stated estimates.

### 11. SE(3) extrinsics and an offline Gauss-Newton calibration solver

**Bullet.** Config-driven **SE(3)** radar-camera extrinsics (quaternion + t) and a
Levenberg-damped **Gauss-Newton** reprojection solver on SE(3); synthetic self-test
recovers a 2.3° mount error to 0.005° / 1 mm / 0.50 px RMS.

**Evidence.** `c3359e4` (P1.1): `include/cuas_fusion/common/types.hpp:94-132`
(quaternion → rotation matrix), `fusion_engine.cpp:61-66` (R·p + t),
`fusion_node.cpp:59-108` (YAML load, FATAL on bad quaternion, TF broadcasts the
inverse (q*, −Rᵀt)), `config/extrinsics.yaml`. `7aa1dab` (P1.2):
`scripts/calibrate_extrinsics.py:102-161` (left so(3) perturbation, analytic 2×6
Jacobian per point, λ ×0.3 on accept / ×10 on reject, 50 iterations),
`:176-213` self-test (40 points 3–14 m, 2.3° error, 0.5 px noise). Run on
2026-09-01: `python3 scripts/calibrate_extrinsics.py --self-test` → rot 0.0046°,
t 1.02 mm, RMS 0.502 px, PASSED. Tests `test_fusion_engine.cpp:147, 180, 216, 236`.

**Defense.** "Each residual is the pixel error of projecting R·p + t; the Jacobian
with respect to a left so(3) perturbation and δt is 2×6 per point, and the damping
scales down on accepted steps and up on rejected ones. The self-test plants a 2.3°
mount error and half-pixel noise on 40 points and recovers it to five thousandths
of a degree."

**Risk.** Never run on real correspondences: `config/extrinsics.yaml:17-25` still
holds the nominal +90° rotation and tape-measure offsets with a comment saying to
replace them. No lens undistortion exists in the C++ path even though k1–k3 are in
the calibration YAML. The overlay engine still uses a hard-coded axis swap and
ignores the YAML (`overlay_engine.cpp:72-91`).

### 12. V4L2 camera driver hardening under a rollback protocol

**Bullet.** Hardened a raw **V4L2** driver (10-bit Bayer, 4-buffer mmap ring):
500 ms `poll()` bound on **DQBUF** so a dead CSI link can't hang shutdown,
kernel buffer-index validation, logged teardown; sensor registers
byte-identical.

**Evidence.** `src/cuas_fusion/src/drivers/camera_driver.cpp`: `S_FMT` 1920×1080
`V4L2_PIX_FMT_SGRBG10` `:47-58`, `REQBUFS` 4 MMAP `:68-77` (`camera_driver.hpp:13`),
`kDqbufTimeoutMs = 500` `:25-28` + `poll()` `:132-148` (R12f `618f113`),
`buf.index` check `:158-165` (R12b `6030bc7`), teardown logging `:196-230`
(R12a `776935b`). Protocol: `ROLLBACK.md` (backup per file in `camera_backups/`,
one fix per commit, restore command per row); `HANDOFF.md` §5.

**Defense.** "DQBUF blocks forever when the link dies and the capture thread can't
check its running flag from inside a blocked ioctl, so the destructor's `join()`
hung; `poll()` with a 500 ms bound, fifteen frame periods, returns false to the
node's existing 1 s reopen loop. Every camera edit was backed up first, one fix per
commit, with a restore command in ROLLBACK.md, and no register or format value
changed."

**Risk.** R12d/e/f are unverified on hardware. Demosaic and white balance are
OpenCV calls (`cvtColor BayerGB2BGR`, `convertTo`), not hand-written. `ROLLBACK.md`
lists pre-rebase commit hashes that no longer exist in `git log` (the current ones
are `f7ac045`, `1973322`, `618f113`).

### 13. Scale and scope

**Bullet.** Solo-built C++17 **ROS 2 Humble** system on a **Jetson Orin Nano**
(8 GB, JetPack 6.2): 22 node executables, 11 static libs, 19 messages (136
fields), 26 topics, 7 launch files, 11.8k lines of C++ + 2.3k of tests, 102
commits.

**Evidence.** Counts and commands in the table at the top of this file.
`src/cuas_fusion/CMakeLists.txt:108-467` (22 `add_executable`), 11 `add_library`;
`msgs/cuas_msgs/msg/` (19); `docs/ICD.md` §3 (26 topic sections);
`src/cuas_fusion/launch/` (7); `/proc/device-tree/model`; `/etc/nv_tegra_release`.

**Defense.** "22 executables are built, 19 of them run in the hardware launch plus
tf2 and RViz; the other three are the offline Hungarian tracker, a frame-capture
utility and the radar simulator. LOC is git-tracked .cpp/.hpp under
`src/cuas_fusion` excluding tests, 9.3k of it non-blank non-comment."

**Risk.** The current resume says 23 nodes and 10.9k LOC; both are stale (see
Section 2). Three of the 19 messages are never published (`RadarFrame`,
`RadarDetection`, `SystemHealthArray`). The commit density on 2026-07-07 (63
commits) invites the AI-assistance question (Section 3, item 12).

### 14. Bounded IDL and covariance on the wire

**Bullet.** Bounded every **cuas_msgs** sequence to producer capacity
(Track[≤32], FusedDetection[≤128], RadarDetection[≤256]) and added a 6×6
covariance as a 21-value upper triangle with layout-anchor tests; additive, so
old bags replay.

**Evidence.** `6d3a438` (P3.1). `msgs/cuas_msgs/msg/TrackArray.msg:3`,
`FusedDetectionArray.msg:3`, `RadarFrame.msg:4`, `TrajectoryWaypoints.msg:4-10`
(`[<=64]`), `GeofenceEventArray.msg:3` (`[<=64]`), `ThreatReport.msg:17`
(`[<=16]`); `Track.msg:25` `float64[21] covariance`, `:28` `source_mask`;
`include/cuas_fusion/common/eigen_types.hpp:28-38` `packUpperTriangle6`;
`test/unit/test_eigen_types.cpp:15` (literal offsets 0, 6, 11, 15, 18, 20), `:39`
round-trip; `CHANGELOG.md` 0.9.0; DEV-011 narrowed to strings.

**Defense.** "rosidl `BoundedVector` makes the wire contract match the
`FixedVector` caps in code, so a consumer can size buffers statically. The trap is
that `push_back` past the bound throws, so every producer is structurally capped
below its bound and the bound is commented in the .msg file."

**Risk.** Five string fields remain unbounded (DEV-011). A bound overflow becomes
`std::length_error`, which the `main()` boundary turns into node exit, not a
dropped element (HANDOFF.md §8).

### 15. TensorRT INT8 detector wrapper

**Bullet.** **YOLOv8s** as a 13 MB **TensorRT 10.3 INT8** engine (3.45× smaller
than FP32 ONNX) behind a hand-written TRT/CUDA wrapper: pinned + device buffers
on one stream, tensor-shape validation (1×3×640×640 / 1×84×8400), per-class NMS.

**Evidence.** `models/yolov8s_int8.engine` 13,007,484 B; `yolov8s.onnx`
44,869,835 B (ratio 0.290); `yolov8s_fp16.engine` 25,456,804 B (INT8 is 1.96×
smaller); engine header bytes identify TensorRT 10.3.0.30 and Ampere IMMA kernels.
`src/cuas_fusion/src/inference/trt_detector.cpp:54-107` (exactly one input/output,
shapes validated; `0620287` A2.8), `:120-155` (stream, `cudaMallocHost`/`cudaMalloc`,
`setTensorAddress`), `:205-265` (decode 84 rows × 8400 anchors), `:267-316`
(greedy per-class NMS, own IoU), `:335-351` (H2D → `enqueueV3` → D2H → sync);
`trt_detector.hpp:21-58` stateless functor deleters; `constants.hpp:29-36`
(conf 0.25, NMS 0.45, 80 classes, max 100). `AUDIT_REPORT.md` §C.1: every CUDA/TRT
status return on the per-frame path is checked.

**Defense.** "At init I require exactly one input and one output tensor and check
their shapes against the compile-time buffers, because a different YOLO variant
would otherwise write past the device buffer with no diagnostic. Decode reads the
84×8400 output as four box rows plus 80 class scores per anchor, thresholds at 0.25
and runs per-class NMS at 0.45."

**Risk.** "35 Hz" exists only in the subject line of commit `4a9e78a`
(2026-04-01); there is no log, CSV or instrumentation. The engine is a prebuilt
blob with no INT8 calibration recipe or build command in the repo. Preprocess is a
plain resize (16:9 squashed to 1:1, no letterbox). COCO has no "drone" class.
The node subscribes with queue depth 1 to a 30 Hz topic, so in-pipeline detection
rate is ≤ 30 Hz.

### 16. Overlay FPS drop analysis (rewritten; the current bullet is wrong)

**Bullet.** Characterized a 28–30 → 13–17 Hz overlay FPS drop with
**tegrastats**: camera ≈83 %, color-correct ≈58 %, RViz2 ≈67 % CPU
oversubscribing 6 cores; GPU (31–68 % GR3D) and thermal (≈57 °C) ruled out; 20
Hz track timer unaffected.

**Evidence.** `docs/latency_budget.md` §4.1–4.5 for the numbers;
`src/cuas_fusion/src/visualization/cuas_overlay_node.cpp:155-186` (30-frame
arrival-stamp ring, `RCLCPP_INFO_THROTTLE` every 5 s); `imm_tracker_node.cpp:68-72`
(50 ms wall timer). Hardware: six Cortex-A78AE at 1728 MHz (this file, header).

**Defense.** "The overlay node keeps a 30-sample ring of arrival stamps and logs
FPS every five seconds; it runs at camera rate for the first ~30 s and settles at
13–17 Hz once the full graph plus RViz2 is up. tegrastats put the three heaviest
processes at about 208 % of one core on top of the other sixteen nodes, GR3D never
passed 68 % and the junction temperature stayed near 57 °C, so it is CPU
oversubscription, not the GPU and not throttling."

**Risk.** The root cause in `latency_budget.md` §4.3 ("4 + 2 layout, two
Cortex-A55 at 729 MHz, 2.37× slower") is false for this board: all six cores are
A78AE with the same 1728 MHz ceiling; 729.6 MHz is an idle DVFS step that
tegrastats shows on parked cores. Do not say "4+2", "efficiency cores" or
"scheduler spill". Also: no tegrastats capture or FPS log is checked in, the
numbers are doc-only, and the observation was made on an unoptimized (-O0) build
(AUDIT A4.6). The "90 °C threshold" should be "≈95 °C trip point".

### 17. Camera intrinsic calibration

**Bullet.** Calibrated the **AR0234** global-shutter camera with a guided 15-pose
chessboard capture (10×7 inner corners, 23 mm squares): **RMS reprojection
1.82 px**, fx/fy 1862.7/1877.7, 5-coefficient distortion.

**Evidence.** `src/cuas_fusion/config/camera_calibration_result.yaml:1-18`
(2026-04-09, 15 frames, RMS 1.8157, fx 1862.67, fy 1877.74, cx 1032.83, cy 426.82,
k1 0.531, k2 −1.715, p1 −0.053, p2 0.017, k3 4.145); `calibration_frames/frame_001..015.png`;
`scripts/calibrate_camera.py:23-44` (poses at 2/3/4/6 ft with tilts), `:196-207`
(`cv2.calibrateCamera`, RMS printed); `constants.hpp:43-46`.

**Defense.** "The script walks the operator through 15 poses at two to six feet
with tilts, finds corners with adaptive thresholding and `cornerSubPix`, and calls
`calibrateCamera`. 1.8 px RMS on a 1920-wide image is usable for a padded-box
association but not tight."

**Risk.** `cv2.calibrateCamera` is library code; the from-scratch part is the
capture procedure. cy sits 113 px above the image centre and k3 = 4.1, which looks
like an over-parameterized fit or a different capture mode (the test-directory copy
of the script uses 1920×1200). The distortion coefficients are unused in C++.

### 18. DBSCAN clustering and clutter histogram

**Bullet.** From-scratch **DBSCAN** (eps 0.7 m, minPts 2) on radar returns with
Doppler-weighted cluster centroids, plus a 40×40 × 0.25 m occupancy-histogram
clutter filter learned over 200 frames and thresholded at 60 % occupancy.

**Evidence.** `radar_parser_node.cpp:31-34` (constants), `:121-226` (`dbscanCluster`),
`:196-214` (w = |doppler| + `kDopplerWeightFloor` 0.1; `constants.hpp:83`);
centroid arrived in `25f3ca7` (2026-04-15). `include/cuas_fusion/clutter_map.hpp:14-20`
(grid, 0.25 m, origin −5 m, 200 frames, 0.6); `src/cuas_fusion/src/clutter_map.cpp:72-114`;
`test_clutter_map.cpp` (7 tests); `3bd7550` (A1.12: runtime params were ignored).

**Defense.** "Cluster position is Σw·p / Σw with w = |Doppler| + 0.1, so the
moving part of a return dominates the centroid, and the cluster velocity is the
strongest-Doppler member. The clutter grid counts hits per 25 cm cell for 200
frames, normalizes once, and any cell above 0.6 is filtered from then on."

**Risk.** "Learned" is generous: it is a one-shot boot-time histogram with no
decay and a ±5 m footprint while the parser passes 15 m; the radar's own on-chip
`clutterRemoval` is also enabled in the profile. DBSCAN is O(n²) and only ever
sees ≤ 32 points. Cluster velocity is not Doppler-weighted (max member).

### 19. Fixed-capacity containers and the Mandatory-rule fixes

**Bullet.** Closed 3 latent **MISRA 11.6.2 Mandatory** paths: unchecked
`clock_gettime` behind every timestamp, `FixedVector::back()` on empty wrapping
to index 0xFFFFFFFF, `resize()` exposing uninitialized slots; 8 tests.

**Evidence.** `4e37a08` (A2.1): `include/cuas_fusion/common/clock.hpp:16-24`.
`0ead798` (A2.2): `include/cuas_fusion/common/fixed_containers.hpp:13, 19, 21,
39-40, 44-55, 60-61, 108`; `test/unit/test_fixed_containers.cpp:24-63`. `FixedVector`
and `FixedMap` are `std::array`-backed (`:72, :166`) and carry all owned steady
state (AUDIT §C.2).

**Defense.** "`back()` did `data_[size_ − 1]` with an unsigned size, so on empty it
read slot four billion; now it clamps to slot 0 and `operator[]` clamps to
capacity − 1, so the worst case is a wrong-but-valid element rather than a read
off the end. Mandatory rules can't be deviated, so these were fixed first."

**Risk.** Clamping hides caller bugs; the `idx < size()` contract is still the
caller's. The containers are ~170 lines; don't present them as a library.

### 20. Watchdog float32 quantization

**Bullet.** Fixed a watchdog storing timestamps as float32 seconds: past 2^24 s
uptime (194 days) the ULP is 2 s, so the 2 s dead-threshold was unresolvable and
rate EMAs rot within days; moved to int64 ns, subtract-then-narrow.

**Evidence.** `e8c5794` (A6.8): old `float32_t last_recv_sec` and
`static_cast<float32_t>(now().seconds())`; new
`include/cuas_fusion/health_monitor.hpp:30-34, 42-44` (500 ms stale, 2 s dead,
EMA α 0.1), `src/cuas_fusion/src/health_monitor.cpp:41-48`. Arithmetic: float32
ULP(x) = 2^(⌊log₂x⌋ − 23); 2^24 s = 16,777,216 s = 194.2 days → ULP 2 s; the 33 ms
camera period drops below the ULP at 2^18 s ≈ 3 days.

**Defense.** "A float32 has 24 significand bits, so once the value passes 2^24
seconds the spacing between representable numbers is two seconds and a half-second
staleness test cannot work; the 33 ms camera period is already below the ULP after
about three days. I keep int64 nanoseconds end to end and only narrow the small
difference."

**Risk.** `expected_hz` is stored but never compared; status is timeout-only and
the camera rate is never published (`SystemHealth.msg` has no camera field). The
tests use timestamps ≤ 3.2 s and don't exercise the large-magnitude regime.

### 21. Downstream engines: FSM, geofence, time-to-intercept

**Bullet.** 5-state threat escalation FSM with dwell timers, ray-cast
polygon/circle **geofence** (≤16 zones, ≤32 verts, fail-loud on overflow),
closed-form **time-to-intercept**; fixed a TTI fed a fabricated outward
velocity.

**Evidence.** `include/cuas_fusion/classification/threat_classifier.hpp:13-19`
(UNKNOWN → TRACKED → IDENTIFIED → THREATENING → ENGAGED),
`src/cuas_fusion/src/classification/threat_classifier.cpp:62-114` (transitions,
dwell 1.0 s, range 4.0 m, closing 0.3 m/s; ENGAGED at < 2 m for ≥ 2 s).
`geofence_engine.cpp:98-141` (even-odd ray cast, half-open straddle),
`geofence_node.cpp:95-105, 148-155` (`2909093` A2.6: over-capacity → FATAL; before,
`(void)push_back` silently dropped vertices 33+). `reachability_engine.cpp:29-42`
(TTI = distance / closing speed); `f23ed78` (A1.3): old `reachability_node.cpp:37-42`
rebuilt velocity as |v| along `atan2(y, x)`; new `:44-45` uses `vx_mps/vy_mps`.
`fb79d63`: bearing wrap at ±180° (`threat_classifier_node.cpp:145-149`, A1.4) and
the YAML key `threat_classifier_node:` that never matched node name
`classifier_node`, so five tuning parameters were silently ignored (A1.5).

**Defense.** "Reachability rebuilt velocity as speed along the position bearing,
which is always radially outward, so closing speed was always negative and
`intercept_possible` could never be true; it now uses the track's vx/vy. The
geofence used to drop vertices past 32 silently, enforcing a different fence than
configured, so over-capacity is now a fatal startup error."

**Risk.** The FSM has no de-escalation path. The intent classifier is a stateless
single-sample rule set (no loiter time, no angular rate). The reachability
covariance ellipse is a constant 0.5 m circle because the node ignores the new
`Track.covariance`. The kinematic and occlusion predictors still contain the
radially-outward velocity bug (`kinematic_predictor.cpp:80-93`,
`kinematic_predictor_node.cpp:114-117`) — open, no audit ID.

### 22. Build enforcement

**Bullet.** 90 warnings → 0 under **-Werror**: SYSTEM third-party includes,
`-Wshadow`/`-Wconversion` on all 11 no-ROS libs, RelWithDebInfo default (was
silent -O0), Debug `EIGEN_NO_MALLOC` guard, guarded CUDA lookup on a no-nvcc
image.

**Evidence.** `AUDIT_REPORT.md` baseline "90 warnings (88 from Eigen via
non-SYSTEM includes, 2 in own code)"; `7fbb23d` (A4.1, "90 → 2"), `46f9eeb`
(R9.1), `7ae7361` (R9.2), `d2e450d` (R9.3), `5dea3b5` (R9.4).
`src/cuas_fusion/CMakeLists.txt:6-8` (build type), `:36-54` (guarded `find_path`/
`find_library` with FATAL_ERROR), `:79-105` (flag sets), `:95-98`
(`option(CUAS_WERROR … ON)`), `:209` (`EIGEN_NO_MALLOC`). Full rebuild
`log/build_2026-07-07_07-18-58`: 344 s, 0 warnings.

**Defense.** "88 of the 90 were Eigen's PacketMath reached through a non-SYSTEM
include, so marking third-party directories SYSTEM left the two real ones. Turning
on -O2 then surfaced `-Wnull-dereference` inside dynamic-size Eigen code, and the
fix was moving that code to fixed-size types rather than suppressing the warning."

**Risk.** No CI; no `.clang-tidy`/`.clang-format`/cppcheck config is checked in;
`-Werror` is enforced only on this machine. The CUDA lookup is a hand-rolled
`find_path` because this JetPack image ships no `nvcc`, so `FindCUDAToolkit` fails.

### 23. Delivery-cadence measurement (the honest replacement for the "jitter" story)

**Bullet.** Logged fused radar-camera output over 3 live runs (733 detections,
75 s): 20 Hz cadence delivered at p50 50 ms / p95 100 ms / p99 442 ms, 86 gaps
>150 ms in one run — the evidence for the 50→150 ms cross-sensor gate.

**Evidence.** `logs/jitter_log.csv` (373 rows, 39.5 s), `jitter_log_centroid.csv`
(121, 19.0 s), `jitter_log_centroid_run2.csv` (239, 16.5 s); recorder
`scripts/jitter_log.py` (rclpy subscriber, `time.monotonic()` at callback). Stats
computed 2026-09-01 with pandas: run 2 inter-arrival p50 50.0 ms / p95 100.2 ms /
p99 441.5 ms, 211 of 238 intervals in 40–60 ms; run 1 bimodal with 86 gaps > 150 ms.
Gate constant `constants.hpp:39` `MAX_TIMESTAMP_DELTA_NS = 150 ms`, changed from
50 ms in commit `f1abc6e` (2026-04-09).

**Defense.** "A Python subscriber stamped every `/fusion/detections` arrival with
monotonic time; the tracker publishes on a 50 ms timer, so a 50 ms median is the
design rate and the p99 tail is DDS and executor delivery jitter under full load.
That tail is why a 50 ms cross-sensor window was dropping matches."

**Risk.** These are delivery times at a Python subscriber, not sensor-stamp skew;
the CSVs were recorded as pixel-jitter logs (the doc says so). The gate-change
commit predates the CSVs and its message says nothing about jitter; the "kernel
wake-up 60–80 ms" narrative in `latency_budget.md` §6 is post-hoc and unmeasured.
Run 1 is bursty (5 messages every ~450 ms), which is itself a scheduling finding
but not a clean cadence.

### 24. Closed-loop auto-exposure (NEEDS-HARDWARE)

**Bullet.** Closed-loop **auto-exposure/gain** over V4L2 for the AR0234:
multiplicative P-step (±25 %/step, 10-count deadband, luma target 110),
exposure-before-gain priority, 8000-count exposure ceiling to bound motion blur;
10 tests.

**Evidence.** `476bdee`. `src/cuas_fusion/src/drivers/auto_exposure.cpp:43-79`
(law), `include/cuas_fusion/drivers/auto_exposure.hpp:5-28` (limits 2/8000/100/1200,
params 110/10/0.25, blur rationale), `auto_exposure_node.cpp:34-42` (0.3 s period),
`:114-131` (`V4L2_CID_EXPOSURE`/`V4L2_CID_ANALOGUE_GAIN`), `:147-149` (luma);
`test/unit/test_auto_exposure.cpp` (10 tests).

**Defense.** "Error is target minus mean luma, the fractional step is clamped to
±25 % and applied multiplicatively; it raises exposure first and gain only at the
exposure ceiling, and sheds gain first when bright, so noise is added last and
removed first. The 0.3 s period lets the sensor settle between steps."

**Risk.** Never run on hardware; the commit is tagged NEEDS-HARDWARE and the V4L2
control IDs are unverified against the Arducam driver. It is P, not PI; full-frame
mean, no ROI or histogram.

### 25. Software-in-the-loop and health monitoring

**Bullet.** Radar simulator (LCG + Box-Muller noise σ 0.10 m, 4 scenarios, ≤8
targets), 7 launch configs incl. rosbag replay on sim time; per-topic health
monitor (EMA α 0.1, 0.5 s stale / 2 s dead) → NOMINAL/DEGRADED/FAILED at 1 Hz.

**Evidence.** `src/cuas_fusion/src/drivers/sim_radar.cpp:10-11, 36-47, 61-67,
73-145`; `constants.hpp:126-129`; `launch/` (full 21 processes, sim 19, replay 10 +
`ros2 bag play --clock`, full_system 5, kinematic 3, occlusion 3, radar_only 1);
`health_monitor.hpp:42-44`, `health_monitor.cpp:41-48, 82-98`,
`health_monitor_node.cpp:34-38, 60`.

**Defense.** "`sim.launch.py` swaps only the radar; the camera and inference still
run for real. The health node timestamps five topics on the steady clock and flags
stale at 0.5 s and dead at 2 s, then folds them into one system status."

**Risk.** "SIL with no hardware" is wrong: `sim.launch.py` launches `camera_node`
and `inference_node` (`:82-96`), and four of the seven launch files need the
physical radar. The monitor is per-topic (5), not per-node (22); `expected_hz` is
never compared.

### 26. Measurement-path time base

**Bullet.** Found two incomparable time bases in flight (drivers on
**CLOCK_MONOTONIC**, tracker on ROS system time); moved the measurement path to
one steady clock and wrote the contract into the ICD.

**Evidence.** `HANDOFF.md` §3; `51c476c` (P2.1); `docs/ICD.md` §2 time-base
contract; `camera_driver.cpp:167-170`, `radar_parser_node.cpp:419-427, 475`,
`sim_radar_node.cpp:21-32`. Interim fix `e8bf689` compared only local monotonic
arrival times for that reason.

**Defense.** "Camera and radar frames were stamped from `CLOCK_MONOTONIC` while the
tracker stamped `/tracks` with the ROS system clock, so any stamp arithmetic across
the two was meaningless. I moved the measurement path to one steady clock and
documented which topics carry which base."

**Risk.** Stamps are host receive times (after DQBUF, after the last serial byte),
not sensor exposure or chirp times; the kernel `buf.timestamp` and the radar's
`timeCpuCycles` are ignored.

### 27. Cursor-on-Target output (weak; use only if the posting cares about TAK)

**Bullet.** **Cursor-on-Target 2.0** XML over UDP multicast (239.2.3.1:6969, TTL
32) at 1 Hz for THREATENING/ENGAGED tracks, 5 s full sweep; hardened
socket/`inet_aton`/`sendto`/`strftime` returns, course wrapped to 0–360°.

**Evidence.** `src/cuas_fusion/src/output/cot_publisher_node.cpp:26-32, 46-72,
118-144, 160-183`; `f110696` (A1.6); `docs/ICD.md` §4.

**Defense / Risk.** Latitude and longitude are hard-coded 0.0 (`:131`); every event
plots at null island; no ATAK client has ever received a packet
(`PROJECT_MAP.md` §7 NEEDS HARDWARE). If used, say "emits CoT 2.0 events;
georeferencing deferred pending a GNSS source."

---

## Section 2 — Corrections to the nine bullets currently on the resume

1. **"23-node ROS 2 Humble C++17 pipeline, 10.9k LOC, 19 message types, IWR6843ISK
   radar + AR0234 camera on Jetson Orin Nano Super, no off-board compute."**
   Two numbers wrong, rest supported. CMake builds **22** executables
   (`auto_exposure_node` was added 2026-07-07); `full.launch.py` runs **19**
   cuas_fusion nodes plus tf2 and rviz2. "23" is the count of `executable=` lines
   in that launch file, which double-counts two conditional nodes and includes the
   two external processes. LOC is **11,827** owned C++ (14,110 with tests); 10.9k
   is the 2026-04-21 figure. 19 messages, Orin Nano Super (device-tree model string
   and MAXN_SUPER confirmed), no off-board compute: all correct.

2. **"30 Hz → 13–17 Hz FPS drop root-caused to 4+2 core scheduler spill; GPU (31–68 %
   GR3D) and thermal (57 °C vs 90 °C) ruled out; 20 Hz track contract unaffected."**
   **The root cause is wrong and a Jetson-literate reviewer will catch it in
   seconds.** This board has six identical Cortex-A78AE cores, all with a 1728 MHz
   ceiling (`/proc/cpuinfo` part 0xd42 ×6; `cpuinfo_max_freq` 1728000 ×6). There
   are no A55 efficiency cores and no 729 MHz ceiling; `latency_budget.md` §4.3's
   "two Cortex-A55 at 729 MHz, 2.37× slower" mechanism does not exist. The 729 MHz
   figure is an idle DVFS step that tegrastats shows on parked cores. The FPS,
   GR3D and temperature numbers exist only in the doc (no tegrastats capture or
   FPS log is checked in) and were observed on an -O0 build. "20 Hz unaffected" is
   by design of the 50 ms timer, not measured under load. The Orin Nano throttle
   trip is ≈95 °C, not 90. Use bullet 16's wording or drop it.

3. **"YOLOv8s INT8 TensorRT at 35 Hz, pinhole projection + IoU association, 20 Hz
   confirmed track output."** "IoU association" is **wrong**: fusion tests whether
   the projected radar point falls inside a YOLO box padded by 25 %
   (`fusion_engine.cpp:84-88`); the only IoU in the codebase is inside the
   detector's NMS. "35 Hz" is supported only by a commit subject line; the node
   itself subscribes at depth 1 to a 30 Hz topic, so in-pipeline rate is ≤ 30 Hz.
   "Confirmed" is not enforced: `imm_tracker_node` publishes tracks with their state
   field, and the ICD's `/tracks/confirmed` producer is not launched. Say "20 Hz
   track output".

4. **"IMM CV/CA/CT Kalman filter, Bayesian mode mixing, Hungarian assignment,
   Mahalanobis chi-square gating at 9.0/16.0."** Every component exists and is
   tested, and the gate values are exact (`constants.hpp:91-92`). Two caveats:
   (a) Hungarian + Mahalanobis live in `TrackManager`, which is **not launched**;
   the launched IMM node uses greedy nearest-neighbour with a 0.8 m Euclidean gate.
   (b) "Bayesian mode mixing" is half true: the interaction step is textbook, the
   mode-probability update uses the prior μ rather than the Markov-predicted
   prior. Also: the IMM was effectively CV-only until 2026-07-06 (A1.2); that fix
   is the stronger story (bullet 1).

5. **"Cross-sensor timestamp gate widened 50 ms → 150 ms after kernel wake-up jitter
   aged out camera frames; 6-frame / 200 ms ring buffer."** The constant change is
   real (`f1abc6e`, 2026-04-09), but: the gate at that time compared radar stamps
   to the latest YOLO-detection stamp, not to buffered camera frames; the
   6-frame image ring (`TimestampAssociator`) has **never been instantiated by any
   node** and exists only as a library plus tests; `8652f22` removed the gate from
   fusion four days later and nothing used it until 2026-07-07, when
   `DetectionSetBuffer` (six **detection sets**, 150 ms) went live. The "kernel
   wake-up jitter 60–80 ms" story appears only in a doc written three months later,
   which itself says no wall-clock period jitter was recorded. `AUDIT_REPORT.md`
   A5.1 credits the wrong commit. Replace with bullet 23 or bullet 7.

6. **"144 JSF AV C++ / MISRA C++:2023 violations cleared across 20 files; math moved
   out of ROS nodes into 43 GoogleTest-covered pure classes; 32-track compile-time
   bound."** "144 / 20 files" is **unsupported**: it appears in CHANGELOG 0.7.0 and
   commit `7e57574`'s body with no list; the rewrite commit `8652f22` touched 56
   .cpp/.hpp files. "43" is **wrong**: it is the number of `TEST(` macros in the
   six files under `src/cuas_fusion/test/`, not a count of classes. Real numbers:
   129 test cases, 18 suites, 20 fixtures/suites covering ~16 pure classes. The
   math/node separation and the 32-track bound (`TRACK_MAX_TRACKS`, `Track[<=32]`)
   are correct. Replace with bullets 5 and 9.

7. **"Radar serial TLV decode, DBSCAN clustering with Doppler-weighted centroid at
   16 Hz, learned static occupancy-grid clutter map."** "16 Hz" is **wrong**:
   `radar_profile.cfg` `frameCfg … 50 …` is a 50 ms period = **20 Hz**, unchanged
   since the first commit; the "16" is the chirp-loop count and was misread into
   the docs, the health monitor and the sim default. No hardware rate log exists to
   argue otherwise. "Learned" overstates a 200-frame one-shot histogram. The
   centroid is Doppler-weighted; the cluster velocity is the max-Doppler member.

8. **"CoT 2.0 UDP multicast to ATAK at 1 Hz, 5-state escalation FSM, 5-class intent
   classifier, polygon geofence with time-to-intercept."** FSM (5 states) and
   intent (5 classes + unknown) are correct, though the ICD names the FSM states
   wrong and the intent classifier is a stateless heuristic. "To ATAK" has no
   evidence: no client ever received a packet and lat/lon are 0.0. "Geofence with
   time-to-intercept" conflates two nodes: geofence reports containment and signed
   distance; TTI is the reachability node, whose velocity input was fabricated
   until `f23ed78`.

9. **"Software radar source + 7 launch configurations for full SIL with no hardware;
   built-in-test node tracking per-node rates by EMA."** 7 launch files and the
   simulator are real. "No hardware" is wrong: `sim.launch.py` starts the real
   camera and TensorRT inference. "Per-node" is wrong: five topics. "BIT" appears
   in the repo only as a reserved-panel note in the ICD. EMA (α 0.1) is correct.

---

## Section 3 — Weak spots (what a reviewer will find)

Each item says whether to omit it or name it.

1. **Zero live hardware runs on the audit/overhaul branch.** 78 commits, 123 files,
   +7,237/−1,226 lines verified by build and unit tests only (HANDOFF.md §1, §8;
   OVERHAUL_PLAN.md hard-limits table). Camera R12d/e/f, auto-exposure and radar
   hardening are tagged NEEDS-HARDWARE. *Name it*: "verified by build, tests and
   inspection; hardware validation is the next milestone." Do not imply the
   post-July system has been run.

2. **The latency budget is not a latency budget.** All 13 stages are marked "not
   instrumented"; the end-to-end figure is a rate-quantization floor; the only
   measured signal is the overlay FPS estimator, and the CPU topology it blames is
   wrong (Section 2, item 2). *Omit* the A55 story entirely; name the FPS
   observation only with bullet 16's wording.

3. **Georeferencing is absent.** CoT lat/lon 0.0 since 2026-04-05 ("GPS/georef
   deferred pending BN 880 hardware"); the georef sources were empty placeholders
   and were deleted. *Name it* if CoT is mentioned at all; otherwise omit CoT.

4. **The tracker in the launch is the weak one.** `imm_tracker_node`: greedy NN
   0.8 m Euclidean gate, 5 consecutive hits to confirm, no miss handling beyond a
   5 s reaper, an OCCLUDED→REACQUIRED arm nothing ever enters. Hungarian +
   Mahalanobis + M-of-N sit in the unlaunched `TrackManager`. Two confirm policies
   (5 vs 3) coexist undocumented. *Name it* as "implemented and unit-tested"; never
   as "the live tracker".

5. **Fusion is a label join.** Camera geometry never enters an estimate; fusion
   consumes tracks, not points; the P3.2 bearing model is not wired; fused range is
   the forward coordinate. *Name it*: "radar-primary tracking with camera class
   fusion".

6. **Prediction is thin and has an open bug.** CV-only after the fake CA/CT blend
   was deleted; horizon is metadata (propagation is a fixed 5 s / 50 steps);
   the kinematic and occlusion predictors rebuild velocity radially outward
   (`kinematic_predictor.cpp:80-93`), the identical defect fixed in reachability.
   Reachability's ellipse is a constant circle. *Fix the bug before any interview*
   (Section 4, item 7); until then omit prediction claims.

7. **Detector.** Stock COCO YOLOv8s; "drone" is not a class; no letterbox (16:9 →
   1:1 distortion); no undistortion; "35 Hz" undocumented; no INT8 calibration
   recipe or model card; engine is an unversioned blob. *Name honestly*: "stock
   COCO model; drone fine-tuning is Phase 4 of the roadmap."

8. **Radar limits.** ±4.7 m/s unambiguous Doppler aliases any real drone; 32-point
   cap before DBSCAN; blocking read and zombie-on-error still open; docs say 16 Hz
   against a 20 Hz config; udev rule and parser disagree on which ttyUSB is the
   data port; `ea60` vs `ea70` PID mismatch between scripts. *Name the Doppler
   limit* as a known constraint; fix the doc/script inconsistencies (Section 4,
   item 2).

9. **Extrinsics were never calibrated on real data**, and the overlay ignores the
   extrinsics file. *Name it*: "solver written and validated synthetically;
   real calibration pending a corner-reflector session."

10. **Documentation drift.** `ROLLBACK.md`/`HANDOFF.md` cite pre-rebase SHAs;
    test counts 97/108/136/129 across docs; ICD says 21 nodes and identity rotation;
    `architecture_diagram.md` says eight cores and lists predictors as tested;
    ICD/architecture name the FSM states wrong; `latency_budget.md` still says
    0.8.0 and 16 Hz. Any reviewer cross-reading two docs finds a contradiction.
    *Fix* (Section 4, item 10).

11. **Process artifacts missing.** No CI, no coverage, no clang-tidy/cppcheck
    config, no README, no requirements or traceability matrix
    (`OVERHAUL_PLAN.md` Phase 6 calls the RTM "the #1 gap"), and the GitHub
    default branch `main` is 78 commits behind `misra-audit-fixes`: a recruiter
    who clicks the link sees the pre-audit tree with no description.

12. **Provenance and velocity.** `git log` shows 18 commits on 2026-07-06 and 63
    on 2026-07-07, including 37 audit-ID fix commits between 05:28 and 06:45 (one
    every ~2 minutes), plus docs that are written for "sessions" and reference
    "context" (`HANDOFF.md` header; commit `3613be7`). A reviewer who opens the
    history will read this as AI-assisted work. That is not disqualifying, but the
    answer must be decided in advance and must be consistent with "solo project":
    what was directed, what was reviewed, and what can be re-derived at a
    whiteboard. Every Defense line in Section 1 was written to be re-derivable
    from the code; if a bullet cannot be walked through without notes, drop it.

13. **Visualizer.** 940-line file, a 562-line `imageCallback`, a mutex held across
    the whole render (A6.8/A6.9 open), and the same non-evicting `FixedMap` bug
    that was fixed in the overlay node. *Omit* the visualizer from the resume.

14. **Test gaps.** Nothing covers the predictors, `IMMTracker`, `TrtDetector`,
    overlay, CoT, or the node-level fixes A1.3/A1.4/A2.6; the health tests never
    reach the magnitude the int64 fix targets. *Name* the 129 honestly with the
    "pure math classes" qualifier.

---

## Section 4 — Gaps worth closing, ranked by hours-to-payoff

1. **README.md on the branch, and make it the landing page** (2–3 h). One
   architecture diagram, a hardware photo, the verified numbers from this file's
   header table, build/test commands, the standards posture with the honesty note,
   and a plain status line ("hardware-validated through 0.8.0; 0.9.0 audit branch
   verified by build and tests"). Then either merge `misra-audit-fixes` to `main`
   or change the GitHub default branch (owner action; 10 min). Today the link
   resolves to a bare file tree of pre-audit code. Highest payoff per hour in the
   whole list.

2. **Fix the wrong facts in the docs** (1–2 h). `latency_budget.md` §4.3 (six
   A78AE; replace spill with oversubscription; 16 → 20 Hz; 90 → 95 °C);
   `architecture_diagram.md` §8 ("eight cores"); ICD/architecture FSM state names;
   `health_monitor_node.cpp:34` and `sim.launch.py:47` 16 → 20 Hz; `ea60`/`ea70`
   and the udev port-role comment. One wrong hardware fact discounts every other
   number in the doc.

3. **One hardware session with artifacts checked in** (half a day). Run
   `full.launch.py`; verify R12d/e/f per `ROLLBACK.md`; record `ros2 topic hz` for
   `/radar/detections` (settles 16 vs 20), `/inference/detections` (settles the
   35 Hz claim), `/tracks`; capture `tegrastats` for 60 s and the overlay FPS log
   into `logs/`. This converts six doc-only numbers into files and clears three
   NEEDS-HARDWARE tags.

4. **Run the extrinsic solver on real correspondences** (2–3 h with a corner
   reflector). Produces a real RMS reprojection residual and replaces the
   tape-measure `extrinsics.yaml`. That residual is the single best fusion number
   the project can own.

5. **Fix the open radially-outward predictor bug and pin it with a test** (1–2 h).
   `kinematic_predictor.cpp:80-93` and `occlusion_predictor_node.cpp:94-96`: use
   `vx/vy` from the track. Same class as A1.3; a reviewer reading the predictor
   after reading the A1.3 commit will find it.

6. **Per-stage latency instrumentation** (4–6 h). `now()` on entry/exit in each
   node adapter around the pure class call, a `/latency_report` message with
   p50/p95/p99 per stage. Fills the 13 "not instrumented" rows and gives an
   end-to-end number, which is the one metric every C-UAS reviewer asks for.

7. **GitHub Actions CI** (4–8 h). Build + `colcon test` on an aarch64 or x86
   runner with the TensorRT targets gated behind a CMake option; badge in the
   README. `-Werror` and 129 tests become an external, checkable claim instead of
   a local one.

8. **Make the Hungarian/Mahalanobis tracker the launched one, or wire it into the
   IMM node** (2–4 h), and unify the 5-vs-3 confirm policy. This makes resume
   bullet 4 true of the live pipeline.

9. **MODEL_CARD.md and the INT8 build recipe** (2 h). The `trtexec` or Ultralytics
   export command, calibration image set, ONNX hash, TRT version, and the shape
   contract. Turns "INT8" from a filename into a defensible artifact.

10. **Sweep the doc drift** (1 h). Current SHAs in `ROLLBACK.md`/`HANDOFF.md`;
    129 everywhere; 22/19 node counts; DEV-001..012 in the architecture doc;
    `PREDICTION_MAX_STEPS` removed; predictors not listed as tested.

11. **Requirements seed + traceability** (3–4 h). 15–20 numbered requirements from
    `OVERHAUL_PLAN.md`'s definition-of-done, each tagged onto an existing gtest
    with a `REQ-` comment and a one-page matrix. Cheap, and it is the artifact
    prime-contractor reviewers ask for first.

12. **gcovr line/branch coverage with a stated target** (2 h). Adds a number to
    bullet 9 and exposes the untested predictors honestly.

13. **Sensor-degradation doc + two fault-injection tests** (3–4 h). Corrupt TLV
    frame test (the notable gap HANDOFF §6.5 names) and a NaN-measurement test
    through the IMM node path.

14. **A 30-second demo capture in the README** (hardware day). Annotated overlay
    video or GIF from `/camera/annotated_enhanced` with a walking person and the
    PPI. For a two-second scan, a picture of the thing working does more than any
    bullet.
