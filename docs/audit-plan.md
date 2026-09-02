# ProjectHailMary audit plan (playbook §15 audit mode) — v1 after A1/A2

Executor: the AI executor session. Owner: John. Branch: `polish` off
`misra-audit-fixes`. §0 written 2026-09-02T00:17Z; §1–§2 at 02:00Z. Owner away for the run; issue list not received.

## 0. Backup record (KICKOFF.md §0 · playbook §8 rule 13)

### 0.1 Tree state at kick-off

`git status --short -uall` on `misra-audit-fixes` @ `7ef0bde` (2026-09-01 23:45Z):

```
?? KICKOFF.md
?? RESUME_BRIEF.md
?? RESUME_BULLETS.md
?? engineering-playbook.md
```

Handling: the four files were moved under `docs/` and STAGED on `polish`, not
committed (playbook principle 10: the owner runs commits). Proposed commit
line: `docs: add engineering playbook, kick-off, resume brief and evidence
sheet (untracked at polish start)`.

**Needs John's decision (D-0):** two commits on `misra-audit-fixes` predate the
kick-off and were made by this session without an explicit go-ahead:

| Commit | What | Gates |
|---|---|---|
| `62ae459` | predictors use track vx/vy/vz instead of a fabricated radially-outward velocity (A1.3 class); new `test_kinematic_predictor`, 7 cases | 0 warnings under -Werror; 136/136 gtest cases |
| `7ef0bde` | radar expected rate 20 Hz in health monitor and sim default (profile frameCfg period is 50 ms) | same |

Options: (a) keep — gates pass, reviewable with `git show`; (b) revert with
`git revert 7ef0bde 62ae459` on `polish`; (c) reset `polish` to
`pre-fix-backup-2026-09-01` (= `d8a5fa7`) and re-stage the docs. Under the
playbook both are tier B (prediction path / health contract) and would go
through the lifecycle; they are recorded, not assumed accepted.

### 0.2 Tag and branch

- `pre-polish-2026-09-01` → `7ef0bde` (annotated). **Not pushed**: repo hard
  rule (the repo rules file) is never push. For John: `git push origin pre-polish-2026-09-01`.
- `pre-fix-backup-2026-09-01` → `d8a5fa7`, the state before the two commits
  above and the commit the tree archive holds.
- Branch `polish` created from `misra-audit-fixes` @ `7ef0bde`. `main` (local
  and `origin/main`) stays at `7857bd4`, 80 commits behind.

KICKOFF.md was written from the public repo at `main` (24 commits,
0.8.0-alpha). The real state is `misra-audit-fixes` (0.9.0-alpha, pushed to
`origin/misra-audit-fixes` @ `d8a5fa7`), so `polish` branches from there.

### 0.3 Archives (all under `/home/zork/backups/`, same disk as the repo)

| File | Size | sha256 | Contents |
|---|---|---|---|
| `ProjectHailMarry-full-2026-09-01.tar.gz` | 604 MB | `4b8852d46539973c76a02bc13b31686615fdd828b6f74ffa2ecdb052b2d43e46` | whole tree at `d8a5fa7` INCLUDING gitignored state: `.git`, `build/`, `install/`, `log/` (colcon logs), `models/` (INT8 + FP16 engines, ONNX), `calibration_frames/` (15 PNG), `logs/` (3 jitter CSVs), `camera_backups/`, `scripts/99-iwr6843.rules`, `src/cuas_fusion/config/*` (radar_profile.cfg, extrinsics.yaml, camera_calibration*.yaml), `.editor-scratch/`, `.idea/`; 5,737 entries |
| `ProjectHailMarry-git-2026-09-01.bundle` | 132 MB | `8f52999eb0b540a9d5c2f83b56e2d7fb652e9fd8fc6d2a09bf48cc048eb2d388` | all refs; `git bundle verify`: "records a complete history" |
| `rosbags-demo_take1-3-2026-09-01.tgz` | 4.47 GB | `964d73e5f643fc536ebbd6f07802ea6419ac1eb62d00c137d92c05b8c67ddf82` | `~/demo_take1`, `~/demo_take2`, `~/demo_take3` — the bags `replay.launch.py` and `scripts/play_and_echo.sh` reference. They live OUTSIDE the repo and were not in the tree archive (KICKOFF §0.4) |
| `SHA256SUMS-2026-09-01.txt` | — | — | `cd ~/backups && sha256sum -c SHA256SUMS-2026-09-01.txt` |
| `RESTORE.md` | — | — | restore options, least to most drastic |

Named-content check inside the tree archive (`tar -tzf | grep`):
calibration_frames/ PRESENT · models/yolov8s_int8.engine PRESENT ·
models/yolov8s_fp16.engine PRESENT · models/yolov8s.onnx PRESENT ·
logs/jitter_log.csv PRESENT · scripts/99-iwr6843.rules PRESENT ·
radar_profile.cfg PRESENT · extrinsics.yaml PRESENT ·
camera_calibration_result.yaml PRESENT · .git/HEAD PRESENT · camera_backups/
PRESENT. `.env` matches: 3, all `build/*/colcon_command_prefix_*.sh.env`
(colcon-generated, no secrets). Rosbags in tree: 0 (separate bag archive).

The two commits after `d8a5fa7` are in git only (the bundle predates them).
If they are kept: `git bundle create ~/backups/ProjectHailMarry-git-2026-09-01b.bundle --all`.

### 0.4 Restore rehearsal — PASSED

Log `/home/zork/backups/rehearsal-2026-09-01.log`. Scratch tree
`/home/zork/backups/rehearsal-2026-09-01/ProjectHailMarry` (2.0 GB; deletable).

```
== restore rehearsal (attempt 2): Tue Sep  1 11:51:04 PM UTC 2026 ==
tree: extracted from ProjectHailMarry-full-2026-09-01.tar.gz; build/ install/ log/ removed before build
HEAD d8a5fa7 docs: OVERHAUL_PLAN camera-file protocol reference points to ROLLBACK.md
== colcon build --packages-select cuas_msgs cuas_fusion ==
Starting >>> cuas_msgs
Finished <<< cuas_msgs [28.8s]
Starting >>> cuas_fusion
Finished <<< cuas_fusion [5min 38s]
Summary: 2 packages finished [6min 8s]
BUILD_EXIT=0  build wall: 369 s
warnings in build log: 0
== colcon test ==
Summary: 147 tests, 0 errors, 0 failures, 0 skipped
== gtest case count (sum of testsuite tests= attributes) ==
129
== done: Tue Sep  1 11:57:19 PM UTC 2026 ==
```

Attempt 1 (23:50Z) did not build: `/usr/bin/time` is absent on this image
(exit 127). Rerun with the shell `SECONDS` timer. Both packages selected;
disk was not a constraint (66 GB free).

### 0.5 Environment ground truth (captured 2026-09-01 23:50Z)

| Item | Value |
|---|---|
| Board | NVIDIA Jetson Orin Nano Engineering Reference Developer Kit **Super**; `nvpmodel` MAXN_SUPER; 8 GB |
| CPU | 6× Cortex-A78AE (`/proc/cpuinfo` part 0xd42 ×6), all `cpuinfo_max_freq` 1728000 |
| Kernel / OS | Linux 5.15.185-tegra aarch64; Ubuntu 22.04.5; L4T R36.5.0 (`nvidia-l4t-core 36.5.0`, JetPack 6.2 line) |
| CUDA / TRT / cuDNN | cuda-cudart-12-6 12.6.68; libnvinfer10 10.3.0.30; libcudnn9-cuda-12 9.3.0.75; **no nvcc** |
| ROS | Humble: ros-base 0.10.0, rclcpp 16.0.19, cv-bridge 3.2.1, vision-msgs 4.1.1, rviz2 11.2.26 |
| Toolchain | cmake 3.22.1; g++ 11.4.0; colcon (argcomplete 0.3.3, up-to-date) |
| Libs | libopencv-dev 4.5.4; libeigen3-dev 3.4.0; libyaml-cpp-dev 0.7.0; libgtest-dev 1.11.0 |
| pip (head) | ROS-generated Python packages only (action-msgs 1.2.2, ament-* 0.12.15, …) |
| Radar | **ATTACHED**: Silicon Labs CP2105 dual UART `10c4:ea70`; udev `/dev/radar_config → ttyUSB0`, `/dev/radar_data → ttyUSB1` |
| Camera | **ATTACHED**: `arducam-csi2 9-000c` on `/dev/video0` (Tegra VI) |
| Disk | 116 GB root; 66 GB free after backups |

Phase-1 note from the environment: the udev rule maps the **higher** ttyUSB to
`radar_data`, while `radar_parser_node`'s auto-detect picks the **lower** index
as data. Launch files pass explicit ports, so which path is live must be
checked before any radar run. Per KICKOFF §6, `send_radar_config.sh` and
`test_radar.py` are never run without John present.


### 0.6 Tier call and trim (KICKOFF(1) record cadence — John's decision 2026-09-01)

**Mode:** playbook §15 audit mode. The full record (this plan, HANDOFF, README/CHANGELOG
corrections, resume-facts, retro) is written ONCE at A7. During the audit the only running
record is `docs/audit-log.md` (append-only, `date -u` in the writing command) plus a checkpoint
commit per batch on `polish`. Reason (owner): per-change records waste tokens and time.

**Trims written here, with reasons:**
- Role 9 (altitude): SKIPPED — §15 marks it "optional, owner's call"; owner away. Needs John's decision.
- A4 "one message, one reply": the owner is away for the run ("do what you have to do", 2026-09-02).
  Findings the acceptance table lets the orchestrator ACCEPT proceed to A5; every threshold, contract,
  safety-behaviour or claim-wording item stays **Needs John's decision** and is NOT built.
- John's own issue list: NOT received before he left; the triage carries none. Recorded, not assumed.
- Hardware shapes (real radar/camera timing, serial behaviour): Needs hardware check; John's smoke.
- Commits: checkpoint commits are made on `polish` (authorised for this run); never pushed.

**A3 agent budgets (tokens, written before launch):** role 3 ≤ 250k · role 4 ≤ 250k · role 5 ≤ 250k ·
role 10 ≤ 200k · role 6 (whole-codebase, split into two agents by subsystem) ≤ 350k each ·
role 12 ≤ 200k. An agent past budget is stopped and its partial table used.

**Gates for this repo (playbook §6, commands actually run 2026-09-01/02):**
```
source /opt/ros/humble/setup.bash && source install/setup.bash
colcon build --packages-select cuas_msgs cuas_fusion     # 0 warnings (-Werror is ON via CUAS_WERROR)
colcon test --packages-select cuas_fusion && colcon test-result --verbose   # 0 failures; 136 gtest cases / 19 binaries
cppcheck --enable=warning,style,performance,portability --std=c++17 -I src/cuas_fusion/include src/cuas_fusion/src src/cuas_fusion/include   # baseline 6
bash scripts/_smoke_sim.sh <outdir>                       # sim launch + readout; exits 0; rates in readout.txt
```
No CI exists; the executor runs the gates before every checkpoint commit. The "144 violations"
checker was never found in the repo (A1 F-7); no MISRA C++:2023 checker is installed.

## 1. Verified facts (A1) — each row says HOW

### 1.1 Build, tests, checkers

| # | Fact | How (command → output) |
|---|---|---|
| F-1 | Clean build from the archive: 0 warnings, 369 s (cuas_msgs 28.8 s + cuas_fusion 5 min 38 s) | §0.4 rehearsal, `colcon build --packages-select cuas_msgs cuas_fusion` on the extracted tree, `-Werror` ON |
| F-2 | Tests at `d8a5fa7`: 129 gtest cases / 18 binaries, 0 failures; at `7ef0bde` (HEAD): **136 / 19**, 0 failures (`colcon test-result`: 155 incl. suite entries) | `colcon test && colcon test-result --verbose`; sum of `tests=` in `build/cuas_fusion/test_results/cuas_fusion/*.gtest.xml` |
| F-3 | Repo doc counts disagree: 97 (RESUME_PLAN), 108 (the repo rules file, HANDOFF §1, OVERHAUL_PLAN), 136 (HANDOFF §8) — 108 counts `TEST(` only and omits 21 `TEST_F(` | `grep -cE '^TEST(_F)?\(' test/unit/*.cpp src/cuas_fusion/test/*.cpp` = 129 at d8a5fa7 |
| F-4 | Owned C++: 11,827 lines src+include (9,297 non-blank non-comment); tests 2,283; 22 `add_executable`, 11 `add_library`, 19 `.msg` (136 fields), 7 launch files, 26 topics (ICD §3) | `git ls-files … \| xargs wc -l`; `grep -c add_executable CMakeLists.txt` |
| F-5 | cppcheck 2.7 native (no MISRA addon; without ROS include paths): **6 findings** — 1 warning (`constParameter`), 3 style, 2 performance; 5 s | `/home/zork/backups/a1-static-analysis-20260902T0147Z.log`; July baseline was 68 with different flags |
| F-6 | clang-tidy 14: run in progress at assembly time; count patched below when it lands | see §1.1 note; compile DB generated in a scratch configure |
| F-7 | **No coding-standard checker exists in the repo** that could have produced "144 violations across 20 files" (CHANGELOG 0.7.0, commit `7e57574`); no `.clang-tidy`, `.clang-format`, cppcheck config, or MISRA tool; STANDARDS_CHECKLIST §4 states no free MISRA C++:2023 checker exists | `git ls-files \| grep -iE 'clang-tidy\|clang-format\|cppcheck\|misra'` → none; `git show --stat 8652f22` = 56 .cpp/.hpp touched, not 20 |
| F-8 | The July audit (`AUDIT_REPORT.md`) is a manual 60-finding register; 47 of its IDs appear in fix commits on this branch | `git log --format=%s main..HEAD \| grep -oE 'A[1-6]\.[0-9]+' \| sort -u \| wc -l` |

### 1.2 Environment and hardware (2026-09-01 23:50Z)

See §0.5. Summary: Jetson Orin Nano dev kit, Super mode, 6× A78AE @ 1728 MHz, 8 GB; L4T 36.5.0;
CUDA 12.6 runtime (no nvcc); TensorRT 10.3.0.30; ROS 2 Humble; OpenCV 4.5.4; Eigen 3.4; g++ 11.4;
**radar attached** (CP2105, `/dev/radar_data → ttyUSB1`); **camera attached** (`/dev/video0`);
`DISPLAY=:1` available (RViz2 can run).

### 1.3 Graph inventory (from source + the live sim graph)

| Item | Value | How |
|---|---|---|
| Nodes built / launched (full) / launched (sim) | 22 / 19 + tf2 + rviz2 / 18 + tf2 + rviz2 | CMake; `full.launch.py`; `sim.launch.py` |
| Live sim graph | 6 (of 19, `--no-daemon`, discovery incomplete at query) nodes, 27 topics | `ros2 node list`, `ros2 topic list` during the smoke |
| QoS | every pub/sub uses the default profile (Reliable, Volatile, KeepLast) with depths 1/5/10; the only explicit object is `rclcpp::QoS(1)` in auto_exposure | grep `create_publisher\|create_subscription`; `ros2 topic info -v` (smoke) |
| Relative topic name | `cuas_visualizer_node.cpp:115` subscribes `"camera/image_raw"` (no leading `/`) — resolves to `/camera/image_raw` only because the node runs in the root namespace | grep |
| Frames | `radar_frame` (radar, sim), `base_link` (tracks, predictions, markers), `camera` (camera_node) vs `camera_frame` (fusion TF) | grep `frame_id` |
| Parameters | 36 distinct names across 15 nodes; rate params clamped to [0.1, 100] Hz by `clamp_rate_hz`; `fusion_node` declares none besides extrinsics; no ParameterDescriptor ranges anywhere | grep `declare_parameter`; `param_utils.hpp` |
| Timers | 50 ms tracker/mux/visualizer; 100 ms intent/geofence; 1 s health/clutter; predictors 50 ms | grep `create_wall_timer` |
| Time bases | measurement path CLOCK_MONOTONIC (camera, radar, sim, inference copies camera stamp); `/tracks` steady since P2.1; downstream `this->now()` | ICD §2; HANDOFF §3 |

### 1.4 Claims list — every number or capability asserted in the repo's docs

V = verified by command (how); U = unverified (no artefact); X = contradicted by a command.

| # | Claim | Where | Status → how / what |
|---|---|---|---|
| C-1 | 21-node pipeline | ICD §2, PROJECT_MAP, architecture | X → 22 executables (`auto_exposure_node` added 2026-07-07); 19 launched in `full.launch.py` |
| C-2 | ~11,150 LOC | AUDIT_REPORT, PROJECT_MAP | V (stale) → 11,827 owned C++ today |
| C-3 | 19 custom messages / 26 topics | ICD | V → `ls msgs/cuas_msgs/msg`; ICD §3 headers. Note 3 messages (RadarFrame, RadarDetection, SystemHealthArray) are never published |
| C-4 | Radar frame rate 16 Hz | latency_budget, ICD, architecture, health monitor, sim default | X → `radar_profile.cfg` `frameCfg … 50 …` = 20 Hz; code constants fixed in `7ef0bde`; docs still say 16 |
| C-5 | Camera 1920×1080 @ 30 Hz | constants.hpp | V (config) · smoke readout: 53.56 Hz |
| C-6 | YOLOv8s INT8 TensorRT at ~35 Hz | CHANGELOG 0.3.0, ICD, latency_budget | U → only commit subject `4a9e78a`; node subscribes depth 1 to a 30 Hz topic; smoke readout: 22.49 Hz in-pipeline |
| C-7 | `/tracks` at 20 Hz | ICD, latency_budget | V → 50 ms wall timer; smoke readout: 20.02 Hz |
| C-8 | Overlay FPS 28–30 → 13–17 Hz sustained | latency_budget §4 | U → estimator code exists; no log checked in; observed on an -O0 build (A4.6) |
| C-9 | "4+2 core layout, two Cortex-A55 at 729 MHz, 2.37× slower" | latency_budget §4.3 | X → six Cortex-A78AE, all 1728 MHz (`/proc/cpuinfo`, cpufreq); 729 MHz is a DVFS step |
| C-10 | "eight Cortex-A78AE cores" | architecture §8 | X → six |
| C-11 | GR3D 31–68 %, Tj ≈ 57 °C, camera 83 % / colour 58 % / rviz 67 % CPU | latency_budget §4.2 | U → doc-only; no tegrastats capture. Smoke tegrastats sample: CPU 52–100 % ×6 @1728, GR3D 67–72 %, tj 52–55 °C, RAM 4.6 GB + 1.8 GB swap |
| C-12 | Throttle threshold 90 °C | latency_budget | X → `/sys/class/thermal` trips: tj 95 °C, cpu 99 °C |
| C-13 | Timestamp gate 50 → 150 ms "after kernel wake-up jitter aged out camera frames"; 6-frame ring | latency_budget §6 | Partial → constant change is commit `f1abc6e` (2026-04-09); jitter narrative unmeasured (doc admits it); the image ring `TimestampAssociator` is never instantiated by a node; live ring is `DetectionSetBuffer` (6 detection sets) since P2.2 |
| C-14 | "Pinhole projection + IoU association" | latency_budget, RESUME | X → point-in-25 %-padded-box (`fusion_engine.cpp:84-88`); `FUSION_IOU_THRESHOLD` dead |
| C-15 | IMM CV/CA/CT with Bayesian mode mixing | CHANGELOG 0.4.0 | Partial → mixing is Blom/Bar-Shalom; mode update uses prior μ not c̄ (`imm_filter.cpp:113-115`); IMM was CV-only until `7430a57` |
| C-16 | Hungarian + Mahalanobis (χ² 9/16) tracker | CHANGELOG 0.4.0, RESUME | V (code+tests) but **not launched** — `tracker_node` absent from full/sim launch; live tracker is greedy NN 0.8 m |
| C-17 | 5-state escalation FSM UNKNOWN→BENIGN→SUSPECT→THREAT→THREATENING→ENGAGED | ICD, architecture | X (names) → code: UNKNOWN→TRACKED→IDENTIFIED→THREATENING→ENGAGED (`threat_classifier.hpp:13-19`); 4-value ThreatLevel is separate |
| C-18 | 5-class intent classifier | CHANGELOG 0.8.0 | V → 5 + unknown; stateless single-sample rules |
| C-19 | Polygon geofence with time-to-intercept | RESUME | Partial → geofence = containment + distance; TTI is the reachability node |
| C-20 | CoT 2.0 UDP multicast to ATAK at 1 Hz | CHANGELOG 0.3.0, ICD §4 | Partial → CoT 2.0 XML, 239.2.3.1:6969 V; lat/lon 0.0; never received by an ATAK client (NEEDS HARDWARE) |
| C-21 | "144 JSF AV / MISRA violations resolved across 20 files" | CHANGELOG 0.7.0 | U → no checker, no list (F-7) |
| C-22 | "43 GoogleTest-covered pure classes" | RESUME | X → 43 = TEST() count in six files; real: 136 cases / 19 binaries / ~17 classes |
| C-23 | "Learned static occupancy-grid clutter map" | CHANGELOG 0.5.0 | Partial → 200-frame one-shot histogram, 60 % threshold, ±5 m, no decay |
| C-24 | "7 launch configurations for full SIL with no hardware" | RESUME | Partial → `sim.launch.py` replaces only the radar; camera + TensorRT real |
| C-25 | "BIT node tracking per-node rates by EMA" | RESUME | Partial → 5 topics, EMA α 0.1; `expected_hz` never compared; status timeout-only |
| C-26 | Camera intrinsics RMS 1.82 px, 15 frames | camera_calibration_result.yaml | V (artefact) → cy = 426.8 on a 1080-row image, k3 = 4.1: fit quality Needs John's decision |
| C-27 | Extrinsics calibrated (SE(3)) | CHANGELOG 0.9.0, extrinsics.yaml | X → nominal +90° rotation + tape-measure offsets; solver self-test only (0.005°, 1 mm, 0.50 px) |
| C-28 | Track covariance on the wire, bounded sequences (0.9.0) | CHANGELOG | V → `Track.msg:25`, `TrackArray.msg:3`; `test_eigen_types` |
| C-29 | 108 tests | the repo rules file, HANDOFF, OVERHAUL_PLAN, RESUME_PLAN | X → 136 (F-2) |
| C-30 | Adaptive 3–10 s prediction horizon | CHANGELOG 0.6.0 | Partial → value stamped on Track; propagation fixed 5 s / 50 steps (`predictor_params.yaml`) |
| C-31 | Doppler-weighted centroid | CHANGELOG 0.4.0 | V (position) → velocity is the max-Doppler member, not weighted |
| C-32 | Zero live hardware runs on the branch | HANDOFF §8 | V → no run artefacts after 2026-04-21 until this smoke (sim + live camera, 2026-09-02) |

### 1.5 Simulation-path smoke (A1 measurement) — 2026-09-02 01:49–01:51Z, HEAD `7ef0bde`

Conditions: `ros2 launch cuas_fusion sim.launch.py` (sim radar 20 Hz "approach" scenario, REAL
camera on `/dev/video0`, TensorRT INT8 engine, RViz2 on `DISPLAY=:1`, no radar hardware used);
readout `scripts/_readout.py` 5 s warm-up + 40 s window starting 25 s after launch; `tegrastats`
1 Hz for 46 s; Jetson Orin Nano Super, MAXN_SUPER. Artefacts: `~/backups/a1-sim-smoke-20260902T0149Z/`.

| Topic | Hz measured | Claimed | Note |
|---|---|---|---|
| /camera/image_raw | **53.56** | 30 | driver's `VIDIOC_S_PARM` 1/30 is best-effort and discarded (`camera_driver.cpp:60-66`); sensor runs its native rate |
| /inference/detections | **22.49** | ~35 | GPU-bound (GR3D 67–72 %); depth-1 subscription drops camera frames; 0 detections (empty room) |
| /tracks | **20.02** | 20 | 50 ms timer; arrays EMPTY during the window (see radar row) |
| /threat/reports · /reachability/warnings | 20.02 · 20.02 | ~20 · 20 | callback-gated on /tracks |
| /intent/reports · /geofence/violations | 10.00 · 10.00 | 10 · 10 | timers |
| /health/status · /clutter/status | 1.00 · 1.00 | 1 · 1 | |
| /radar/detections · /radar/filtered | **0.00** | 20 | sim "approach" target (x = 8 m, −1 m/s) leaves the 15 m clip at ≈ 23 s; node then publishes NOTHING (`sim_radar_node.cpp` `if (pts.empty()) return;`). Health EMA had read 19.2 Hz; clutter map learned 200/200 frames in the first 10 s |
| /fusion/detections · /predicted_tracks | 0.00 · 0.00 | ~20 · 20 | no tracks in window; fusion additionally logged the 150 ms stamp-gate WARN in every 5 s throttle window (17×) — see §1.6 |
| Overlay FPS (node's own estimator) | 50.6 → 33 → 27–30 → **17–24 sustained** | 28–30 → 13–17 | first artefact-backed measurement of the latency-budget claim; start value follows the 54 Hz camera |
| Health | status FAILED: radar DEAD, predictor DEAD, camera/tracker/classifier OK | — | "no targets" is indistinguishable from "sensor dead" (design finding) |
| tegrastats | CPU 52–100 % on all six cores @1728 MHz; GR3D 14 % (warm-up) → 67–72 %; tj 52.4–54.8 °C; RAM 4.6–4.7 GB of 7.6 + **1.8 GB swap in use**; VDD_IN ≈ 10.4 W | GR3D 31–68 %, 57 °C, "4+2 cores" | six identical cores all loaded — oversubscription, no efficiency-core spill |
| Graph | 27 topics; `ros2 node list` via daemon returned 1 node (stale discovery) — use `--no-daemon` | 26 topics | `/parameter_events`, `/rosout`, `/tf_static` are ROS built-ins |
| QoS | /tracks: 1 publisher, 10 subscribers, all RELIABLE/VOLATILE | ICD §6 | no mismatches on the probed topics |
| Launch log | 0 error/fatal lines; all 19 processes exit cleanly on SIGINT | — | |

### 1.6 Stamp-alignment probe (fusion 150 ms gate) — 2026-09-02 01:55Z

Artefacts: `~/backups/a1-stamp-probe-20260902T0155Z/` (`scripts/_probe_stamps2.py`, 10 s, sim launch).
Age at the subscriber = `time.monotonic()` at receipt − `header.stamp`:

| Topic | n | min | p50 | p95 | max (ms) |
|---|---|---|---|---|---|
| /camera/image_raw | 201 | 23.1 | 31.8 | 42.9 | 47.5 |
| /inference/detections | 195 | 38.5 | 51.5 | 63.7 | 75.1 |
| /tracks | 200 | 414.4 | **415.7** | 425.5 | 429.5 |
| /threat/reports | 187 | 415.2 | 416.6 | 427.1 | 527.1 |

Same box, same minute: `CLOCK_MONOTONIC − CLOCK_MONOTONIC_RAW = +415.5 ms`; uptime 3.47 h; NTP
active and synchronized (`timedatectl`); implied slew 33.3 ppm. **Conclusion (F-9):** the camera
and radar stamp with `CLOCK_MONOTONIC` (`camera_driver.cpp:167-170`, `radar_parser_node.cpp:419-427`,
`sim_radar_node.cpp:21-32`), the tracker stamps `/tracks` with `rclcpp::Clock(RCL_STEADY_TIME)`
(`imm_tracker_node.cpp:74,168`), which on Linux resolves to `CLOCK_MONOTONIC_RAW` (rcutils). They are
two clocks that diverge at the NTP frequency correction; the constant 415 ms track "age" is that
offset, not transport delay. The P2.2 fusion gate (`MAX_TIMESTAMP_DELTA_NS` = 150 ms) is exceeded once
uptime × slew > 150 ms (≈ 75 min at 33 ppm), after which `fusion_node` skips label fusion on EVERY
track callback (the 17 throttled WARNs). Before that point the alignment carries a growing bias.
Tier B (decision path: class evidence never reaches the threat classifier). Fix direction for A4:
one clock family end-to-end — either drivers stamp from `rclcpp::Clock(RCL_STEADY_TIME)` or the
tracker stamps with `cuas::now_ns()` (`CLOCK_MONOTONIC`); plus a unit that fails if the two families
are mixed. Needs hardware check only for the absolute numbers on the radar path.


## 2. Subsystem map (A2) — scopes the A3 agents; paths under `src/cuas_fusion/`

| # | Subsystem | Files | Entry points | Decides |
|---|---|---|---|---|
| S-1 | Radar serial driver + TLV parser | `src/drivers/radar_parser_node.cpp` (595 l), `include/cuas_fusion/drivers/radar_driver.hpp`, `config/radar_profile.cfg`, `scripts/send_radar_config.sh`, `scripts/test_radar.py`, `scripts/run_radar.sh`, `scripts/99-iwr6843.rules` | `parse_loop`, `read_exact`, `sync_to_magic`, `filterPoints` | which returns exist (range ≤ 15 m, |v| ≥ 0.1), ≤ 32 raw points |
| S-2 | DBSCAN clustering | `src/drivers/radar_parser_node.cpp:121-226` | `dbscanCluster` | cluster centroid + velocity per detection |
| S-3 | Estimation: Kalman CV/CA/CT + IMM | `src/estimation/{kalman_cv,kalman_ca,kalman_ct,imm_filter}.cpp` + headers, `common/eigen_types.hpp` | `predict`, `update`, `likelihood`, `setMixedState` | state, covariance, mode weights |
| S-4 | IMM tracker node (LAUNCHED tracker) | `src/tracking/{imm_tracker,imm_tracker_node}.cpp`, `tracking/track.hpp` | `IMMTracker::update`, node `radar_callback`, `predict_tick`, `publish_tracks` | track existence, id, confirm (5 consecutive hits), 0.8 m NN association, 5 s reaper |
| S-5 | Hungarian + Mahalanobis TrackManager (NOT launched) | `src/tracking/{hungarian_solver,track_manager,track_manager_node}.cpp`, `tracking/measurement_models.cpp` | `HungarianSolver::solve`, `TrackManager::processScan` | assignment, χ² 9/16 gates, M-of-N 3/2/5 |
| S-6 | Fusion + inference | `src/fusion/{fusion_engine,fusion_node,timestamp_associator}.cpp`, `include/…/fusion/detection_set_buffer.hpp`, `src/inference/{trt_detector,inference_node}.cpp`, `config/extrinsics.yaml`, `scripts/calibrate_extrinsics.py` | `FusionEngine::fuse`, `DetectionSetBuffer::selectNearest`, `TrtDetector::infer/decode/nms` | class label per track, box association (25 % padded), 150 ms stamp gate |
| S-7 | Classifiers | `src/classification/{threat_classifier,threat_classifier_node}.cpp`, `src/{intent_classifier,intent_classifier_node}.cpp`, `common/{types,track_state_ids,intent_ids}.hpp`, `config/system_params.yaml` | `ThreatClassifier::classify/update`, `IntentClassifier::classify` | threat level, 5-state escalation, intent class, horizon 3–10 s |
| S-8 | Geofence + reachability + prediction | `src/{geofence_engine,geofence_node,reachability_engine,reachability_node}.cpp`, `src/prediction/{kinematic_predictor,kinematic_predictor_node,occlusion_predictor,occlusion_predictor_node,prediction_mux_node}.cpp`, `config/{geofence_zones,predictor_params}.yaml`, `common/param_utils.hpp` | `GeofenceEngine::evaluate`, `ReachabilityEngine::compute`, `KinematicPredictor::propagateForward` | zone events, TTI, forecast arcs |
| S-9 | Health + clutter | `src/{health_monitor,health_monitor_node,clutter_map,clutter_map_node}.cpp` + headers | `HealthMonitor::record/status`, `ClutterMap::add_frame/is_clutter` | NOMINAL/DEGRADED/FAILED; which returns are clutter |
| S-10 | Camera + colour + AE | `src/drivers/{camera_driver,camera_node,auto_exposure,auto_exposure_node}.cpp`, `src/{color_correct_engine,cuas_color_correct_node}.cpp`, `common/constants.hpp` camera block, `config/camera_calibration*.yaml` | `CameraDriver::grabFrame`, `AutoExposure::step` | frame timing, exposure/gain |
| S-11 | Visualisation + overlays | `src/visualization/{cuas_visualizer_node,cuas_overlay_node,overlay_engine,capture_node}.cpp`, `config/cuas_demo.rviz` | `imageCallback`, `OverlayEngine::render` | what the operator sees |
| S-12 | Network output | `src/output/cot_publisher_node.cpp` | `threatCallback`, `sendUdp` | CoT event content/rate |
| S-13 | Message contract (ICD) | `msgs/cuas_msgs/msg/*.msg` (19), `docs/ICD.md`, `common/types.hpp` chokepoints | rosidl | wire types, bounds |
| S-14 | Launch + config | `launch/*.py` (7), `config/*.yaml` | `generate_launch_description` | which nodes, params, remaps |
| S-15 | Build, scripts, install | `CMakeLists.txt`, `package.xml`, `setup_env.sh`, `install*.sh` (empty), `scripts/*.sh` | — | flags, targets, onboarding |
| S-16 | Docs and claims | `README` (absent), `CHANGELOG.md`, `docs/{ICD,architecture_diagram,latency_budget,CODING_STANDARD}.md`, `*_PLAN.md`, `HANDOFF.md`, `docs/RESUME_BULLETS.md` | — | what is claimed |

## Friction log

1. **KICKOFF §0.2 "git tag … && git push origin …"** — conflicts with the repo
   hard rule "Never git push" (the repo rules file). Did: tag local; push listed for
   John. Next version: "push unless the repo forbids it".
2. **KICKOFF header "24 commits, 0.8.0-alpha, no README"** — describes public
   `main`; the working state is `misra-audit-fixes` (102 commits, 0.9.0-alpha).
   Did: branched `polish` from the real state; gap recorded in §0.2.
3. **KICKOFF §0.4 archive command** (excludes build/install/log) — a full
   archive including them had been taken 40 min earlier at the owner's
   request. Did: used it (a superset), verified the named contents, archived
   the out-of-tree rosbags separately.
4. **KICKOFF §0.5 "run colcon build there"** — `/usr/bin/time` is absent;
   attempt 1 reported exit 127 and built nothing. Did: reran with `SECONDS`.
   Next version: never wrap a gate in a tool that might be absent.
5. **Playbook §4 "one heredoc per command, ≤ 5 KB"** — the first attempt to
   write this record was one 9 KB unquoted heredoc chained after a 6.4 GB tar;
   the command timed out at 2 min and the partial bag archive had to be
   deleted. Did: bag archive as its own background job; record written as
   three quoted parts ≤ 3 KB and concatenated. Ledger row L7–L11 applies.
6. **Playbook principle 10 vs KICKOFF §0.1 "commit or stash"** — did: staged,
   did not commit; commit line in §0.1.

## Timings

| Step | Start (UTC) | End (UTC) | Notes |
|---|---|---|---|
| Full tree archive + bundle + checksums | 23:32 | 23:35 | taken before KICKOFF existed |
| Tag, branch, stage docs, env capture | 23:45 | 23:50 | |
| Restore rehearsal (extract 42 s; build 369 s; tests 4 s) | 23:50 | 23:57 | attempt 1 void (no /usr/bin/time) |
| Rosbag archive (6.4 GB) | 00:07 | 00:16 | |
| This record | 00:05 | 00:17 | |
