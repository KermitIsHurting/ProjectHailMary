# HARDWARE_TEST_PLAN — verify branch `misra-audit-fixes` on the Jetson

Everything below was already verified locally where possible (build warning-clean,
97/97 gtests green at the last full run). These procedures cover what only live
hardware can prove. Run them in order. **Do not merge until all PASS.**

## 0. Build & test gate (5 min)

```bash
cd ~/ProjectHailMarry
git checkout misra-audit-fixes
source /opt/ros/humble/setup.bash
rm -rf build/cuas_fusion install/cuas_fusion
colcon build --packages-select cuas_msgs cuas_fusion 2>&1 | grep -cE 'warning:|error:'
# PASS: prints 0
colcon test --packages-select cuas_fusion && colcon test-result
# PASS: "97 tests, 0 errors, 0 failures"
```

## 1. Sim smoke test — no sensors needed (10 min)

```bash
source install/setup.bash
ros2 launch cuas_fusion sim.launch.py
```
- PASS: all nodes start, no FATAL lines, `/tracks` publishes (`ros2 topic hz /tracks`).
- PASS: `ros2 param get /classifier_node threatening_range_m` returns **4.0**
  (this proves the system_params.yaml rename A1.5 now reaches the node —
  before the fix the param existed only as a hardcoded default; also try
  editing the YAML value and relaunching: the change must take effect).

## 2. IMM maneuver behavior (sim, 10 min)

The IMM fix (7430a57) changes tracker dynamics: CT/CA models now retain their
private states, so maneuvering targets should hold track better and
`imm_ct_probability` on `/tracks` should rise during turns.

```bash
ros2 launch cuas_fusion sim.launch.py   # circle scenario if available:
ros2 param set /sim_radar_node scenario circle   # or relaunch with scenario:=circle
ros2 topic echo /tracks --field tracks | grep -A1 imm_ct
```
- PASS: during circular motion, `imm_ct_probability` climbs well above ~0.33
  and track position error stays bounded (no oscillation/divergence).
- FAIL/WATCH: if tracks become jumpy vs. main, note it — CT Jacobian sign fix
  changes filter gains; retune `sigma` params rather than reverting.

## 3. Reachability geometry (sim, 5 min)

```bash
ros2 topic echo /reachability/warnings
```
- PASS: for an inbound target (approach scenario), `intercept_possible: true`
  appears with plausible `time_to_intercept_s` (= range/closing speed).
  Before the fix this was never true for magnitude velocities.

## 4. Live radar (IWR6843 attached, 15 min)

```bash
ros2 launch cuas_fusion radar_only.launch.py
ros2 topic hz /radar/detections     # PASS: ~scan rate, stable
```
- Walk a target across the field of view **behind/through the ±180° bearing
  seam** relative to the sensor; with the camera pipeline running
  (full.launch.py), verify the track keeps its camera class label across the
  seam (bearing-wrap fix fb79d63).
- Unplug the radar USB mid-run: **known remaining defect** — the parse thread
  currently busy-spins on EOF (fix is queued in RESUME_PLAN.md item R1). Just
  confirm the node doesn't crash the rest of the launch; plug back in requires
  node restart. Do NOT treat as a regression of this branch.

## 5. Full pipeline + camera (30 min)

```bash
ros2 launch cuas_fusion full.launch.py
```
- PASS: camera → inference → fusion → tracks → threat/overlay all publish;
  visualizer image looks unchanged (no PROTECTED camera file was touched —
  verify with the command in §7).
- PASS: threat reports show `predicted_impact_*` consistent with
  `prediction_horizon_s` per threat level (fix fb79d63: impact was previously
  always extrapolated 5 s regardless of stamped horizon — impact points for
  THREAT-level tracks will now be farther out; this is the correction).

## 6. Soak sanity (optional, overnight)

- Leave sim.launch.py running overnight; `/health/status` should stay stable.
  (The float32 uptime-quantization defect in the health monitor is NOT yet
  fixed — RESUME_PLAN item R7 — it only matters after ~11 days uptime.)

## 7. PROTECTED-files invariant — **OBSOLETE as of R12**

> Owner authorized camera-file edits (RESUME_PLAN R12); this
> diff-must-be-empty check is intentionally obsolete for those files.
> Replacement check: (1) `ROLLBACK.md` exists at repo root with a
> per-commit log and restore commands, and (2) the camera-file diff
> against main has been reviewed by the owner. Register/format values
> (device path, 1920x1080@30, BA10, black level 2752, WB gains,
> intrinsics, V4L2 S_FMT/S_PARM setup) remain byte-identical — verify
> with the section 7b command below.

### 7 (original, superseded)

```bash
git diff main...misra-audit-fixes --stat -- \
  src/cuas_fusion/src/drivers/camera_driver.cpp \
  src/cuas_fusion/include/cuas_fusion/drivers/camera_driver.hpp \
  src/cuas_fusion/src/drivers/camera_node.cpp \
  src/cuas_fusion/config/camera_calibration.yaml \
  src/cuas_fusion/config/camera_calibration_result.yaml \
  src/cuas_fusion/src/color_correct_engine.cpp \
  src/cuas_fusion/include/cuas_fusion/color_correct_engine.hpp \
  src/cuas_fusion/src/cuas_color_correct_node.cpp
```
- PASS: **empty output** (camera files untouched). Note: `constants.hpp` was
  never modified on this branch either; `cuas_color_correct_node.cpp` shows
  only the main() catch-all wrapper (b3a97df) — if you consider that file
  strictly PROTECTED, revert that one hunk with:
  `git checkout main -- src/cuas_fusion/src/cuas_color_correct_node.cpp && git commit`

## 7b. Register/format-values invariant (replacement check)

```bash
git diff main...misra-audit-fixes -- \
  src/cuas_fusion/include/cuas_fusion/common/constants.hpp | grep '^[-+]' | \
  grep -E 'CAMERA_(WIDTH|HEIGHT|FPS|BLACK_LEVEL|WB_GAIN|TONE_SCALE|FX|FY|CX|CY|DEVICE)'
```
- PASS: no changed *values* (R12g re-expresses CAMERA_IMAGE_W/H as
  `= CAMERA_WIDTH/HEIGHT`, values unchanged). Also confirm the V4L2
  S_FMT/S_PARM block in camera_driver.cpp still sets the same
  format/rate (`git diff main...misra-audit-fixes -- src/cuas_fusion/src/drivers/camera_driver.cpp`).

## Merge / discard

```bash
# If all PASS:
git checkout main && git merge misra-audit-fixes        # do NOT push until you decide to

# Revert a single bad commit on the branch:
git revert <sha>

# Throw the whole branch away (main is untouched):
git checkout main && git branch -D misra-audit-fixes
```
