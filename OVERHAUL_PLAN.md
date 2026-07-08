# OVERHAUL_PLAN — 10x: from demo pipeline to genuine fast-drone radar-camera fusion

Written 2026-07-07 by the session that completed the MISRA audit branch
(see HANDOFF.md). This is the roadmap to turn the current
indoor-demo-scale pipeline into a real counter-UAS sensor-fusion system
that detects and tracks fast drones. Phases are ordered by dependency —
do not reorder 0→1→2→3; 4 and 5 can interleave after 3.

## Definition of done (measurable, not vibes)

- Detect and maintain a confirmed, correctly-labeled track on a drone
  crossing the field of view at ≥ 10 m/s radial or tangential, out to the
  radar's redesigned range, with < 1 s track initiation and no track loss
  through a 1 s single-sensor dropout.
- Projected radar track lands inside the drone's image bounding box
  (not a 25%-padded guess) across the full FOV.
- End-to-end latency (photon/chirp → /threat/reports) measured and bounded,
  with a published budget per stage.
- Every claim above demonstrated by a repeatable rosbag replay test in CI.

## Where the system actually is today (hard limits found in audit)

| Limit | Value today | Consequence for fast drones |
|---|---|---|
| Radar unambiguous Doppler | ≈ ±5 m/s (profileCfg: 3-TX TDM, idle 30 µs + ramp 57.14 µs ⇒ ~261 µs per-TX chirp period at 60 GHz — **recompute precisely before changing**) | a 15 m/s drone's Doppler aliases; velocity gating and clutter rejection actively work against exactly the targets we care about |
| Radar frame rate | 20 Hz (frameCfg period 50 ms) | 0.75 m of travel between scans at 15 m/s; association gates must grow accordingly |
| Software range gate | `MAX_RANGE_M` = 15 m, `CLUTTER_VEL_THRESH` = 0.1 m/s (radar_parser hardcoded) | indoor-demo scale; both must become profile-derived parameters |
| Detector | Stock COCO YOLOv8s (640²) | **"drone" is not a COCO class.** The deleted THREAT_DRONE_CLASSES hack (bird/airplane/kite) was the tell. The detector cannot currently say "drone" at all, and small distant quadcopters are exactly what COCO models miss |
| Extrinsics | Translation-only, rotation = identity | 1–2° mount error ⇒ 25–50 cm projection error at 15 m; the 25% box padding papers over it |
| Time base | Camera/radar CLOCK_MONOTONIC vs tracker ROS time; fusion uses latest-box + 500 ms staleness gate | no per-measurement temporal alignment; at 15 m/s a 50 ms mismatch is 0.75 m |
| Estimator | Cascaded: IMM(radar) → fusion(label join) → TrackManager(position-only KF); Track.msg carries no covariance, no acceleration, no turn rate | camera contributes only labels, never geometry; CV-only prediction (the CA/CT blend was fake and was honestly deleted); two different confirm policies (5 vs 3) |
| Fusion modes | Output only when radar AND fresh camera agree | camera-only target ⇒ silence; no bearing-only initiation |
| Georeference | lat/lon hardcoded 0,0 (ICD §4) | CoT output plots at null island |
| Validation | Zero live hardware runs on this branch; 3 camera commits NEEDS-HARDWARE (ROLLBACK.md) | everything below must start by building ground truth |

What is already solid and must be preserved: the process discipline
(-Werror zero-warning build, 108 green tests, commit-per-finding, deviation
records), the fixed-size/no-heap hot path, the NaN-safe math idioms, the
fault-tolerant node boundaries. Read HANDOFF.md before touching anything.

---

## Phase 0 — Ground truth and a validation harness (do this FIRST)

You cannot 10x what you cannot measure. Everything later is judged here.

1. **Hardware bring-up check**: run `full.launch.py`, verify the three
   NEEDS-HARDWARE camera commits per ROLLBACK.md, verify radar frames at
   20 Hz, `ros2 topic hz` on every stage.
2. **Rosbag corpus**: record synchronized runs — static scene, walking
   person (benign), small drone slow, drone fast passes, drone + person
   together, sensor-dropout runs (cover the radar; cap the lens). These
   bags are the project's most valuable asset from this point on.
3. **Projection-overlay truth tool**: extend the visualizer (or a small new
   node) to draw *raw projected radar clusters* (not tracks) on the image.
   This single view exposes extrinsic error, time skew, and association
   quality at a glance. Acceptance: screenshot per bag.
4. **Metrics node/script**: per-bag report — association rate, track
   continuity (ID switches, gaps), initiation latency, projection residual
   (px) against hand-labeled boxes, end-to-end latency (stamp deltas).
5. **Replay tests in CI**: bag-driven integration tests asserting the
   metrics don't regress. (CI itself: see Phase 5; a local script gate is
   fine to start.)

## Phase 1 — Calibration (unlocks everything geometric)

1. **Full SE(3) extrinsics**: `ExtrinsicTransform` gains a rotation
   (quaternion); fusion projection becomes `p_cam = R·(p_radar) + t`.
   Config-file driven (`config/extrinsics.yaml`), not constants.
2. **Radar-camera extrinsic calibration procedure**: corner reflector (or
   any strong isolated reflector) moved through the shared FOV; collect
   (radar 3D point, hand/detector-clicked pixel) pairs; solve R,t by
   minimizing reprojection error (a 50-line ceres-free Gauss-Newton or even
   scipy offline script is fine). Store residuals with the result — the
   residual IS the association gate floor.
3. **Shrink the association padding** from 25% to calibrated-residual-based.
   Acceptance: projected reflector within ±10 px across the FOV; overlay
   tool shows radar clusters centered on targets.

## Phase 2 — One time base

1. Pick the pipeline clock: simplest robust choice is CLOCK_MONOTONIC
   everywhere (camera and radar already use it); make imm_tracker and
   every `this->now()` stamp follow, or move everything to ROS time from
   one source. One decision, applied everywhere, documented in ICD.
2. **Use the TimestampAssociator pattern in fusion**: buffer recent YOLO
   detection sets with stamps; on each radar/track update, select (or
   linearly interpolate boxes of) the nearest camera frame, and extrapolate
   the track state to that stamp with its velocity. Kill the
   latest-box-within-500ms heuristic.
3. Acceptance: projection residual on a *moving* target ≈ residual on a
   static one (today it grows with speed).

## Phase 3 — Fusion core rebuild (the actual 10x)

Replace the cascaded IMM→label-join→position-KF chain with **one central
tracker** that ingests both sensors as measurements. Keep the existing
libraries as parts: the estimation stack (fixed-size, tested), the
Hungarian solver (stride-flexible Ref), FixedVector/Map.

1. **State & messages**: 9-state per track (pos, vel, acc) or 7-state CT —
   the *real* IMM this time; Track.msg grows covariance (6x6 or 9x9
   triangle), acceleration, and drops nothing (additive = wire-compatible).
   This is the interface change DEV-011 anticipated; bound the sequences
   while you're in there.
2. **Measurement models**:
   - Radar: 3D position + radial Doppler (Doppler is currently carried but
     never used in the update — use it; it's the best fast-target signal).
   - Camera: **bearing-only** — pixel → calibrated ray (az/el) with px-scaled
     noise, plus class evidence. Camera finally contributes *geometry*.
3. **Association**: global nearest neighbor via the existing Hungarian
   solver, per-sensor Mahalanobis gates in each measurement space (project
   track covariance into image space for camera). Best-match, not
   first-padded-box. JPDA only if crossing-target bags prove GNN insufficient.
4. **Asymmetric modes**: radar-only tracks coast with honest covariance
   growth (exists); **camera-only (bearing-only) initiation** — hold an
   unranged candidate track that fuses to full 3D on first radar hit.
   Fusion output never goes silent because one sensor blinked.
5. **Class fusion**: Bayesian label accumulation over time (log-odds per
   class per track) instead of last-write-wins.
6. **Confirm policy**: one M-of-N policy, one place (kills the 5-vs-3
   discrepancy flagged in HANDOFF §6.4).
7. Acceptance: crossing-targets bag with zero ID switches; dropout bag with
   no track loss; unit tests in the existing style (assert the old failure
   mode is impossible).

## Phase 4 — Fast-target capability

1. **Radar waveform redesign** (config-only, no code): new profileCfg for
   the drone envelope — target ≥ ±20 m/s unambiguous Doppler (shorter
   chirps / fewer TX in TDM / frame redesign; trade range resolution),
   longer range. Derive `MAX_RANGE_M`, clutter threshold, and gate sizes
   from the profile (one YAML both the radar script and parser read) instead
   of hardcoding. **Recompute the ambiguity numbers from first principles;
   the ±5 m/s figure above is an estimate to be verified.** Keep the old
   profile as `radar_profile_indoor.cfg`.
2. **Detector that knows what a drone is**: fine-tune YOLOv8 (s or n) on
   drone datasets (Anti-UAV, Det-Fly, VisDrone-UAV, plus own bags), export
   → ONNX → TRT engine (pipeline exists; A2.8 shape validation will catch a
   class-count change — update `INFERENCE_NUM_CLASSES` and the 1×84×8400
   expectation together). For small/distant targets evaluate: higher input
   res (960/1280 — re-measure TRT latency), or radar-cued ROI crops
   (radar tells you *where* to look — run detection on a crop around the
   projected gate; this is the cheap 10x for small targets and only works
   after Phases 1–2).
3. **Latency budget enforcement**: measure per-stage (stamps exist), publish
   in docs/latency_budget.md with real numbers, alert in health monitor
   when violated. The allocation work on this branch already bought the
   WCET headroom.
4. **Camera exposure for motion**: AR0234 is global-shutter (good); verify
   exposure time short enough at 30 fps that a 15 m/s crossing target
   doesn't smear (register work = camera-file protocol per the repo rules file).
5. Acceptance: the Definition-of-done drone passes, on bags and live.

## Phase 5 — Deployment hardening (interleave after 3)

1. **Georeferencing**: GNSS/pose source → WGS84 in Track/ThreatReport →
   real CoT lat/lon (the ICD §4 stub and DEV-011 migration path are
   waiting for this). Without it, "detection" never reaches a real ATAK
   consumer meaningfully.
2. **CI**: build + colcon test + cppcheck/clang-tidy + bag-replay metrics.
   Caveat from HANDOFF §2: no nvcc on target and aarch64+TRT deps — gate
   TRT targets behind an option or use a self-hosted runner.
3. **Degradation-modes doc + tests**: what the system does when each sensor
   dies (health monitor + retry loops exist; write it down, fault-inject it).
4. R11 polish list (HANDOFF §6.4) as filler work between phases.

## Sequencing summary

Phase 0 (days, mostly hardware time) → 1 (days) → 2 (days) →
3 (the big one — weeks, do it incrementally behind the existing tests) →
4.1/4.2 (parallel, config + ML work) → 5 ongoing. At every step the branch
rules hold: 0 warnings under -Werror, all tests green, one concern per
commit, deviations documented, camera-file protocol per ROLLBACK.md.

The single highest-leverage first week: Phase 0 items 1–3 plus the Phase 1
calibration. Everything else is guesswork until the overlay tool shows
radar returns sitting on top of the things that caused them.
