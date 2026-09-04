# ProjectHailMary — counter-UAS radar + camera fusion on a Jetson

A 22-node (19 launched) ROS 2 Humble / C++17 pipeline that turns a TI IWR6843ISK 60 GHz mmWave radar and an
AR0234 camera into tracked, classified, geofenced targets with a live operator view and a
CoT/ATAK network feed. Runs on a Jetson Orin Nano 8 GB. Written to MISRA C++:2023 / JPL
Power-of-10 discipline: fixed-width types, no heap on hot paths, bounded loops, fixed-capacity
containers, one exception boundary per process.

```
 radar (serial TLV) ──► parse + filter ──► DBSCAN ──► IMM tracker (CV/CA/CT) ──┐
                                                                             ├─► threat + intent ──► geofence / reachability ──► RViz2 · camera overlay · CoT UDP
 camera (V4L2) ──► YOLOv8s INT8 (TensorRT) ──► radar→camera fusion ───────────┘
```

Full diagram and dataflow: `docs/architecture_diagram.md`. Message contract: `docs/ICD.md`.

## Hardware

| Part | Notes |
|---|---|
| TI IWR6843ISK mmWave radar | CP2105 USB serial: interface 00 = config, 01 = data (`scripts/99-iwr6843.rules` → `/dev/radar_config`, `/dev/radar_data`); profile `src/cuas_fusion/config/radar_profile.cfg`, 20 Hz frames |
| AR0234 global-shutter camera | V4L2 `/dev/video0`, 1920×1080, intrinsics in `src/cuas_fusion/config/` |
| NVIDIA Jetson Orin Nano 8 GB | JetPack 6 / L4T 36.5, CUDA 12.6, TensorRT 10.3, ROS 2 Humble |

## Run it in simulation (no radar needed; the camera and TensorRT engine are real)

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select cuas_msgs cuas_fusion        # 0 warnings, -Werror on
source install/setup.bash
ros2 launch cuas_fusion sim.launch.py scenario:=approach     # approach | lateral | circle | two_targets | crossing | clutter | max_range | hover
ros2 topic hz /tracks                                        # ~20 Hz; RViz2 opens with the demo config
```

What "working" looks like: `/radar/detections` and `/tracks` at 20 Hz, `/health/status`
status 0 (NOMINAL), a track marker and label in RViz2 walking down the +Y axis, the camera
overlay showing a health strip at the top.

## Run it on hardware

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch cuas_fusion full.launch.py            # sends the radar profile, then starts the parser only if that succeeded
```

Stop with Ctrl-C and check nothing survived: `pgrep -af install/cuas_fusion` must print
nothing before the next launch. If the radar is unplugged and replugged while running, the parser
reopens the port but the sensor comes back idle: re-send the profile with
`scripts/send_radar_config.sh` (unverified on hardware, audit F5). Rollback:
`git checkout pre-polish-2026-09-01 && colcon build`.

## Measured (2026-09-02, sim radar + real camera, Orin Nano Super mode)

| Quantity | Value |
|---|---|
| `/radar/detections`, `/tracks`, `/threat/reports`, `/fusion/detections` | 20 Hz |
| YOLOv8s INT8 in the full pipeline | 22–27 Hz across 27 runs (≈24 typical); GPU load 10–90 % over a run, so "GPU-bound" is inferred, not isolated |
| `/tracks` stamp age at the subscriber | 2.7 ms p50 |
| Unit tests | 160 GoogleTest cases in 22 binaries, 0 failures (`colcon test-result` prints 182: it adds one row per binary) |
| Static analysis | cppcheck 2.7: 6 findings (1 warning, 3 style, 2 performance); clang-tidy 14: 710 warnings / 0 errors (A1 baseline, not re-run); `-Werror` clean |

Every number above has its artefact and conditions in `docs/claims-evidence.md`; the shape-by-shape
proof runs are in `docs/audit-plan.md` §9.

## Status

Alpha. Done: the pipeline above end to end in simulation with the real camera and TensorRT engine
in the loop; the radar-hardware path was last exercised in the April 2026 bags on an older tree.
Not done / unverified:
extrinsics are nominal (translation sign unverified), the CoT feed has never been received by an
ATAK client, no hardware soak, and the open decisions listed in `docs/claims-evidence.md`
(network exposure, threat class policy, de-escalation, clutter relearn, geofence hysteresis).

## Reading order for a reviewer

`docs/claims-evidence.md` (claims → evidence) → `docs/audit-plan.md` (the 2026-09 whole-project
audit: 37 root causes, 7 fix batches, every gate result) → `docs/ICD.md` → `docs/HANDOFF.md`.
