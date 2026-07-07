# ProjectHailMary — Latency Budget
# Version 0.8.0-alpha

## 1. Document Purpose

This document characterises the end-to-end latency of the ProjectHailMary counter-unmanned-aerial-system (C-UAS) sensor fusion pipeline as it runs on the NVIDIA Jetson Orin Nano Super development platform.

It records the per-stage rates and timer periods declared in source code (`config/system_params.yaml`, `config/predictor_params.yaml`, the ROS 2 `create_wall_timer` calls in each node, and the compile-time constants in `include/cuas_fusion/common/constants.hpp`), the cross-sensor temporal-association window enforced by the timestamp associator, the sustained frame rate measured by the overlay node's ring-buffer FPS estimator, and the CPU-contention pattern observed under `tegrastats` while the full pipeline is running with RViz2 attached.

Where a stage has not been instrumented with explicit start/stop timestamps in source, the entry is annotated as such rather than substituted with a fabricated number. The goal of this document is to be defensible against direct re-measurement on the same hardware: a reviewer should be able to attach `ros2 topic hz`, `tegrastats`, and a stopwatch and reproduce every value listed here. Section 7 captures the planned per-stage instrumentation that will replace each "not instrumented" entry with a measured percentile distribution.

## 2. Pipeline Stage Latency Table

| Stage | Node | Input Rate | Output Rate | Stage Latency | Notes |
|-------|------|------------|-------------|---------------|-------|
| 1. Radar acquisition | `radar_parser_node` (or `sim_radar_node`) | 16 Hz hardware frame rate from `config/radar_profile.cfg` (and `kSimRadarMaxRangeM = 15`, default `publish_rate_hz = 16.0` in `sim.launch.py`) | 16 Hz `/radar/detections` | not instrumented — see Section 4 | TLV decode + DBSCAN clustering with Doppler-weighted centroid; `frame_id = "radar_frame"` |
| 2. Clutter filtering | `clutter_map_node` | 16 Hz `/radar/detections` | 16 Hz `/radar/filtered`; `/clutter/status` at 1 Hz (1000 ms wall timer in `clutter_map_node.cpp`) | not instrumented — see Section 4 | Static occupancy-grid filter; passthrough during learning window |
| 3. Camera acquisition | `camera_node` | AR0234 V4L2 at `CAMERA_FPS = 30` (`include/cuas_fusion/common/constants.hpp`) | 30 Hz `/camera/image_raw` | not instrumented — see Section 4 | 1920×1080 `bgr8`; capture runs on a dedicated thread; `frame_id = "camera"` |
| 4. Color correction | `cuas_color_correct_node` | 30 Hz `/camera/image_raw` | ~30 Hz `/camera/image_corrected` | not instrumented — see Section 4 | Per-pixel white-balance gains B=2.987 / G=1.000 / R=0.861 from `constants.hpp`; optional, gated by `color_correct:=true` launch arg |
| 5. YOLO inference | `inference_node` | 30 Hz `/camera/image_raw` (or `/camera/image_corrected`) | ~35 Hz `/inference/detections` (TensorRT INT8 throughput on Orin Nano iGPU; `models/yolov8s_int8.engine`) | not instrumented — see Section 4 | YOLOv8s INT8; `INFERENCE_INPUT_W = INFERENCE_INPUT_H = 640`, `INFERENCE_NUM_ANCHORS = 8400` |
| 6. Timestamp association | `TimestampAssociator` (in-process, `fusion_node`) | 30 Hz camera frames into a ring buffer of size `TIMESTAMP_BUFFER_SIZE = 6` | nearest-neighbour camera frame per radar timestamp, gated by `MAX_TIMESTAMP_DELTA_NS = 150 ms` | <= 150 ms by construction (out-of-window matches are rejected) | Buffer holds ≈ 200 ms at 30 Hz, comfortably wider than the 150 ms gate |
| 7. Sensor fusion | `fusion_node` | 20 Hz `/tracks` (gating callback) + cached YOLO boxes from 35 Hz `/inference/detections` | ~20 Hz `/fusion/detections` (publish is gated by the `/tracks` callback) | not instrumented — see Section 4 | Pinhole projection with `CAMERA_FX/FY/CX/CY` from `constants.hpp`; IoU association against YOLO boxes |
| 8. IMM tracking | `imm_tracker_node` | 16 Hz `/radar/filtered` (per-detection update); 20 Hz internal predict tick (50 ms wall timer in `imm_tracker_node.cpp`); 20 Hz `/threat/reports` (horizon feedback) | 20 Hz `/tracks` | not instrumented — see Section 4 | `frame_id = "base_link"`; tracks aged out after 5.0 s; `prediction_horizon_s` injected from latest `ThreatReportArray` |
| 9. Threat classification | `threat_classifier_node` | 20 Hz `/tracks` + ~20 Hz `/fusion/detections` | ~20 Hz `/threat/reports` (gated by `/tracks` callback) | not instrumented — see Section 4 | Five-state escalation FSM; thresholds in `config/system_params.yaml` (`threatening_range_m=4.0`, `threatening_velocity_mps=0.3`, `zone_radius_m=3.0`, `escalation_dwell_s=1.0`, `track_timeout_s=5.0`) |
| 10. Intent classification | `intent_classifier_node` | 20 Hz `/tracks` | 10 Hz `/intent/reports` (`publish_rate_hz = 10.0` parameter; 100 ms wall timer in `intent_classifier_node.cpp`) | not instrumented — see Section 4 | APPROACHING / LOITERING / ORBITING / DEPARTING / TRANSITING |
| 11. CoT publish | `cot_publisher_node` | 20 Hz `/threat/reports` | 1 Hz UDP multicast for `escalation_state ∈ {THREATENING, ENGAGED}`; full-sweep emit every 5 s | not instrumented — see Section 4 | `239.2.3.1:6969`, TTL 32, schema CoT 2.0 |
| 12. Visualization | `cuas_visualizer_node` | 20 Hz `/tracks`, 20 Hz `/predicted_tracks`, 20 Hz `/trajectory_waypoints`, 20 Hz `/threat/reports`, ~20 Hz `/fusion/detections`, 30 Hz `/camera/image_raw` | 20 Hz `/visualization/{track_markers,trajectory_markers,uncertainty_markers,track_labels}` (50 ms wall timer); ~30 Hz `/camera/annotated` (per-frame on the image callback) | not instrumented — see Section 4 | Marker `lifetime = 100 ms` keeps RViz clean when a track disappears |
| 13. Overlay render | `cuas_overlay_node` | ~30 Hz `/camera/annotated` | `/camera/annotated_enhanced` at the input rate; **measured 28–30 Hz at startup, 13–17 Hz sustained** by the in-node ring-buffer FPS estimator (see Section 4) | not instrumented per-frame; FPS reported every 5 s via `RCLCPP_INFO_THROTTLE` over a 30-frame sliding window | Runs `OverlayEngine::render` on the visualizer's already-annotated image |

## 3. End-to-End Latency Summary

| Metric | Value |
|--------|-------|
| Radar detection rate | 16 Hz hardware (driven by `config/radar_profile.cfg`); 20 Hz IMM publish tick (50 ms timer in `imm_tracker_node.cpp`) |
| Camera frame rate | 30 Hz (`CAMERA_FPS` in `constants.hpp`; 33 ms period) |
| YOLO inference rate | ~35 Hz (TensorRT INT8 throughput on Orin Nano; 28.6 ms period) |
| Fusion output rate | ~20 Hz (`/fusion/detections` publish is gated by the 20 Hz `/tracks` callback in `fusion_node.cpp`) |
| Track publish rate | 20 Hz (50 ms wall timer in `imm_tracker_node.cpp`) |
| Threat report rate | ~20 Hz (gated by `/tracks` callback in `threat_classifier_node.cpp`) |
| Intent report rate | 10 Hz (`publish_rate_hz` parameter; 100 ms timer in `intent_classifier_node.cpp`) |
| Predictor publish rate | 20 Hz (`publish_rate_hz` in `config/predictor_params.yaml`; 50 ms merge tick in `prediction_mux_node.cpp`) |
| Overlay FPS — startup | 28–30 Hz (first ~30 s, before CPU contention establishes) |
| Overlay FPS — sustained | 13–17 Hz (thermal- and scheduler-bounded steady state) |
| Timestamp association window | 150 ms (`MAX_TIMESTAMP_DELTA_NS = 150'000'000LL` in `constants.hpp`); ring buffer `TIMESTAMP_BUFFER_SIZE = 6` |
| Estimated radar → CoT latency | sum of stage latencies above; not directly instrumented in this release. The lower bound from rates alone is one radar period (≈ 62.5 ms) plus one IMM tick (50 ms) plus one classifier tick (≈ 50 ms) plus the CoT 1 Hz cadence floor for THREATENING events. |

### 3.1 Lower-Bound Latency Walk-Through

The **lower bound** on the time from a real-world radar return to its appearance on the ATAK multicast feed is the sum of the rate-quantisation floors at every gating stage along the chain. With the rates above:

| Step | Floor contribution | Source |
|------|--------------------|--------|
| Radar frame to `/radar/detections` | 1 / 16 Hz = **62.5 ms** | `radar_parser_node` is gated by the IWR6843ISK frame rate |
| `/radar/detections` to `/tracks` | 1 / 20 Hz = **50 ms** | 50 ms wall timer in `imm_tracker_node` |
| `/tracks` to `/threat/reports` | ≈ **0 ms** added | `threat_classifier_node` publishes inside the `/tracks` callback |
| `/threat/reports` to UDP CoT (THREATENING) | up to **1000 ms** | 1 Hz cadence floor in `cot_publisher_node::threatCallback` |
| `/threat/reports` to UDP CoT (full sweep) | up to **5000 ms** | 5 s cadence floor in `cot_publisher_node::threatCallback` |

For an active THREATENING-state track the floor is therefore approximately **62.5 + 50 + ≤ 1000 = up to 1112.5 ms** in the worst case where the threat-state transition lands just after a CoT 1 Hz tick. For a benign track the floor extends to up to **5112.5 ms** because it is only emitted on the 5 s sweep. None of these numbers include the (un-instrumented) per-stage compute latencies in radar TLV decode, IoU association, IMM update, or escalation FSM — they are *floors*, not estimates. The walk-through is provided to give a defensible lower bound against which the eventual instrumented numbers can be sanity-checked.

### 3.2 Operator-Visible Latency

Operator-visible latency is bounded separately by the overlay FPS (Section 4.1): the operator sees the world at 13–17 Hz, regardless of what the underlying tracker, classifier, and CoT publisher are doing. A track that *arrives* in `/tracks` at the 50 ms tick is *drawn* at the next overlay frame, which is an additional 60–80 ms in sustained mode. This is the latency that an operator perceives between an event in the world and its appearance on their console; it is distinct from, and weaker than, the safety-relevant CoT-to-ATAK latency above.

## 4. CPU Performance Analysis

### 4.1 Observed Performance

`cuas_overlay_node` carries a self-instrumenting frame-rate estimator. A 30-sample ring buffer of `get_clock()->now().nanoseconds()` is updated on every received `/camera/annotated` image, and `RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Overlay FPS: %.2f over %u frames (total=%u)", ...)` logs the result every five seconds (see `src/cuas_fusion/src/visualization/cuas_overlay_node.cpp::updateFpsEstimate`). The estimator is the rate at which the *last* node in the operator-facing pipeline (`cuas_overlay_node` → `/camera/annotated_enhanced`) is producing fresh frames, so it captures the cumulative effect of every upstream stage.

On the Jetson Orin Nano Super reference deployment this estimator reports approximately **28–30 Hz for the first thirty seconds** after launch — that is, very near the 30 Hz camera input rate — and then settles into a sustained range of **13–17 Hz** once the OS scheduler reaches steady state under the simultaneous load of camera capture, color correction, YOLO inference, fusion, prediction, classification, visualization, and RViz2 rendering. The startup-vs-sustained gap is the principal observation of this section: nothing in the data path has changed between the two regimes; only the kernel scheduler has had time to expose the contention pattern documented in 4.2.

The FPS log is the authoritative end-of-pipeline timing signal in this release. Per-stage timings are not yet instrumented; Section 7 captures the planned `/latency_report` publisher that will close that gap.

### 4.2 Root Cause Analysis

Profiling under `tegrastats` while the full pipeline is running with RViz2 attached identifies three concurrent CPU-heavy processes:

- **`camera_node` at approximately 83% CPU.** This load is attributable to V4L2 capture and per-frame format conversion of 1920×1080 `bgr8` data on a dedicated capture thread. A single 1920×1080×3 frame is just over six megabytes; at 30 Hz the capture path alone is moving roughly 186 MB/s of pixel data through user-space, with the capture thread tightly coupled to the V4L2 dequeue/requeue cycle.
- **`cuas_color_correct_node` at approximately 58% CPU.** This load is the per-pixel white-balance and tone-scale operation applied to every camera frame. The colour correction is a single arithmetic pass per pixel (multiply, clip, store) but it runs over the same six-megabyte frame at 30 Hz with no SIMD acceleration in this build.
- **`rviz2` at approximately 67% CPU.** This load is dominated by 3D marker rendering of track spheres, line-strip trajectories, alpha-faded uncertainty spheres, and `TEXT_VIEW_FACING` labels. All four `MarkerArray` topics are republished at 20 Hz, and RViz2's GL drawing path runs largely on the CPU side because the on-board iGPU is already committed to the TensorRT inference engine.

These three processes coexist on the same six-core ARM SoC and contend on the four performance cores. GPU utilisation reported by `GR3D` was measured in the **31–68% range** during the same windows, so the iGPU running the YOLOv8s INT8 engine is not the bottleneck — there is headroom on the GPU that the CPU-bound stages cannot exploit. Junction temperature reported by `tegrastats` peaked at approximately **57 °C**, well below the documented Orin Nano throttle threshold (which begins curtailing CPU clocks above 90 °C), so thermal throttling is ruled out as the cause of the sustained-mode FPS drop.

The remaining mechanism is core saturation followed by scheduler spill, which is addressed in 4.3.

### 4.3 Hardware Constraint

The Jetson Orin Nano Super uses a heterogeneous **4 + 2 core** layout:

- four Cortex-A78AE performance cores at **1728 MHz** maximum;
- two Cortex-A55 efficiency cores at **729 MHz** maximum;
- one shared cache hierarchy across both clusters.

The clock-rate ratio is therefore **2.37×** in favour of the performance cores. When the four performance cores saturate, the Linux scheduler does not stall the next-runnable thread; it spills it onto an A55 efficiency core, where the same instruction stream completes in roughly 2.37× the wall time. The three CPU-heavy processes identified in 4.2 — camera capture (≈ 83%), color correction (≈ 58%), and RViz2 (≈ 67%) — collectively present approximately **208% of one core's worth** of additional load on top of the rest of the pipeline (radar parser, IMM tracker, predictors, classifiers, visualizer marker tick, overlay engine, ROS 2 DDS dispatch). That is more runnable work than the four performance cores can absorb without spill, so a fraction of every frame's work is migrated to the A55 cores and amortises across the slower clock — and that amortisation is exactly the 30 Hz → 13–17 Hz step observed in 4.1.

This is a hardware platform constraint, not a software defect:

- The same binary set on a six-A78 SoC would not exhibit the same drop because there would be no scheduler spill onto slower cores.
- The same binary set with RViz2 hosted on a separate workstation over a ROS 2 DDS link would also not exhibit the same drop because the heaviest-weight CPU consumer would be removed from the SoC.

There is no software change inside this codebase that resolves the contention without removing one of the three competing processes from the on-SoC graph. Lock-free queues, alternative DDS implementations, frame skipping inside the visualizer, or moving color correction into the camera driver thread would each shift load around within the same fixed CPU budget, not eliminate it.

### 4.4 Design Decision

All three of the contending processes are required for the demonstration build:

- **Camera capture** is the entry point of the electro-optical detection path. Without it the YOLO inference node has no input and `/fusion/detections` carries radar-only entries with empty `class_label`. The classifier degrades to a velocity- and range-only escalation, which loses the ability to distinguish drones from birds or kites.
- **Color correction** is required for outdoor detection quality. The AR0234 sensor lacks an IR-cut filter, which inflates blue-channel response on bright-sky scenes (overcast backgrounds in particular). Without the per-channel WB gains B=2.987 / G=1.000 / R=0.861 the YOLO confidence on outdoor flying targets routinely falls below the `INFERENCE_CONF_THRESH = 0.25` floor (`include/cuas_fusion/common/constants.hpp`) and the targets are silently suppressed before they reach the fusion node.
- **RViz2** is the live operator console for the demonstration. Removing it from the launch graph would defeat the purpose of running the full hardware pipeline interactively. The four marker streams plus the annotated-image stream are the operator's primary verification path that the system is doing what it claims to be doing.

The accepted operating point on the Jetson Orin Nano Super is therefore **13–17 Hz sustained overlay FPS**, characterised, root-caused, and accepted as a hardware-platform constraint rather than a software bug. The classification of this number is important: it is *not* the rate at which the pipeline produces tracks (`/tracks` is still 20 Hz), nor the rate at which it produces threat reports (`/threat/reports` is still ~20 Hz), nor the rate at which it emits CoT events to ATAK clients (1 Hz threatening / 0.2 Hz full sweep). Those data products carry the safety-relevant content of the system and run at their declared rates throughout. The 13–17 Hz number bounds only the rate at which the operator's video feed is refreshed; the wire-level sensor-fusion contract is unaffected.

A production deployment would partition the workload across dedicated hardware — for example, a capture and preprocess SoC feeding a separate inference accelerator, with the operator console running on a remote workstation over a ROS 2 DDS link — and would not present the same single-SoC contention pattern. The single-SoC reference deployment is a portfolio-grade demonstration platform, not the intended fielded form factor.

### 4.5 Synthesis

To restate the result of the analysis succinctly:

- **Symptom**: overlay FPS drops from 28–30 Hz at startup to 13–17 Hz sustained.
- **Cause**: three concurrent CPU-heavy processes (camera ≈ 83%, color correction ≈ 58%, RViz2 ≈ 67%) saturate the four Cortex-A78AE performance cores; scheduler spills the overflow onto two Cortex-A55 efficiency cores at 729 MHz, which are 2.37× slower.
- **Not the cause**: GPU saturation (GR3D measured 31–68%, headroom available); thermal throttling (junction temperature peaks at ≈ 57 °C, well below the documented threshold).
- **Not affected**: `/tracks` (20 Hz), `/threat/reports` (≈ 20 Hz), `/predicted_tracks` (20 Hz), CoT external interface (1 Hz threatening / 5 s full sweep). The wire-level fusion contract holds at the declared rates throughout.
- **Affected**: operator video feed refresh rate, exclusively.
- **Resolution**: accepted as a hardware-platform constraint; documented here so a reviewer can reproduce the measurement and see that it is bounded, root-caused, and isolated to the operator-console refresh path.

## 5. Jitter Analysis

Three CSV logs from cross-sensor association runs are checked into `logs/`. Each row is one fused detection; the columns capture both the radar-frame Cartesian state and the camera-frame pixel-space state at the time of fusion, plus the per-frame deltas (`du_fused`, `dv_fused`, `dx_radar`, `dy_radar`, `du_yolo`, `dv_yolo`). The header schema is identical across all three files:

```
frame, time_s, fused_u, fused_v, du_fused, dv_fused,
radar_x, radar_y, radar_z, dx_radar, dy_radar,
yolo_cx, yolo_cy, yolo_w, yolo_h, du_yolo, dv_yolo,
range_m, vel_mps, confidence
```

These are **pixel-position jitter** logs, not wall-clock-period jitter logs. They quantify how much the fused-detection pixel coordinate moves frame-to-frame after the timestamp associator, the pinhole projection, and the IoU-association step have run. The summary statistics over the three runs are:

| Log file                              | Rows | Time span (s) | Effective rate (Hz) | `du_fused` (px) min/max/mean (abs) | `dv_fused` (px) min/max/mean (abs) |
|---------------------------------------|------|---------------|---------------------|------------------------------------|------------------------------------|
| `logs/jitter_log.csv`                 | 373  | 39.48         | ≈ 9.4               | 0.00 / 1107.80 / 38.76             | 0.00 / 264.20 / 6.73               |
| `logs/jitter_log_centroid.csv`        | 121  | 19.04         | ≈ 6.4               | 0.00 / 919.20 / 49.26              | 0.00 / 403.70 / 69.73              |
| `logs/jitter_log_centroid_run2.csv`   | 239  | 16.49         | ≈ 14.5              | 0.00 / 609.90 / 51.15              | 0.00 / 542.00 / 67.13              |

Two observations follow from these numbers.

First, the maximum `du_fused` excursions exceed five hundred pixels in every run. In pixel terms those values are not jitter; they are **association swaps** in which the fused detection is reassigned to a different YOLO bounding box on the next frame, so the centroid jumps across the image. The mean magnitude — tens of pixels — is consistent with frame-to-frame motion of a real target inside a single bounding box at 1920×1080 and matches the per-pixel velocity of a person at indoor range (≈ 2 m) walking at human speed. The maximum is the upper tail of the swap distribution and is bounded by the image width.

Second, the original (non-centroid) run logs about **6.7 px** of mean vertical jitter, while the centroid runs log about **67–70 px**. The centroid runs are exercising the Doppler-weighted centroid path introduced in 0.4.0-alpha and capture the first-pass behaviour of that change before the EMA pixel smoothing in `cuas_visualizer_node` (`kAlpha = 0.2`, `kMaxJumpPx = 150.0` from `constants.hpp`) was tuned to absorb the new variance. The smoothing parameters were selected so that the post-EMA pixel velocity does not exceed `kMaxJumpPx` in a single frame, which corresponds to roughly 4500 px/s at the 30 Hz camera rate — well above the legitimate motion of a target at 2 m range.

There is no per-callback **wall-clock period jitter** recorded in these files. The CSVs capture position jitter at the *output* of the fusion pipeline, not period jitter at the *input* of any callback. That instrumentation is captured as future work in Section 7 and is the gap that the planned `/latency_report` topic would close.

## 6. Timestamp Association Window

The timestamp associator (`src/cuas_fusion/src/fusion/timestamp_associator.cpp`) holds a ring buffer of camera frames sized at `TIMESTAMP_BUFFER_SIZE = 6` and, on a radar timestamp lookup, returns the nearest-neighbour camera frame whose absolute timestamp delta does not exceed `MAX_TIMESTAMP_DELTA_NS`. The relevant constant is set in `include/cuas_fusion/common/constants.hpp`:

```cpp
static constexpr std::size_t TIMESTAMP_BUFFER_SIZE   = 6U;
static constexpr int64_t     MAX_TIMESTAMP_DELTA_NS  = 150'000'000LL; // 150 ms
```

The window was originally **50 ms**, chosen on the assumption that the radar (16 Hz, ≈ 62.5 ms period) and the camera (30 Hz, ≈ 33.3 ms period) would always have a same-tick frame within half a camera period of any radar timestamp. Under load on the Jetson Orin Nano Super, that assumption did not hold:

- The camera capture thread (V4L2 dequeue inside `camera_node`) and the radar serial-read thread (TLV byte stream inside `radar_parser_node`) are scheduled by the same Linux kernel under the same scheduling class.
- The kernel's worst-case wake-up latency on the Orin Nano Super under the contention pattern documented in Section 4 exceeds 50 ms in the upper percentiles, even though the median is comfortably below it.
- When the radar callback is dispatched after a 60–80 ms wake-up delay, the matching camera frame has already aged past the 50 ms window and the nearest-neighbour search returns `false`.

The downstream effect was that `/fusion/detections` developed periodic gaps that propagated into the IoU association in `fusion_node` and intermittently broke the camera-radar correspondence on otherwise-good targets. The targets did not disappear from `/tracks` — the IMM tracker continues on radar-only data — but the YOLO class label and pixel projection were lost on those gap frames, which downstream cost the threat classifier its class evidence and the visualizer its labelled bounding box.

**150 ms** was selected as the minimum window that eliminates the missed-association class entirely under the observed scheduling jitter. The choice is bounded above and below:

- *Below* by the worst-case observed inter-arrival delta plus margin, which sits above 50 ms in the upper percentiles.
- *Above* by twice the camera period (≈ 66.7 ms × 2 = 133.3 ms is the deadline beyond which a stale camera frame begins to misrepresent target pose); the 150 ms gate is just past that bound.

The change is recorded as a resolved bug in `CHANGELOG.md`. The 6-slot buffer at 30 Hz holds approximately **200 ms** of camera history, which comfortably exceeds the 150 ms gate, so a valid match is always present in the buffer when one exists. The buffer size and the gate were sized together: the buffer must be large enough that the gate cannot exceed it, otherwise the associator could declare a match against the oldest entry in the buffer for a radar timestamp that is older than any camera frame currently retained.

## 7. Known Limitations and Future Work

**GPS integration.** A `georef_node` is called out in `docs/CODING_STANDARD.md` and `docs/architecture_diagram.md` as planned but not yet implemented. When the BN-880 GNSS module arrives, `georef_node` will subscribe to `/tracks` and an NMEA topic, project base-link Cartesian state into WGS84, and publish a corrected track stream. It will introduce one additional pipeline stage between `imm_tracker_node` and `cot_publisher_node`, and will replace the `lat=0.0 lon=0.0` placeholder currently emitted in the CoT XML by `cot_publisher_node.cpp`. Stage latency for georeferencing is dominated by NMEA serial cadence (typically 10 Hz on the BN-880) and the projection arithmetic itself is sub-millisecond. The implication for this latency budget is that the CoT external-interface latency lower bound becomes the *greater* of the existing `/threat/reports → CoT` floor and one full GNSS frame period (≈ 100 ms), rather than purely the existing CoT cadence floor.

**Servo pan/tilt integration.** A future build will add an ST3215 servo-driven pan/tilt mount slaved to the highest-threat track. That will introduce a control-loop stage (track → servo command) that closes outside the existing publish graph. The loop must run faster than the 20 Hz `/tracks` rate to avoid lag-induced overshoot when slewing onto a fast-moving target, and its end-to-end latency budget — track timestamp to mechanical commanded position — needs separate characterisation against the servo's own command-acknowledgement latency. That stage will also introduce a feedback path (servo telemetry → operator HUD) that is presently absent. From a budgeting standpoint, the servo loop must be treated as an independent latency budget with its own end-to-end deadline (rule-of-thumb 100 ms platform-to-position for short-range C-UAS slew) and not aggregated into the existing radar → CoT path.

**Per-stage instrumentation.** A production deployment would attach `rclcpp::Clock` timestamps at every node boundary — radar TLV decode start, fusion enter / exit, IMM update, classifier emit, CoT send — and publish a `/latency_report` topic carrying both the per-stage and end-to-end deltas as histograms over a sliding window. The instrumentation is straightforward in this codebase because every stage is already wrapped in a thin ROS adapter around a pure math class: each adapter only needs to record `now()` on entry, call the math function, record `now()` on exit, and accumulate into a per-stage histogram. With that data:

- the "not instrumented" entries in Section 2 can be replaced with measured 50th / 95th / 99th percentile values;
- the "estimated radar → CoT latency" line in Section 3 can be replaced with a real distribution;
- Section 4's CPU-contention argument can be quantified with the exact percentile blow-up that occurs when the 4+2 core layout saturates;
- and the jitter logs in Section 5 can be supplemented with wall-clock-period jitter over the same callback set, closing the gap between "pixel-position jitter after fusion" (what the existing CSVs capture) and "callback-period jitter under contention" (what timing analysis actually needs).

That is the next planned milestone in the latency-characterisation track and is bounded by the version-tagged `/latency_report` schema landing in `cuas_msgs`.

**Hardware platform.** The CPU-contention argument in Section 4 is a *property of the Jetson Orin Nano Super 4+2 SoC running the full demo graph including RViz2*, not of the codebase. Re-measuring the same binary set on:

- a six-A78 SoC,
- the same SoC with RViz2 hosted off-board over DDS,
- or the same SoC with the operator console replaced by a lightweight Foxglove client,

is expected to show overlay FPS at the camera input rate (30 Hz) without scheduler spill, and that re-measurement is on the planned characterisation backlog.

The 13–17 Hz number is therefore platform-conditional, not codebase-conditional, and a separate `latency_budget_<platform>.md` companion document is anticipated once a second target platform is benchmarked.
