# Polish retro — 2026-09-01/02 whole-project audit

## What was broken (the ones that changed a decision)

- **The threat path could never fire.** `Track.doppler_mps` was hard-wired 0, so
  IDENTIFIED→THREATENING (which needs a closing speed) was unreachable, and the CoT alert path
  with it (RC-1). Fixed: the tracker publishes the line-of-sight speed of its own estimate.
- **Fast targets were pinned near 0 m/s.** A fixed 0.15 m/s-per-update velocity gate against a
  zero-velocity birth rejected every real velocity innovation; a 0.8 m nearest-neighbour gate
  then spawned a new tentative track per frame (RC-2, RC-3). Fixed: covariance-scaled gates.
- **The IMM stopped being an IMM.** Mixing kept stale cross-covariance and left P indefinite
  within 5 s; the mode update used the posterior instead of the mixed prior and μ_CT
  underflowed to 1e-323 (RC-34, RC-7). Fixed: block-diagonal injection, mixed-prior update.
- **Two clocks.** Drivers stamped CLOCK_MONOTONIC, the tracker RCL_STEADY_TIME
  (CLOCK_MONOTONIC_RAW); the 415 ms drift broke the 150 ms fusion gate after ~75 min (RC-6).
- **Nothing was ever pruned.** Five FixedMap<32> keyed by an unbounded track id: after 32 ids
  the geofence, reachability, classifier and visualizer silently stopped (RC-4).
- **Stale labels.** Fusion never published empty arrays; the classifier kept the last non-empty
  set forever and re-applied it by bearing alone (RC-5).
- **Silence looked like death, and death looked like nothing.** Producers skipped empty frames
  so health reported DEAD on an empty scene; a missing TensorRT engine exited 0 (RC-12, RC-22).
- **The sim never flew where the camera looks.** Scenes moved along X (azimuth) in a Y-forward
  frame, so fusion had published 0 Hz in every smoke for a reason that was the scene (RC-38).

## What was fixed
37 root causes triaged (+1 found in B6): 30 fixed in code, 1 partial, 2 documentation-only, 5 owner
decisions; seven checkpointed batches (B1 parser … B7 tooling), each behind build (-Werror,
0 warnings) → tests (136 → 160 GoogleTest cases) → cppcheck baseline → sim smoke.
Then one role-6 bug-finder pass over the cumulative diff (FIX-FIRST: six findings, five fixed,
one hardware note), the shape runs (A6, two passes: the first found twin tracks from the sim's
raw multi-return targets, a clutter scene outside the map, and scene timing; the second is the
record), and a second role-6 pass on the fixes only.

## What was reworded because it could not be proven
"144 MISRA/JSF violations resolved" (no checker exists), "35 Hz YOLOv8s" (22–25 Hz measured
in-pipeline), "16 Hz radar" (20 Hz), "108 tests" (160 GoogleTest cases; colcon counts 182 with suite rows), "Hungarian tracker" (not launched),
"calibrated extrinsics" (nominal), "IoU association" (padded-box containment). See
`docs/claims-evidence.md` and CHANGELOG.

## Open decisions
D-1 network exposure · D-2 threat class policy · D-3 de-escalation · D-4 clutter relearn ·
D-5 extrinsics sign · D-0 keep the two pre-kickoff commits · the hardware smoke (S-7, S-9, S-12).

## Lessons for the playbook (friction log 7–11)
Scratch on tmpfs dies with the box; process patterns never go on the caller's command line;
shared headers cost a 5-minute rebuild per touch; the review pass must check every producer's frame
against the ICD; name the build worker count in the gates.
