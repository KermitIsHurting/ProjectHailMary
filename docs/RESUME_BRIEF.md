# Resume Bullet Extraction Brief — ProjectHailMary

Paste this whole file into the AI executor running at the root of the ProjectHailMary repo.

---

## Who this is for

John Lopez. BS Computer Science, Cal State LA, graduated May 2026. No professional
software experience — no internships, no co-ops, only minimum-wage jobs. ProjectHailMary
is a solo project built specifically to get past that gap and land an entry-level role at
a defense technology company (Anduril, Northrop Grumman, Lockheed Martin, and similar).

This project is roughly 90% of the resume. It has to carry the whole thing.

## Your task

Read the entire repository — source, headers, tests, configs, launch files, docs, commit
history, and logs — and produce the strongest possible set of resume bullets for it.
Do not write the resume. Produce ranked, evidence-backed candidate bullets that a human
will select from.

## The bar you are writing against

These criteria come verbatim from a hiring manager at a defense tech company who reviews
these resumes himself. This is the actual rubric, not a guess:

1. **Technical depth is the only thing that matters.** In his words: you can spend all day
   changing colors and grammar, but it is the technical depth that matters.
2. **He skips resumes that are not detailed enough**, and it is "super obvious" when
   someone is using jargon for no reason. Jargon that does not carry information is a
   negative signal, not a neutral one.
3. **Do not start a bullet with the word "Engineered."** He calls this out by name.
   Also avoid "Utilized," "Leveraged," "Spearheaded," "Architected" as a reflex opener.
4. **Every line needs a quantity** — an absolute number, a rate, a percentage, or a
   before/after delta. His exact standard: it "doesn't need to be very accurate as long as
   you can defend where it came from."
5. **He spends about two seconds per resume on first scan**, reading company names, then
   titles, then school. Bolded technical details are what make a resume scannable in that
   window.
6. **Building things with your hands beats credentials.** He describes hiring someone with
   no direct experience who had built a lot of real things, because it demonstrated the
   ability to learn.

## The hard rule: evidence or it does not exist

For every bullet you propose, cite the exact source: `path/to/file.cpp:123`, a config key,
a commit SHA, a CSV column, or a doc section. If you cannot point to something in the repo
that supports a number, do not write the bullet. There is no acceptable rounding-up here —
John will be asked to defend these in an interview by someone who may know this domain
better than he does. A number he cannot trace to a file is worse than no number.

If a claim is *true but not yet measured* (for example, a latency you believe is low but
which the latency budget explicitly marks "not instrumented"), say so and propose what
measurement would substantiate it. Do not launder an estimate into a fact.

## What to dig for

Go well beyond the changelog. Specifically look for:

- **Hard problems with a before/after.** Bugs found, root-caused, and fixed. Wrong
  approaches abandoned and why. Parameters that were changed and the failure that forced
  the change. These are the highest-value bullets because they show engineering judgment,
  not just feature checklists.
- **Real measurements.** Anything in `logs/`, the FPS estimator, `tegrastats` observations,
  the jitter CSVs, calibration results. Compute statistics yourself from the CSVs if that
  yields a defensible number.
- **Scale and scope.** Node count, LOC, message types, test count, launch configurations,
  compile-time capacity bounds, number of distinct algorithms implemented.
- **Algorithms actually implemented from scratch** versus libraries called. From-scratch
  implementations are worth far more. Verify which is which by reading the source.
- **Standards compliance work.** The JSF AV C++ / MISRA pass, the documented deviations,
  what specific classes of violation were fixed and what the fix pattern was.
- **Systems engineering artifacts.** The ICD, architecture doc, latency budget, coding
  standard. Note what makes each defensible.
- **Commit history.** `git log` shows the actual arc of the work — what was built when,
  what got reworked, what was deferred and why. Sequence and rework are evidence of
  iteration.
- **Test coverage.** What is actually tested, what is not, and whether the ROS/math
  separation genuinely enables the testing it claims to.

## Bullets already on the resume — beat these or confirm them

These nine are currently on the resume, extracted from the changelog and docs. Verify each
one against the source (flag anything that is wrong or overstated), then tell me which of
your findings are stronger and should replace them.

1. 23-node ROS 2 Humble C++17 pipeline, 10.9k LOC, 19 message types, IWR6843ISK radar +
   AR0234 camera on Jetson Orin Nano Super, no off-board compute
2. 30 Hz → 13–17 Hz FPS drop root-caused to 4+2 core scheduler spill; GPU (31–68% GR3D)
   and thermal (57 °C vs 90 °C) ruled out; 20 Hz track contract unaffected
3. YOLOv8s INT8 TensorRT at 35 Hz, pinhole projection + IoU association, 20 Hz confirmed
   track output
4. IMM CV/CA/CT Kalman filter, Bayesian mode mixing, Hungarian assignment, Mahalanobis
   chi-square gating at 9.0/16.0
5. Cross-sensor timestamp gate widened 50 ms → 150 ms after kernel wake-up jitter aged out
   camera frames; 6-frame / 200 ms ring buffer
6. 144 JSF AV C++ / MISRA C++:2023 violations cleared across 20 files; math moved out of
   ROS nodes into 43 GoogleTest-covered pure classes; 32-track compile-time bound
7. Radar serial TLV decode, DBSCAN clustering with Doppler-weighted centroid at 16 Hz,
   learned static occupancy-grid clutter map
8. CoT 2.0 UDP multicast to ATAK at 1 Hz, 5-state escalation FSM, 5-class intent
   classifier, polygon geofence with time-to-intercept
9. Software radar source + 7 launch configurations for full SIL with no hardware;
   built-in-test node tracking per-node rates by EMA

## Output format

Produce a file `RESUME_BULLETS.md` containing:

**Section 1 — Ranked candidate bullets.** At least 20, ordered by strength. For each:

| Field | Content |
|---|---|
| Bullet | The text, under 2 lines at 10pt Arial on a Letter page, technical terms marked for bolding |
| Evidence | File path and line, commit SHA, or doc section |
| Defense | The two-sentence answer John gives when an interviewer says "walk me through that number" |
| Risk | What a sharp interviewer would attack, and how to answer |

**Section 2 — Corrections.** Anything in the nine bullets above that is wrong, overstated,
or unsupported by the source. Be blunt. A wrong number on a resume is a disqualifier.

**Section 3 — Weak spots.** What is genuinely thin or unfinished in this project, so John
knows where he is exposed before someone else finds it. Include the un-instrumented latency
stages, the deferred GPS/georef work, and anything else you find. For each, note whether it
is better to omit it or to name it honestly.

**Section 4 — Gaps worth closing.** Concrete work that would meaningfully strengthen the
resume, ranked by hours-to-payoff. Include the missing README — the repo currently has no
description, so a recruiter who clicks the link sees a bare file tree. Suggest what else
would move the needle most per hour spent.

## Tone

Write plainly. No filler, no salesmanship, no adjectives doing work that numbers should do.
If a bullet is weak, say it is weak. John needs an accurate picture of what he has, not
encouragement.
