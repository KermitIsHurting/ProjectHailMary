# ProjectHailMary — polish kick-off for the executing the executor session

Written 2026-09-01 23:34Z by John's VS Code session (reversegarage) from a read of the public repo
`https://github.com/KermitIsHurting/ProjectHailMary` (24 commits, `0.8.0-alpha`, no README). John is
applying to Anduril (primary) and Northrop Grumman; this repo — a counter-UAS radar + camera fusion
stack on ROS 2 Humble in C++ — is his most relevant project. Two goals, in this order:

1. **Make it safe to work on, then find and fix what is broken across the WHOLE project** — by
   `docs/engineering-playbook.md` (copy it from this folder into the repo's `docs/` first), in
   **§15 audit mode**: one ground-truth pass, one lens sweep over every subsystem, one triage
   message to John, batched fixes with gates and a checkpoint commit per batch, one bug-finder
   pass on the cumulative diff, one harness run on the final tree.
2. **Make it presentable and provable** — a README a hiring panel can read in two minutes, and a
   *facts sheet* where every résumé claim has its evidence (file, measurement, commit).

**Record cadence — John's decision (2026-09-01):** the full record (audit plan, HANDOFF, README /
CHANGELOG corrections, facts sheet, retro) is written **ONCE, at the end (A7)** — not after every
change. During the audit the only running record is `docs/audit-log.md`, append-only, one line per
event written with `date -u` in the same command, plus checkpoint commits per batch. Write this
trim into `docs/audit-plan.md` §0 as the tier call with the reason ("owner: per-change records
waste tokens and time"). Tier-A findings still interrupt John immediately, in one line.

John decides; you build; you stop at the commit boundary with exact paths and commit lines.

## 0. BACKUP FIRST — nothing else happens before this is done and verified (§8 rule 13)

Run these before the first edit, in the repo root, and paste the outputs into `docs/audit-plan.md` §0:

1. `git status --short -uall` — record it. If the tree is dirty, commit or stash with a message
   naming what it is; do not carry unknown local changes into the polish.
2. `git tag pre-polish-2026-09-01 && git push origin pre-polish-2026-09-01` — the point of no argument.
3. `git switch -c polish` — all work on this branch; `main` stays as it was until John merges.
4. Archive the WHOLE tree **including gitignored state** to a path OUTSIDE the repo, dated:
   `tar --exclude=build --exclude=install --exclude=log -czf ~/backups/ProjectHailMary-pre-polish-$(date -u +%Y%m%dT%H%MZ).tgz -C <parent dir> ProjectHailMary`
   then `sha256sum` it and `tar -tzf` it, and confirm these are INSIDE the archive by name:
   `calibration_frames/`, `models/` (the TensorRT engines / YOLO weights), `logs/` (recorded runs),
   `scripts/99-iwr6843.rules`, any `.env`, any radar config files, any rosbags. If a directory is on a
   different disk (Jetson external storage), archive it separately and say so.
5. **Restore rehearsal:** extract the archive into a scratch folder and run `colcon build` there. Only
   when that build passes is the backup real. Record the archive path, its sha256 and the rehearsal
   result in the plan §0. If the machine is a Jetson with limited disk, the rehearsal may be
   `--packages-select cuas_msgs cuas_fusion`; say which.
6. Record the environment as ground truth: `uname -a`, ROS distro, `nvidia-smi` / JetPack version,
   TensorRT version, CMake, compiler, `pip freeze | head`, the radar's serial device path, and whether
   the radar and camera are attached right now. Everything later is judged against this list.

## 1. Read, in this order

- `docs/engineering-playbook.md` §0, §2 + §2b (tiers for a non-web project), §6 (gates — write this
  repo's four commands into it), §7 (simulation / replay harness), §8, §11, §14.
- The repo's own `docs/`: `CODING_STANDARD.md`, `ICD.md`, `architecture_diagram.md`,
  `latency_budget.md`, plus `CHANGELOG.md` and `VERSION`. These are claims; the polish proves or
  corrects them.
- `install.sh`, `install_prerequisites.sh`, `setup_env.sh`, `scripts/*` — the onboarding path a
  stranger would take.

## 2. Ground truth (phase 1) — the questions the polish must answer with commands

- Does it **build clean from the archive** on this machine? (`colcon build`, warnings counted.)
- Do the **tests pass**, and how many are there? (`colcon test` + `colcon test-result --verbose`;
  the changelog claims math-class unit tests are integrated — count them.)
- Does the **coding-standard claim hold**? The changelog says "JSF AV C++ / MISRA C++:2023 full
  compliance pass, 144 violations resolved across 20 files". Find the checker that produced that
  number (clang-tidy profile? a script?). Run it. Record the current violation count. If no checker
  exists in the repo, the claim is unprovable and the README must say "guided by" not "compliant" —
  Needs John's decision.
- Does the **simulation path run without hardware**? (`simulation radar node` from 0.5.0) — launch it,
  confirm topics publish, RViz2 shows tracks. This is the demo path for anyone without an IWR6843.
- Do the **latency budget numbers** in `docs/latency_budget.md` come from a measurement? Find or
  build the readout (`scripts/jitter_log.py` looks like it) and re-measure on this machine; record
  numbers with the date and the conditions (sim vs live radar, camera on/off, TensorRT INT8 engine).
- What is the **35 Hz YOLOv8s INT8 TensorRT** claim measured with? Reproduce or mark unverified.
- Which nodes exist, which launch files start them, which are dead code? (grep `add_executable`,
  `launch/*.py`, `package.xml` deps.)
- What does `CHANGELOG.md` promise that the code does not do? Every such gap is either fixed, or the
  changelog line is rewritten — never left as an unbacked claim on a résumé project.

Everything above goes into `docs/audit-plan.md` §1 as numbered verified facts, each saying HOW.

## 3. Tiers for this repo (from playbook §2b) — John's issues sort into these

| Tier | Here | Review set |
|---|---|---|
| A | The build is broken on the target; a node crashes on real sensor input; a geofence / threat / reachability decision is wrong at runtime | Smallest diff → gates → role 6 within 24 h |
| B | Fusion math, IMM / Hungarian / Mahalanobis changes, the `cuas_msgs` contracts (the ICD), threat + intent classifiers, geofence + reachability logic, the latency budget, calibration handling, CoT/ATAK output format | Full lifecycle; roles 3, 4, 9, 10 on the plan (role 5 only for the network-facing pieces: CoT UDP multicast, config loading, serial parsing of untrusted TLV bytes); role 6 on the diff |
| C | Launch files, RViz config, overlays, health monitor thresholds, scripts, install/setup scripts | Short plan, ONE combined reviewer, harness = sim launch |
| D | README, docs, comments, formatting-only | Gates only |

The MISRA/JSF pass is tier **D only if** it is formatting/naming; any rule that changes control flow,
integer widths, or floating-point comparisons in the math classes is tier **B** (a "compliance" edit
that flips a `<=` is a fusion bug).

## 3b. The roles, in this repo's terms (paste these mission lines into the §10 prompt wrapper)

The lifecycle and the numbering are the playbook's; only the vocabulary changes. "Money path" here
is the **decision path**: sensor bytes → detections → tracks → threat / intent / geofence /
reachability → operator + CoT output. A wrong decision is this project's "wrong charge".

**Main session**

- **Role 1 — Planner.** Own ground truth → plan → triage. "Needs John's decision" covers: any change to a
  threshold that moves a threat / intent / geofence / reachability decision; any `cuas_msgs` field
  (the ICD); the latency budget; safety behaviour on sensor loss; and every README claim that could
  not be proven by a command. State what was NOT verified as loudly as what was.
- **Role 2 — Embedded engineer (the builder).** Implement the reviewed plan exactly, with: one producer
  per constant that decides behaviour (gating distance, horizon bounds 3–10 s, threat-level enums,
  DBSCAN eps/minPts, clutter thresholds) guarded by a test that scans for copies; typed fields over
  string compares (0.7.0 already did this — keep it); units converted once, at the boundary, named in
  the type or the variable; fail-closed: a missing parameter, a corrupt model or calibration file, or a
  malformed frame → last-known-good behaviour + one log line, never a node that keeps publishing
  garbage; bounded work in every callback on the hot path (the TLV parser, association) — no
  unbounded loops, no allocation surprises, no exceptions escaping a callback; no inline default that
  feeds a parameter callback or a timer; comments state the constraint the code cannot show (why
  this eps, why this QoS).
- **Role 7 — Empirical prover.** Drive the real pipeline with the simulation radar node and recorded
  bags, one shape at a time, with a readout that prints the numbers: single target approaching;
  two targets crossing (does association swap?); clutter only (false tracks per minute); target
  lost and reacquired (track id continuity); geofence entry / exit; target at max range; camera
  detection with no radar track and the reverse; a malformed TLV frame mid-stream; serial
  disconnect; timestamps that jump. Record shape × observed × expected; delete the harness launch
  files after.
- **Role 8 — Operator lens.** Read the run procedure as a stranger with a Jetson and the box of parts:
  is the udev rule installed, was the radar config sent, is the camera calibrated, which launch
  file, what "working" looks like in RViz2 and on the readout, how to stop safely, and rollback =
  `git checkout pre-polish-2026-09-01 && colcon build`. Re-read after the last code change.

**Plan-review agents (parallel, read-only)**

- **Role 3 — Failure-forcing reviewer.** For each work item, name the frame, state or timing that makes
  it wrong: an empty point cloud, NaN range or Doppler, a timestamp going backwards, two detections
  in one range bin, more tracks than detections (and the reverse) in the Hungarian matrix, a horizon
  shrinking mid-prediction, a self-intersecting geofence polygon, a CoT packet over the MTU, a
  serial frame cut in half, a model file that loads but has the wrong input shape. Attack the plan's
  own "failure modes" section hardest.
- **Role 4 — Cross-file tracer / removed-guarantees auditor.** Grep every consumer of every touched
  topic, message field, parameter name, enum value and frame_id; inventory table file:line →
  affected → why. List every guarantee the old code made — units, frame conventions, publish rates,
  QoS profiles, thread-safety of callbacks, ownership of the prediction horizon (0.6.0 moved it to
  the IMM tracker) — and check each with the change in and out.
- **Role 5 — Adversarial-input and safety reviewer** (the black-hat, rescoped). Attack only the rows the
  change touches: **S1** untrusted bytes — the serial TLV parser's bounds, lengths and checksums; **S2**
  the decision path — can any single input flip a threat / intent / geofence decision without
  corroboration, and can a friendly ever render as a threat?; **S3** time — replayed or skewed
  timestamps, clock jumps; **S4** state machines — threat classifier and track lifecycle: every state
  has an exit, no stuck HIGH; **S5** network output — the CoT/ATAK multicast group: who can join,
  message rate, malformed inbound if anything listens; **S6** file inputs — model, calibration and
  config loading: path handling, corrupt or truncated files fail closed; **S7** resource exhaustion —
  track table growth, clutter-map memory, log growth over a long run; **S8** parameters — ranges
  validated at declaration, a bad value cannot start the node in a dangerous mode. Write "checked,
  already safe" explicitly; end with "Top N to bake in first".
- **Role 9 — Altitude / architecture.** Is the shape right and what gets expensive later: fusion at
  track level vs detection level; IMM model set; node boundaries and what crosses them; the ICD
  once a downstream consumer exists; judged against where John wants the project to go (a demo, a
  paper, a portfolio piece, a base for the next sensor).
- **Role 10 — Systems integration ("connect the lines").** Map the ROS graph and prove every line
  connects: every topic has a publisher AND a subscriber (no orphans), QoS compatible on both
  ends, the tf tree consistent, parameters declared where they are read, launch order and
  lifecycle, the latency budget summed across nodes against the measured numbers, the CoT output
  actually parsing on an ATAK client. Close with "connected and fine (verified)" and "non-issues checked".
- **Role 13 — ICD / message-contract reviewer** (replaces the DB lens; only when `cuas_msgs` changes).
  For every field added, removed, renamed, re-typed or re-defaulted: every publisher and subscriber
  updated; recorded bags still replay or the break is recorded; `VERSION` bumped; `docs/ICD.md`
  updated in the same commit; proven by building both packages clean and replaying one old bag.

**Post-build agents**

- **Role 6 — Bug finder.** Line-by-line on `git diff` + every new file, C++ eyes: signed/unsigned and
  overflow in range and bin arithmetic, off-by-one in matrix sizes, uninitialised members, float
  equality, unit mixing (m vs cm, deg vs rad, ms vs s), thread safety of shared state touched from
  callbacks, exceptions on the hot path, memory growth over time, a "compliance" edit that flipped
  a comparison. Check the plan was actually implemented; audit the tests (which test fails if this
  breaks?); walk the run procedure as John will type it; verdict SHIP / FIX-FIRST.
- **Role 11 — Operator-view reviewer** (replaces rendered-UI). Look at what the operator sees — RViz2
  markers and overlays, the health monitor's output, the CoT icons on ATAK — as the operator, not the
  author: labels, units, colours and threat levels unambiguous; a friendly never looks like a threat;
  nothing readable only by the person who wrote it.
- **Role 12 — Cold reader of the record.** Read README, CHANGELOG, `docs/latency_budget.md`,
  `docs/resume-facts.md` and the plan as a hiring panel would, and check every claim against a
  command: build output, test count, checker violation count, measured rates with their conditions.
  Corrected-lines list; an unprovable claim is reworded, never left.
- **Built-in passes.** The coding-standard checker (the one that produced "144 violations", once
  found), `clang-tidy`, and the tooling's `/code-review high`; `/security-review` for the parser and
  network pieces.

**Which roles per tier here:** A = role 2 → gates → role 6 within 24 h. B (the decision path, the
ICD, the math, the latency budget) = all of them; role 5 only on the rows the change touches. C
(launch, RViz, scripts, health thresholds) = roles 1, 2, 7 + ONE combined agent playing 3+4+10 +
role 11. D = gates. The prompt wrapper is playbook §10; put the repo path, the files in scope and
the READ-ONLY rule around each mission line above.

## 4. How the audit runs here (playbook §15, applied)

- **A0** = §0 above. **A1** = §2 above, written straight into `docs/audit-plan.md` §1 as tables.
- **A2** subsystem map for this repo (fill file lists from the tree): radar serial driver + TLV parser ·
  DBSCAN clustering · IMM tracker + motion models · Hungarian association + Mahalanobis gating ·
  camera–radar fusion + YOLOv8s TensorRT inference · threat classifier + intent classifier ·
  geofence + reachability engine · health monitor · clutter map · RViz2 visualiser + overlays +
  stream arbitration · CoT/ATAK UDP output · `cuas_msgs` (the ICD) · launch + config · install /
  setup scripts + udev · docs and claims.
- **A3** one message launches roles 3, 4, 5, 10, 6 (whole-codebase), 12 (claims) with the §3b mission
  lines, each scoped to its subsystems' files, each with a token budget written in §0 first. Nothing
  is fixed during A3 except tier A.
- **A4** one triage message to John: the ranked findings table + the decisions that are his
  (thresholds, claim wording, anything Needs hardware check). **John's own known issues are pasted
  into this triage as findings** — each reproduced first (command, observed, expected); one he
  reports that you cannot reproduce is "Needs John's decision", not a fix.
- **A5** batches in dependency order: parser → clustering → tracker → association → fusion →
  classifiers → geofence/reachability → health/viz/network → tooling → docs. Gates + checkpoint commit
  + one log line per batch. ONE role-6 pass on `git diff pre-polish-2026-09-01..HEAD -- src/ msgs/`
  after the last tier-B batch.
- **A6** the §3b role-7 shape list once, on the final tree, via the simulation radar node + any bags in
  `logs/`; the production-defaults pass; hardware shapes = John's smoke with the readout open.
- **A7** the record, once — §5 and §7 below.

## 5. Presentability deliverables (tier D unless they touch code)

1. **`README.md`** — the two-minute read a panel engineer does before an interview:
   what it is (one paragraph, counter-UAS radar + camera fusion, ROS 2 Humble, C++), a system
   diagram (promote `docs/architecture_diagram.md`), the hardware (TI IWR6843 mmWave radar, camera,
   Jetson — whatever `install_prerequisites.sh` and the udev rule prove), the pipeline in order
   (serial TLV → DBSCAN → IMM tracker → Hungarian association + Mahalanobis gating → camera fusion
   with YOLOv8s INT8 → threat + intent classifiers → geofence / reachability → RViz2 + CoT/ATAK),
   measured numbers WITH conditions and date, how to run in **simulation** in five commands, how to
   run on hardware, test count, coding standard status (proven wording only), what is alpha / not
   done, and a 30–60 s demo GIF or video link from the sim path.
2. **`docs/resume-facts.md`** — one row per claim John might put on a résumé or say in an interview:
   claim → evidence (file:line, command + output, commit id) → confidence (measured / implemented,
   unmeasured / planned). Anduril's presentation round attacks unowned or hand-waved claims; this
   sheet is what John rehearses from. Include the decisions HE made and why (IMM over a single KF;
   Hungarian over greedy nearest-neighbour; DBSCAN parameters; the adaptive 3–10 s horizon; INT8
   over FP16; UDP multicast CoT) — each with the trade-off in one line.
3. A **`docs/polish-retro.md`** at the end (playbook §9.7): what was broken, what was fixed, what
   was reworded because it could not be proven.

## 6. Hard rules for this repo (add to playbook §8 as you confirm them)

- Never run a radar-config or firmware script (`send_radar_config.sh`) against attached hardware as
  part of a "test" without John present — some radar configs persist.
- Model and calibration files are never regenerated as a side effect; a script that writes into
  `models/` or `calibration_frames/` is tier B and runs only on John's word.
- The `cuas_msgs` package is an interface contract: a field change is an ICD change (tier B) and
  bumps `VERSION`.
- Every measured number in the README carries its conditions and date; unmeasured claims are
  worded as such.

## 7. Hand-back to John (A7 — the one record, then the commit boundary)

Written once, in this order, from the audit log and the checkpoint commits: `docs/audit-plan.md`
completed (§0 tier call + trim, §1 facts, the findings table with outcomes, the decision log with
"Needs John's decision" for every wording that could not be proven and every behaviour change in
the math, harness results, timings); README and CHANGELOG corrected so every claim is verified,
reworded or removed; `docs/resume-facts.md`; `HANDOFF.md` PICK UP HERE; ledger rows for any new
mistake; the retro section. Then role 12 reads the whole record once. Then: the path list from
`git status --short -uall`, the checkpoint commits already on `polish` listed by hash, any final
commit lines, and the literal `git` commands. John merges to `main` himself after reading the facts
sheet. The audit log's last line is `A7 record complete`.
