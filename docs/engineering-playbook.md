# Engineering Playbook — building on a sensor-to-decision system (ROS 2 / embedded edition, v1.0)

A method for a small team — an owner who decides, an engineer or AI executor who builds, and a set
of review agents — to change a system that turns sensor data into decisions (radar → detections →
tracks → threat / intent / geofence → operator and network output) without breaking the decision
path, the hardware, the interface contract, or the record. Derived from a playbook that ran nine
features on a live e-commerce product in three days; every web term has been translated, not just
renamed. Copy into `docs/engineering-playbook.md`, fill the `<placeholders>` (§6 gates, §8 rules),
keep a friction log per feature, and bump the version when the log changes this file.

**Thesis.** "It builds and the tests pass" is the start of a health check, never the end. On a
system like this a bug is not a red test — it is a track that swaps identity when two targets
cross, a friendly rendered as a threat, a node that keeps publishing after the sensor went silent,
or a README number nobody can reproduce in the interview.

---

## 0. How to use this file

1. Put it in `docs/engineering-playbook.md`. Fill §6 (this repo's gate commands) and §8 (this
   repo's hard rules) with commands you have actually run.
2. **First session on the project: back it up before touching it (§8 rule 1).** Tag, branch,
   archive outside the tree *including gitignored state* (calibration data, models, recorded logs,
   udev rules, device configs), checksum, and a restore rehearsal that builds.
3. Before any change: pick the **tier** (§2). The tier decides everything else.
4. Run the **lifecycle** (§3) with the **roles** (§4). One plan doc per feature (§9).
   **For "go through the whole project and find what is wrong" — use §15 audit mode instead:**
   one sweep, batched fixes, the record written once at the end.
5. Before phase 1 and before the first patch, read **§11 (traps)** and **§14 (the ledger)**. Add a
   ledger row the day a mistake happens, with its cost.
6. Keep a **friction log** in the plan: every line of this file you had to guess at or deviate
   from — quote it, say what you did instead. Fold it back in at the end.

---

## 1. Principles (each one is a rule further down)

| # | Principle | The failure it prevents |
|---|---|---|
| 1 | **Evidence, not belief.** A finding needs `file:line` and a frame, bag or parameter set that reproduces it; a claim about ROS, a driver, a library or the hardware needs the installed docs, the header, or a probe on the device. | Confident wrong answers about how a QoS profile, a filter or a sensor "probably" behaves |
| 2 | **Ground truth before opinion.** Build from clean, list the running graph, read the actual device state and installed versions first. A 30-second `ros2 topic hz` settles what an hour of argument cannot. | Designing against an imagined pipeline |
| 3 | **Independent redundancy.** Several reviewers with different lenses and no shared state; accept what two find or one proves. | The blind spot everyone shares |
| 4 | **The diff is where the bugs are.** Plan review saturates fast; keep budget for the line-by-line hunt on real code. | Triple-reviewed plans that ship an off-by-one in the association matrix |
| 5 | **One producer per behaviour-deciding constant.** Every threshold, bound, enum or unit factor that changes a decision has exactly one home, guarded by a test that scans for copies. | A gating distance that drifts between two files; a threat level compared as a string in one place and an enum in another |
| 6 | **Fail closed, degrade to last-known-good.** A missing parameter, a corrupt model or calibration file, a malformed frame, a lost sensor → last-known-good behaviour plus one log line — never a node that keeps publishing plausible garbage. | Downstream consumers acting on data the source never validated |
| 7 | **The operator's screen and the readout are the truth.** What RViz, the health monitor and the network consumer show — with numbers printed by a readout — not what the source says. | A friendly rendered as a threat; a latency budget that only exists in a document |
| 8 | **Records carry intent.** A parameter flipped, a config sent to the sensor, a model swapped — recorded with why, for how long and who reverts it. | The next session treating a deliberate state as a fault |
| 9 | **Decisions belong to the owner** — thresholds that move a decision, interface-contract changes, safety behaviour, and every claim that will be printed on a résumé — logged as "Needs <owner>'s decision", never assumed. | Engineers quietly changing what the system calls a threat |
| 10 | **The executor stops at the commit boundary.** Gates green, exact paths, one-line messages, the git commands handed over; the owner runs them, merges and flashes. | Surprise merges; an unreviewable tree on the target |

---

## 2. Sizing — pick the tier first

| Tier | When (this domain) | What runs | Ship |
|---|---|---|---|
| **A — Incident** | The build is broken on the target; a node crashes or stalls on real sensor input; a safety / geofence / threat decision is wrong at runtime | Smallest possible diff → gates → rebuild on the target → verify with the readout → one-line handoff. Within 24 h: the bug finder (role 6) on that diff, verdict recorded | Immediately |
| **B — Decision path / contract / timing** | Anything on the path from sensor bytes to decision or output: parsers, clustering, filters and motion models, association and gating, fusion, threat / intent / reachability / geofence logic, calibration handling, the message contract (ICD), the latency budget, network output format, safety behaviour on sensor loss | Full lifecycle, all roles (role 5 scoped to the rows touched) | Behind a parameter (default = old behaviour) if the owner wants a soak first; otherwise on the feature branch until the owner merges |
| **C — Tooling and operator surface** | Launch files, RViz configs and overlays, health thresholds, logging, scripts, install/setup scripts, visualisation | Phases 1, 2 (short plan), 5, 7 with ONE combined reviewer (roles 3+4+10 in one prompt) + role 11 on the operator view | On merge |
| **C-fix — same-day correction of a batch reviewed today** (no new decision-path change) | Fixes scope, a script, a launch parameter, an overlay | Build → gates → harness on the changed behaviour → ONE post-build review (role 6) → role 12 only if the run procedure changed shape. Earlier findings carry over as the acceptance list | On merge |
| **D — Docs / comments / formatting** | README, ICD text, comments, formatting-only | Gates only. **Exception:** a "compliance" or "style" edit that changes control flow, integer widths, or a floating-point comparison inside the math is tier B — a flipped `<=` in a gate is a fusion bug wearing a linter's clothes | On merge |

**Batches:** an ask that bundles several changes runs at the highest tier of its parts; each part
gets its own one-line commit; grouping decided at plan time from file overlap.

**Trimming is a plan decision, never an executor shortcut.** A run that drops a role the tier calls
for writes the trim into the plan's §0 next to the tier call, with the reason. Every lifecycle pass
ends with a line in the plan: `RUN (time, verdict)` or `SKIPPED (reason) — Needs <owner>'s
decision`. A record with an empty pass line is not "gates green".

**A scope question is answered with an EXAMPLE, never with options.** "Tracks jump", "the
classifier is too eager", "the overlay is cluttered" — ask for ONE concrete instance in the owner's
words: a bag and a timestamp, a screenshot, the parameter set they were running. Offering your own
readings lets the owner pick the nearest one and ships the wrong fix.

---

## 3. The lifecycle (tier B; C and D use the subset above)

| # | Phase | Do | Produce |
|---|---|---|---|
| 1 | **Ground truth** | Build from a clean tree (`rm -rf build install log`, warnings counted). List the graph: nodes, topics, publishers/subscribers, QoS, tf frames, parameters and their declared ranges. Record the device state: which sensors are attached, driver versions, the config last sent to the sensor, JetPack / CUDA / TensorRT / ROS versions. Verify every claim in the repo's docs and changelog with a command — rates with `ros2 topic hz`, latency with the readout, test counts with the runner, the coding-standard count with the checker that produced it. A claim with no command is "unverified" in the plan, in those words. Stamp the clock. | Numbered **verified facts**, each saying *how* |
| 2 | **Plan v1** | §0: the ask verbatim + the owner's one concrete example. Decision log (D-1..n: Fixed / Needs <owner>'s decision / Open, with numbers). Work items (W-1..n: exact files, build order). Failure modes considered. **Run procedure + rollback written now** — every command executed before it is written down. Commit grouping. Timings table. Friction notes | `docs/<feature>-plan.md` |
| 2b | **Pure module first** | Build the pure part — the math class, the parser, the gate, the state machine — and its unit tests (gtest / pytest), wired into no node. Then **freeze** the tree. | Real code for the reviewers |
| 3 | **Adversarial plan review** | Roles 3, 4, 5 (scoped), 9, 10 (+13 when the message contract changes) as **parallel read-only agents launched in one message**, given the plan AND the pure module. Dedupe by root cause. Apply the acceptance table (§5). Record the losing argument of every disagreement. **An unresolved disagreement on the decision path blocks merge.** Contract findings are PROVEN: build both packages and replay an old bag. | Review-outcome table |
| 4 | **Plan v2** | Fold findings in; **re-sync** facts, decisions and the run procedure to the code that now exists | Updated plan |
| 5 | **Build + gates** | Role 2 disciplines (§4). Clean build before the gates when CMake caches or symlink installs could lie. | Green tree |
| 6 | **Empirical proof** | The harness (§7) drives **every behavioural shape** through the real pipeline — simulated source or bag replay — with a readout printing the numbers. One pass with **production defaults** (no injected fake, no sim-only parameter). A shape that cannot be driven without hardware is recorded as such and becomes the owner's hardware smoke, watched. Role 11 on the operator view. Role 6 may run in parallel. | Shape × observed × expected table |
| 7 | **Bug hunt on the diff** | **First rewrite the handoff / run procedure for THIS tree** — role 6 and role 12 walk it as the owner will type it. Then role 6 on `git diff` + every new file → SHIP / FIX-FIRST (fix → gates → role 6 again). A reviewer's fix DIRECTION is a hypothesis: measure it in the still-open harness before taking it. Then the built-in passes (the coding-standard checker, `clang-tidy`, the tooling's `/code-review high`; `/security-review` for parsers and network code). Then role 12 reads the record. | Verdict; corrected-lines list |
| 8 | **Ship** | Executor hands over: exact path list, one commit line per part, the literal `git` commands. Owner commits, merges, rebuilds on the target. | Commits |
| 9 | **Record + verify on target** | Handoff entry (§9.4), notes, the post-merge checks (§8 rule 10), the first run on real sensors watched with the readout open | A record the next session can trust |

---

## 4. Roles — mission lines (paste verbatim into the §10 prompt wrapper)

"Money path" in the parent playbook is the **decision path** here: sensor bytes → detections →
tracks → threat / intent / geofence / reachability → operator and network output.

### Main session

**Role 1 — Planner.** Own phases 1–4 and the acceptance table. "Needs <owner>'s decision" covers:
any threshold that moves a threat / intent / geofence / reachability decision; any message-contract
field; the latency budget; safety behaviour on sensor loss or corrupt input; and every README or
changelog claim that could not be proven by a command. State what was NOT verified as loudly as
what was.

**Role 2 — Embedded engineer (the builder).** Implement the reviewed plan exactly, with:
- one producer per behaviour-deciding constant (gating distance, horizon bounds, threat-level
  enums, clustering parameters, clutter thresholds, unit factors), guarded by a source-scan test
  that matches code tokens, never prose;
- typed fields and enums over string compares; units converted once at the boundary and named in
  the type or the variable (`range_m`, `yaw_rad`, `dt_s`);
- fail-closed: a missing parameter, a corrupt model or calibration file, a malformed frame, a lost
  sensor → last-known-good behaviour + one log line; never keep publishing garbage;
- bounded work in every callback on the hot path (parsers, association, prediction): no unbounded
  loops, no allocation surprises, no exception escaping a callback, no blocking I/O in an executor
  thread;
- parameters declared with ranges and validated at declaration; a bad value cannot start the node
  in a dangerous mode;
- **no inline default for anything that feeds a re-execution key** — a lambda or object literal
  passed as a default to a parameter callback, a timer, or a reactive binding is a new identity
  every call; hoist it to a named constant or member;
- an injectable dependency for the harness (a simulated source, a fake clock) means the
  PRODUCTION default is the path the harness never exercises — cover it with the
  production-defaults pass (§7);
- comments state the constraint the code cannot show (why this eps, why this QoS, why this horizon).

**Patch-tooling disciplines (any AI executor).** Read the exact anchor lines in the previous command
before patching them; patches are script FILES from a quoted heredoc ≤ 4 KB, never inline
one-liners with backslashes, backticks or `$`; one heredoc per command; a generated regex is tested
against the file before its "0 hits" is believed; every timestamp and count in a record comes from
a command in the same pass (`date -u`, `wc -l`, the test runner's summary).

**Role 7 — Empirical prover.** Drive the real pipeline with the simulated source node and recorded
bags, one shape at a time, with a readout that prints the numbers. Shapes for a tracking / fusion
system: single target approaching; two targets crossing (does association swap ids?); clutter only
(false tracks per minute); target lost and reacquired (id continuity); geofence entry and exit;
target at maximum range; camera detection with no radar track and the reverse; a malformed frame
mid-stream; sensor disconnect; timestamps that jump or repeat; a long soak (memory and track-table
growth over 30+ minutes). Record shape × observed × expected; delete the harness launch files after.

**Role 8 — Operator lens.** Read the run procedure as a stranger with the target board and the box
of parts: is the udev rule installed, was the sensor config sent, is the camera calibrated, which
launch file, what "working" looks like in RViz and on the readout, how to stop safely, and the
rollback (`git checkout <tag> && colcon build`). Re-read after the last code change.

### Plan-review agents (parallel, read-only)

**Role 3 — Failure-forcing reviewer (plan AND diff).** For each work item or hunk, name the frame,
state or timing that makes it wrong: an empty point cloud; NaN or infinite range / Doppler; a
timestamp going backwards; two detections in one range bin; more tracks than detections (and the
reverse) in the assignment matrix; a prediction horizon shrinking mid-prediction; a self-intersecting
geofence polygon; a network packet over the MTU; a serial frame cut in half; a model file that loads
but has the wrong input shape; a parameter at the edge of its declared range. Attack the plan's own
"failure modes" section hardest.

**Role 4 — Cross-file tracer / removed-guarantees auditor.** Grep every consumer of every touched
topic, message field, parameter name, enum value and frame id; deliver an inventory table (file:line
→ affected y/n → why). List every guarantee the old code made — units, frame conventions, publish
rates, QoS profiles, thread-safety of callbacks, which node owns a piece of state — and check each
with the change in and out.

**Role 5 — Adversarial-input and safety reviewer (scoped).** Attack ONLY the rows the change
touches, starting from ground truth (where bytes enter, where a decision is made, what listens on
the network):

| Row | Change touches… | Attacks |
|---|---|---|
| S1 | Parsers of untrusted bytes (serial, UDP, files) | length and bounds checks, checksums, truncated and oversized frames, integer overflow in length fields, a frame that parses but is semantically impossible |
| S2 | The decision path (threat / intent / geofence / reachability) | can ONE input flip a decision without corroboration; can a friendly ever render as a threat; hysteresis and debounce; what the decision does when its inputs are stale |
| S3 | Time | replayed, skewed or repeated timestamps; clock jumps; `use_sim_time` mismatches; wall clock vs sensor clock |
| S4 | State machines (threat classifier, track lifecycle) | every state has an exit; no stuck-HIGH; transitions only through the dedicated path; every consumer of the state re-checked |
| S5 | Network output / input (multicast, TCP, serial out) | who can join or send; message rate; malformed inbound if anything listens; what a consumer does with a half-written message |
| S6 | File inputs (models, calibration, configs) | path handling, corrupt or truncated files fail closed, version mismatch detected, never regenerated as a side effect |
| S7 | Resource exhaustion | track-table growth, clutter-map memory, log growth, queue depth under a slow consumer over a long run |
| S8 | Parameters and launch arguments | ranges validated at declaration; a bad value cannot start the node in a dangerous mode; who can change it at runtime |
| — | Pure tooling / visualisation / docs | skip this role; role 11 covers the operator view |

Write "checked, already safe" explicitly; every finding = plan section → attack → evidence file:line
→ mitigation; end with "Top N to bake in first".

**Role 9 — Altitude / architecture.** Is the whole approach right, and what gets expensive later?
Fusion at track level vs detection level; the motion-model set; node boundaries and what crosses
them; the message contract once anything downstream depends on it. Compare against at least one
alternative the plan did not argue; judge "expensive later" against where the owner wants the
project to go (a demo, a paper, a portfolio piece, a base for the next sensor).

**Role 10 — Systems integration ("connect the lines").** Map the running graph and prove every
line connects: (0) a mechanism table — mechanism → where (file:line) → the fact that matters — for
every subsystem touched (drivers, parsers, filters, association, fusion, classifiers, geofence,
health, visualisation, network output); (A) every topic has a publisher AND a subscriber (no orphans,
no dead ends), QoS compatible on both ends, tf tree consistent; (B) state machines: every state has
an exit and every consumer is listed; (C) installed versions support what the plan assumes; (D)
collisions: what ELSE changes when this changes; (E) sequencing: parameters, configs sent to the
sensor, model files and the rebuild in an order that can never leave the system half-connected;
(F) the latency budget summed across nodes against the measured numbers; (G) the network output
actually parses on the real consumer. Close with "connected and fine (verified)" and "non-issues checked".

**Role 13 — Interface-contract (ICD) reviewer (only when a message package changes).** For every
field added, removed, renamed, re-typed or re-defaulted: every publisher and subscriber updated;
recorded bags still replay or the break is recorded and dated; the version file bumped; the ICD
document updated in the same commit; proven by building both packages clean and replaying one old
bag through the changed consumer.

### Post-build agents

**Role 6 — Bug finder (line-by-line, on the diff).** Read `git diff` and every new file in full,
with C++ / real-time eyes: signed/unsigned and overflow in range and bin arithmetic; off-by-one in
matrix and buffer sizes; uninitialised members; floating-point equality; unit mixing (m vs cm, deg
vs rad, ms vs s); thread safety of shared state touched from callbacks; exceptions on the hot path;
memory growth over time; a "compliance" edit that flipped a comparison; an inline default feeding a
re-execution key. Check the plan was actually implemented, not what the builder believes; audit the
tests (for each promised behaviour, which test fails if it breaks?); walk the run procedure as the
owner will type it; list the candidates you killed; verdict SHIP or FIX-FIRST with ranked findings.

**Role 11 — Operator-view reviewer.** Look at what the operator sees — RViz markers and overlays,
the health monitor output, the icons on the network consumer — as the operator, not the author:
labels, units, colours and levels unambiguous; a friendly never looks like a threat; a stale track
looks stale; nothing readable only by the person who wrote it. Each finding = view → what it shows →
what it should show → file:line. Cheapest lens; runs on any visible change.

**Role 12 — Cold reader of the record.** Read the README, changelog, latency document, facts sheet,
the plan's run procedure and decision log as a hiring panel or the next engineer would — with no
context — and check every claim against a command: build output, test count, checker violation
count, measured rates with their conditions and date, `git log`, `git status`. Flag every
contradiction, stale number, command that no longer works as written, decision recorded as settled
that the owner has not answered, and recorded state with no intent. An unprovable claim is reworded,
never left. Deliver a corrected-lines list. Runs last.

**Built-in passes.** After role 6: the coding-standard checker the project claims compliance with
(the violation count is the number), `clang-tidy`, the tooling's `/code-review high`; `/security-review`
for parsers and network code. Fold findings through the acceptance table.

---

## 5. Evidence contract, acceptance table, impact ranking

**Evidence contract.** A finding is only a finding with exact `file:line` AND a concrete failure
scenario (frame / state / parameters → wrong outcome), after the finder tried to kill it.

| Finding state | Rule |
|---|---|
| Accept → work item | ≥2 independent finders agree, OR the orchestrator confirms it (reads the path end-to-end, reproduces it with a bag or a unit test, probes the device read-only) |
| Needs probe | One finder, plausible, not yet demonstrable → probe before finalising; if unprobeable, record as "unverified risk" with the mitigation cost |
| **Needs hardware check** | Anything about real-sensor timing, serial behaviour, thermal or GPU throughput, or the physical setup is NOT accepted on reasoning: driven on the device first, or built behind the owner's hardware smoke |
| **Hypothesis → measure** | A reviewer's *fix direction* is measured in the harness or a test before it is taken; the outcome row names what was measured |
| Discard | Nobody can produce a concrete scenario after trying — record the losing argument |

**Impact ranking:** an unsafe or wrong decision on real input > a node crash / build broken on the
target > wrong data on the bus (units, contract) > wrong operator display > latency budget missed >
polish.

---

## 6. Gates — literal commands, all of them, in order, zero failures

```
# pre-step: when the target, toolchain or CMake options changed, build from clean
rm -rf build install log
colcon build --symlink-install                      # 0 new warnings (record the baseline count)
colcon test && colcon test-result --verbose         # 0 failed (record the count)
<coding-standard checker command>                   # e.g. clang-tidy with the MISRA/JSF profile; record the violation count
<static analysis / cppcheck if used>
<simulation smoke>                                  # a sim/replay launch that prints its readout and exits 0
```

Gates are entry conditions for review and merge — never proof of correctness. If there is no CI,
say so here and name who runs the gates before committing. The checker that produced any
"N violations resolved" claim must be named here; if it cannot be found, the claim is reworded.

---

## 7. Harnesses — observe, never infer (delete after use)

**Simulated-source harness** (`launch/_harness_<feature>.launch.py`, gitignored): the simulated
sensor node feeding the REAL pipeline, one scripted scene per behavioural shape (§4 role 7 list),
plus a **readout node** that prints the numbers the plan promised — track count, id continuity,
association swaps, false tracks per minute, end-to-end latency percentiles, decision transitions —
to the console or a topic. The readout is the record; a screenshot is not.

**Bag-replay harness:** `ros2 bag play <bag> --clock` with `use_sim_time` on every node under
test; the same readout; one bag per shape that hardware produced and simulation cannot.

**Hardware-in-the-loop smoke (owner present):** the real sensors, the real board, the readout open,
one shape at a time, never as part of an unattended "test". Nothing that writes to the sensor
(config, firmware) runs without the owner's word.

**Caveats learned the hard way:**
- A **fixed numeric readout** is the record; counts, rates and states are read by script, never
  eyeballed off RViz.
- **One pass with production defaults** — no simulated source, no sim-only parameter, no fake clock
  — with message rates counted from the readout. The injected path is the one the harness proves;
  the default path is the one that runs on the target.
- **Time sources:** `use_sim_time` mismatches silently break every time-based filter; verify the
  clock every node is on before believing a latency number.
- **QoS mismatches drop silently**: a reliable subscriber on a best-effort publisher receives
  nothing and logs nothing by default — check with `ros2 topic info -v`.
- A visualisation window that is **occluded or minimised** may not render; a screenshot taken then
  shows nothing new. Bring it to the front and say so in the record.
- Long-run shapes (soak, memory growth) are timed by the clock in the record, not by feel.

**The visual gate (owner-judged views).** When the owner will judge an operator view (overlay,
marker set, layout), the harness IS the mockup: scripted scenes, real nodes, nothing merged.
Screenshots per state go to the owner as named PNG files, side by side when there is a choice, and
the owner picks or strikes BEFORE the wiring. Direction is written as rules a reviewer can check
(colour = one meaning; units on every number; a stale track visibly stale), not adjectives.

**Two-session split (when the executor cannot see images).** A session that drives the tools but
cannot view screenshots pairs with one that can: the driver saves PNGs under one folder with
descriptive names, messages the reader the absolute paths plus ONE question, and the reader answers
the driver only. The driver stays the owner's single interface. Paths are sent only after `ls` shows
the files.

---

## 8. Hard rules for this kind of system (every tier, always)

| # | Rule | Prevents |
|---|---|---|
| 1 | **Back up before the first change of any polish / refactor / compliance batch.** `git tag pre-<batch>-<date>` pushed; a working branch; an archive of the whole tree INCLUDING gitignored state (calibration data, models, recorded logs, udev rules, device configs) written OUTSIDE the tree with a checksum list; a restore rehearsal that builds. The record names the tag, the archive path and the rehearsal result. | Losing the only copy of calibration or model files; an unrecoverable half-refactor |
| 2 | **Nothing writes to a sensor or a board as part of a "test."** Config-send and firmware scripts run only with the owner present and named in the record. | A persisted radar config nobody can undo |
| 3 | **Models and calibration are never regenerated as a side effect.** A script that writes into `models/` or `calibration_frames/` is tier B and runs on the owner's word. | Silent replacement of the artefact every measured number depends on |
| 4 | **A message-contract change is one commit:** every publisher and subscriber, the version bump, the ICD document, and the bag-compatibility note together. | Half the graph on the old contract |
| 5 | **Fail closed, degrade to last-known-good** (principle 6) — enforced by a test per input class: bad parameter, corrupt file, malformed frame, lost sensor. | Plausible garbage on the bus |
| 6 | **Every measured number carries its conditions and date** (sim or live, which sensor, which board, which model precision, clock source). An unmeasured number is worded as unmeasured. | A README rate nobody can reproduce in the interview |
| 7 | **The first run on real sensors after any decision-path change is watched** by a named person with the readout open. | Silent regression on live input |
| 8 | **Rollback is written before merge**, names its trigger and what it does NOT undo (a sensor config already sent, a model already replaced). | Improvised rollbacks |
| 9 | **Every session ends with the handoff entry and notes updated.** | Lost context |
| 10 | **After every merge / rebuild on the target:** clean build, launch, readout sane for one full shape, logs scanned for new warnings. | Broken targets nobody noticed |
| 11 | **No credentials in scripts or launch files** (network consumers, cloud endpoints); they come from a gitignored env file. | Secrets in history |
| 12 | **Throwaways live in gitignored paths** (`launch/_harness_*`, `scripts/_*`) and the run procedure lists the exact paths to commit; "git status is the file list" is not a step. | Harness launches shipping in the tree |
| 13 | **A parameter or config state is never left unexplained.** After every smoke, the parameter dump is recorded; each non-default row is "intended (who, when)" or reverted before the record closes. | The next run inheriting a test parameter |
| 14 | **Stability is claimed only after a soak** with the duration, memory and track-table numbers in the record. | "It ran fine" |

Add this repo's own rules below, each with the failure it prevents.

---

## 9. Documents and templates

### 9.1 Plan doc skeleton — `docs/<feature>-plan.md`

```
# <Feature> — plan v1
## 0. The ask (verbatim, who asked, when) + the owner's one concrete example; tier call + any trim with reason
## 1. Verified facts (numbered; each says HOW it was verified — command + output)
## 2. Decision log      | D-n | decision, with numbers | Fixed / Needs <owner>'s decision / Open |
## 3. Work items (W-1..n: exact files, build order)
## 4. Failure modes considered
## 5. Empirical proof plan (shape → how driven → expected numbers)
## 6. Run procedure + rollback (written BEFORE the build; every command executed first)
## 7. Decisions still open (the owner's)
## 8. Review outcomes (v2: finding → found by → decision → where it landed; losing arguments)
## 9. Harness results (shape → observed → expected → ✓)
## 10. Post-build bug hunt — verdict; pass lines: RUN (time, verdict) / SKIPPED (reason)
## Friction log (playbook line quoted → what was done instead)
## Timings (phase → start → end → tokens/notes)
```

### 9.2 Rows

- Decision: `| D-n | <what and why, with the numbers> | Fixed (who) · Needs <owner>'s decision · Open — <who decides> |` — a decision that moves a threat / geofence decision, changes the contract, or goes on a résumé is never "Fixed" until the owner answers.
- Review outcome: `| R-n | <root cause> | <roles that found it> | Accept / Needs probe / Needs hardware check / Discard (+ losing argument) | <where it landed> |`
- Harness result: `| S-n | <how driven: scene / bag> | <observed, from the readout> | <expected> | ✓/✗ |`
- **Facts-sheet row** (`docs/resume-facts.md`): `| claim | evidence (file:line · command + output · commit) | confidence: measured / implemented-unmeasured / planned | the decision behind it and its trade-off, one line |`

### 9.3 Run procedure + rollback

Numbered steps naming the exact command and the expected output; the readout check between
steps; "…then rebuild on the target and confirm the new binary is the one running" after every
change; a smoke with pass criteria; the rollback with trigger + window + what survives it (configs
sent, files replaced); re-read after the last code change; read once more by role 12.

### 9.4 Handoff entry — `HANDOFF.md`, under `## PICK UP HERE`, newest first

feature + date · commit ids (or "uncommitted — paths below") · branch · parameters / configs
changed and their intent, duration, who reverts · what is on the target right now · what is
pending and who owns it · decisions still open (the owner's) · exact path list + per-part commit
lines + the literal git commands.

### 9.5 Retro — `docs/<feature>-retro.md` (when the owner asks)

Timeline (UTC) · what was built · findings ledger (# → finding → severity → found by → outcome →
would it have shipped?) · agent scorecard (role → tokens → minutes → raised → accepted → unique
catches → judgment) · orchestrator self-review · reward (which lenses to keep) · change (next
playbook version) · still open.

### 9.6 Friction log entry

`n. **§x "<quoted playbook line>"** — <what was unclear or wrong>. Did: <what was done instead>. Next version: <the fix>.`

---

## 10. Agent prompt template and orchestration rules

```
You are a review lens for the repo at <absolute repo path>
(<one line: ROS 2 <distro>, C++<std>, target <board>; sensors attached right now: <yes/no, which>>).
Read first: docs/<feature>-plan.md. In scope: <PLANNED new paths — "do not exist yet, review the
plan's spec"> + <touched existing files, read in full> <role 11: the views/screenshots to check>
<role 12: the handoff entry, the run procedure + decision log, README, CHANGELOG, resume-facts>.
Role: <number + name>. Mission: <paste the Mission line verbatim from §4>.
<Role 5 only: paste the applicable S-rows.>
Evidence contract: every finding needs exact file:line AND a concrete failure scenario (frame /
state / parameters → wrong outcome); try to kill each candidate before reporting; no scenario = not
a finding. Verify ROS / library / hardware claims against the installed docs, headers or a
read-only probe, never memory.
Rules: READ-ONLY — do not create, edit or delete files; small scripts, `colcon test` and read-only
`ros2` introspection are fine; NEVER launch anything against attached hardware, send a sensor
config, or write into models/ or calibration_frames/.
Report (final message, under ~1,800 words): ranked findings worst-first (impact ranking) each with
scenario + fix direction; then the claims/candidates you VERIFIED or KILLED; <role 6: end with SHIP
or FIX-FIRST> <role 12: end with the corrected-lines list>.
```

Orchestration: launch all plan-review agents in **one message** (parallel), with the plan AND the
pure module; **freeze the tree** until they return; record each agent's tokens, tool uses and
duration in the Timings table; dedupe by root cause; write down the losing argument of every
disagreement; the orchestrator never edits a decision-path file to make its own test pass — it
fixes the test.

---

## 11. Traps this method exists to catch

| Trap | What to do instead |
|---|---|
| Believing ROS / a driver / a library behaves like it did in another version | Read the installed docs or headers; probe on the device; the installed version is the truth |
| A QoS mismatch that drops every message silently | `ros2 topic info -v` on both ends before debugging the algorithm |
| Filters fed by a mix of wall clock and sensor clock | One clock per graph, `use_sim_time` verified on every node under test |
| Units mixed at a boundary (m/cm, deg/rad, ms/s) | Convert once at the boundary; name the unit in the type or the variable; a unit test per boundary |
| A string compared where an enum should be | Typed fields; the compiler is the test |
| Floating-point equality on a gate or a state transition | Tolerances or ordered comparisons, with the tolerance's home being the one producer |
| Integer bin arithmetic that overflows or goes negative at the edges | Explicit widths, checked conversions, tests at range 0 and range max |
| Blocking I/O or unbounded work inside a callback | Bounded per-callback work; I/O on its own thread; a watchdog in the readout |
| A "compliance" or "style" pass that flipped a comparison | Tier B when the edit touches the math; role 6 checks every changed operator |
| An inline default feeding a re-execution key (callback, timer, reactive binding) | Named constant or member; a source pin; the production-defaults harness pass |
| A stale CMake cache or symlink-install after a target or toolchain change | Build from clean before the gates |
| A guard test that reads prose (comments, docs) | Match code tokens, or strip comments with the language's parser |
| Building while the reviewers read | Pure module first, then freeze; re-sync the plan after |
| Reviewers launched against a run procedure written for the previous batch | Rewrite the handoff for THIS tree first; diff the staging line against `git status --short -uall` |
| A reviewer's fix direction taken on faith | Measure in the still-open harness; record hypothesis → measurement → fix |
| A clarifying question that offers the executor's own interpretations | Ask for ONE concrete example (a bag + timestamp, a screenshot, the parameter set) |
| Two full post-build review sets on one feature family in one day | Tier C-fix: one review; earlier findings carry over |
| A lifecycle pass skipped without a trace | `RUN (time, verdict)` or `SKIPPED (reason) — Needs <owner>'s decision` |
| Timestamps predicted, counts typed from memory | `date -u` and `wc -l` in the same command that writes the line |
| A generated regex trusted on "0 hits" | Test it against the file first |
| Commands over ~5 KB; backslashes or backticks in inline one-liners | Script files from a quoted heredoc ≤ 4 KB; one heredoc per command |
| Anchors patched from memory | Read the exact lines in the previous command; the script computes every replacement, writes once |
| A screenshot of an occluded or minimised window | Bring it to the front; say so in the record |
| A harness launch file left in the tree | Gitignored `launch/_harness_*`; exact commit path list |
| A README number with no conditions | Sim or live · sensor · board · model precision · clock · date, next to every number |
| A parameter left non-default after a smoke | Parameter dump in the record; intended (who, when) or reverted |
| "It ran fine" as a stability claim | A timed soak with memory and track-table numbers |
| Two sessions and one owner watching both | The tool-driving session is the only interface; the image-reading session answers it, never the owner |

---

## 12. New-project setup checklist

- [ ] `docs/engineering-playbook.md` (this file) with §6 and §8 filled with commands actually run.
- [ ] The backup of §8 rule 1 done and recorded before the first change.
- [ ] `HANDOFF.md` with a `## PICK UP HERE` section; notes for the executor.
- [ ] `.gitignore`: build output, `.env*` (keep `.env.example`), `launch/_harness_*`, `scripts/_*`; large artefacts (models, bags, calibration) either LFS or listed in the backup procedure.
- [ ] A simulated source node and at least one recorded bag per hardware-only shape.
- [ ] A readout node/script that prints the numbers the README claims.
- [ ] Single-producer modules for every behaviour-deciding constant, each with a source-scan test.
- [ ] Gate commands scripted (a `Makefile` or `scripts/gates.sh`); the baseline warning and violation counts written down.
- [ ] `docs/resume-facts.md` opened on day one: every claim, its evidence, its confidence.
- [ ] A first friction log opened in the first plan.

---

## 13. Definition of done

| Tier | Done means |
|---|---|
| **B** | plan with decision log, verified facts, review-outcome table, harness results (incl. the production-defaults pass), run procedure + rollback, Timings · code + tests (source-scan guard where a constant got a single producer) · gates green with counts · role 6 SHIP · built-in passes triaged · role 11 on every changed view · role 12 on the record · first run on real sensors watched · handoff entry + notes · git commands handed to the owner · decisions the owner has answered, not assumed · facts sheet updated |
| **C** | short plan note · code + tests · gates · one combined review · role 11 · handoff line |
| **C-fix** | build · gates · harness on the changed behaviour · ONE post-build review · run procedure · handoff line |
| **D** | gates · role 11 if a view changed · handoff line if it changes anything an operator reads |
| **A** | smallest diff · gates · verified on the target with the readout · handoff line · role 6 within 24 h |

---

## 14. Never-again ledger — template and seed rows

One row per real event, dated, never deleted (retire with a date). The **rule** is absolute; the
**check** is the mechanical thing that makes it hold without good intentions. Read before phase 1
and before the first patch; add the day it happens, with its cost.

`| L-n | What happened (date) | Cost | NEVER … | Check that prevents it |`

Seed rows (generalised from the parent project's first fifteen; keep what applies, add your own):

| # | What happened | Cost | NEVER | Check |
|---|---|---|---|---|
| L1 | A category-scoped ask was built from the executor's own reading; the owner picked the nearest option; the wrong thing shipped and was rebuilt | ≈ half a session; two review sets | Never build a category-scoped change without ONE concrete example in the owner's words | Plan §0 "verbatim + example" required; W-1 waits for it |
| L2 | Lifecycle passes skipped silently, then run in the retro → a second review round | An extra cycle | Never skip a pass without writing it down | `RUN` / `SKIPPED (reason)` on every pass line |
| L3 | Reviewers read a run procedure written for the previous batch; its steps would have broken the target | A review pass on a docs fault | Never launch post-build reviewers before the handoff is rewritten for THIS tree | Runbook first; staging line diffed against `git status` |
| L4–L5 | Timestamps predicted; counts typed from memory; the cold reader spent its pass on arithmetic | A review pass on numbers | Never write a time or count you did not read from a command | `date -u` / `wc -l` in the writing command; one source of truth |
| L6 | Reviewer fix directions nearly taken on faith; one did the opposite of the goal | A harness cycle | Never apply a fix direction unmeasured | Hypothesis → measurement → fix |
| L7–L11 | Tooling: oversized commands, collapsed backslashes, anchors from memory, wrong path forms, a regex trusted on "0 hits" | ≈ 25 failed commands | Never inline what belongs in a script file | Script files ≤ 4 KB from quoted heredocs; anchors read first; regex tested |
| L12 | A same-day correction got a second full review set | The largest token line of the session | Never two full sets on one family in one day without the owner's word | Tier C-fix |
| L13 | A state left flipped by a smoke with no recorded intent caused an owner-visible regression | Hours of wrong behaviour | Never leave a state unexplained | Rule 13 |
| L15 | An inline default fed a re-execution key; every update re-armed the work at network speed; the harness had injected a stable value so no lens saw it | Thousands of spurious calls in 15 min; a user locked out (tier A) | Never an inline function/object default on anything that feeds a dependency or re-execution key | Source pin; reviewer checklist; the production-defaults harness pass |

---

## 15. Whole-project audit mode — find everything first, fix in batches, record once

Use this instead of the feature lifecycle when the ask is "go through the whole project and find
what is wrong" — taking over, polishing, or preparing a codebase to be shown. The feature lifecycle
records after every change because a live product needs a trail per deploy. An audit does not:
per-change plans, per-change handoff rewrites and per-change review sets would spend most of the
budget on paperwork about paperwork. **In audit mode the full record is written ONCE, at the end
(A7).** This is the owner's decision and is written into the audit plan's §0 as the tier call.

### What is different from feature mode

| Feature mode | Audit mode |
|---|---|
| One plan doc per change | ONE `docs/audit-plan.md`, tables only, opened at A1 and completed at A7 |
| Handoff rewritten before every review | Handoff rewritten once, at A7 |
| Reviewers on each change's plan and diff | Reviewers ONCE over the whole codebase by subsystem (A3), then ONE bug-finder pass on the cumulative diff (A5) |
| Harness after each change | Harness once, on the final tree (A6) — unless a batch changes something the next batch depends on |
| Prose status after each step | An append-only `docs/audit-log.md`: one line per event, written by the same command that does the thing |
| Retro when asked | Retro is part of A7 |

### The safety net that replaces the running record

- **Backup first** (§8 rule 1). Nothing else starts before the restore rehearsal builds.
- **Checkpoint commits** on the audit branch after every batch that passes gates, one-line
  messages (`audit: B2 tracker — bounded association matrix, NaN guard`). Work survives a dead
  session; nothing is rewritten for them.
- **`docs/audit-log.md`**, append-only, one line per event, e.g.
  `21:04Z · A3 role 5 done · 9 findings · 148k tok` or `22:10Z · B2 gates green 41/41 · 3f2a9c1`.
  Written with `date -u` in the same command. This is the ENTIRE running record; it is what A7
  is reconstructed from if the session dies.
- **Tier-A findings interrupt.** Anything found during A3 that is tier A (broken build on target,
  a crash on real input, a wrong safety decision) is told to the owner immediately in one line
  and fixed first. Everything else waits for the triage message (A4).

### Phases

**A0 — Backup** (§8 rule 1, KICKOFF §0). Recorded as the first lines of the audit log.

**A1 — Ground truth of the whole system, one pass.** Clean build with the warning count; test
count; the coding-standard checker and its violation count (or "checker not found"); graph
inventory — nodes, topics with publisher/subscriber pairs and QoS, tf frames, parameters with
declared ranges; the environment (board, ROS, CUDA/TensorRT, driver versions, sensors attached);
and the **claims list**: every number or capability asserted in README / CHANGELOG / latency /
ICD docs, each marked *verified (command)* or *unverified*. Written straight into
`docs/audit-plan.md` §1 as tables — this is the one upfront document, because every later phase
depends on it.

**A2 — Subsystem map.** One table: subsystem → files → entry points → what it decides. For a
sensor-to-decision system: source driver / parser · clustering · tracker and motion models ·
association and gating · fusion · classifiers (threat, intent) · geofence and reachability ·
health · visualisation and overlays · network output · launch and configuration · build, scripts
and install · docs and claims. This table is what scopes the agents — they get file lists, never
"explore the repo".

**A3 — Lens sweep, one message, all agents in parallel, each scoped by the map:**

| Agent | Scope | Output |
|---|---|---|
| Role 3 failure-forcing | the decision-path subsystems | findings table |
| Role 4 cross-file / guarantees | the message contract, parameters, topics, frames — every consumer | inventory + findings |
| Role 5 adversarial-input + safety (S1–S8) | parsers, network in/out, file inputs, parameters, state machines | findings table + "checked, already safe" |
| Role 10 systems integration | the whole graph: orphans, QoS pairs, tf, launch order, the latency budget summed | connected / not-connected table |
| Role 6 bug finder, whole-codebase mode | the math classes and hot-path callbacks, file by file, C++ eyes | findings table |
| Role 12 cold reader | the claims list from A1 against the code and the commands | claim → holds / reword / unverified |
| Role 9 altitude (optional, owner's call) | architecture of the pipeline | one page: what gets expensive later |

Every agent returns a **compact table** — `id · file:line · scenario · impact rank · fix
direction · proposed tier` — not prose. Budget per agent is written in §0 before launch; the
completion notice's tokens and minutes go into the log line. Nothing is fixed during A3.

**A4 — Triage, one message to the owner.** Dedupe by root cause; apply the acceptance table (§5);
rank by impact; tier each finding; group into **batches**: A (already done), B by subsystem in
dependency order (parser before tracker before classifiers), C, D. The message to the owner
contains the ranked table and, separately, every decision that is theirs: thresholds that would
move a decision, claims that must be reworded or removed, anything Needs hardware check. One
message, one reply; then the batches run without further check-ins except tier A.

**A5 — Fix batches.** Per batch: pure module + unit tests first if it touches math; build →
gates → checkpoint commit → one log line. No plan rewrite, no handoff rewrite, no reviewer.
After ALL tier-B batches: **ONE role-6 pass on the cumulative diff**
(`git diff <pre-audit tag>..HEAD -- <decision-path files>`), fix → gates → re-run role 6 on the
fixes only. Tier C and D batches get gates only.

**A6 — Proof, once, on the final tree.** The full shape list (§4 role 7) through the harness with
the readout; the production-defaults pass; a soak if stability is claimed anywhere; the operator
view (role 11) if anything visible changed. Hardware shapes are the owner's smoke. Results go
into the audit plan §9 as one table.

**A7 — The record, once.** In this order: `docs/audit-plan.md` completed (findings table with
outcomes, decision log, harness results, timings from the log); README and CHANGELOG corrections
(every claim now verified, reworded, or removed); `docs/resume-facts.md`; `HANDOFF.md` PICK UP
HERE; ledger rows (§14) for any new mistake; the retro section; then **role 12 reads the whole
record once**; then the commit lines and the literal git commands for the owner. The audit log's
last line is `A7 record complete`.

### Token economy rules for audit mode

- Never re-read a file already read in the same phase; agents receive file lists from the A2 map.
- Tables, not prose — in agent reports, in the plan, in the triage message.
- The orchestrator does not narrate progress; the audit-log line IS the narration.
- One gate run per batch, not per file; one harness run per audit, not per batch.
- No screenshots except in A6 (operator view / visual gate).
- One message to the owner at A4 and one at A7; tier-A interrupts only.
- Budget written per agent before launch; an agent that runs past it is stopped and its partial
  table used.

### What audit mode does NOT skip

The backup; gates after every batch; tier-A interrupts; the ONE role-6 pass on the cumulative
decision-path diff; the harness on the final tree; role 12 on the record; the owner's decisions
asked before they are assumed. Those are the parts that catch bugs. Everything trimmed is
paperwork cadence, and the trim is written in the plan's §0 with the reason.

---

*Provenance: the parent playbook ran nine features on a live e-commerce product between 2026-08-30
and 2026-09-01 (three cold executions, two correction batches, one incident). This edition
translates it for a ROS 2 / embedded sensor-to-decision system; its first cold execution is the
ProjectHailMary polish — keep the friction log, and this file's v1.1 comes from it.*
