# Safety-Critical C++ Standards — Audit Checklist

**Purpose**: Audit basis for the `cuas_fusion` Counter-UAS ground station (ROS 2 Humble, C++17, Jetson Orin Nano).
Built from primary sources before any code was audited. Every rule ID below was verified against the
cited source — nothing is paraphrased from memory.

**Sources**
- MISRA C++:2023 rule table: Perforce QAC enforcement doc (complete 179-guideline listing) — https://help.perforce.com/qac/enforcement/doc/MISRA_M2CPP.pdf
- MISRA Compliance:2020 (deviation process): https://misra.org.uk/app/uploads/2021/06/MISRA-Compliance-2020.pdf
- JPL Power of 10: G. J. Holzmann, IEEE Computer 39(6):95–99, June 2006 — preprint https://spinroot.com/gerard/pdf/P10.pdf
- JPL Institutional Coding Standard D-60411 v1.0 (2009): https://everyspec.com/NASA/NASA-JPL/JPL-D-60411_VER-1_32832/
- JSF AV C++ (Lockheed, Doc 2RDU00001 Rev C): https://www.stroustrup.com/JSF-AV-rules.pdf
- DO-178C/DO-330/DO-332 overviews: NASA NTRS 20120016835; tasking.com/do-332; ldra.com/do-330
- ROS 2 real-time design docs: https://design.ros2.org/articles/realtime_background.html

---

## 1. MISRA C++:2023 — structure you must be able to explain

- **179 guidelines = 175 Rules + 4 Directives.** Rules are statically checkable (decidable or undecidable);
  Directives need engineering judgment (Dir 0.3.1 float arithmetic, Dir 0.3.2 preconditions, Dir 5.7.2 no
  commented-out code, Dir 15.8.1 self-assignment safety).
- **Numbering mirrors the ISO C++17 standard clauses** (Rule 8.2.2 lives in ISO clause 8 "Expressions") —
  a deliberate change from MISRA C++:2008's X-Y-Z scheme.
- **Classifications** (MISRA Compliance:2020 §5):
  - **Mandatory** — violation never permitted; no deviation possible. Only **5 Mandatory rules exist**:
    6.8.2 (no dangling return), 8.18.1 (no overlapping copy), 11.6.2 (no read-before-set),
    25.5.2 / 25.5.3 (getenv/strerror-family pointer misuse).
  - **Required** — violable only with a formal deviation record.
  - **Advisory** — follow as reasonably practical; violations identified but need no deviation.
  - Re-categorization may only strengthen, never weaken (Required can't become Advisory).
- **Lineage (interview question)**: MISRA C++:2008 (C++03, 228 rules) → AUTOSAR C++14 (2017, stopgap,
  +modern-C++ rules) → 2019 merger → **MISRA C++:2023** (C++17, 179 guidelines, published Oct 2023).
  Dropped style-only rules — notably **there is NO single-point-of-exit rule anymore** (2008's 6-6-5 was
  deleted) and **no rule restricting `auto`**. Don't "fix" either.

### Key rules for this audit (verified IDs)

| # | Rule | Cat | Checklist item | Applies to |
|---|------|-----|----------------|-----------|
| M1 | 21.6.1 | Advisory | No dynamic memory at all (aspirational) | hot path |
| M2 | 21.6.2 | Required | All dynamic memory managed automatically (RAII, no naked new/delete) | everywhere |
| M3 | 21.6.3 | Required | No placement new / advanced allocators | everywhere |
| M4 | 18.4.1 | Required | Destructors/move ops noexcept | everywhere |
| M5 | 18.3.2 | Required | Catch class exceptions by (const) reference | everywhere |
| M6 | 18.3.1 | Advisory | Catch-all handler exists for otherwise-unhandled exceptions | main() |
| M7 | 8.2.2 | Required | No C-style or functional casts | everywhere |
| M8 | 8.2.3 | Required | No cast removing const/volatile | everywhere |
| M9 | 8.2.5 | Required | No reinterpret_cast | everywhere (deviation likely: V4L2/TLV parsing) |
| M10 | 7.0.1/7.0.2 | Required | No implicit conversion to/from bool | everywhere |
| M11 | 7.0.5/7.0.6 | Required | No signedness-changing promotions; no narrowing assignment | everywhere |
| M12 | 7.11.1 | Required | nullptr is the only null-pointer constant (no 0/NULL) | everywhere |
| M13 | 11.6.1 | Advisory | All variables initialized at declaration | everywhere |
| M14 | 11.6.2 | **Mandatory** | No read-before-set | everywhere |
| M15 | 6.8.2 | **Mandatory** | Never return ref/pointer to a local | everywhere |
| M16 | 12.3.1 | Required | No union keyword | everywhere (deviation likely: TLV parsing) |
| M17 | 8.7.1/8.7.2 | Required | No invalid pointer arithmetic; subtraction only within one array | parsers |
| M18 | 8.2.10 | Required | No recursion, direct or indirect | everywhere |
| M19 | 13.3.1 | Required | virtual/override/final used appropriately on every override | class hierarchies |
| M20 | 13.3.2 | Required | Overrides don't change default arguments | class hierarchies |
| M21 | 15.1.1 | Required | No dynamic-type use in ctor/dtor (no virtual calls in ctors) | class hierarchies |
| M22 | 8.1.1/8.1.2 | Req/Adv | Lambdas: no implicit ref-capture in non-transient lambdas; capture explicitly | ROS callbacks |
| M23 | 0.0.1 | Required | No unreachable statements | everywhere |
| M24 | 0.1.2 | Required | Return values used (or explicitly discarded) | everywhere |
| M25 | 0.2.2 | Required | Every named parameter used | everywhere |
| M26 | 9.6.5 | Required | Non-void function returns on all paths | everywhere |
| M27 | 6.7.2 | Required | No global variables | everywhere |
| M28 | 9.6.1 | Advisory | No goto | everywhere |
| M29 | Dir 5.7.2 | Advisory | No commented-out code | everywhere |
| M30 | Dir 15.8.1 | Required(Dir) | Copy/move assignment handles self-assignment | value classes |

### Deviation process (MISRA Compliance:2020 — what a real MISRA shop does)

A **deviation record** must contain (§4.2): (1) guideline violated, (2) circumstances in which violation
is acceptable, (3) reason — one of four approved: **code quality (incl. performance), access to hardware,
adopted-code integration, non-compliant adopted code**, (4) background/context, (5) risk assessment +
precautions. Every violation location must be traceable (file/line register or tags). Sign-off by a
designated technical authority is required. Deviations are prohibited for mere convenience or when a
reasonable compliant alternative exists.

**Permit vs record**: a *permit* is a reusable pre-approved template for a use case; a *record* is the
project-specific instance covering identified locations.

**Compliance claim** (§7): per-project, delivered as a Guideline Compliance Summary (GCS) — one line per
guideline: Compliant / Deviations / Violations / Disapplied. "Compliant with deviations" = zero Mandatory
violations + every Required violation backed by an approved record + Advisory violations at least identified.

---

## 2. JPL Power of 10 (Holzmann, 2006) — with C++17 translation

| # | Rule (near-verbatim) | C++17 / ROS 2 translation |
|---|---------------------|---------------------------|
| P1 | Simple control flow: no goto, setjmp/longjmp, no direct or indirect recursion | Also: exceptions are the modern setjmp/longjmp — ban throw in flight code. Acyclic call graph enables static stack bounding. |
| P2 | All loops have a statically provable fixed upper bound (or provably non-terminating for scheduler loops) | Bound loops over runtime containers with a constexpr cap; guarded main loops (`while (rclcpp::ok())`) are the sanctioned non-terminating case. |
| P3 | No dynamic memory allocation **after initialization** | Init-phase allocation OK. Steady-state: no vector growth, no map insert, no string build, no make_shared in callbacks. Use reserve()+capacity guards, std::array, fixed-capacity containers, pmr pools. |
| P4 | Functions ≤ ~60 lines | clang-tidy readability-function-size. |
| P5 | Assertion density ≥ 2 per function average; assertions side-effect-free; failed assertion triggers explicit recovery (not abort) | Project CHECK() macro returning an error, not abort(); static_assert for compile-time class. D-60411 relaxes to ≥1 assertion per function >10 LOC. |
| P6 | Data declared at smallest possible scope | const by default, declare-at-first-use, if-with-initializer, no non-const globals. |
| P7 | Check all non-void return values (or explicitly cast to void); validate parameters | [[nodiscard]] on status returns; `(void)`/std::ignore for deliberate discards; validate at public API boundaries. |
| P8 | Preprocessor: includes + simple macros only; no token pasting/varargs/recursive macros; minimal conditional compilation | constexpr/if constexpr/enum class replace macros; each #ifdef beyond include guards needs justification (2^N version explosion). |
| P9 | Pointers: ≤1 level of dereference; no dereference hidden in macros/typedefs; no function pointers | references/span/string_view/not_null; no T**; virtual dispatch and std::function ARE function pointers to an analyzer — hierarchies must be final/sealed so the call graph stays bounded (D-60411 allows const function-pointer tables). |
| P10 | All warnings on, most pedantic setting, zero warnings from day one; daily static analysis, zero findings | -Wall -Wextra -Wpedantic -Wconversion -Wshadow (+ -Werror); clang-tidy + second analyzer in CI, gating merges. Rewrite code that confuses the analyzer even if the warning looks false. |

**ROS 2 reality (design.ros2.org)**: the sanctioned pattern is three-phase — (1) non-RT init: construct
nodes/pubs/subs, preallocate everything; (2) RT spin: allocation-free, lock-free, blocking-free callbacks;
(3) non-RT teardown. Middleware allocation is contained via custom executor/publisher Allocator template
args (TLSF), bounded message types, and loaned messages. Power-of-10 rules are enforced on *code you own*
in the callback path; rclcpp's own allocations are an accepted, documented deviation.

---

## 3. DO-178C-relevant practices (honest framing: ground station ≠ airborne)

**Framing to use in interviews**: DO-178C governs *airborne* software. A C-UAS ground station falls under
**MIL-STD-882E** system safety (Software Criticality Index → Level of Rigor) or **DO-278A** (ground
CNS/ATM, AL1–AL6). The defensible claim is "**DO-178C/DO-332-inspired practices**", never "DO-178C
compliant" (compliance requires a certification authority).

**Facts to know cold**
- DAL A–E by worst failure condition (Catastrophic → No effect); ~71 objectives at Level A, 69 B, 62 C, 26 D;
  independence requirements grow with DAL (~30 objectives with independence at A).
- Structural coverage per DAL: **C = statement, B = +decision, A = +MC/DC** plus source-to-object
  traceability. Coverage must come from requirements-based tests; a gap means missing requirement, missing
  test, or dead code — you never "write a test to hit the line."
- Supplements: DO-330 tool qualification (TQL-1..5 via Criteria 1/2/3), DO-331 model-based, **DO-332 OOT**,
  DO-333 formal methods.
- **DO-332 C++ vulnerabilities**: dynamic dispatch (mitigate via *local type consistency* — LSP verified by
  formal proof, parent-test inheritance, or pessimistic per-call-point testing), dynamic memory (OO.11:
  verify robustness or avoid), templates (verify each instantiation), overloading (verify intended overload
  resolves), exceptions (non-local flow breaks WCET/coverage arguments).
- **JSF AV C++ anchors** (Lockheed 2RDU00001C — name-drop these): AV 206 no heap after init,
  AV 208 no exceptions, no recursion, no multiple implementation inheritance.
- **Multicore (Jetson-relevant)**: AC 20-193 (ex CAST-32A, now retired) — interference channels (shared L2,
  DRAM controller, GPU/DMA contention) must be identified and bounded for WCET. A Jetson with CUDA is
  effectively uncertifiable for airborne DAL A/B; that's *why* certified multicore avionics use
  partitioned RTOSes (INTEGRITY-178 tuMP, VxWorks 653). Knowing this limitation is the interview win.

**Checklist items derived**
| # | Item | Applies to |
|---|------|-----------|
| D1 | Deterministic execution: no unbounded loops, no unbounded recursion, WCET-analyzable hot path | hot path |
| D2 | No heap after initialization (JSF AV 206) | hot path |
| D3 | Exception policy documented: no throw in owned code; library throws → terminate/fault-handler policy (JSF AV 208 adapted for rclcpp) | everywhere |
| D4 | Virtual dispatch bounded: hierarchies final/sealed, LSP-consistent overrides (DO-332) | class hierarchies |
| D5 | Every template instantiation exercised by a test (DO-332) | templates |
| D6 | Requirements traceability: req ID → code → test (RTM artifact) | project |
| D7 | Structural coverage measured (statement/decision as DAL-B imitation) | tests |
| D8 | Tool qualification notes: classify gcc/gtest/coverage tools per DO-330 criteria | project |
| D9 | Fault detection & graceful degradation on sensor loss (882E LOR flavor) | system |
| D10 | Health monitoring / watchdog with defined fault responses | system |

---

## 4. Tooling honesty (cite this in the audit report)

- **cppcheck's open-source `misra.py` addon implements MISRA C:2012 for C only** — it is *not* MISRA C++
  evidence. No free tool checks MISRA C++:2023 (Cppcheck Premium, Helix QAC, Polyspace, Parasoft, LDRA do;
  clang-tidy has no MISRA checker, largely for licensing reasons).
- Honest audit claim: **"manual MISRA C++:2023 audit against the verified rule set, plus cppcheck (native
  checks) and clang-tidy (clang-analyzer/bugprone/cert/cppcoreguidelines) as automated defect evidence."**
- Only the 175 Rules are even theoretically automatable; the 4 Directives and undecidable rules require
  manual review — which is why a manual audit is legitimate MISRA practice, not a workaround.

---

## 5. Audit method for this repo

1. Hot-path files (per-frame/per-scan callbacks) audited against every item; init-time code exempt from
   P3/D2/M1 by definition ("after initialization").
2. Each finding → AUDIT_REPORT.md §A (fixable) or §B (formal deviation record per Compliance:2020 §4.2
   field set) or noted in §C (already compliant).
3. Required-rule violations without a defensible deviation MUST be fixed; Advisory violations fixed when
   cheap, else documented. The 5 Mandatory rules admit no deviation — any hit is a must-fix.
4. PROTECTED camera files: audited, findings reported, never edited.
