# Drone-Mapper-ex3 — Agent Guide

TAU Advanced Topics in Programming, Assignment 3. Splits ex2's monolithic simulator into three
separately built projects — `simulator_<ids>` executable that `dlopen`s an `Algorithm_<ids>.so` and a
`MissionControl_<ids>.so` — and adds multithreaded comparative/competitive run modes.

**Status:** implementation is feature-complete enough for a full instructor-doc audit
(`docs/assignment-compliance-pickup.md` — **start there**). Default composition
`inputs/sim_compose.yaml` scores **24/24 Completed** with `mission_score >= 0`. Submission
`README.md` and root `HLD.pdf` are done (2026-08-27). Instructor-catalog follow-up Points 1–4 are
done (2026-08-28) — see `docs/instructor-test-catalog-followup-roadmap.md`; re-verify with
`verify-instructor-test-catalog`. Independent-component variants (VAR-01…04) harness is on
branch `independent-component-variants` — re-verify with `verify-independent-component-variants`.
Remaining mandatory work is Known Issues excel export at zip time and pre-submission packaging,
not the Sagi/Yoav track split. Deadline: **Sep 6, 2026, 23:30**.

## Start here

1. Read `.cursor/rules/project-context.mdc` (always applies) and `docs/assignment3-checklist.md`.
2. Read `docs/assignment-compliance-pickup.md` for where we left off, then `docs/workplan.md` for
   the historical Sagi/Yoav split (do not restart that split unless the pickup file says so).
3. Before git branches, commits, or PRs: `git-workflow.mdc`.
4. Before touching a published header: `frozen-interfaces.mdc` and `docs/api-delta-ex2-to-ex3.md` —
   ex2 code does not compile as-is (moved headers, changed namespaces, changed types).
5. Before deciding where new code lives: `docs/component-placement.md`.
6. Before writing the plugin loader, CLI, or threading: `plugin-architecture.mdc`,
   `simulator-cli-and-outputs.mdc`, `threading-model.mdc`.
7. Before error paths: `error-handling-logging.mdc` and `docs/error-handling-matrix.md`.
8. Before editing C++: `adv-cpp-standards.mdc`, `mp-units-strong-types.mdc`.

## Skills (invoke by name)

| Skill | Use when |
|-------|----------|
| `port-ex2-component` | Moving a component from `../Drone-Mapper-ex2/` into the ex3 layout |
| `plugin-loading-and-registration` | Implementing or debugging `dlopen`/registration/`dlclose` |
| `pre-submission-review` | Checking the 5-folder/ID-naming submission structure before zipping; also grep checks for `new`/`delete` (`ZIP-15`), mock placement (`ZIP-13`), `inputs/` presence (`ZIP-17`), and produced-zip archive name/root-placement (`ZIP-01`/`ZIP-04`/`ZIP-05`) |
| `verify-frozen-interfaces` | Confirming `common/` (and the other frozen folders) weren't touched |
| `gather-instructor-context` | Extracting / re-diffing Assignment 3 from `context/` via subagents (docx/pdf win over `docs/`) |
| `populate-known-issues` | Adding a row to `docs/known-issues.md` (optional excel export later) |
| `advcpp-rubric-review` | Subjective AdvCpp rubric review (`e01`–`e23`): dispatches one explore subagent per code group (headers/API, C++ idiom, structure/flow, HLD alignment), reports findings as a judgment-call table — not a pass/fail script. Run before submission or after major refactors. |
| `verify-instructor-test-catalog` | End-to-end instructor-catalog verification: Docker build + `ctest` + `run_all.sh` + `pre-submission-review` + `advcpp-rubric-review` → one PASS/FAIL/AMBIGUOUS report keyed by catalog ID |
| `verify-independent-component-variants` | Independence harness (VAR-01…04): Docker build + `check_foreign_host` / `check_foreign_mission_control` / `check_adversarial_plugins` / optional `check_baseline_algorithm` → PASS/FAIL/SKIP per variant (not catalog IDs) |

## Key docs

| Path | Purpose |
|------|---------|
| `docs/assignment-compliance-pickup.md` | Where to pick up: instructor-doc audit verdict and ordered remaining work |
| `docs/instructor-test-catalog-followup-roadmap.md` | Catalog follow-up Points 1–4 — **done 2026-08-28** (harness gaps, skills, orchestrator) |
| `docs/superpowers/specs/2026-08-28-independent-component-variants-design.md` | Blind foreign host / MC / adversarial / baseline independence design |
| `docs/superpowers/plans/2026-08-28-independent-component-variants.md` | Implementation plan + execution notes (VAR-01…04) |
| `docs/instructor-test-suite-approach.md` | Why/how the blind catalog + Phase B harness were built |
| `docs/simulator_runtime_test_catalog.md` | Phase A blind catalog (IDs keyed by `verify-instructor-test-catalog`) |
| `docs/workplan.md` | Historical work items split between Sagi and Yoav (see pickup file first) |
| `docs/assignment3-checklist.md` | Mandatory requirements, condensed from the assignment docx |
| `docs/api-delta-ex2-to-ex3.md` | Header/type/namespace deltas between the ex2 and ex3 skeletons — **and** where our own `Drone-Mapper-ex2` deviated from the frozen ex2 headers itself (those need reverting, not porting) |
| `docs/component-placement.md` | Which ex2 file goes in which of the 5 folders, and why |
| `docs/error-handling-matrix.md` | Mandatory + optional fault-handling table (course staff PDF) |
| `docs/map3d-contract.md` | `.npy` dtype rules (maps are **mixed** `int8`/`uint8`), world↔voxel mapping |
| `docs/review-error-codes.md` | AdvCpp rubric codes (`e*`, `b*`) — no ex3-specific guideline published yet |
| `docs/known-issues.md` | Working Known Issues rows (excel export is a later zip step) |
| `docs/known-issues-guidelines.md` | Optional Known Issues excel — grade-neutral-or-better to submit |
| `docs/open-questions.md` | Genuine ambiguities with a working assumption each — check the forum |
| `docs/ex2-grading-handoff.md` | Lessons from the Ex2 grade (appealed to **87.5/100**) — frozen-API drift, obsolete bugs, and the ALG28 unbounded-BFS hang; not a plan for sequencing Ex3 |
| `context/` | Assignment docx/pdf sources |

## Hard rules (see `.cursor/rules/` for the full versions)

- **Never** add files to `common/`, or change a published header's signature.
- **Never** cache plugin instances between runs; **never** reload a `.so` that's already loaded.
- No `new`/`delete`; never `exit()`/`abort()`; the Simulator must not crash except on a plugin crash.
- Every artifact name carries both student IDs — see the naming table in `docs/assignment3-checklist.md`.
- Log every error immediately; never defer to shutdown.

## What's genuinely uncertain right now

Assignment 3 doesn't specify a scoring metric, a comparative-grouping rule, or any testing requirement —
`docs/open-questions.md` has the full list with working assumptions. Don't invent ex2-style ceremony
(bug-injection ceilings, coverage bars) for things that were never asked for in this assignment; the doc
also ends mid-sentence (`[TBD]`) and left draft mode Jul 26 — re-diff `context/Advanced Topics TAU 2026B -
Assignment 3.docx` before submission.

## Note on this infrastructure

The rules/skills here were written for ex3's own requirements, not ported wholesale from
`../Drone-Mapper-ex2/.cursor/`. Generic ex2 team-process conventions (workplan-phase-naming avoidance,
bug-injection coverage bureaucracy) were deliberately **not** copied verbatim — they addressed problems
specific to that team's workplan, not this assignment. Where the same underlying concern genuinely
applies here too, it's rewritten against ex3's own workplan shape instead of reused as-is: `docs/workplan.md`
splits work by owner and dependency (no phases, no gates), and `.cursor/rules/git-workflow.mdc` keeps
*our* workplan's shorthand (item codes, owner names) out of git history the same way ex2's rule kept
its phase/gate labels out — same problem, different labels, so the rule was rewritten, not copied. Where
the same course rubric genuinely still applies (AdvCpp code-quality rules, mp-units usage), it's cited
from ex3's own `context/` docs, not from ex2's files.
