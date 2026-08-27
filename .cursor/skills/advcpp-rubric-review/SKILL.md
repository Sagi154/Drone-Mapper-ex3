---
name: advcpp-rubric-review
description: Walks the AdvCpp rubric (e01–e23) and the AdvCpp Review Guideline against the actual codebase, dispatching one explore subagent per code group, and reports findings as a table of judgment calls (not boolean PASS/FAIL). Use before submission to surface code-quality deductions before a grader does, or after major refactors to re-check rubric alignment.
---

# AdvCpp Rubric Review

**This skill produces judgment calls, not a pass/fail script.** The AdvCpp rubric (`e01`–`e23`)
awards points based on a reviewer's reading of the code. A subagent can identify probable
deductions and flag suspicious patterns, but it cannot replace a human code-review pass.
Read the findings table as "things worth a second look," not as a deterministic test result.

Source material (check all three exist before starting):

| File | Role |
|------|------|
| `docs/review-error-codes.md` | Condensed `e01`–`e23` + `b01`–`b08` table with severity weights |
| `context/AdvCpp Review Guideline.docx` | Qualitative happy-flow / code-quality checklist (no `e*` codes in the doc itself) |
| `context/Error Code Key.xlsx` | Staff original — use `docs/review-error-codes.md` only after confirming it matches |

> **`b*` codes are run-time deduction labels, not Assignment-3 prose requirements.**
> `b05` (timeout), `b04` (scenario error), `b06` (missing input handling), etc. appear in the
> review spreadsheet and inform how graders score a run — they are **not** additional functional
> requirements beyond what `docs/assignment3-checklist.md` already states. Do not promote `b*`
> codes into hard requirements or use them as test-assertion targets. Use them only as a reminder
> of grading consequences when discussing a bug or missing feature.

## When to invoke

- Pre-submission final review pass (run after `pre-submission-review`).
- After a major refactor or port from ex2 (e.g. `port-ex2-component` skill).
- When a grader's feedback references a specific `e*` code and you need to locate the offending site.

**Not this skill:** zip structure / ID naming (use `pre-submission-review`); frozen-header check
(use `verify-frozen-interfaces`); extracting new requirements from `context/` (use
`gather-instructor-context`); Known Issues rows (use `populate-known-issues`).

## Code groups for parallel dispatch

Dispatch one `explore` subagent per group in **one** `Task` batch. Each agent gets a focused
scope so it can be thorough without being overwhelmed.

| Group | Codes | Scope hint |
|-------|-------|------------|
| A — Header / API design | `e01` `e02` `e06` `e07` `e08` `e17` | `include/`, `common/`, published headers; `#include` graph |
| B — C++ idiom / stdlib | `e03` `e04` `e13` `e16` `e22` `e23` | All `.cpp` + `.h` in `Simulator/src/`, `Algorithm/src/`, `MissionControl/src/`, `UserCommon/` |
| C — Structure / flow | `e05` `e09` `e10` `e11` `e21` | All shippable sources; also CMake for `e11` |
| D — HLD alignment | `e14` `e15` | `docs/HLD.md` or `HLD.pdf` vs. actual class declarations and sequence in source |

> **Do not use `new`/`delete` grep here** — that deterministic check lives in
> `pre-submission-review` §5a. The focus here is qualitative judgment (rule of 3/5, single-
> responsibility, const-correctness, short functions, lean APIs, …).

### Prompt skeleton for each subagent (fill the bracketed parts)

```text
You are reviewing code for TAU AdvCpp Assignment 3 against the rubric codes [GROUP CODES].
Project root: c:\Users\sagi1\Projects\DroneMapper\Drone-Mapper-ex3
Scope: [SCOPE PATHS].

Rubric definitions (from docs/review-error-codes.md):
[PASTE THE RELEVANT ROWS FROM THE TABLE]

For each code in the group:
1. Search the scoped paths for probable violations.
2. Report every suspect site as: code | file:line | severity (m/n/s per the table) | one-line description | suggested fix.
3. If no violations found, write "no obvious violations."
4. Do NOT emit PASS/FAIL — these are judgment calls.
5. Do NOT look outside the scoped paths unless a header is #included from there.

Read-only. Do not edit files.
```

## Output format

After agents return, compile their findings into **one table** in the session:

```markdown
| Code | File : Line | Severity | Finding | Suggested fix |
|------|------------|----------|---------|---------------|
| e09  | Simulator/src/RunMatrixOrchestrator.cpp:183 | n | Function `runAll` is 120 lines | Split run-dispatch from aggregation |
| …    | …           | …        | …       | …             |
```

Severity abbreviations: **m** = minor, **n** = normal, **s** = severe (matches
`docs/review-error-codes.md` column headers).

If the table is empty (no findings across all groups), note that explicitly — do not silently
produce an empty table.

## HLD alignment check (`e14` / `e15`)

Group D deserves extra detail because HLD drift is hard to spot programmatically:

- **`e14`** — class diagram incomplete: every concrete class we ship (`SimulationRun*`,
  `RunMatrixOrchestrator*`, `DroneControlImpl`, `MappingAlgorithmImpl`, `MissionControlImpl`,
  `Map3DImpl`, `MapsComparison`, `*Factory*`, …) should appear in the diagram with its key
  relationships (inheritance, composition, dependency injection via the struct types).
- **`e15`** — sequence diagram incomplete: the full call chain for one comparative run (CLI parse
  → plugin load → run matrix → per-run mission loop → score → report YAML) should be traceable
  from the diagrams.
- The agent should list classes/sequences present in code but absent from the HLD and vice versa.

Cross-reference: `docs/open-questions.md` entry on HLD format if one exists; the HLD should be a
PDF in the submission root per `docs/assignment3-checklist.md` §6 and `pre-submission-review` §6.

## Known Issues cross-reference

If this review surfaces deductions we will **not fix** before the deadline, add a row to
`docs/known-issues.md` using the `populate-known-issues` skill. Common candidates:

- `e09` long functions we don't have time to split.
- `e23` magic numbers that would take structural changes to eliminate.
- `e10` duplication inherited from ex2 and deliberately kept.

See `docs/known-issues-guidelines.md` and `.cursor/skills/populate-known-issues/SKILL.md`
for column schema and filing rules.

## AdvCpp Review Guideline items (qualitative)

These come from `context/AdvCpp Review Guideline.docx` and are not mapped to individual `e*`
codes but still drive grader judgment. The Group B/C agents should also flag these:

- Happy flow completes on all default scenarios, **including from a different working directory**
  and after minor valid config edits (`b07`/`b08` risk if paths are hardcoded).
- Invalid config / input → graceful exit, not a crash.
- Coherent structure; clear separation of responsibilities; acyclic `#include` DAG.
- `const`-correctness, maximal encapsulation, lean classes; helpers without class state in
  anonymous namespace in `.cpp` (not exposed in a header).
- Rule of 3/5 for any class owning a raw resource (rare after no-`new` rule, but check).
- No magic numbers (`e23`); no expensive recomputation (`e21`); correct container choices (`e04`);
  short single-responsibility functions (`e09`).
- `HLD.pdf` and `README.md` accurately reflect the code (`e14`/`e15` + guideline prose).
- `bonus.txt` present only if a bonus is actually claimed.

## After the review

1. Fix any **severe** (`s`) findings before submission if at all feasible.
2. For anything not fixed: file a Known Issues row (`populate-known-issues`).
3. Re-run `pre-submission-review` to catch any regressions introduced during the fix.
4. Update `docs/HLD.md` / `HLD.pdf` if Group D finds drift (use the HLD section of
   `docs/superpowers/plans/` or ask the user to regenerate diagrams).
