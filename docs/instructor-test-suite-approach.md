# Instructor-style test suite — approach

Advice captured 2026-08-26; **status updated 2026-08-28.** Do not treat this as a requirement
catalog; it explains how to build a black-box suite that mimics how a grader would probe the built
binary. Phase A (blind catalog) and Phase B (harness + skills) are **done** — see
`docs/instructor-test-catalog-followup-roadmap.md` and
`.cursor/skills/verify-instructor-test-catalog/SKILL.md`.

Authoritative instructor sources still win: `context/Advanced Topics TAU 2026B - Assignment 3.docx`,
`context/Common issues and handling.pdf`, AdvCpp guideline + Error Code Key. Condensed
`docs/*.md` may already encode our working assumptions.

Related: `docs/assignment-compliance-pickup.md`, `docs/open-questions.md`,
`docs/assignment3-checklist.md`, `docs/simulator_runtime_test_catalog.md`.

---

## What an "instructor test suite" can even mean here

There is no published Ex3 review guideline or bug-injection scheme (`docs/open-questions.md` #6 —
the Assignment 3 docx never mentions tests). So there is no real artifact to "impersonate" — we
would be **inferring** a black-box grading harness from three sources, in this priority order:

1. `Assignment 3.docx` — CLI contract, output/report schema, threading rules, error-handling
   requirements ("Argument rules… all of these are graded behaviors").
2. `Common issues and handling.pdf` — the mandatory/optional fault table (who detects, who
   handles).
3. `AdvCpp Review Guideline.docx` + `Error Code Key.xlsx` — but this part is **manual code
   review** (RAII, magic numbers, const-correctness, `e*`/`b*` codes), not something a runtime
   test suite can grade. Don't try to force that into pass/fail tests — treat it as a separate
   static/manual checklist.

So the real deliverable is closer to "a black-box CLI/contract compliance suite that mimics how a
grader would probe the built binary," not a literal reproduction of a secret instructor script.

---

## Is a blind, implementation-unaware agent the right way to build it?

Partially yes, but the isolation boundary matters — split it into two phases:

### Phase A — spec-only, blind agent (this is where isolation earns its keep)

Give it only:

- `context/` (docx/pdf, **not** the `docs/` condensations, which already carry our
  working-assumption bias — e.g. sum-based scoring, `(total_score, total_steps)` grouping)
- `ex_3_skeleton` (published headers + skeleton README)

Its job: produce an exhaustive, objective **test catalog** — every mechanically-observable
requirement, phrased as "given this CLI invocation / input, assert this observable output,"
explicitly separating mandatory vs. optional vs. genuinely-unspecified (don't let it invent a
scoring formula the docx never gives).

This is exactly the failure mode isolation prevents: an agent that's seen our code tends to write
tests that confirm what we already built rather than what the spec actually demands, and it'll
unconsciously encode our own working assumptions (from `docs/open-questions.md`) as if they were
requirements.

`ex_3_skeleton` is a sibling folder to this repo and already ships its own `context/` copy, so a
Cursor window opened at `ex_3_skeleton` root is naturally, physically isolated — no exclusion
rules needed, it simply cannot see this repo.

**Source priority for that isolated agent** (revised from the first draft): the skeleton is
ground truth for anything it actually defines — published header signatures under `common/`,
`Simulator/common_simulator/`, `MissionControl/common_mission_control/`, plus its root
`CMakeLists.txt` / `CMakePresets.json` / `README.md` (build structure, artifact naming pattern).
The docx governs everything the skeleton does **not** encode: CLI argument semantics,
output/report YAML formats, threading rules, error-handling requirements, submission zip
contents. When both cover the same thing and disagree, the skeleton wins and the agent should
flag the conflict rather than silently preferring the docx's prose.

**The catalog targets a built submission zip, not the skeleton repo.** The agent must be told
explicitly that the eventual test target is an unzipped, built **submission** (5 folders, 4 build
files, `students.txt`, `README.md` — a filled-in copy of this skeleton), not `ex_3_skeleton`
itself, and that it doesn't know real student IDs — every test invocation and filename in the
catalog must use `<ids>` / `<algorithm_so>` / `<mission_control_so>` placeholders exactly as the
assignment doc itself does, never a concrete name. Naming/ID wiring is a Phase B concern.

**No `.cursor/skills` exist inside `ex_3_skeleton`** (confirmed empty), so the isolated agent
cannot invoke this repo's `gather-instructor-context` skill by name. Inline the equivalent
instruction directly in the prompt instead: dispatch one extraction subagent per source document
(or per short PDF pair) in parallel, each returning a structured partial catalog with quotes and
IDs, then synthesize into the single final document. Don't invoke the brainstorming skill for this
— it's a fully-specified, deterministic extraction task, not ambiguous creative work needing
back-and-forth.

#### Phase A prompt (paste into the isolated `ex_3_skeleton` window)

```text
You are extracting a black-box grading test catalog for a course assignment. Do NOT write or
modify any code. Do NOT look outside this workspace folder — everything you need is here.

IMPORTANT CONTEXT: this catalog will eventually be used to test a graded SUBMISSION ZIP, not this
skeleton repo. A submission is this skeleton's structure filled in by some team: 5 folders
(Simulator/, Algorithm/, MissionControl/, common/ as-is, UserCommon/), 4 build files, students.txt,
README.md, unzipped and built with the submitting team's own build system, producing an executable
named simulator_<their_ids> that dlopens Algorithm_<their_ids>.so and MissionControl_<their_ids>.so
from wherever the CLI points. You do not know real IDs — describe every test invocation and
filename generically using <ids> / <algorithm_so> / <mission_control_so> placeholders exactly as
the assignment doc itself does, never hardcode a concrete name.

Sources and priority:
1. HIGHEST for anything they actually define: the header files under common/,
   Simulator/common_simulator/, MissionControl/common_mission_control/ (published, frozen
   interfaces every team builds against as-is), plus this skeleton's root CMakeLists.txt,
   CMakePresets.json, and README.md (build structure, executable/artifact naming pattern,
   folder layout). These are the actual compiled/build-time ground truth, more reliable than
   the docx's prose for anything they cover.
2. context/Advanced Topics TAU 2026B - Assignment 3.docx — governs everything the skeleton does
   NOT encode: CLI argument semantics, output/report YAML formats, threading rules, error-handling
   requirements, submission zip contents. This is a zip; extract word/document.xml text rather
   than trusting any summary. It was in DRAFT mode and its last section is literally "[TBD]" —
   note that.
3. context/Common issues and handling.pdf — mandatory vs optional runtime fault table.
4. context/Structuring the project.pdf — where mocks/registration live, project boundaries.

When the skeleton and the docx conflict on something the skeleton actually defines (e.g. exact
header signatures, namespace of published interfaces), the skeleton wins and you should flag the
conflict rather than silently picking the docx's wording.

Ignore: Assignment 1/2 docx, Exercise 2 Review Guideline.docx, Submission Guidelines docx (it
describes a different, older exercise — flag anything relevant from it separately, don't merge it
in), AdvCpp Review Guideline.docx / Error Code Key.xlsx (that's manual human code review, not
something a runtime test can check — list it in a separate "not testable by a runtime suite"
section instead of skipping it silently).

Approach: given the number of source documents, dispatch one extraction subagent per document (or
per short PDF pair, e.g. Common issues + Structuring the project together) in parallel, in the
same turn. Each subagent gets the absolute path to its one document and returns a structured
partial catalog (quotes, tentative IDs, classification) without looking at anything else. Then
synthesize all partial catalogs yourself into the single final markdown document below. Do not
dispatch subagents to write or modify code, and do not let them look outside this workspace.

Task: produce an exhaustive TEST CATALOG for the Simulator's runtime, CLI, and file-output
behavior — the parts a grader could check by unzipping a submission, building it, running the
resulting executable, and inspecting its output, without reading that team's source code. For
every requirement you find that is objectively, mechanically observable this way, write one
catalog entry with:

- ID (e.g. CLI-01, OUT-03, ERR-02, THREAD-01, PLUGIN-01)
- Verbatim or near-verbatim quote of the requirement from its source
- Exact test setup: what CLI invocation / input files / folder state to construct (using <ids>
  placeholders, never concrete names)
- Exact expected observable outcome: stdout/stderr content expectations, exit behavior, files
  created (names, locations, formats), YAML field names and structure, sort order, log line
  format — quote field names and CLI flag syntax verbatim, don't paraphrase them
- Classification: MANDATORY / OPTIONAL (bracketed args, "may") / BONUS / GENUINELY UNSPECIFIED
  (the doc requires an outcome but never defines the exact metric/rule — e.g. scoring formula,
  what makes two mission control results "the same") — for unspecified ones, describe only the
  observable SHAPE/CONTRACT you can test (e.g. "results_summary is sorted descending by group
  size" — testable) without inventing the missing definition

Also produce a short separate list of:
- Contradictions or ambiguities between sources (quote both conflicting passages, and say which
  source should win per the priority rules above)
- Anything left to "your decision" per the docx (e.g. exact usage/error text) — note a test can
  only check these loosely (e.g. "usage text is printed and mentions the missing arg name"), not
  exact wording
- Requirements that are graded but NOT testable by running a built binary — build-time/structural
  things (folder layout, no `new`/`delete`, frozen headers unchanged vs this skeleton, no binaries
  in the zip, students.txt/README present) — list separately, since checking these means
  inspecting the unzipped submission tree/source, not running it
- Requirements that only make sense once you assume a specific submission's build system exists
  (e.g. "run the root build file") — note that a generic tester needs to discover/invoke whatever
  build tooling the submission provides, since it may differ per team

Do not guess at anything the assignment doesn't specify. If unsure whether something is mandatory,
quote the exact sentence and mark it "ambiguous" rather than picking a side.

Output as a single markdown document with the sections: Mandatory CLI/runtime behaviors, Optional/
bonus behaviors, Genuinely unspecified (shape-only tests), Static/structural checks on the unzipped
submission (not runtime-testable), Not testable by any automated suite (manual review items),
Contradictions/ambiguities found across sources.
```

### Phase B — wiring pass, necessarily implementation-aware

**Status 2026-08-28:** done — see `docs/instructor-test-catalog-followup-roadmap.md` Points 2–4
and `Simulator/tests/manual/run_all.sh`.

Isolation can't extend to *execution*: someone has to know our binary is
`simulator_207190406_209543255`, our `.so` names, our CMake preset, and where `build/` outputs
land. That pass takes Phase A's catalog and turns it into a runnable harness (shell/pytest driving
the real executable, inspecting real YAML/files).

This phase should stay mechanical — don't let it "soften" assertions to match what our code
currently does; if a real gap shows up, that's the point.

---

## What's actually testable black-box (high value)

Matches "graded behaviors" in the docx:

- **CLI arg validation:** any order, `=` with no spaces, missing args (lists *all* missing),
  unsupported args (lists *all* unsupported), missing/unopenable file,
  missing/untraversable/empty-of-right-kind folder — each should print usage + error and exit
  cleanly (no crash, no `exit()`/`abort()`).
- **`num_threads`:** absent/`1` → behaves as single-threaded; `>=2` → doesn't silently ignore it
  (exact thread-count assertions are hard from outside, but at least check results stay correct
  and nothing hangs/deadlocks under `num_threads=8` etc.).
- **Output dir naming:** `comparative_results_<time>` / `competition_<time>` freshly generated,
  no collision on repeat runs.
- **Report YAML shape:** `comparative_report` / `competitive_report` field names,
  `results_summary` sort order (group size descending / score desc-then-steps-asc),
  `errors: [...]` populated for unloadable plugins (this is exactly gap #3 in
  `docs/assignment-compliance-pickup.md` — a real instructor test would catch it).
- **Per-plugin Simulation Result Output File:** YAML present per mission control / per algorithm,
  named to include the plugin name.
- **`error.log`:** format and immediacy (can fake a bad `.npy` or bad composition entry and check
  the log line appears even if the run keeps going).
- **`-verbose` on/off:** verbose files appear only when the flag is set (another flagged gap).
- **Mandatory Common-issues pair:** feed a real map with a wall in the drone's path and confirm it
  doesn't crash and the mission ends sanely (algorithm shouldn't fly into a *known* obstacle; the
  mock-movement wall-collision throw should be caught, not propagate).
- **Algorithm minimum bar:** run against a small known map, assert no wall collisions and some
  minimum mapping coverage — close to "instructor grades the algorithm," worth having even though
  the docx gives no numeric bar.
- **Mechanical build/structure checks** (static, not runtime): 5 folders, 4 build files, no
  `common/` additions, no binaries in the tree, compiler flags, `students.txt` filled, frozen
  headers byte-identical to skeleton.

---

## What's *not* worth building

- Anything that asserts a specific score/points value — the metric is explicitly unspecified
  (open questions #3/#4); test the **shape and ordering contract**, not a number.
- Ex2-style bug-injection ceilings/coverage bars — nothing in the docx asks for it, and
  `AGENTS.md` already warns against inventing that ceremony.
- Automated stand-ins for the AdvCpp manual rubric (`e*`/`b*` codes) — that's a human/static-
  analysis pass, not a test suite concern.

---

## Cross-team plugin test (maps to "another team's implementation")

The docx says each part must work with **another team's** implementation of the others. The
strongest, most instructor-like test isn't just CLI-level — it's a minimal harness that:

- links only against `common/` (published, frozen), and
- `dlopen`s *just* our `Algorithm_<ids>.so` or `MissionControl_<ids>.so` against a deliberately
  independent counterpart (a tiny reference/mock Simulator+MissionControl or Simulator+Algorithm,
  built without ever reading our source),

rather than always testing through our own full Simulator. That's the cheapest way to catch "it
only works because it secretly assumes our own MissionControl/Algorithm" bugs — arguably closer to
what a real grader running your `.so` against *their* simulator would hit.

---

## Recommendation (executed)

1. ~~Dispatch a **blind agent** (only `context/` + `ex_3_skeleton`) to produce the test catalog.~~
   **Done** — `docs/simulator_runtime_test_catalog.md` (also mirrored under `ex_3_skeleton/`).
2. ~~Human sanity-check against `docs/open-questions.md`.~~ **Done** — follow-up Point 1 added
   UNSPEC-07 as `docs/open-questions.md` §9; remaining unspecified rows stay AMBIGUOUS in reports.
3. ~~Wire Phase B against the real build.~~ **Done 2026-08-28** — catalog gaps closed in
   `Simulator/tests/manual/` (plan:
   `docs/superpowers/plans/2026-08-28-close-instructor-test-catalog-gaps.md`); skills
   `pre-submission-review` (extended), `advcpp-rubric-review`, and orchestrator
   `verify-instructor-test-catalog`. Cross-team independence variants are a **separate** follow-on
   (`docs/superpowers/specs/2026-08-28-independent-component-variants-design.md`) — catalog
   roadmap gate cleared.
