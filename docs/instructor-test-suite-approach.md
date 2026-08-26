# Instructor-style test suite — approach (no implementation yet)

Advice captured 2026-08-26. Do not treat this as a requirement catalog; it is a plan for how to
build a black-box suite that mimics how a grader would probe the built binary.

Authoritative instructor sources still win: `context/Advanced Topics TAU 2026B - Assignment 3.docx`,
`context/Common issues and handling.pdf`, AdvCpp guideline + Error Code Key. Condensed
`docs/*.md` may already encode our working assumptions.

Related: `docs/assignment-compliance-pickup.md`, `docs/open-questions.md`,
`docs/assignment3-checklist.md`.

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

### Phase B — wiring pass, necessarily implementation-aware

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

## Recommendation

1. Dispatch a **blind agent** (only `context/` + `ex_3_skeleton`) to produce the test catalog as a
   reviewable doc first (e.g. `docs/instructor-test-catalog.md`).
2. Human sanity-check it against `docs/open-questions.md` for anything it over-specified.
3. **Then** wire it into a runnable suite against the real build (Phase B: harness choice, where
   it lives, CMake integration).

Do not implement the suite until the catalog is reviewed.
