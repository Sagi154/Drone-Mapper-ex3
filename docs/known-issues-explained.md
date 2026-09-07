# Known Issues — explanation

Plain-language walkthrough of the remaining rows in `docs/known-issues.md`.
That file is the working list for the optional staff Known Issues excel; this
doc is for us, not for the zip.

Resolved items were already pruned. Rows **#1–#6** are **skipped optional
recovery** (or bonus) from the Common-issues PDF, not mandatory bugs.
Rows **#7–#13** are deferred AdvCpp rubric nits from the 2026-09-06 findings
plan (`docs/superpowers/plans/2026-09-06-advcpp-rubric-findings-fix.md`) plus
the same-day re-verify; they are code-quality leftovers, not runtime failures. CI9 (step `Error`: log and
continue), CI2 (ignore world/map OOB), CI10 (clamp to mission bounds), CI3
(retry invalid `nextStep` then throw), and CI8 (split oversize into in-limit
fragments) are implemented and claimed in `bonus.txt`; they are not remaining
rows. Unmapped-as-passable and mapping-track band gaps are not listed.

## What the list is for

The course lets you submit a Known Issues spreadsheet. It is optional and
grade-neutral-or-better: you document things you did not implement, things you
implemented differently, and remaining bugs so graders do not treat them as
surprises. The markdown table is the working copy; it is **not** submitted
as-is. At zip time it gets copied into the staff sheet and exported as `.xlsx`.

If you later **implement** another optional / bonus row (#1–#6), do not leave
it here. Remove it from `docs/known-issues.md` and **append** the claim in
`bonus.txt` (what you did, plus file:line). Staff will not infer extra credit
from the code alone. Skip `bonus.txt` for #7–#13 (deferred rubric nits); those
are not bonuses. CI9, CI2,
CI10, CI3, and CI8 are already claimed in zip-root `bonus.txt` — keep that
file in the submission.

---

## Implemented optional rows

- **CI9 — Drone step `Error`:** log `DRONE_STEP_FAILED` and continue the
  `max_steps` loop (`runMissionSteps` in `MissionControlImpl.cpp`). Claimed in
  `bonus.txt`. Not a remaining Known Issues row.
- **CI2 — ignore world/map OOB:** `applyMovement` drops Advance/Elevate whose
  predicted center is outside `output_map_.isInBounds` (`predictedDestination`
  in `DroneControlImpl.cpp`). No Movement call. Hover/Rotate skip the check.
- **CI10 — clamp to mission bounds:** when `mission_bounds` is set (not
  all-zero), `clampMovementToMissionBounds` shortens Advance/Elevate so the
  predicted center stays inside that AABB. Zero leftover distance is treated
  as ignore. **Order:** clamp first, then CI2 ignore if the center is still
  world-OOB (mission AABB larger than the map).
- **CI3 — invalid command retry then throw:** `step()` re-calls `nextStep` up
  to `kMaxInvalidCommandRetries` when the movement type is unsupported.
  After N failures it throws out of `DroneControlImpl` (propagates; not caught
  on the wall-collision path). `SimulationRunImpl::run` maps that to
  `MISSION_EXCEPTION`. Oversize is not invalid (CI8 splits it).
- **CI8 — split oversize movement:** `splitWithinLimits` breaks oversize
  Advance/Elevate/Rotate into in-limit fragments queued on
  `pending_movements_`. The original oversize command gets one `nextStep`;
  later fragment `step()`s drain the queue and skip `nextStep` (and skip an
  extra scan). That is an intentional exception to “one `nextStep` per
  `step()`” and only runs when the algorithm exceeded drone max.

---

## Optional Common-issues rows (#1–#5)

The staff PDF lists recovery behaviors for bad algorithm/sensor/movement cases.
Two rows are **mandatory** (your own algorithm never emits a move it knew was
illegal; `MockMovement` throws on a wall). Those are done. `DroneControl`
catches the wall throw and Continues;
`SimulationRun` still catches other exceptions (CI3), not wall throws. CI9,
CI2, CI10, CI3, and CI8 are implemented (see above).

These five are **optional** and still skipped. You log them so a grader who
expects the PDF’s extra recovery does not mark them as forgotten bugs. All are
**Low**. All are **Always** reproducible. Reason: lack of time. Numbers match
`docs/known-issues.md` after the CI8 compact.

| # | What the PDF wanted | What you actually do |
|---|---------------------|----------------------|
| **1 (CI4)** | Empty move **and** empty scan (NOOP): retry N times, then throw | No NOOP retry; the mission just continues or ends |
| **2 (CI6)** | Empty LiDAR result: re-scan N times, then throw | No empty-scan retry |
| **3 (CI7)** | Movement returns `false` (not a wall throw): retry N, then throw | Any `success==false` becomes Continue (no string match, no Error). No N-retry-then-throw. Distinct from mandatory CI5 (the throw path) |
| **4 (CI11)** | GPS reports OOB: compare to internal pose; ignore or throw | No compare/ignore/throw path |
| **5 (CI12)** | After a successful move, impossible GPS: re-read N times, then Error | No GPS retry loop |

In short: `DroneControlImpl` still skips the PDF’s remaining retry machinery
for empty NOOP, empty LiDAR, Movement `false`, and GPS recoveries. That is a
conscious skip, not a mystery failure. MissionControl implements CI9
continue-on-`Error`. `applyMovement` implements CI10 clamp then CI2 ignore (no
Movement call) for world-OOB Advance/Elevate. `step()` implements CI3
invalid-type retry then throw before movement, and CI8 split-oversize via
`pending_movements_`.

---

## Bonus skip (#6)

**#6 — eager plugin load (Low, Feature/Missing)**  
Lazy load/unload of `.so` files is an explicit **bonus**. You load every
required plugin on the main thread before the run matrix, and `dlclose` at
shutdown. The assignment allows that. Do not claim the bonus in `bonus.txt`.

---

## How to read this as a student

- **CI9:** Implemented. Log `DRONE_STEP_FAILED` and keep the `max_steps` loop.
  Claimed in `bonus.txt`.
- **CI2 / CI10:** Implemented. Clamp Advance/Elevate to `mission_bounds` when
  set, then ignore if the predicted center is still world-OOB
  (`output_map_.isInBounds`). Claimed in `bonus.txt`.
- **CI3:** Implemented. Retry `nextStep` up to `kMaxInvalidCommandRetries`, then
  throw. `DroneControl` propagates; `SimulationRun` logs `MISSION_EXCEPTION`.
  Claimed in `bonus.txt`.
- **CI8:** Implemented. `splitWithinLimits` queues in-limit fragments on
  `pending_movements_`. One `nextStep` per original oversize command; fragment
  steps skip `nextStep`. Intentional exception to “one `nextStep` per `step()`”,
  only when the algorithm exceeded drone max. Claimed in `bonus.txt`.
- **#1–#6:** “We did the mandatory path; we did not implement the remaining
  optional/bonus recovery or lazy `.so` loading.” If one of these is later
  implemented, drop the row and list it in `bonus.txt` with file:line. Do not
  claim a nearby workaround that does not match the PDF (e.g. Continue on
  Movement `false` is not CI7).
- **#7–#13:** Deferred AdvCpp rubric leftovers (libm unwraps, `double` APIs,
  raw pointers, leaky `detail` types, remaining e21 walks, remaining e10
  duplication, leftover magic numbers). Listed so
  graders see we looked and chose not to touch the hot path. Not `bonus.txt`.

---

## Deferred AdvCpp rubric leftovers (#7–#13)

These came from `docs/advcpp-rubric-review.md` and were **consciously skipped**
by the 2026-09-06 fix plan (Tasks 1–16 landed the cheap, gated sites). The
2026-09-06 re-verify confirmed those leftovers and leftover e23 literals.
The 2026-09-07 HLD pass closed the e15/e14 leftover (comparative sequence now
routes expand/distribute through `runPluginMatrix`; class diagram includes
`runMissionSteps`, `ConfigParseResult`, `TimeFormat`, `BeamMath`, `LidarCone`).
Type is `Code`. Reproducibility is **Not relevant**.

| # | Rubric | What is still true |
|---|--------|--------------------|
| **7 (e03)** | Hot-path `std::sin`/`std::cos`/`sqrt` unwraps | Tasks 9–10 converted MockMovement limit checks and output-map resolution. MockLidar ring offsets now use `si::sin`/`si::cos`. Remaining trig (including `DroneControlImpl::predictedDestination`) stays libm so the 24-cell scores do not drift. |
| **8 (e16)** | `double` cm/radian APIs on cone/frontier/planner | Cosmetic type-safety; blast radius is the score table. |
| **9 (e13)** | Raw `const T*` / out-params (`MatrixCell`, `WavefrontPlanner::plan`, …) | Structural API change, not a mechanical rename. |
| **10 (e22)** | `ConeTemplateCache::get` returns `const vector&`; `detail` types in ScanPlanning/ExplorationPlan headers | Same class of invasive header change. |
| **11 (e21)** | String-pull re-walk, `VoxelStamp::mark` re-quantize, `findPathTo` without the `runBoundedSearch` memo, plus extra double walks | Task 13 only fused the scan-supplement walk in `ScanResultToVoxels`. |
| **12 (e10)** | LidarCone/ConeTemplate walk overlap, MockLidar ring rebuild, PluginLoader directory loops, plus bind/CLI/`isUnsetBoundaries` copies | Tasks 6–8 shared load-one, `wrapDeg`, and `keyToPoint` only. |
| **13 (e23)** | Leftover magic numbers after Task 14 | One row covers all remaining literals (`+ 2.0`, `0.5`, `1e-9`, `1.0` cm, `360.0 * deg`). |
