# Known Issues — explanation

Plain-language walkthrough of the remaining rows in `docs/known-issues.md`.
That file is the working list for the optional staff Known Issues excel; this
doc is for us, not for the zip.

Resolved items were already pruned. Rows **#1–#10** are **skipped optional
recovery** (or bonus) from the Common-issues PDF, not mandatory bugs. **#11**
is Unmapped policy. The one remaining mapping-score problem is **#12**.
Rows **#13–#18** are deferred AdvCpp rubric nits from the 2026-09-06 findings
plan (`docs/superpowers/plans/2026-09-06-advcpp-rubric-findings-fix.md`); they
are code-quality leftovers, not runtime failures. CI9 (step `Error`: log and
continue) is implemented and claimed in `bonus.txt`; it is not a remaining row.

## What the list is for

The course lets you submit a Known Issues spreadsheet. It is optional and
grade-neutral-or-better: you document things you did not implement, things you
implemented differently, and remaining bugs so graders do not treat them as
surprises. The markdown table is the working copy; it is **not** submitted
as-is. At zip time it gets copied into the staff sheet and exported as `.xlsx`.

If you later **implement** an optional / bonus row (#1–#10), do not leave it
here. Remove it from `docs/known-issues.md` and **claim it in `bonus.txt`**
(what you did, plus file:line). Staff will not infer extra credit from the
code alone. Skip `bonus.txt` for #11 (Unmapped, different design), #12 (bug), and
#13–#18 (deferred rubric nits); those are not bonuses. CI9 is already claimed
in `bonus.txt`. If nothing optional was implemented, do not add `bonus.txt`.

---

## Implemented optional rows

- **CI9 — Drone step `Error`:** log `DRONE_STEP_FAILED` and continue the
  `max_steps` loop (`runMissionSteps` in `MissionControlImpl.cpp`). Claimed in
  `bonus.txt`. Not a remaining Known Issues row.

## Optional Common-issues rows (#1–#9)

The staff PDF lists recovery behaviors for bad algorithm/sensor/movement cases.
Two rows are **mandatory** (your own algorithm never emits a move it knew was
illegal; `MockMovement` throws on a wall and `DroneControl`/`SimulationRun`
catch it). Those are done. CI9 is implemented (see above).

These nine are **optional** and still skipped. You log them so a grader who
expects the PDF’s extra recovery does not mark them as forgotten bugs. All are
**Low** except #6 (**Medium**). All are **Always** reproducible. Reason: lack
of time. Numbers match `docs/known-issues.md` after the CI9 compact.

| # | What the PDF wanted | What you actually do |
|---|---------------------|----------------------|
| **1 (CI2)** | Ignore a move that would leave the **world/map** | The command is forwarded to Movement anyway |
| **2 (CI3)** | Invalid command: retry `nextStep` N times, then throw | First bad command → `DroneStepStatus::Error`, no retry |
| **3 (CI4)** | Empty move **and** empty scan (NOOP): retry N times, then throw | No NOOP retry; the mission just continues or ends |
| **4 (CI6)** | Empty LiDAR result: re-scan N times, then throw | No empty-scan retry |
| **5 (CI7)** | Movement returns `false` (not a wall throw): retry N, then throw | Any `success==false` becomes Continue (no string match, no Error). No N-retry-then-throw. Distinct from mandatory CI5 (the throw path) |
| **6 (CI8)** | Oversize Advance/Elevate/Rotate: **split** into several legal steps | Rejected immediately as Error (`movementWithinLimits`) |
| **7 (CI10)** | Move that would leave **mission** bounds: clamp/shorten it | Not amended. `DroneControlImpl` also dropped unused `mission_` (AdvCpp e08), so bounds are not in the controller. Different from #1 (world OOB ignore vs mission-bounds clamp) |
| **8 (CI11)** | GPS reports OOB: compare to internal pose; ignore or throw | No compare/ignore/throw path |
| **9 (CI12)** | After a successful move, impossible GPS: re-read N times, then Error | No GPS retry loop |

In short: `DroneControlImpl` is a fairly strict “one command, one outcome”
path. It does not implement the PDF’s retry/ignore/split machinery. That is a
conscious skip, not a mystery failure. MissionControl does implement CI9
continue-on-`Error`.

#6 is Medium because oversize commands are a realistic algorithm mistake, and
splitting them would have been the most useful optional recovery.

---

## Bonus / different design (#10–#11)

**#10 — eager plugin load (Low, Feature/Missing)**  
Lazy load/unload of `.so` files is an explicit **bonus**. You load every
required plugin on the main thread before the run matrix, and `dlclose` at
shutdown. The assignment allows that. Do not claim the bonus in `bonus.txt`.

**#11 — Unmapped cells are walkable (Low, developed differently)**  
Unknown voxels are **not** treated as walls. Dijkstra prefers Empty (cost 1)
over Unmapped (cost 4). If a “path through unknown” hits a hidden wall, you
recover with Continue + replan (soft stall, a couple of ticks), not
`MISSION_EXCEPTION`.

This is an Ex2 lesson: treating unknown as a hard wall stalled exploration.
Reproducibility is **Rare** because it only shows up when the planned path goes
through Unmapped that turns out to be a wall. Reason here is “lack of
knowledge” (of the right policy), not lack of time.

A 2026-09 Occupied-AABB execution gate plus shrink / floor-support / attic-first
was reverted (`backup/var01-2026-09-05`; `docs/archive/2026-09-05-var01-attempt.md`).
Independence VAR-01 (`HOST_ILLEGAL_MOVE_ATTEMPTS=0`) now PASSes via **corner-anchored**
`sphereIntersectsCellBox` plus a 3000-node local replan cap (2026-09-06, Approach A).
Do not revive the Occupied-AABB gate. Write-up:
`docs/benchmarks/2026-09-06-var01-approach-a.md`.

---

## The remaining mapping-track gaps (#12)

**Some cells still sit below the ex2 score bands** after the 2026-09-06 landing.
This is no longer the 2026-09-03 `large_out` short-lidar cliff (44.56 / 32.00 at
~46s WARN). Current serial Release baseline: score_sum **1832.7**, wall_max **3.0s**,
`cells_ge_60s=0`. `large_out` shorts are **74.95 / 72.02**.

Still short of the bands:

- `house_full` mean **25.86** (47.46 / 21.00 / 20.08 / 14.89) vs ex2 ~56–62.
- `small_out` large+long **45.38**.
- `small_room` large+short **67.65**.

Queued runner-up plans can still assume a stale start pose; the clearance +
search-cap change mostly changed *which* frontier is picked and how expensive
each replan is. Fixing remaining band gaps is mapping-track work, not
`bonus.txt`.

---

## How to read this as a student

- **CI9:** Implemented. Log `DRONE_STEP_FAILED` and keep the `max_steps` loop.
  Claimed in `bonus.txt`.
- **#1–#10:** “We did the mandatory path; we did not implement the remaining
  optional/bonus recovery or lazy `.so` loading.” If one of these is later
  implemented, drop the row and list it in `bonus.txt` with file:line. Do not
  claim a nearby workaround that does not match the PDF (e.g. Continue on
  Movement `false` is not CI7).
- **#11:** Unmapped (different design, not `bonus.txt`).
- **#12:** “`house_full` / `small_out` large+long / `small_room` large+short still
  underperform vs ex2 bands.” Fixing it helps the algorithm contest; it
  is not an extra-credit line in `bonus.txt`. The 2026-09-03 `large_out` short
  cliff is no longer the measured baseline.
- **#13–#18:** Deferred AdvCpp rubric leftovers (libm unwraps, `double` APIs,
  raw pointers, leaky `detail` types, remaining e21 walks, remaining e10
  duplication). Listed so graders see we looked and chose not to touch the
  hot path. Not `bonus.txt`.

---

## Deferred AdvCpp rubric leftovers (#13–#18)

These came from `docs/advcpp-rubric-review.md` and were **consciously skipped**
by the 2026-09-06 fix plan (Tasks 1–16 landed the cheap, gated sites). Type is
`Code`. Reproducibility is **Not relevant**.

| # | Rubric | What is still true |
|---|--------|--------------------|
| **13 (e03)** | Hot-path `std::sin`/`std::cos`/`sqrt` unwraps | Tasks 9–10 only converted MockMovement limit checks and output-map resolution. Remaining trig stays libm so the 24-cell scores do not drift. |
| **14 (e16)** | `double` cm/radian APIs on cone/frontier/planner | Cosmetic type-safety; blast radius is the score table. |
| **15 (e13)** | Raw `const T*` / out-params (`MatrixCell`, `WavefrontPlanner::plan`, …) | Structural API change, not a mechanical rename. |
| **16 (e22)** | `ConeTemplateCache::get` returns `const vector&`; `detail` types in ScanPlanning/ExplorationPlan headers | Same class of invasive header change. |
| **17 (e21)** | String-pull re-walk, `VoxelStamp::mark` re-quantize, `findPathTo` without the `runBoundedSearch` memo | Task 13 only fused the scan-supplement walk in `ScanResultToVoxels`. |
| **18 (e10)** | LidarCone/ConeTemplate walk overlap, MockLidar ring rebuild, PluginLoader directory loops | Tasks 6–8 shared load-one, `wrapDeg`, and `keyToPoint` only. |
