# Known Issues — explanation

Plain-language walkthrough of the remaining rows in `docs/known-issues.md`.
That file is the working list for the optional staff Known Issues excel; this
doc is for us, not for the zip.

Resolved items were already pruned. Almost all remaining rows are **skipped
optional recovery** from the Common-issues PDF, not mandatory bugs. The one
real behavioral problem is **#13**.

## What the list is for

The course lets you submit a Known Issues spreadsheet. It is optional and
grade-neutral-or-better: you document things you did not implement, things you
implemented differently, and remaining bugs so graders do not treat them as
surprises. The markdown table is the working copy; it is **not** submitted
as-is. At zip time it gets copied into the staff sheet and exported as `.xlsx`.

If you later **implement** an optional / bonus row (#1–#11), do not leave it
here. Remove it from `docs/known-issues.md` and **claim it in `bonus.txt`**
(what you did, plus file:line). Staff will not infer extra credit from the
code alone. Skip `bonus.txt` for #12 (Unmapped, different design) and #13 (bug);
those are not bonuses. If nothing optional was implemented, do not add
`bonus.txt`.

---

## Optional Common-issues rows (#1–#10)

The staff PDF lists recovery behaviors for bad algorithm/sensor/movement cases.
Two rows are **mandatory** (your own algorithm never emits a move it knew was
illegal; `MockMovement` throws on a wall and `DroneControl`/`SimulationRun`
catch it). Those are done.

These ten are **optional**. You log them so a grader who expects the PDF’s extra
recovery does not mark them as forgotten bugs. All are **Low** except #6
(**Medium**). All are **Always** reproducible. Reason: lack of time.

| # | What the PDF wanted | What you actually do |
|---|---------------------|----------------------|
| **1 (CI2)** | Ignore a move that would leave the **world/map** | The command is forwarded to Movement anyway |
| **2 (CI3)** | Invalid command: retry `nextStep` N times, then throw | First bad command → `DroneStepStatus::Error`, no retry |
| **3 (CI4)** | Empty move **and** empty scan (NOOP): retry N times, then throw | No NOOP retry; the mission just continues or ends |
| **4 (CI6)** | Empty LiDAR result: re-scan N times, then throw | No empty-scan retry |
| **5 (CI7)** | Movement returns `false` (not a wall throw): retry N, then throw | Some “blocked/boundary” cases become Continue; otherwise Error. No N-retry-then-throw. Distinct from mandatory CI5 (the throw path) |
| **6 (CI8)** | Oversize Advance/Elevate/Rotate: **split** into several legal steps | Rejected immediately as Error (`movementWithinLimits`) |
| **7 (CI9)** | Step `Error`: log and **keep** the `max_steps` loop | Log `DRONE_STEP_FAILED` and **stop** the mission |
| **8 (CI10)** | Move that would leave **mission** bounds: clamp/shorten it | Not amended. Different from #1 (world OOB ignore vs mission-bounds clamp) |
| **9 (CI11)** | GPS reports OOB: compare to internal pose; ignore or throw | No compare/ignore/throw path |
| **10 (CI12)** | After a successful move, impossible GPS: re-read N times, then Error | No GPS retry loop |

In short: `DroneControlImpl` is a fairly strict “one command, one outcome”
path. It does not implement the PDF’s retry/ignore/split/continue machinery.
That is a conscious skip, not a mystery failure.

#6 is Medium because oversize commands are a realistic algorithm mistake, and
splitting them would have been the most useful optional recovery.

---

## Bonus / different design (#11–#12)

**#11 — eager plugin load (Low, Feature/Missing)**  
Lazy load/unload of `.so` files is an explicit **bonus**. You load every
required plugin on the main thread before the run matrix, and `dlclose` at
shutdown. The assignment allows that. Do not claim the bonus in `bonus.txt`.

**#12 — Unmapped cells are walkable (Low, developed differently)**  
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

## The remaining mapping-track gaps (#13)

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

- **#1–#11:** “We did the mandatory path; we did not implement optional/bonus
  recovery or lazy `.so` loading.” If one of these is later implemented, drop
  the row and list it in `bonus.txt` with file:line. Do not claim a nearby
  workaround that does not match the PDF (e.g. Continue on Movement `false` is
  not CI7).
- **#12:** Unmapped (different design, not `bonus.txt`).
- **#13:** “`house_full` / `small_out` large+long / `small_room` large+short still
  underperform vs ex2 bands.” Fixing it helps the algorithm contest; it
  is not an extra-credit line in `bonus.txt`. The 2026-09-03 `large_out` short
  cliff is no longer the measured baseline.
