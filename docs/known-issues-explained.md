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

---

## The actual bug (#13)

**`large_out` short-lidar scores got worse after plan-batching.** Medium
severity, always reproducible on Release serial timing of the `large_out`
group.

What happened:

- The mapper queues several “runner-up” plans and pops them later.
- Those queued plans still assume the **old start pose**. After the drone has
  moved, the plan is stale.
- An observed-progress stall then fires, so missions **end early** (4100 /
  2200 steps instead of ~10k).
- Short-lidar cells are hit hard: small+short **44.56** (was 63.59),
  large+short **32.00** (was 85.58).
- Long-lidar held or improved (small+long 82.27, large+long 74.01).
- Wall times for all four `large_out` cells stay ≤41s, so this is a **score**
  problem, not a timeout.

A fix that repath’d on every queue pop restored some large+short score
(32→56) but wrecked small+long (82→34) and pushed small+short wall to 102s
(grader-risk). That was reverted. Shipped behavior: popping a queued plan does
**not** count as a low-rate replan, and there is no Dijkstra per pop.

This is the only row that is a real remaining defect you might still want to
fix for the mapping-algorithm track. The rest are documentation of skipped
optional work and intentional design.

---

## How to read this as a student

- **#1–#11:** “We did the mandatory path; we did not implement optional/bonus
  recovery or lazy `.so` loading.” If one of these is later implemented, drop
  the row and list it in `bonus.txt` with file:line. Do not claim a nearby
  workaround that does not match the PDF (e.g. Continue on Movement `false` is
  not CI7).
- **#12:** Unmapped (different design, not `bonus.txt`).
- **#13:** “Short-lidar `large_out` (and small+short) still underperform
  because batched plans go stale.” Fixing it helps the algorithm contest; it
  is not an extra-credit line in `bonus.txt`.
