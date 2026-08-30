# Exploration Policy (Next-Best-View) — Design

**Date:** 2026-08-29
**Status:** Accepted 2026-08-29
**Goal:** Replace `Algorithm/`'s action policy with a budget-aware, frontier-anchored next-best-view
policy that stops terminating missions early, carries a scan on every movement step, and chooses
viewpoints by expected information per step — so the `honest` benchmark column approaches ex2's
recorded score bands instead of the 6–35% it posts on four of six scenario groups today.

This is project **D**, the last of four. See
`docs/superpowers/specs/2026-08-29-mapping-algorithm-roadmap.md` for the sequence and the two
standing decisions (honest-step accounting as the optimization target; beat-ex2 as the success bar).
Depends on A (harness), B (one scan per step, move+scan honored), C (real clearance geometry,
lidar-derived cone helpers in `UserCommon/`).

---

## Problem

Post-C measurements (`docs/benchmarks/2026-08-29-post_c_honest.csv`) show C did what it set out to do
— 17693 → 9101 steps, `MAX_STEPS` cells 8 → 2 — and barely moved the score: 1335.4 → 1331.0. The
remaining gap to ex2 is not sensor geometry. It is the action policy, and three specific properties
of it.

### 1. The algorithm quits while the budget is nearly untouched

In `handleFrontierCleanupPhase` (`Algorithm/src/MappingAlgorithmImpl.cpp:333-348`), one planning cycle
in which `findPath` and all three fallbacks (`findExplorePath`, `findUnstickPath`,
`findAnyPassableNeighbor`) come back empty sets `finished = true` and returns
`FinishedWithUnmappableVoxels`. There is no retry, no local recovery, no second look after more of the
map is known.

The cost is visible per cell. On `house_mission_full`, `max_steps` is 10000 and the four cells finish
in **27, 35, 44, and 232 steps** — all `COMPLETED` — scoring 6.31, 6.31, 7.10, 12.61 against an ex2
band of 56–62. `small_out` and `large_out` show the same shape at smaller magnitude.

Terminating early buys nothing. `SimulationRunImpl.cpp:140-148` scores any run that is not
`Error`; a `MAX_STEPS` run is scored exactly like a `COMPLETED` one, and
`Simulator/src/io/CompetitiveReportWriter.cpp:32-37` sorts by `total_score` descending with
`total_steps` only as a tiebreak. Unused budget is discarded score.

Two mechanisms make it worse. `blocked_cells` and `frontier_visit_counts` are never cleared, so the
search graph shrinks monotonically over a mission and frontiers are retired after 3 visits (5 near the
floor) no matter how much unknown space still borders them. And the `kNoProgressLimit = 100` /
`kNoFrontierStuckLimit = 100` counters are stagnation heuristics standing in for the question the
policy should be answering directly: is there any reachable viewpoint left that would observe
something new within the remaining budget?

### 2. Every travel step throws away a free scan

`MappingStepCommand` carries `movement` and `scan_orientation` independently, and since project B our
`DroneControlImpl::step()` executes movement, then at most one scan at the resulting pose
(`MissionControl/src/DroneControlImpl.cpp:190-228`). No handler in `MappingAlgorithmImpl` ever sets
both — `handleMovingPhase` returns movement, `handleScanningPhase` returns a scan. Every step spent
travelling observes nothing, and travel dominates the step count on the large scenarios.

### 3. Frontier information density is not the competitive objective

`findPath` scores frontiers by unmapped-neighbour count in a radius-2 sphere divided by path cost
(`MappingAlgorithmFrontier.cpp:407`). That is a proxy: it counts unknown voxels *near* the target
rather than the voxels the sensor would actually observe from it, ignores occlusion, ignores the
`z_min` = 20 cm dead zone and the `z_max` = 80/150 cm range, and its cost term is Dijkstra cell cost
rather than the steps the drone will actually spend. Cell cost and step cost differ substantially:
`drone_large` advances 50 cm per step but rotates only 45°, so a right-angle turn costs it two steps,
while `drone_small` advances 30 cm and turns 90° in one.

Alongside these, the roadmap catalogues correctness debt this project retires:
`explore_dist_cache` is cleared only on `findPath` success and reused exactly on the stale path;
the mid-search edge-set change at `MappingAlgorithmFrontier.cpp:428` voids Dijkstra's optimality;
`findFarthestPath` and `findGreedyUnknownStep` have no callers; and the ALG28 hang class
(`docs/ex2-grading-handoff.md:149-166`) is still live because every search is bounded only by
occupancy.

---

## Decisions taken before the design

These were settled explicitly in the design conversation. Do not re-litigate without raising it.

1. **`Unmapped` stays soft-cost traversable; the safety invariant is "never command a move whose
   drone-sphere footprint contains `Occupied` or `OutOfBounds`."** The roadmap's Project D bullet
   ("no emitted movement may enter a cell the belief map hasn't cleared") is superseded, because it
   is unachievable with the shipped sensor: lidar `z_min` is 20 cm against a 10 cm output grid, so a
   scan from the current pose physically cannot carve the adjacent cell `Empty`, and
   `markDroneFootprintEmpty` (`MissionControl/src/DroneControlImpl.cpp:24-56`) clears only the voxel
   the drone occupies at radius 4–7.5 cm. An `Empty`-only planner cannot take its first step from
   spawn. C's sphere-box clearance check plus stall-triggered replanning is the safety mechanism.
2. **Scope:** a new NBV policy module; `MappingAlgorithmFrontier` is kept as the reachability and path
   substrate with its dead helpers and never-cleared blacklists removed.
3. **mp-units:** new code is written in strong types. The frontier substrate's internal
   `force_numerical_value_in(cm)` doubles are left alone and converted by a dedicated follow-up
   project E, after D's scores are locked in, so a mechanical refactor cannot silently move the score.
4. **Candidate selection is enumerated, never sampled.** There is no RNG anywhere in D, which makes
   project A's determinism requirement structural rather than dependent on holding a seed fixed.

---

## Design

### Components

| Unit | Responsibility | Depends on |
|------|----------------|------------|
| `Algorithm/src/NbvPlanner.{h,cpp}` (new) | The policy. Given map, pose, configs and remaining budget, returns an `ExplorationPlan`. | frontier substrate, `UserCommon` cone helpers |
| `Algorithm/src/MappingAlgorithmImpl.cpp` (rewritten policy, same public class) | Plan execution only: hold the current plan, emit one `MappingStepCommand` per `nextStep`, decide when to replan. | `NbvPlanner` |
| `Algorithm/src/MappingAlgorithmFrontier.{h,cpp}` (trimmed) | Reachability, Dijkstra costs, frontier detection, line-of-sight clearance. No policy. | — |
| `UserCommon/include/user_common_207190406_209543255/LidarCone.h` (extended) | Add a counting form of C's cone walk. | `BeamMath.h` |

`ExplorationPlan` is the whole interface between policy and execution:

```cpp
struct ExplorationPlan {
    std::vector<common::Position3D> waypoints;      // smoothed, first element is the next target
    std::vector<common::Orientation> terminal_scans; // world-frame; converted to the drone frame at emission
    double expected_gain = 0.0;                      // voxels, for logging and termination
    bool valid = false;
};
```

### Objective

For a candidate viewpoint `v`:

```
utility(v) = expected_new_voxels(v) / step_cost(v)
```

`expected_new_voxels(v)` casts C's direction set from `v` — `fibonacciSphereOrientations(
directionCountForHalfAngle(coneHalfAngleRad(lidar_config_)))`, the same set the drone can actually
scan — and counts distinct `Unmapped` voxels reached before an `Occupied`/`OutOfBounds` terminates the
ray, considering only the interval `[z_min, z_max]`. Reusing C's beam walk means the gain estimate and
the executable scans agree by construction, and occlusion and the dead zone are respected for free.

`step_cost(v)` is computed on the *smoothed* path against the real command set, not on Dijkstra cell
cost: `ceil(run_length / max_advance)` per straight run, `ceil(turn_angle / max_rotate)` per turn,
`ceil(dz / max_elevate)` per elevation change, plus one step per orientation that will be scanned at
the viewpoint.

The current pose is always a candidate, which is what makes the drone sweep in place at spawn (when
nothing is reachable yet) and again whenever standing still is the best information per step.

### Candidates

The Dijkstra pass over the passable component already visits every reachable cell. Candidates are the
visited cells that are adjacent to unresolved space, deduplicated onto a coarse lattice
(`kCandidateStrideCells = 3`) to bound their number, plus the current pose.

Scoring every candidate with a full cone raycast is too expensive, so scoring is two-stage:

1. **Cheap prefilter, constant work per candidate:** count `Unmapped` cells in the candidate's
   26-neighbourhood, divide by its Dijkstra cost. Keep the best `kScoredCandidates = 16`.
2. **Full raycast gain** on those 16 only, then rank by `utility`.

This bounds a replan at roughly `16 × directions × (z_max / ray_step)` map lookups — a few tens of
thousands, with `ray_step` half the grid resolution as in C's `coneCoversUnresolved`.

### Budget-awareness is a feasibility filter, not a mode switch

Remaining budget is `mission_config_.max_steps - state.step_index` — the first production read of
either field. Any candidate whose `step_cost` exceeds the remaining budget is discarded before
ranking.

That single filter produces the anytime behavior the roadmap asked for, with no thresholds to tune:
while the budget is loose, distant high-gain viewpoints compete on gain-per-step; as it drains, they
become infeasible and only near candidates survive, so the policy turns greedy on its own. Nothing is
reserved for a return trip, because the mission has no return requirement.

### Co-emission

While a plan is being executed, each step emits the movement toward the next waypoint **and** a scan.
Because MissionControl moves first and scans at the resulting pose, the planner predicts the post-move
pose — including the heading change a `Rotate` produces — and emits the highest-gain orientation
expressed relative to *that* heading. The scan is omitted only when no orientation has positive gain,
which is C's gain-gate reused. `enable_scan_during_travel` and `spacing_cells` are deleted; periodic
travel scans are subsumed.

### Path smoothing

The 6-connected Dijkstra path is string-pulled: from each waypoint, extend to the farthest later
waypoint whose straight segment is clear, testing C's sphere clearance at half-resolution samples
along the segment.

Smoothing applies **only to segments at constant altitude**. Elevation changes stay as their own
waypoints because `movementToward` emits `Elevate` before any horizontal motion, so the drone flies an
L through a mixed segment; checking line-of-sight along a 3D diagonal would validate a trajectory the
drone never flies.

### Termination

`Finished` is returned only when no `Unmapped` voxel remains in bounds.
`FinishedWithUnmappableVoxels` is returned only when no feasible candidate has positive expected gain
**and** `kRecoveryAttempts = 3` consecutive recovery plans (move to any reachable cell with unresolved
neighbourhood, ignoring the blocked set) have also failed. Otherwise the algorithm keeps working.
`kNoProgressLimit` and `kNoFrontierStuckLimit` are deleted — "no feasible positive-gain viewpoint" is
the direct form of the question those counters approximated.

### Retired state

- `frontier_visit_counts` — deleted outright. A frontier that has been exhausted scores zero gain, so
  NBV deprioritizes it by construction instead of blacklisting it.
- `blocked_cells` — kept, but each entry records the `step_index` at which it was inserted and expires
  after `kBlockedTtlSteps = 50` steps. A cell that is genuinely solid gets re-blocked on the next
  stall, or is `Occupied` in the map and blocked by occupancy anyway.
- `explore_dist_cache` — deleted. It was correct only when unused; the distance field is recomputed
  per replan, which the replan cadence below bounds.
- `findFarthestPath`, `findGreedyUnknownStep` — deleted with their declarations (~120 lines, no
  callers).
- The mid-search edge-set change at `MappingAlgorithmFrontier.cpp:428` — deleted. Dijkstra returns
  shortest paths under a fixed edge set; frontier preference belongs in the NBV objective, not inside
  the search.

### ALG28 bound

Every search in the substrate takes an explicit expansion cap equal to the number of voxels in the
mission bounds, and returns "no path" when the cap is hit. Today the visited set is bounded only by
`isSpherePassable` returning false, so a mutation making passability always-true walks an unbounded
integer grid — the exact hang the ex2 grader injected
(`docs/ex2-grading-handoff.md:160-166`). The bound is verified by a test, not by inspection.

### Replan cadence

Replan when the plan is exhausted, when movement stalls (`kMaxMovingStallTicks = 2`, unchanged), or
every `kReplanIntervalSteps = 10` steps, whichever comes first. `IMap3D` exposes no version counter,
so a fixed interval is the available proxy for "the map changed enough to reconsider"; it also bounds
planning compute per mission.

### What does not change

- C's clearance geometry, cone half-angle formula, and `UserCommon` placement.
- B's `step()` contract and `isRecoverableMovementFailure`.
- The `IMappingAlgorithm` interface, the plugin class name, and registration.
- `Unmapped` soft cost (4) versus `Empty` (1) in the traversal cost.

---

## Test plan

### `NbvPlanner`

| Test | Asserts |
|------|---------|
| Gain counts occluded voxels once and stops at `Occupied` | Rays terminate on hits; a voxel behind a wall contributes nothing |
| Gain ignores voxels closer than `z_min` and beyond `z_max` | Dead zone and range respected |
| Utility prefers the cheaper of two equal-gain viewpoints | Ranking uses step cost, not cell count |
| Step cost charges two rotations for a 90° turn at `max_rotate = 45°` | Cost model matches the real command set (`drone_large`) |
| Candidate exceeding remaining budget is discarded | Feasibility filter, with a candidate feasible at 1000 remaining and infeasible at 10 |
| Same map and configs produce an identical plan across runs | Determinism required by project A |
| Current pose is a candidate when nothing is reachable | Spawn behavior: sweep in place rather than fail |

### Execution and safety

| Test | Asserts |
|------|---------|
| A travel step emits both `movement` and `scan_orientation` | Co-emission, the property B enabled and nothing used |
| Scan orientation is relative to the post-move heading | A plan whose next command is a `Rotate` emits the orientation for the rotated frame |
| Scan omitted when every orientation has zero gain | C's gate still applies |
| No emitted movement targets a cell failing sphere clearance | Safety invariant from decision 1 |
| Smoothed segment preserves clearance | String-pulling across a wall corner is rejected |
| Smoothing does not merge across an altitude change | Mixed segments stay split, matching the executed L |
| Smoothing reduces command count on a staircase path | The point of the feature, measured not assumed |

### Termination and retired state

| Test | Asserts |
|------|---------|
| Does not finish while a feasible positive-gain viewpoint remains | The `house_full` failure, as a unit test |
| Finishes with `Finished` when no `Unmapped` remains in bounds | True completion still terminates |
| Finishes with `FinishedWithUnmappableVoxels` after recovery attempts are exhausted | Bounded, and not on the first failed cycle |
| A blocked cell becomes plannable again after its TTL | Blacklist expiry |
| Search returns "no path" rather than hanging when passability is forced true | ALG28 mutation, with a test-level timeout |

### Harness

Re-run the `honest` column and record `docs/benchmarks/<date>-post_d_honest.{csv,md}` with per-group
band verdicts against ex2. Re-run the adversarial column as a robustness check and confirm no cell
errors. Expect total steps to **rise** — missions that used to quit at 27 steps now use their budget —
and treat that as intended, since steps are only the competitive tiebreak.

---

## Success criteria

1. No cell terminates with substantial unused budget while reachable unknown space and positive
   expected gain remain, demonstrated on `house_full` (today: 27–232 steps of 10000, 6–13%).
2. `honest` score sum materially exceeds post-C's 1331.0; the target is ex2's band midpoints, roughly
   2030 across the 24 cells, with ≥ 1800 treated as "clearly approaching" and a per-group verdict
   recorded either way.
3. `house_full` moves toward its ex2 band of 56–62. The sub-`z_min` floor layer is a physical ceiling
   ex2 hit too — do not spend effort below it.
4. Zero cells with `ERROR` status in either column; `MAX_STEPS` cells are acceptable.
5. Every travel step carries a scan unless gain is zero, verified by test.
6. `frontier_visit_counts`, `explore_dist_cache`, `findFarthestPath`, `findGreedyUnknownStep`, the
   mid-search edge-set change, and the two 100-cycle counters are gone from the tree.
7. Every search is expansion-bounded, verified by the forced-passability test.
8. New code uses `mp-units` strong types; no new `force_numerical_value_in` outside the boundary with
   the legacy substrate.
9. Full `ctest` green; docs below updated.

---

## Risks

- **Harness runtime grows.** Missions that finished in tens of steps will now run to hundreds or
  thousands. The per-replan candidate caps and the 10-step replan interval bound planning compute, but
  expect the sweep to take longer than the current 2–5 minutes per column and budget for it.
- **More flying means more wall contact.** Our MC absorbs blocked moves as recoverable; the
  adversarial column is the check that this has not become a real hazard.
- **Gain estimation is a model, not truth.** It assumes the map is the belief and that a scan resolves
  what its rays touch. If measurements disagree with predicted gain, the prefilter is the first thing
  to inspect, not the objective.

---

## Out of scope

- Frontier clustering with tour ordering (FUEL/TARE) — deferred past D by the roadmap; revisit only if
  measurements show revisit oscillation survives NBV.
- Raycasting `latest_scan` into an independent belief map — `output_map_` was sufficient for C's
  gain-gating and is sufficient for D's scoring. Only a foreign host that passes `nullptr` scans and
  skips Empty-carving would force it, and that host is a robustness check, not the target.
- Converting the frontier substrate to `mp-units` — project E.
- Any change to `MissionControl/`, `common/`, or the harness.

---

## Related docs

| Doc | Role |
|-----|------|
| `docs/superpowers/specs/2026-08-29-mapping-algorithm-roadmap.md` | A/B/C/D sequence and the standing decisions |
| `docs/superpowers/specs/2026-08-29-sensor-model-clearance-belief-design.md` | C — clearance geometry and cone helpers D builds on |
| `docs/superpowers/specs/2026-08-29-missioncontrol-step-honesty-design.md` | B — the step contract that makes co-emission pay |
| `docs/superpowers/specs/2026-08-29-algorithm-benchmark-harness-design.md` | A — harness, ex2 bands, determinism requirement |
| `docs/mapping-algorithm-analysis.md` | The review that identified all three policy gaps |
| `docs/ex2-grading-handoff.md` | ALG28 unbounded-search hang this project bounds |
| `docs/benchmarks/2026-08-29-post_c_honest.{csv,md}` | The baseline D is measured against |

## Documentation updates

| Doc | Change |
|-----|--------|
| `docs/mapping-algorithm-analysis.md` | Mark policy gaps 1 (no move+scan) and 3 (blind to `max_steps`) resolved; mark the implementation findings retired by D (blacklists, stale cache, mid-search edge change, dead code) |
| `docs/HLD.md:89-91` | Replace "uses an internal BFS frontier planner to choose scan directions" with the NBV policy over a frontier reachability substrate |
| `docs/superpowers/specs/2026-08-29-mapping-algorithm-roadmap.md` | Project D status, measured impact table, and the superseded clearance-invariant note from decision 1 |
| `docs/mapping-algorithm-rewrite-pickup.md` | D done; next queue item is project E plus PR/packaging |
| `docs/known-issues.md` | Revisit #20 given the new step profile |
