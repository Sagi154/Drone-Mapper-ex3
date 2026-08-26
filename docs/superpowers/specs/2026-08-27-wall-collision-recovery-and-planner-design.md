# Wall collision recovery and planner improvements

**Date:** 2026-08-27  
**Status:** Accepted 2026-08-27  
**Goal:** All 24 `inputs/sim_compose.yaml` cells finish as `Completed` or `MaxSteps` with `mission_score >= 0`, with compose wall-clock in the same order of magnitude as ex2 (minutes), without thrashing on wall bumps.

## Problem

Ex2 scored 24/24 with the same Unmapped-as-passable planner because `MockMovement` returned `{false, …blocked…}` and `DroneControl` continued; stall detection then marked cells blocked and replanned.

Ex3 made wall collision a **throw** (Common-issues mandatory). Ported `DroneControlImpl` did **not** catch; `SimulationRunImpl` turned the first bump into `MISSION_EXCEPTION` / score `-1`. That is a wiring regression, not a skeleton-forced algorithm rewrite.

Catch-and-Continue alone recreates the recovery channel but can thrash: several missions use `max_steps` up to 10000, `kMaxMovingStallTicks` is 8, and `enable_scan_during_travel` is disabled when `max_steps > 500` — so long missions bump more and learn slowly.

House spawn (`height_cm` 10→150) is a separate, already-identified inputs fix and is assumed present.

## Non-goals

- Reverting MockMovement to return `false` instead of throw (violates Common-issues mandatory throw).
- Treating Unmapped as fully impassable (ex2 rejected; stalls exploration).
- Rewriting the frontier algorithm from scratch.
- README / HLD PDF (separate pickup items).
- Editing frozen `common/`, `Simulator/common_simulator/`, `MissionControl/common_mission_control/`.

## Constraints (project rules)

- **Frozen interfaces:** no published header signature or folder changes.
- **Common-issues mandatory:** MockMovement throws on real-map wall; DroneControl and SimulationRun both catch (SimulationRun remains backstop for non-recoverable escapes).
- **Assignment algorithm bar:** do not fly into walls when the algorithm can avoid them; map relevant surroundings; be efficient.
- **AdvCpp:** named constants for costs / stall thresholds (e23); no `new`/`delete`; Docker-only verify.
- **Git:** feature branch; conventional commits; human approval before each commit.
- **Placement:** recovery in MissionControl (`DroneControlImpl`); planner/sensing in Algorithm; MockMovement stays throwing in Simulator.

## Architecture

```text
Algorithm plans (Empty preferred over Unmapped; Unmapped still passable)
    → DroneControl executeMovement
        → MockMovement throw on real wall
        → DroneControl catch blocked/boundary → Continue (no pose change)
        → other exceptions rethrow → SimulationRun MISSION_EXCEPTION
    → Algorithm sees same position → fast stall → blocked_cells → replan
    → Scan during travel (including long max_steps) reduces blind Unmapped entry
```

## Design details

### 1. Control and recovery (`DroneControlImpl`)

- Wrap `executeMovement` in `try/catch (const std::exception&)`.
- If `isRecoverableMovementFailure(ex.what())` (`blocked` / `boundary`): increment step index, return `DroneStepStatus::Continue` (same as false `MovementResult` path).
- Otherwise rethrow so `SimulationRunImpl` logs `MISSION_EXCEPTION`.
- Keep MockMovement throws unchanged.
- Unit tests: recoverable throw → Continue; non-recoverable message → escapes `step()`.

### 2. Fast stall learning (`MappingAlgorithmImpl`)

- Reduce effective wait before marking a stuck waypoint blocked: target **1–2** same-position ticks after a failed move (named constant; replace or supersede `kMaxMovingStallTicks = 8` for the moving-phase stall path).
- Existing behavior after threshold: insert waypoint into `blocked_cells`, enter Planning, replan.
- No new frozen API; algorithm still only observes GPS (no explicit collision callback).

### 3. Soft Unmapped path cost (`MappingAlgorithmFrontier`)

- Keep `isSpherePassable`: Unmapped does not hard-block; Occupied / OutOfBounds / `blocked_cells` do.
- In BFS / path scoring, apply a **higher step cost** for Unmapped than Empty (named constant, e.g. `kUnmappedTraversalCost` vs `kEmptyTraversalCost = 1`).
- Prefer paths through known Empty when lengths would otherwise be similar; still allow Unmapped when needed for exploration.

### 4. Sensing on long missions (`MappingAlgorithmImpl`)

- Stop disabling scan-during-travel solely because `max_steps > 500`.
- **Locked rule:** set `enable_scan_during_travel = true` regardless of `max_steps` (remove the `max_steps <= 500` gate). Revisit only if compose timing regresses badly after soft Unmapped cost + fast stall.

### 5. Verification

- Docker `ctest` (at least DroneControl + Algorithm frontier tests).
- Comparative CLI on `inputs/sim_compose.yaml`: **24/24** with `mission_score >= 0`; zero wall-driven `MISSION_EXCEPTION` in the happy path.
- Wall-clock: minutes-scale for full compose (not tens of minutes).
- Frozen-interface git check empty after changes.

### 6. Docs after green

- `docs/assignment-compliance-pickup.md`, `docs/known-issues.md` (rows 2, 17, 18 as appropriate), `AGENTS.md`, `docs/workplan.md`, integration note / canvas if used.

## Success criteria

| Criterion | Pass |
|-----------|------|
| Scores | 24/24 cells `Completed` or `MaxSteps`, `mission_score >= 0` |
| Exceptions | No routine `MISSION_EXCEPTION` from wall bumps on default compose |
| Time | Full comparative finishes in minutes (ex2 ballpark), not ~20+ minutes of thrash |
| Rules | MockMovement still throws; frozen trees untouched; Common-issues catch targets satisfied |

## Out of scope follow-ups

If a cell still fails after this design, diagnose that cell’s error log only — do not expand into optional Common-issues rows (CI2–CI12) in the same change set.
