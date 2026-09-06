# VAR-01 Approach B — `house_full` Collapse Fix Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fix `MappingAlgorithmImpl_207190406_209543255::reachedWaypoint` so it declares a
waypoint "reached" by voxel membership (matching `quantizePosition`'s own `floor` convention)
instead of a symmetric continuous-distance tolerance, so that Approach B's voxel-center
lattice (worktree `ex3-var01-center-lattice`, branch `sphere-clearance-voxel-center-lattice`)
stops collapsing all four `house_full` cells to score ≈0.061, while keeping VAR-01 passing and
not reintroducing the corner-lattice/clearance mismatch that motivated Approach B.

**Architecture:** One self-contained, one-function change in `Algorithm/src/MappingAlgorithmImpl.cpp`
(no header/signature change), plus a regression test that reproduces the exact
`gps_resolution_cm == map resolution_cm` coincidence found in `house_mission_full.yaml`.
Measured with the same three harnesses the parent plan
(`docs/superpowers/plans/2026-09-05-var01-sphere-clearance-fix.md`) used: `Algorithm/tests`,
`verify-independent-component-variants`, `verify-cell-runtime`.

**Tech Stack:** C++20, CMake presets (`default` = Debug, `opt` = Release), Docker image
`drone-mapper-ex3-dev`, GoogleTest, Python 3 (`time_each_cell.py`).

## Global Constraints

(From `docs/superpowers/specs/2026-09-05-var01-sphere-clearance-fix-design.md` §3, §4 and the
executed plan's Global Constraints — copied verbatim, apply to every task below.)

- Never touch `common/`, `Simulator/common_simulator/`, `MissionControl/common_mission_control/`.
- No wall-clock abort in `Algorithm/` or `MissionControl/`. `max_steps` is the only
  mission-length limit (`.cursor/skills/verify-cell-runtime/SKILL.md`).
- 24-cell score sum on `inputs/sim_compose.yaml` must stay close to baseline **1769.835**
  (`docs/benchmarks/2026-09-03-*`); treat an aggregate drop beyond ~50-90 points as a flag to
  discuss with the user, not an auto-reject. Individual cell drift is acceptable — indeed this
  plan's whole point is to make `house_full`'s 4 cells drift *up*, from ≈0.06 back toward
  Approach A / baseline's 37.19 / 48.33 / 20.99 / 15.34 (documented in
  `.cursor/skills/verify-cell-runtime/SKILL.md`'s baseline table).
- No cell may newly exceed the ~60s per-cell wall-clock budget.
- VAR-01 (`HOST_ILLEGAL_MOVE_ATTEMPTS=0`) must remain PASS; VAR-02/03 must remain at their
  current status.
- Never silently withhold a movement on a clearance refusal — this fix does not touch
  clearance/passability at all, only waypoint-arrival bookkeeping.
- One mechanism per commit, one benchmark run per commit. Do not squash this fix with any
  other change.
- Git workflow (`.cursor/rules/git-workflow.mdc`): branch names are kebab-case and name the
  component/fix, never an item code or owner. This work continues on the existing worktree
  branch `sphere-clearance-voxel-center-lattice` (already checked out in
  `ex3-var01-center-lattice`) — do not rename it. Human approval required before every
  `git commit` — present the diff and message, wait for explicit approval.
- `Algorithm/` code style: `.cursor/rules/adv-cpp-standards.mdc` (no magic numbers — name
  constants; `-Wall -Wextra -Werror -pedantic` must stay clean) and
  `.cursor/rules/mp-units-strong-types.mdc` (strong types at API boundaries; unwrap to
  `double` only inside a `.cpp` for hot-path math, exactly as the existing code already does).
- Work happens in the existing worktree `C:\Users\sagi1\Projects\DroneMapper\ex3-var01-center-lattice`
  (branch `sphere-clearance-voxel-center-lattice`). Do not create a second worktree unless the
  existing one has been repurposed for something else by the time this plan is executed — if
  so, create a fresh worktree off the same branch's tip and note that deviation when reporting
  back.

---

## 1. Root cause (confirmed empirically this session)

### 1.1 Symptom

All four `house_full` cells (`drone_small`/`drone_large` × `lidar_long`/`lidar_short`) under
Approach B (`ex3-var01-center-lattice` @ `5bc4631`) score **exactly 0.0612369871402327** at
**exactly 400 steps**, `status=COMPLETED`, wall ≈0.1-0.2s — reproduced directly:

```
[1] PASS house_simulation+house_mission_full|drone_small|lidar_long   score=0.0612369871402327 steps=400 status=COMPLETED wall=0.1s
[2] PASS house_simulation+house_mission_full|drone_small|lidar_short  score=0.0612369871402327 steps=400 status=COMPLETED wall=0.2s
[3] PASS house_simulation+house_mission_full|drone_large|lidar_long   score=0.0612369871402327 steps=400 status=COMPLETED wall=0.1s
[4] PASS house_simulation+house_mission_full|drone_large|lidar_short  score=0.0612369871402327 steps=400 status=COMPLETED wall=0.2s
```

(Approach A / baseline score these cells 37.19 / 48.33 / 20.99 / 15.34 per the documented
2026-09-03 baseline in `.cursor/skills/verify-cell-runtime/SKILL.md`. `steps=400` well under
`house_mission_full.yaml`'s `max_steps: 10000` proves this is an early-terminate bug, not a
runtime/step-budget issue — VAR-01's own "no wall-clock abort" constraint is not violated,
this is the algorithm choosing to stop.)

### 1.2 Instrumented evidence: the drone never moves

Uncommitted diagnostic prints added to `nextStep`/the waypoint-skip loop in
`MappingAlgorithmImpl.cpp` (reverted after this investigation — see §1.4), run against
`house_simulation+house_mission_full|drone_small|lidar_long`:

```
SCRATCH_DIAG nextStep call step=0 pos=(150.000,200.000,300.000) has_plan=0 waypoints=0 wp_idx=0 offset=(0.000,0.000,150.000)
SCRATCH_DIAG step=1 wp_idx=0 skip pos=(150.000,200.000,300.000) target=(155.000,205.000,295.000) step_cm=10.000 tol=5.000
SCRATCH_DIAG nextStep call step=1 pos=(150.000,200.000,300.000) has_plan=1 waypoints=1 wp_idx=1 offset=(0.000,0.000,150.000)
SCRATCH_DIAG nextStep call step=2 pos=(150.000,200.000,300.000) has_plan=0 waypoints=1 wp_idx=0 offset=(0.000,0.000,150.000)
SCRATCH_DIAG step=3 wp_idx=0 skip pos=(150.000,200.000,300.000) target=(155.000,205.000,295.000) step_cm=10.000 tol=5.000
... (repeats identically every 2 steps through step=79, position never once changes) ...
```

The drone's *reported* position is **frozen at (150.000, 200.000, 300.000) for the entire
run**. Every replan produces a fresh one-waypoint plan whose target is a neighbour voxel
centre (here `(155, 205, 295)`), and that single waypoint is treated as **already reached**
(`skip`) on the very next `nextStep` call, without a single `Advance`/`Elevate` movement
command ever being emitted. The plan is discarded, a new one is built two steps later, and the
cycle repeats — the drone does only its initial arrival scans from the spawn point and never
travels anywhere. This exactly matches "essentially an early-abort/no-op" and "doing
effectively nothing useful" from the task brief.

### 1.3 Root cause: the arrival-tolerance boundary exactly coincides with GPS quantization error

`MappingAlgorithmImpl_207190406_209543255::reachedWaypoint`
(`Algorithm/src/MappingAlgorithmImpl.cpp:78-91`):

```cpp
constexpr double kHalfStepTolerance = 0.5;
...
bool MappingAlgorithmImpl_207190406_209543255::reachedWaypoint(
    const types::DroneState& state,
    const Position3D& target,
    const types::MapConfig& map_config) const {
    const double step = map_config.resolution.force_numerical_value_in(cm);
    const double dx = std::abs(state.position.x.force_numerical_value_in(cm) -
                               target.x.force_numerical_value_in(cm));
    const double dy = std::abs(state.position.y.force_numerical_value_in(cm) -
                               target.y.force_numerical_value_in(cm));
    const double dz = std::abs(state.position.z.force_numerical_value_in(cm) -
                               target.z.force_numerical_value_in(cm));
    return dx <= step * kHalfStepTolerance && dy <= step * kHalfStepTolerance &&
           dz <= step * kHalfStepTolerance;
}
```

This treats "reached" as "within `step/2` (5 cm on this 10 cm grid) of the target on every
axis" — a **symmetric distance window**, using an inclusive `<=`.

Two facts combine to make this window degenerate specifically for `house_full`:

1. **Approach B's center-anchored lattice** places every waypoint at
   `offset + (k + 0.5) * step` (`keyToPoint`, `MappingAlgorithmFrontier.cpp:423-434`) — i.e.
   every reachable waypoint coordinate sits at `...5` mod 10 on a 10 cm grid (with `offset=0`
   on the relevant axes here; `offset.z = 150` per `map_axes_offset.height_offset`, which does
   not change the `...5` residue since 150 is itself a multiple of 10).
2. **`house_mission_full.yaml`'s `gps_resolution_cm: 10`** (`inputs/mission/house_mission_full.yaml:12`)
   is the *only* one of the 6 (simulation, mission) pairs in `inputs/sim_compose.yaml` where
   `gps_resolution_cm` equals `map_resolution_cm` (`house_simulation.yaml:2`, both 10; the
   other 5 pairs use `gps_resolution_cm: 5`, exactly half the map resolution).
   `MockGPS::setPosition` (`Simulator/src/MockGPS.cpp:15-20`) snaps every reported position to
   the nearest multiple of `gps_resolution_cm` measured from world 0 (`std::round(value/res)*res`).
   With `res=10` and `offset=0`, **every GPS-reported coordinate is an exact multiple of 10** —
   it can never end in `...5`.

The minimum possible distance between an achievable GPS-reported coordinate (multiple of 10)
and any lattice-centre waypoint coordinate (`...5` mod 10) is therefore **exactly 5 cm — not
approximately, exactly, in floating point, every single time** (both values are exact binary
multiples of 5 with no accumulated rounding error). That is precisely `step * kHalfStepTolerance`
(`10 * 0.5 = 5`), and the comparison is `<=`. So **every neighbour waypoint the planner ever
proposes is judged "already reached" on the very first check, before any movement command is
ever computed** — the `while (... reachedWaypoint(...)) ++waypoint_index;` loop at the top of
`nextStep` (`MappingAlgorithmImpl.cpp:408-411`) consumes the (always single-waypoint) plan
immediately, `emitMovementOrScan` jumps straight to the arrival-scan branch, and
`movementToward` — the function that would actually issue an `Advance`/`Elevate` command — is
**never called for any waypoint** in this mission. With zero net motion, each replan's
`expected_rate`/observed information rate stays effectively at zero once the spawn-point's
lidar cone is exhausted, and the existing low-rate-replan / low-observed-information-window
give-up logic (`handleReplan`, `updateProgressWindow`,
`MappingAlgorithmImpl.cpp:276-339`) correctly (and by design) calls it quits — hence
`status=COMPLETED` at step 400, well short of the 10000-step budget, with only whatever the
spawn-point arrival scans mapped (score ≈0.06).

**This is not a "5 cm is close enough, minor score dip" situation as the original design spec's
hazard #2 speculated** ("Center lattice points... snap... 5 cm off its intended point... This
is a real risk... must be measured directly, not estimated" —
`docs/superpowers/specs/2026-09-05-var01-sphere-clearance-fix-design.md` §6.3.2). The actual
severity is much worse: because the 5 cm residual sits **exactly** on the `<=` tolerance
boundary, in **every** case, for **every** possible waypoint in this mission (the offset
between the lattice's phase and the GPS grid's phase is a mission-wide constant, not a
per-waypoint coincidence), the drone is deterministically deceived into believing it has
already arrived at any target it could ever be assigned, and physically never needs to move.

### 1.4 Why a smaller tolerance tweak (e.g., `<=` → `<`) does not work — must not repeat this mistake

A tempting one-character "fix" is changing `<=` to strict `<`. This was evaluated and rejected
by direct calculation, not by trial: flipping the comparison converts the bug into the
*opposite* failure mode, not a fix. If dx must be strictly `< 5`, the target `295` still has a
distance of exactly 5 cm to *both* of the only two GPS-achievable neighbouring coordinates,
290 and 300 (`std::round(29.5) == 30`, i.e. GPS reports can only ever be 290 or 300, both
exactly 5 cm from 295, never closer) — so the drone would now **never** be able to satisfy
`reachedWaypoint` for this waypoint at all, regardless of how it moves, and would fall back on
the `moving_stall_ticks` stall detector blocklisting every waypoint it is ever given,
replanning indefinitely without progress. A symmetric distance comparison — with either `<=`
or `<` — cannot distinguish "the drone is one full step short of the target" from "the drone
has just arrived," because under this exact coincidence **both states report the identical 5 cm
residual**. Any fix confined to tuning the comparison operator or the tolerance constant
inherits this same ambiguity. The fix must use a test that is *not* a symmetric distance
window (§2).

### 1.5 Scratch instrumentation used, and cleanup confirmation

The diagnostic prints in §1.2 were added directly to
`ex3-var01-center-lattice/Algorithm/src/MappingAlgorithmImpl.cpp` (gated behind
`std::getenv("SCRATCH_DIAG")`, capped to print only the first few calls) and a one-line
temporary stderr-passthrough in `.cursor/skills/verify-cell-runtime/scripts/time_each_cell.py`
(to surface the child process's stderr even on a non-failing run). Both were reverted via
`git checkout -- <path>` after gathering the evidence above; `git status --short` in the
worktree shows no tracked-file changes remain (the only untracked item is a pre-existing copy
of `docs/superpowers/plans/2026-09-05-var01-sphere-clearance-fix.md` that predates this
investigation and was not created or modified by this work).

## 2. Fix design

### 2.1 Chosen fix: voxel-membership arrival, not symmetric distance

Replace `reachedWaypoint`'s distance-window test with **the same voxel-identity test the rest
of Approach B's lattice code already uses** — `quantizePosition` equality (`GridKey ==`).
`quantizePosition` (`MappingAlgorithmFrontier.cpp:408-421`) already implements the correct,
*directional* notion of "which voxel is this position inside": `floor((pos - offset) / step)`.
A waypoint is reached exactly when the drone's reported position quantizes to the **same
voxel** as the target:

```cpp
bool MappingAlgorithmImpl_207190406_209543255::reachedWaypoint(
    const types::DroneState& state,
    const Position3D& target,
    const types::MapConfig& map_config) const {
    // Voxel-membership arrival: "reached" means occupying the SAME voxel as the target,
    // matching quantizePosition's own floor() convention, not a symmetric distance window.
    //
    // A symmetric ±step/2 distance tolerance is ambiguous whenever the residual sits exactly
    // on the boundary -- which happens on EVERY waypoint in house_mission_full, because its
    // gps_resolution_cm (10) equals the map's resolution_cm (10), so every GPS-quantized
    // report is a multiple of 10 while every centre-lattice waypoint sits at "+5" mod 10; the
    // two are always exactly step/2 apart, and a distance check cannot tell "one full voxel
    // short" from "just arrived" apart -- both report the identical residual. floor()-based
    // voxel membership is directional and resolves the ambiguity correctly: the GPS reading
    // on the "arrived" side of the boundary quantizes into the target's own voxel; the one on
    // the "not yet" side does not, even though both are equidistant in raw cm.
    // See docs/superpowers/plans/2026-09-06-var01-approach-b-house-full-fix.md §1-2.
    return detail::quantizePosition(state.position, map_config) ==
           detail::quantizePosition(target, map_config);
}
```

`detail::quantizePosition` is already visible in this translation unit and already used for
blocked-cell keys a few lines below the existing `reachedWaypoint` call site
(`MappingAlgorithmImpl.cpp:395-397`), so this requires no new include or export. `GridKey`
already supports `operator==` (used directly in the existing round-trip test
`CenterLatticeQuantizeKeyToPointRoundTrip` added by the parent plan's Task 7).

The now-unused `kHalfStepTolerance` constant (`MappingAlgorithmImpl.cpp:34`) is deleted in the
same change — leaving it in place with no callers would fail the `-Wall -Wextra -Werror`
build (unused-variable-style warning does not normally fire for an unused `constexpr` at
namespace scope, but leaving a stale, misleadingly-named unused constant around violates
`adv-cpp-standards.mdc`'s "no magic numbers / no dead code" spirit — remove it, don't leave it
orphaned).

### 2.2 Why this is correct and stays targeted

- **Fixes the root cause, not the symptom.** It removes the ambiguous symmetric-distance test
  entirely rather than retuning its threshold (§1.4 already ruled out threshold tuning).
- **Consistent with the rest of Approach B.** Every other lattice-aware comparison in this
  codebase already reasons in terms of `GridKey` identity (`start_key == goal_key` in
  `findPathTo`, `MappingAlgorithmFrontier.cpp:489`; blocked-cell lookups via `quantizePosition`
  keys). `reachedWaypoint` was the one holdout still using raw continuous distance — this
  fix removes an inconsistency, it doesn't introduce a new mechanism.
- **No change to clearance/passability.** `isSpherePassable`/`sphereIntersectsCellBox` are
  untouched, so VAR-01's corner-vs-center geometry fix (already correct under Approach B, per
  the parent plan) is unaffected. This fix cannot reintroduce the corner-lattice mismatch that
  motivated Approach B, because it does not touch geometry at all — it only changes when the
  *planner's own bookkeeping* advances `waypoint_index`.
- **No new per-tick cost.** `quantizePosition` is O(1) (three `floor` calls), same asymptotic
  cost as the three `std::abs` distance calls it replaces.
- **Does not depend on `gps_resolution_cm`.** The fix does not plumb GPS resolution into the
  Algorithm at all (confirmed `gps_resolution` is currently unused anywhere in
  `Algorithm/` production code — grep turns up only the test fixture's mission-config
  builder). It resolves the coincidence generically: voxel-membership arrival is correct
  regardless of what any mission's GPS resolution happens to be, so it does not create a
  second `gps_res == map_res`-shaped landmine for some future mission YAML.
- **Expected effect on the other 20 cells: none or strictly neutral.** For every mission
  except `house_full`, `gps_resolution_cm` (5) is exactly half of `map_resolution_cm` (10), so
  GPS-reported coordinates land on both `...0` and `...5` mod 10 — i.e., a well-converged
  approach can report a GPS position with **zero** residual to the target voxel centre, well
  inside both the old symmetric-distance window and the new voxel-membership test. The two
  tests differ only in a half-open-vs-closed boundary case at exactly `step/2`, which does not
  occur for these missions' GPS resolution. Task 3 measures this directly rather than assuming
  it.

### 2.3 Rejected alternatives (and why)

- **Adjust `MockGPS` snapping to align with the center lattice.** Rejected: `MockGPS.cpp` is a
  generic Simulator-level sensor model shared by all 24 cells and all 4 VAR checks; making it
  aware of the Algorithm's internal voxel-center choice would be a layering violation (the GPS
  sensor should not need to know how any particular algorithm plugin voxelizes space), and it
  risks changing behavior for the 20 cells that currently work correctly. It is also a strictly
  bigger, less targeted change than fixing the one comparison that is actually wrong.
- **Adjust `quantizePosition`/`keyToPoint` rounding at the boundary.** Rejected: these already
  implement the correct, consistent center-anchored lattice (confirmed by the existing
  round-trip test); the bug is not in how the lattice is defined, it is in how *arrival at* a
  lattice point is tested. Changing the lattice definition to dodge this one comparison's bug
  would risk exactly the kind of "adapt the pattern instead of understanding it" mistake the
  systematic-debugging process warns against.
- **Fix the `[0, n-2]` bounds clamp.** Rejected: unrelated. That hazard (top-layer
  out-of-bounds voxel centres, resolved by the parent plan's Task 6/7 with `kVoxelCenterOffset`
  and the `min_x/max_x` clamp in `forEachInBoundsVoxel`) is orthogonal to this GPS-snap
  coincidence; `house_full`'s spawn point and waypoints in the repro above are nowhere near a
  map boundary (`(150,200,300)` inside a `290x300x150`-boundary map with a `150`cm height
  offset — comfortably interior).
- **Unwind the whole center-lattice re-anchor (abandon Approach B).** Not needed. The root
  cause is one bookkeeping predicate reachable and fixable with a single self-contained
  change; there is no evidence of a deeper architectural problem with the center-anchored
  lattice itself (per systematic-debugging's "3+ failed fixes ⇒ question architecture" bar,
  this is fix attempt #1 with a confirmed, narrow root cause — no reason to escalate to
  reconsidering the approach).

---

## 3. Implementation tasks

### Task 1: Write the failing regression test

**Files:**
- Modify: `Algorithm/tests/test_mapping_algorithm.cpp` (add a new test; insert after
  `KeepsWorkingWhenUnmappedCountKeepsDropping`, before `FinishesCleanlyWhenNothingIsUnmapped`
  — re-locate the exact insertion point by searching for those two test names, since other
  work may have shifted line numbers since this plan was written)

**Interfaces:**
- Consumes: `Impl::nextStep(const ct::DroneState&, const ct::LidarScanResult*) ->
  ct::MappingStepCommand` (existing, unchanged signature); `ct::MovementCommand` fields
  `type`, `distance`, `angle`, `rotation` (existing, used identically to
  `MappingAlgorithmImpl.cpp`'s own `predictPose`); `ct::MissionConfigData::gps_resolution`
  (existing field, currently only set by test fixtures — this task is what finally exercises
  it meaningfully).
- Produces: nothing consumed by later tasks — this task only adds a test assertion that Task 2
  makes pass.

This test simulates what `MockGPS` + `MockMovement` would do to a real drone: after each
`nextStep` call, if a movement command was emitted, apply it to a locally-tracked physical
position/heading (mirroring `MappingAlgorithmImpl_207190406_209543255::predictPose`'s own
Advance/Elevate/Rotate math, since that is the algorithm's own model of its movement effects),
then **snap the result to the nearest multiple of a 10 cm GPS resolution measured from world
0** — reproducing `Simulator/src/MockGPS.cpp:15-20`'s `snapToCm` exactly. Assert the drone
actually leaves its starting voxel within a small number of ticks.

- [ ] **Step 1: Add the test**

```cpp
// What: reproduces the VAR-01 Approach-B house_full collapse (see
// docs/superpowers/plans/2026-09-06-var01-approach-b-house-full-fix.md §1). When a mission's
// gps_resolution_cm equals the map's resolution_cm (house_mission_full: 10cm each), every
// GPS-quantized position report is an exact multiple of 10, while every centre-lattice
// waypoint sits at "+5" mod 10 -- exactly step/2 away, every time. A symmetric ±step/2
// distance tolerance in reachedWaypoint cannot tell "one full voxel short" from "just
// arrived" in this case (both report the identical 5cm residual), so the drone never issues
// a real movement command and is frozen at its GPS-quantized spawn point forever.
// Expected: the drone must leave its starting voxel within a handful of ticks once given a
// clear path, i.e. reachedWaypoint must not be satisfied merely because the residual sits
// exactly on the old tolerance's boundary.
TEST(MappingAlgorithm, DroneActuallyMovesWhenGpsResolutionMatchesMapResolution) {
    const ct::MapConfig config = makeCorridorConfig();  // resolution 10cm, offset {}
    Map output_map{{11, 11, 11}, config};
    fillEmptyBox(output_map, 0, 8, 0, 10, 0, 10, config);

    auto mc = makeMissionConfig();
    mc.gps_resolution = 10.0 * cm;  // == map resolution_cm: house_mission_full's exact hazard
    const auto lc = makeLidarConfig();
    const auto dc = makeDroneConfig();
    Impl algorithm{common::MappingAlgorithmDependencies{mc, lc, dc, output_map}};

    const auto snap_to_gps_cm = [](double value_cm) {
        return std::round(value_cm / 10.0) * 10.0;
    };
    const auto snap_position = [&](const Position3D& pos) {
        return Position3D{
            snap_to_gps_cm(pos.x.force_numerical_value_in(cm)) * x_extent[cm],
            snap_to_gps_cm(pos.y.force_numerical_value_in(cm)) * y_extent[cm],
            snap_to_gps_cm(pos.z.force_numerical_value_in(cm)) * z_extent[cm],
        };
    };
    const auto apply_movement = [&](Position3D pos, const ct::MovementCommand& cmd,
                                     Orientation& heading) {
        switch (cmd.type) {
            case ct::MovementCommandType::Elevate:
                pos = Position3D{pos.x, pos.y,
                                  (pos.z.force_numerical_value_in(cm) +
                                   cmd.distance.force_numerical_value_in(cm)) * z_extent[cm]};
                break;
            case ct::MovementCommandType::Advance: {
                const double heading_rad = heading.horizontal.force_numerical_value_in(deg) *
                                            (std::numbers::pi / 180.0);
                pos = Position3D{
                    (pos.x.force_numerical_value_in(cm) +
                     cmd.distance.force_numerical_value_in(cm) * std::cos(heading_rad)) *
                        x_extent[cm],
                    (pos.y.force_numerical_value_in(cm) +
                     cmd.distance.force_numerical_value_in(cm) * std::sin(heading_rad)) *
                        y_extent[cm],
                    pos.z};
                break;
            }
            case ct::MovementCommandType::Rotate: {
                const double delta = cmd.angle.force_numerical_value_in(deg) *
                    ((cmd.rotation == ct::RotationDirection::Left) ? 1.0 : -1.0);
                heading.horizontal = (heading.horizontal.force_numerical_value_in(deg) + delta) * deg;
                break;
            }
            case ct::MovementCommandType::Hover:
                break;
        }
        return snap_position(pos);
    };

    Position3D position = snap_position(gridPoint(2, 5, 5, config));
    Orientation heading{0.0 * deg, 0.0 * deg};
    const Position3D start_position = position;

    const auto has_moved = [&](const Position3D& p) {
        return std::abs(p.x.force_numerical_value_in(cm) -
                         start_position.x.force_numerical_value_in(cm)) > 1e-6 ||
               std::abs(p.y.force_numerical_value_in(cm) -
                         start_position.y.force_numerical_value_in(cm)) > 1e-6 ||
               std::abs(p.z.force_numerical_value_in(cm) -
                         start_position.z.force_numerical_value_in(cm)) > 1e-6;
    };

    bool moved = false;
    for (int step = 0; step < 40; ++step) {
        const ct::DroneState state{position, heading, static_cast<std::size_t>(step)};
        const ct::MappingStepCommand cmd = algorithm.nextStep(state, nullptr);
        ASSERT_EQ(cmd.status, ct::AlgorithmStatus::Working)
            << "finished prematurely at step " << step;
        if (cmd.movement.has_value()) {
            position = apply_movement(position, *cmd.movement, heading);
        }
        if (!moved && has_moved(position)) {
            moved = true;
        }
    }
    EXPECT_TRUE(moved) << "drone never left its GPS-quantized starting position in 40 ticks";
}
```

- [ ] **Step 2: Run it and confirm it fails against the current (unfixed) `reachedWaypoint`**

```bash
docker run --rm -e VCPKG_ROOT=/usr/local/vcpkg -v "${PWD}:/work" -w /work drone-mapper-ex3-dev bash -lc '
  cmake --build --preset default --target algorithm_test
  ./build/default/Algorithm/algorithm_test --gtest_filter="MappingAlgorithm.DroneActuallyMovesWhenGpsResolutionMatchesMapResolution"
'
```

Expected: **FAIL** — `moved` stays `false` (`EXPECT_TRUE(moved)` fails), confirming the test
exercises the bug before Task 2 fixes it. If this test unexpectedly passes, stop — the fixture
does not reproduce the coincidence, re-derive `makeCorridorConfig`'s resolution/offset against
`house_simulation.yaml`'s actual values before continuing (do not weaken the assertion to make
it pass).

- [ ] **Step 3: Commit the failing test**

```bash
git add Algorithm/tests/test_mapping_algorithm.cpp
git commit -m "test: reproduce house_full freeze when gps_resolution_cm == map resolution_cm"
```

(Wait for human approval per `git-workflow.mdc` before running this commit.)

### Task 2: Fix `reachedWaypoint`

**Files:**
- Modify: `Algorithm/src/MappingAlgorithmImpl.cpp:32-91` (delete `kHalfStepTolerance`, replace
  `reachedWaypoint`'s body)

**Interfaces:**
- Consumes: `detail::quantizePosition(const Position3D&, const types::MapConfig&) -> GridKey`
  (existing, unchanged — already used elsewhere in this file for blocked-cell keys).
- Produces: `reachedWaypoint(const types::DroneState&, const Position3D&, const
  types::MapConfig&) const -> bool` — same signature, corrected semantics. Called only from
  `nextStep`'s waypoint-advance loop (`MappingAlgorithmImpl.cpp:408-411`); no other caller.

- [ ] **Step 1: Delete the now-dead tolerance constant**

Remove this line (`MappingAlgorithmImpl.cpp:32` area, inside the anonymous namespace with
`kMaxArrivalScans` and `kPositionEpsilon`):

```cpp
constexpr double kHalfStepTolerance = 0.5;
```

- [ ] **Step 2: Replace `reachedWaypoint`'s body**

```cpp
bool MappingAlgorithmImpl_207190406_209543255::reachedWaypoint(
    const types::DroneState& state,
    const Position3D& target,
    const types::MapConfig& map_config) const {
    // Voxel-membership arrival: "reached" means occupying the SAME voxel as the target,
    // matching quantizePosition's own floor() convention, not a symmetric distance window.
    //
    // A symmetric ±step/2 distance tolerance is ambiguous whenever the residual sits exactly
    // on the boundary -- which happens on EVERY waypoint in house_mission_full, because its
    // gps_resolution_cm (10) equals the map's resolution_cm (10), so every GPS-quantized
    // report is a multiple of 10 while every centre-lattice waypoint sits at "+5" mod 10; the
    // two are always exactly step/2 apart, and a distance check cannot tell "one full voxel
    // short" from "just arrived" apart -- both report the identical residual. floor()-based
    // voxel membership is directional and resolves the ambiguity correctly: the GPS reading
    // on the "arrived" side of the boundary quantizes into the target's own voxel; the one on
    // the "not yet" side does not, even though both are equidistant in raw cm.
    // See docs/superpowers/plans/2026-09-06-var01-approach-b-house-full-fix.md §1-2.
    return detail::quantizePosition(state.position, map_config) ==
           detail::quantizePosition(target, map_config);
}
```

- [ ] **Step 3: Build and run the new test plus the full `Algorithm` suite**

```bash
docker run --rm -e VCPKG_ROOT=/usr/local/vcpkg -v "${PWD}:/work" -w /work drone-mapper-ex3-dev bash -lc '
  cmake --build --preset default --target algorithm_test
  ./build/default/Algorithm/algorithm_test
'
```

Expected: **all** tests pass, including the new
`DroneActuallyMovesWhenGpsResolutionMatchesMapResolution`. If any pre-existing test in
`test_mapping_algorithm.cpp`, `test_wavefront_planner.cpp`, `test_scan_planning.cpp`, or
`test_path_shaping.cpp` newly fails, read its assertion: `reachedWaypoint`'s only caller is
the `nextStep` waypoint-advance loop, so a regression here means some existing test relied on
the old symmetric-tolerance's boundary behavior for its own pass/fail expectation. Re-derive
that test's expectation using the same voxel-membership reasoning as this task, don't revert
the fix to make it pass.

- [ ] **Step 4: Commit**

```bash
git add Algorithm/src/MappingAlgorithmImpl.cpp
git commit -m "fix: reachedWaypoint uses voxel membership, not symmetric distance tolerance"
```

(Wait for human approval before running.)

### Task 3: Re-measure VAR-01/02/03 and the 24-cell suite in `ex3-var01-center-lattice`

**Files:** none modified — measurement only.

**Interfaces:** N/A.

- [ ] **Step 1: Wipe stale build/bench-out state before measuring (clean paired comparison)**

```bash
Remove-Item -Recurse -Force build\opt -ErrorAction SilentlyContinue
Remove-Item -Recurse -Force tmp\bench-out -ErrorAction SilentlyContinue
```

Run from `C:\Users\sagi1\Projects\DroneMapper\ex3-var01-center-lattice`. This avoids stale
`.so`/object timestamp issues across the fix commit and matches the parent plan's "clean
paired measurement" discipline — do not reuse a `build/opt` that predates Task 2's commit.

- [ ] **Step 2: Rebuild Release and run `verify-cell-runtime`, `house_full` group first**

```bash
docker run --rm -e PYTHONUNBUFFERED=1 -e VCPKG_ROOT=/usr/local/vcpkg -v "${PWD}:/work" -w /work drone-mapper-ex3-dev bash -lc '
  set -euo pipefail
  cmake -S . -B build/opt -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE="$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake"
  cmake --build build/opt --target Algorithm_207190406_209543255 simulator_207190406_209543255 MissionControl_207190406_209543255
  python3 .cursor/skills/verify-cell-runtime/scripts/time_each_cell.py --build-dir /work/build/opt --only-group house_full
'
```

Expected: all 4 `house_full` cells `COMPLETED` with score materially above 0.0612 (compare
against the Approach A / baseline reference values 37.19 / 48.33 / 20.99 / 15.34 — exact
match is not required since Approach B's lattice/clearance model differs from Approach A's,
but a return to the same order of magnitude, and comfortably nonzero steps beyond 400, is the
bar). If any `house_full` cell still shows `steps` in the low hundreds with a near-zero score,
STOP — the fix did not fully resolve the coincidence; re-open Phase 1 investigation rather
than tuning the fix further (do not attempt a second unvalidated fix on top of this one, per
systematic-debugging's "one fix at a time, verify before continuing").

- [ ] **Step 3: Run the full 24-cell suite**

```bash
docker run --rm -e PYTHONUNBUFFERED=1 -e VCPKG_ROOT=/usr/local/vcpkg -v "${PWD}:/work" -w /work drone-mapper-ex3-dev bash -lc '
  python3 .cursor/skills/verify-cell-runtime/scripts/time_each_cell.py --build-dir /work/build/opt
'
cp tmp/bench-out/per-cell-wall.csv tmp/approach-b-house-full-fix-per-cell-wall.csv
```

Expected per Global Constraints: `cells_ge_60s=0`, `FAIL=0`. Compare the 20 non-`house_full`
cells' scores against this worktree's own pre-fix numbers (re-run Task 2's commit's parent, or
diff against the parent plan's Approach-B measurement notes if still available in
`tmp/approach-b-per-cell-wall.csv`, per the parent plan's Task 10) — they should be unchanged
or only trivially different (§2.2's boundary-case argument), not regressed. Record the new
24-cell score sum and compare to baseline **1769.835** per Global Constraints' ~50-90 point
flag threshold.

- [ ] **Step 4: Re-run `verify-independent-component-variants` VAR-01/02/03**

```bash
docker run --rm -e VCPKG_ROOT=/usr/local/vcpkg -v "${PWD}:/work" -w /work drone-mapper-ex3-dev bash -lc '
  set -euo pipefail
  cmake --build --preset default --target \
    skeleton_host \
    simulator_207190406_209543255 \
    Algorithm_207190406_209543255 \
    MissionControl_207190406_209543255 \
    foreign_hits_only_mission_control_plugin \
    adversarial_throw_algorithm_plugin \
    adversarial_never_finish_algorithm_plugin \
    adversarial_into_occupied_algorithm_plugin \
    adversarial_bad_scan_orientation_algorithm_plugin \
    adversarial_throw_mission_control_plugin \
    adversarial_empty_mission_control_plugin \
    adversarial_implausible_steps_mission_control_plugin
  chmod +x Simulator/tests/manual/check_foreign_host.sh Simulator/tests/manual/check_foreign_mission_control.sh Simulator/tests/manual/check_adversarial_plugins.sh
  bash Simulator/tests/manual/check_foreign_host.sh build/default
  bash Simulator/tests/manual/check_foreign_mission_control.sh build/default
  bash Simulator/tests/manual/check_adversarial_plugins.sh build/default
'
```

Expected: VAR-01 `PASS`, `HOST_ILLEGAL_MOVE_ATTEMPTS=0` (this fix does not touch clearance
geometry, so this should be unaffected — confirm, don't assume). VAR-02 diagnostic-only; VAR-03
must `PASS`.

- [ ] **Step 5: Record results**

Write PASS/FAIL, the `house_full` before/after score table, the 24-cell sum before/after, and
wall-clock sum before/after into a scratch note, e.g.
`tmp/approach-b-house-full-fix-results.md` (git-ignored, not part of any commit).

### Task 4: Report back to the user before any further action

**Files:** none — reporting only.

**Interfaces:** N/A.

- [ ] **Step 1: Present Task 3's numbers**

State whether the fix: (a) restores `house_full` to a non-degenerate score, (b) leaves the
other 20 cells' scores unchanged or only trivially different, (c) keeps VAR-01/03 passing, and
(d) keeps every cell under the 60s budget. If all four hold, recommend proceeding to land this
fix as part of Approach B's eventual real feature-branch implementation (a follow-up decision
already flagged as out of scope for the parent plan's own Task 11 — this plan does not decide
Approach A vs. B, it only fixes Approach B's `house_full`-specific defect so that comparison
can be made on equal footing).

- [ ] **Step 2: Do not merge into `known-issues-fixes` or delete the worktree**

Per the parent plan's Task 11/§8.4, cleanup and the Approach A vs. B decision remain the
user's call after seeing both approaches' final numbers side by side. This plan's commits stay
on `sphere-clearance-voxel-center-lattice` in `ex3-var01-center-lattice` until the user decides.
