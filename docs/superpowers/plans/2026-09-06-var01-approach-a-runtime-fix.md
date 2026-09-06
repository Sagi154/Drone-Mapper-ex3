# VAR-01 Approach A — `large_out` Runtime Regression Fix — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fix the `large_out|drone_small|lidar_short` (and related `large_out`) >60s
`verify-cell-runtime` regression introduced by Approach A's corner-anchored clearance fix
(`sphere-clearance-corner-lattice-fix`, worktree `../ex3-var01-corner-fix`), by bounding the
per-replan reachability search instead of always searching the whole map, while keeping
VAR-01 (`HOST_ILLEGAL_MOVE_ATTEMPTS=0`) passing.

**Architecture:** One targeted change to `Algorithm/src/WavefrontPlanner.cpp`'s `plan()`
function: cap `MappingAlgorithmFrontier::exploreReachable`'s `max_expansions` to a small,
constant "local" bound instead of the whole-map `maxExpansionsForMap(map)` value, with a
one-shot escalation to the full-map bound only when the local search finds no frontier
cluster at all. No change to clearance geometry (`sphereIntersectsCellBox` stays exactly as
Approach A already fixed it) and no change to VAR-01's mechanism.

**Tech Stack:** C++20, CMake presets (`default` = Debug, `opt` = Release), Docker image
`drone-mapper-ex3-dev`, GoogleTest, Python 3 (`time_each_cell.py`), bash harness scripts under
`Simulator/tests/manual/`.

## Global Constraints

(Carried over from `docs/superpowers/specs/2026-09-05-var01-sphere-clearance-fix-design.md`
§3/§4, plus this investigation's findings — apply to every task below.)

- Never touch `common/`, `Simulator/common_simulator/`, `MissionControl/common_mission_control/`.
- No wall-clock abort in `Algorithm/` or `MissionControl/`. `max_steps` is the only
  mission-length limit (`.cursor/skills/verify-cell-runtime/SKILL.md`).
- Do not change `sphereIntersectsCellBox` or any other clearance-geometry code — VAR-01 already
  passes on top of Approach A's geometry fix; this plan's job is runtime only, and re-touching
  geometry would reopen a closed, verified investigation.
- No cell may newly exceed the ~60s per-cell wall-clock budget (`verify-cell-runtime`), and no
  cell that currently passes may regress into WARN/FAIL territory beyond noise.
- 24-cell score sum should stay close to the Approach-A baseline (score_sum **1746.720**,
  itself already ~23 points below the pre-Approach-A clean baseline of **1769.835** — see
  Root Cause section). Prefer neutral-to-positive score movement; a large aggregate drop is a
  flag to discuss, not an auto-reject.
- VAR-01/02/03 must remain at PASS. Re-verify after this change, since bounding the search
  could in principle change which frontier a replan targets (it does not change *whether* a
  waypoint is passable — that predicate is untouched).
- Never silently withhold a movement on a clearance refusal — this plan does not change
  `DroneControlImpl::applyMovement`'s existing catch-and-continue behavior (that mechanism is
  pre-existing, not introduced by Approach A, and is explicitly out of scope for this plan; see
  Root Cause §3 for why it is not the thing being fixed here).
- No binary search / iterative refinement on the per-tick hot path. This plan's fix is a single
  bounded search with **at most one** fallback re-search (not a loop, not a shrink-until-clear
  pattern) — consistent with this constraint.
- One mechanism per commit, one benchmark run per commit. Do not combine the cap change with
  any other tuning in the same commit.
- Git workflow (`.cursor/rules/git-workflow.mdc`): branch names are kebab-case, name the
  component/fix (e.g. `bound-wavefront-replan-search`), never an item code or owner. Human
  approval required before every `git commit`.
- `Algorithm/` code style: `.cursor/rules/adv-cpp-standards.mdc` (no magic numbers — name
  constants; `-Wall -Wextra -Werror -pedantic` must stay clean).

---

## Root cause (confirmed empirically this session)

### 1. Symptom

Clean paired measurement (wipe `build/opt` + `tmp/bench-out`, rebuild, one process per cell,
`num_threads=1`), worktree `ex3-var01-corner-fix`:

| | score_sum | wall_sum | wall_max | cells_ge_60s | `large_out\|drone_small\|lidar_short` |
|---|---:|---:|---:|---:|---|
| Baseline (`08f3941`, pre-Approach-A) | 1769.835 | 186.3s | 50.1s | 0 | score 44.56, steps 4100, wall 50.1s **WARN** |
| Approach A (`f1844a6`) | 1746.720 | 239.9s | 79.9s | **1** | score 72.97, steps 5500, wall 79.9s **FAIL** |

`large_out|drone_small|lidar_short` was already the worst cell at baseline (WARN, 50.1s —
within a few seconds of the 60s bar), and Approach A's clearance fix pushed it over.

### 2. Instrumented evidence (uncommitted, worktree `ex3-var01-corner-fix`, reverted after use)

Added two throwaway counters (not committed, already reverted from the worktree):

- In `WavefrontPlanner::plan()`: count calls and total/avg/max wall time spent inside
  `MappingAlgorithmFrontier::exploreReachable` (the bounded BFS every replan runs).
- In `MappingAlgorithmImpl`: count `handleReplan`'s three trigger booleans
  (`plan_exhausted`, `interval_elapsed`, `cluster_dead`) and the movement-command type
  in effect when the stall/blocklist mechanism (`kMaxMovingStallTicks`) fires.

Running the single cell directly (`simulator_207190406_209543255 -comparative ...`, Release,
`DIAG_PERF=1`) gives, isolating the geometry as the only variable (same instrumentation, only
`Algorithm/src/MappingAlgorithmFrontier.cpp` swapped between the old buggy formula and
Approach A's fix via `git checkout <sha> -- <path>`):

| | plan() calls (= replans) | time in `exploreReachable` | `plan_exhausted` | of which stall-triggered | wall (this run) |
|---|---:|---:|---:|---:|---:|
| Old (buggy) geometry + instrumentation | 942 | 39.1s | 1161 | 1139 | 44.9s |
| Approach A geometry + instrumentation | 1235 | 57.3–58.1s | 1299 | 1279 | ~65s (noise: 65–80s across runs) |

Stall breakdown by the movement command in effect when the stall fired (both geometries):
mostly `Advance`/`Elevate` (genuine rejected moves, not natural rotation pauses) — e.g. Approach
A: `advance=520 elevate=613 rotate=146`.

`maxExpansionsForMap` for this cell's map (300×300×300cm world, 10cm resolution, from
`inputs/simulation/large_simulation_out.yaml` + `inputs/mission/large_mission_out.yaml`) is
31×31×31 = **29,791** — the search bound `WavefrontPlanner::plan()` passes to
`exploreReachable` on **every single replan**, regardless of whether the replan was triggered
by a genuine map-wide event or by one single waypoint getting blocklisted.

### 3. Interpretation

Two compounding facts, both confirmed by the numbers above, not guessed:

1. **The full-map-search-per-replan pattern is pre-existing, not introduced by Approach A.**
   Even with the old, buggy geometry, this cell already replans ~942 times over ~4100 steps
   (roughly every 4.3 steps) and spends 39.1s of its ~45–50s wall budget inside
   `exploreReachable`. `handleReplan`'s `interval_elapsed` (12) and `cluster_dead` (2) triggers
   are negligible; **`plan_exhausted`, driven almost entirely by the stall→blocklist
   mechanism (1139 of 1161), is what forces nearly every replan.** The stall mechanism itself
   fires because the planner's own lattice-point clearance check
   (`isSpherePassable`/`hasClearLineOfSight`, evaluated at plan time) and the simulator's
   independent, differently-sampled clearance check
   (`MockMovement::advance/elevate` → `sphereHitsOccupiedOrOutOfBounds` →
   `forEachSphereSample`, evaluated at the drone's actual — frequently off-lattice, because
   `movementToward` clamps each Advance/Elevate to `max_advance_cm`/`max_elevate_cm` and a
   waypoint can be several grid steps away — execution-time position) occasionally disagree.
   When they disagree, `MockMovement` throws, `DroneControlImpl::applyMovement` catches it and
   silently no-ops (pre-existing "refuse and hope" behavior, not something this plan adds),
   the drone's position doesn't change for `kMaxMovingStallTicks` (2) ticks, the pending
   waypoint's cell gets blocklisted, `has_plan` is cleared, and the next tick is
   `plan_exhausted` → a fresh **full 29,791-node bounded search**.
2. **Approach A's corrected clearance geometry amplifies both sides of this pre-existing
   cost.** Correctly treating corner-touching neighbour voxels as blocked (distance 0, not the
   old wrong 5cm) makes more of the outer layer next to any obstacle impassable, which (a)
   increases the replan rate by ~31% (942→1235) because more waypoints end up adjacent to
   newly-blocked cells, and (b) increases the average cost of each bounded search (41.5ms→
   46.7ms avg) because more neighbours are probed-and-rejected during each BFS expansion.
   Combined, time inside `exploreReachable` rises **from 39.1s to 57.3–58.1s (+47%)** — enough
   to push an already-marginal WARN-tier cell (44.9–50.1s) over the 60s FAIL bar (65–80s).

**Root cause, in one sentence:** every replan — regardless of whether it was triggered by a
single blocklisted waypoint or a genuine map-wide event — pays the cost of a bounded search
sized to the *entire* map (29,791 nodes for `large_out`), and Approach A's necessary clearance
correction increases both how often that full-cost search runs and how expensive each run is,
tipping an already near-the-bar cell into FAIL. The fix targets the search's cost, not the
clearance geometry (which is correct and must not change) and not the stall mechanism itself
(which is a separate, pre-existing concern — see "Explicitly out of scope" below).

### 4. Fix validated experimentally (uncommitted, already reverted)

Prototype: cap `exploreReachable`'s `max_expansions` in `WavefrontPlanner::plan()` to
`min(maxExpansionsForMap(map), 3000)`, with **one** fallback re-search at the full map bound
only if the capped search finds `reach.clusters.empty()`. Rebuilt Release, re-ran with
Approach A's geometry (unchanged):

| Cell | Approach A (uncapped) | Approach A + 3000-node cap |
|---|---|---|
| `large_out\|small\|long` | score 79.77, steps 4700, wall 38.0s | score 73.50, steps 1700, wall 0.8s |
| `large_out\|small\|short` | score 72.97, steps 5500, wall 79.9s **FAIL** | score 74.95, steps 4100, wall 2.5s |
| `large_out\|large\|long` | score 72.96, steps 3800, wall 33.5s | score 75.59, steps 3400, wall 1.4s |
| `large_out\|large\|short` | score 37.72, steps 1300, wall 22.4s | score 72.02, steps 3500, wall 2.6s |
| **group wall_sum** | **173.8s** | **7.4s** |
| **group score_sum** | **263.4** | **296.1** |

All four `large_out` cells drop to sub-3s wall time (from a group that previously summed to
174s with one FAIL), and the group's score sum *improves* by ~33 points — the full-map search
was not just slow, it was also picking worse (more distant / more obstacle-adjacent, thus
more stall-prone) targets than a nearby-first search does. `plan_calls` for the `short`-lidar
cell dropped from 1235 to 222 with the cap, confirming the cap also reduces the stall rate
itself (a smaller search naturally prefers closer, less obstacle-hugging frontier targets), not
just the cost per replan.

This is strong first evidence, not a final answer — Task 3 below runs the full 24-cell suite
(not just `large_out`) to confirm no other group regresses, and Task 4 tunes the exact cap
value.

### 5. Explicitly out of scope for this plan

- **The stall/blocklist mechanism's false-positive rate itself** (why `MockMovement`'s
  independent geometry and the planner's lattice geometry disagree at off-lattice execution
  points ~1100–1300 times per `large_out` mission). This is a genuine, separate, pre-existing
  planner/executor mismatch that predates Approach A (measured 1139 stall events with the *old*
  geometry too) and is not required to fix VAR-01 or the 60s budget — Task 2's cap fixes the
  *cost* of reacting to those stalls, which is sufficient. Root-causing and fixing the mismatch
  itself (e.g. validating the predicted post-move pose before emitting a movement, or aligning
  `movementToward`'s step size to the lattice) is a larger, riskier change touching the
  movement-emission hot path and the constraint against "refuse and hope" gates (design spec
  lesson 1) — worth its own future investigation, not bundled here.
- **`DroneControlImpl::applyMovement`'s catch-and-continue-silently behavior.** Pre-existing,
  not introduced by this investigation's fix, not touched.
- **Any change to `sphereIntersectsCellBox` or other clearance-geometry code.** VAR-01 already
  passes; do not re-open it.
- **The pre-existing WARN-tier baseline itself** (this cell was already at 44.9–50.1s before
  Approach A). This plan's bar is "does not newly regress past 60s and does not materially
  hurt scores," not "make this cell as fast as possible."

---

## Task 1: Add the bounded local-search cap (with full-map fallback) to `WavefrontPlanner::plan()`

**Files:**
- Modify: `Algorithm/src/WavefrontPlanner.cpp` (the `plan()` function's call to
  `frontier_.exploreReachable`, currently unconditionally passed `maxExpansionsForMap(in.map)`)
- Modify: `Algorithm/tests/test_wavefront_planner.cpp` (add coverage for the new bound; locate
  existing tests first — see Step 1)

**Interfaces:**
- Consumes: `MappingAlgorithmFrontier::exploreReachable(map, start, radius, blocked,
  max_expansions) -> ReachabilityResult` (existing, unchanged signature — this task only
  changes what `max_expansions` value `WavefrontPlanner::plan()` passes in, not the function
  itself). `maxExpansionsForMap(map) -> std::size_t` (existing, unchanged, in
  `Algorithm/src/MappingAlgorithmFrontier.h`/`.cpp`).
- Produces: `WavefrontPlanner::plan()`'s public behavior is unchanged in *signature*; only its
  internal search cost and (for maps with more than `kLocalSearchExpansionCap` reachable
  voxels) its target-selection distribution changes. No caller of `WavefrontPlanner::plan()`
  needs to change.

Before writing code, locate the exact current call site (it may have shifted a few lines from
this investigation's session):

```bash
grep -n "exploreReachable\|maxExpansionsForMap" Algorithm/src/WavefrontPlanner.cpp
```

- [ ] **Step 1: Read the existing `test_wavefront_planner.cpp` to find its `FakeMap3D`/`Map`
  helper and an existing test that already exercises `WavefrontPlanner::plan()` end-to-end**

```bash
grep -n "TEST(WavefrontPlanner\|WavefrontInputs{" Algorithm/tests/test_wavefront_planner.cpp
```

Use whichever existing helper builds a `WavefrontInputs` (map, drone state, lidar config,
drone config, remaining steps, blocked cells) — this task's new test reuses it, not a new one.

- [ ] **Step 2: Write a failing test — a large map with a distant-only frontier must still be
  found via the fallback**

Add to `Algorithm/tests/test_wavefront_planner.cpp` (near the other `plan()` tests):

```cpp
// What: a map larger than the local search cap, with the ONLY unmapped (frontier-worthy)
// region far from the drone's start position — farther than a small bounded search would
// reach. Expected: plan() must still find it via the full-map fallback (this is the guard
// against the local cap silently making distant frontiers unreachable — design spec
// docs/superpowers/plans/2026-09-06-var01-approach-a-runtime-fix.md Root Cause §4).
TEST(WavefrontPlanner, PlanFallsBackToFullMapSearchWhenLocalCapFindsNoFrontier) {
    // 41x41x3 voxels at 10cm resolution = 5043 reachable-search-space voxels, comfortably
    // larger than the intended ~3000-node local cap (Task 4 tunes the exact constant; this
    // test only needs "larger than whatever the cap turns out to be" to be meaningful, so
    // assert the map's total voxel count is bigger than the cap constant directly).
    const ct::MapConfig config = makeCm10Config();  // adjust to this file's actual helper name
    Map map{{41, 41, 3}, config, ct::VoxelOccupancy::Empty};
    // Fill everything Empty except one small Unmapped pocket at the far corner from start.
    const Position3D start = pointCm(10, 10, 10);
    const Position3D far_pocket = pointCm(390, 390, 10);
    map.set(far_pocket, ct::VoxelOccupancy::Unmapped);

    ASSERT_GT(41 * 41 * 3, static_cast<int>(detail::kLocalSearchExpansionCap));

    const WavefrontPlanner planner;
    const WavefrontInputs inputs{
        map, /* state at start, heading 0 */ makeState(start), makeLidarConfig(),
        makeDroneConfig(), /* remaining_steps */ 5000, /* blocked */ {}, /* ignore_blocked */
        false, /* prefer_descend */ false,
    };
    const ExplorationPlan result = planner.plan(inputs);
    EXPECT_TRUE(result.valid);
    EXPECT_FALSE(result.waypoints.empty());
}
```

(Adjust the `WavefrontInputs`/`DroneState`/`LidarConfigData` construction to match this file's
actual existing helpers — do not guess field names; grep the file for the struct literal used
by an existing passing test and mirror it exactly.)

- [ ] **Step 3: Run it and confirm it currently passes (sanity: full-map search already finds
  distant frontiers) — this test is a regression guard, not a red/green TDD test, because the
  fallback doesn't exist as a distinct code path yet; it's still testing real behavior**

```bash
docker run --rm -e VCPKG_ROOT=/usr/local/vcpkg -v "${PWD}:/work" -w /work drone-mapper-ex3-dev bash -lc '
  cmake --build --preset default --target algorithm_test
  ./build/default/Algorithm/algorithm_test --gtest_filter="WavefrontPlanner.PlanFallsBackToFullMapSearchWhenLocalCapFindsNoFrontier"
'
```

Expected: PASS (this test's job is to catch a *future* regression once Step 4 adds the cap —
run it again after Step 4 to confirm it still passes with the cap in place).

- [ ] **Step 4: Add the capped search with one-shot full-map fallback**

In `Algorithm/src/WavefrontPlanner.cpp`, add the named constant near the top of the anonymous
namespace (alongside `kMaxSweepReserve`):

```cpp
// Reachability search bound for an ordinary replan. Chosen so a single replan's bounded BFS
// (runBoundedSearch, MappingAlgorithmFrontier.cpp) costs low-single-digit milliseconds even on
// the largest sim_compose.yaml map (large_out, 31x31x31 = 29,791 voxels), instead of scaling
// with the whole map on every stall-triggered replan. See root-cause investigation in
// docs/superpowers/plans/2026-09-06-var01-approach-a-runtime-fix.md. Tuned empirically in
// Task 4; if you change this value, re-run verify-cell-runtime on the full 24-cell suite.
constexpr std::size_t kLocalSearchExpansionCap = 3000;
```

Replace the existing unconditional call:

```cpp
const ReachabilityResult reach = frontier_.exploreReachable(
    in.map, in.state.position, in.drone.radius, blocked, maxExpansionsForMap(in.map));
```

with:

```cpp
const std::size_t full_map_cap = maxExpansionsForMap(in.map);
const std::size_t local_cap = std::min(full_map_cap, kLocalSearchExpansionCap);
ReachabilityResult reach =
    frontier_.exploreReachable(in.map, in.state.position, in.drone.radius, blocked, local_cap);
if (reach.start_passable && reach.clusters.empty() && local_cap < full_map_cap) {
    // The bounded local search found no frontier cluster at all (as opposed to "found
    // clusters but they were all too far to afford" — buildCandidatePlans already handles
    // that via the `travel + reserve > remaining_steps` budget check). Escalate once to a
    // full-map search rather than reporting "nothing left to explore" prematurely. This is a
    // single fallback attempt, not a loop/binary-search (Global Constraints: no iterative
    // refinement on the hot path) — if the full-map search also finds nothing, plan()
    // correctly falls through to its existing "no clusters" empty-plan return below.
    reach = frontier_.exploreReachable(in.map, in.state.position, in.drone.radius, blocked,
                                       full_map_cap);
}
```

Note `reach` changes from `const` to non-`const` (it is conditionally reassigned once). No
other line in `plan()` needs to change — every subsequent use of `reach` (cluster ranking,
candidate building, `reach.start_key`, `reach.parent_of`) already operates on whichever
`ReachabilityResult` ends up bound to the name.

- [ ] **Step 5: Rebuild and run both the new test and the full `Algorithm` suite**

```bash
docker run --rm -e VCPKG_ROOT=/usr/local/vcpkg -v "${PWD}:/work" -w /work drone-mapper-ex3-dev bash -lc '
  cmake --build --preset default --target algorithm_test
  ./build/default/Algorithm/algorithm_test
'
```

Expected: 100% pass, including
`WavefrontPlanner.PlanFallsBackToFullMapSearchWhenLocalCapFindsNoFrontier`. If any pre-existing
`WavefrontPlanner`/`MappingAlgorithm` test that asserts specific waypoints/targets on a map
larger than 3000 voxels starts failing, that test was relying on the full-map search finding a
*specific* far-away cluster that the capped search now finds a *closer* one instead — read the
assertion, confirm whether the closer target is still a *reasonable* answer (usually yes, since
`rankClusters` already prefers cheaper/closer clusters when scores tie), and update the
expected value with a comment explaining the new, closer expected target. Do not weaken the
assertion to "any valid plan" — keep it specific.

- [ ] **Step 6: Commit**

```bash
git add Algorithm/src/WavefrontPlanner.cpp Algorithm/tests/test_wavefront_planner.cpp
git commit -m "fix: bound replan reachability search to avoid full-map cost per stall"
```

(Wait for human approval per `git-workflow.mdc` before running this commit.)

---

## Task 2: Re-verify VAR-01/02/03 are unaffected

**Files:** none modified — measurement only.

**Interfaces:** N/A.

The cap changes *which* frontier a replan targets when the map has more than 3000 reachable
voxels; it does not change the clearance predicate VAR-01 exercises
(`isSpherePassable`/`sphereIntersectsCellBox`, untouched by this plan). This task confirms that
empirically rather than assuming it.

- [ ] **Step 1: Rebuild the VAR-01/02/03 targets**

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
'
```

- [ ] **Step 2: Run VAR-01**

```bash
docker run --rm -e VCPKG_ROOT=/usr/local/vcpkg -v "${PWD}:/work" -w /work drone-mapper-ex3-dev bash -lc '
  chmod +x Simulator/tests/manual/check_foreign_host.sh
  bash Simulator/tests/manual/check_foreign_host.sh build/default
'
```

Expected: `PASS: check_foreign_host`, `HOST_ILLEGAL_MOVE_ATTEMPTS=0` for both `small_room` and
`house_lower` — identical to the result already recorded for Approach A before this plan's
change (`tmp/approach-a-var-results.md` in the worktree, if still present). If it regresses,
stop — that would mean the cap somehow changed clearance behavior, which Task 1 should not have
done; re-read the diff before proceeding.

- [ ] **Step 3: Run VAR-02 and VAR-03**

```bash
docker run --rm -e VCPKG_ROOT=/usr/local/vcpkg -v "${PWD}:/work" -w /work drone-mapper-ex3-dev bash -lc '
  chmod +x Simulator/tests/manual/check_foreign_mission_control.sh Simulator/tests/manual/check_adversarial_plugins.sh
  bash Simulator/tests/manual/check_foreign_mission_control.sh build/default
  bash Simulator/tests/manual/check_adversarial_plugins.sh build/default
'
```

Expected: both exit 0 (VAR-02 diagnostic; VAR-03 must PASS including the `bad_scan` timeout
canary).

- [ ] **Step 4: Record results** in a scratch note (`tmp/approach-a-runtime-fix-var-results.md`,
  git-ignored) for the comparison table in Task 5.

---

## Task 3: Measure the full 24-cell suite (clean paired, same methodology as the original fix)

**Files:** none modified — measurement only.

**Interfaces:** N/A.

- [ ] **Step 1: Wipe and rebuild Release**

```bash
docker run --rm -e PYTHONUNBUFFERED=1 -e VCPKG_ROOT=/usr/local/vcpkg -v "${PWD}:/work" -w /work drone-mapper-ex3-dev bash -lc '
  set -euo pipefail
  rm -rf build/opt tmp/bench-out
  cmake -S . -B build/opt -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE="$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake"
  cmake --build build/opt -j$(nproc) --target Algorithm_207190406_209543255 simulator_207190406_209543255 MissionControl_207190406_209543255
'
```

(Wiping the whole `build/opt` here, not just the Algorithm object files, because this is the
"clean paired measurement" bar the original fix's plan used for its final comparison table —
see `docs/superpowers/plans/2026-09-05-var01-sphere-clearance-fix.md` Global Constraints. A
targeted object-file wipe is fine for iterating during Task 1/4, but use the full wipe for the
number that goes in this plan's final report.)

- [ ] **Step 2: Run the full 24-cell suite**

```bash
docker run --rm -e PYTHONUNBUFFERED=1 -e VCPKG_ROOT=/usr/local/vcpkg -v "${PWD}:/work" -w /work drone-mapper-ex3-dev bash -lc '
  python3 .cursor/skills/verify-cell-runtime/scripts/time_each_cell.py --build-dir /work/build/opt
'
cp tmp/bench-out/per-cell-wall.csv tmp/approach-a-runtime-fix-per-cell-wall.csv
```

Expected per Global Constraints: `cells_ge_60s=0`, no cell that currently passes newly
WARNs/FAILs, score_sum comparable to or better than Approach A's **1746.720** (Task 1's
`large_out`-only prototype improved that group's score_sum by ~33 points and its wall_sum by
~166s — confirm the full 24-cell number here, not just the one group).

- [ ] **Step 3: Diff against the Approach-A-uncapped baseline**

Compare `tmp/approach-a-runtime-fix-per-cell-wall.csv` against
`tmp/approach-a-clean-per-cell-wall.csv` (already present in the worktree from the original
investigation, if not cleaned up — regenerate via Task 1's original methodology if missing:
checkout `f1844a6` cleanly with no local-search cap, wipe+rebuild, run the full suite once,
before re-applying this plan's Task 1 commit). Record per-cell deltas, flag any cell whose
score moved by more than a few points in either direction with an explanation (e.g. "fewer
steps used because the mission finished via `FinishedWithUnmappableVoxels` earlier — check
`low_observed_windows`/`kLowObservedWindows` termination, not a bug, just a different explore
order").

---

## Task 4: Tune `kLocalSearchExpansionCap` if Task 3 shows a problem

**Files:**
- Modify: `Algorithm/src/WavefrontPlanner.cpp` (the `kLocalSearchExpansionCap` constant only)

**Interfaces:** unchanged from Task 1.

Only do this task if Task 3's full 24-cell run shows a regression the 3000 default doesn't
already fix cleanly (e.g. a different cell now WARNs, or a score drops more than a few points
somewhere). If Task 3 is already clean, skip this task entirely — do not tune a value that
isn't causing a problem (YAGNI).

- [ ] **Step 1: If a cell's score regressed, check whether raising the cap helps**

Try one alternative value (e.g. `6000`, double the default) via the existing `DIAG_LOCAL_CAP`
pattern from this investigation is **not** available anymore (that was throwaway, already
reverted) — instead, edit the constant directly, rebuild, re-run just the affected
`--only-group`, compare. Change **one** value per rebuild+measure cycle (Global Constraints:
one mechanism per commit — this applies to tuning cycles too, don't change the cap and
something else in the same test).

- [ ] **Step 2: If a cell's wall time regressed (still fine on score), check whether lowering
  the cap helps**

Same method, try a smaller value (e.g. `1500`).

- [ ] **Step 3: Once a value passes Task 3's full-suite bar, update the constant's comment**
  with the final chosen value and a one-line justification, then re-run Task 3's Step 2 in full
  to confirm, and commit as a separate, clearly-labeled follow-up commit (not squashed into
  Task 1's commit):

```bash
git add Algorithm/src/WavefrontPlanner.cpp
git commit -m "tune: adjust replan search cap after full 24-cell measurement"
```

---

## Task 5: Build the before/after comparison and report

**Files:**
- Create: `tmp/approach-a-runtime-fix-comparison.md` (scratch, git-ignored — a report to paste
  into chat; copy into `docs/` only if the user asks to keep it)

**Interfaces:** N/A — reporting only.

- [ ] **Step 1: Build the comparison table**

Columns: cell, Approach-A-uncapped (score/steps/wall/verdict), Approach-A-capped
(score/steps/wall/verdict), delta. Highlight the `large_out` group specifically (the group this
plan targets) and the 24-cell aggregate (score_sum, wall_sum, wall_max, cells_ge_60s).

- [ ] **Step 2: State whether the fix is ready to land**

Decision bar (same spirit as the original fix's spec §8.3): qualifies if VAR-01/02/03 still
PASS (Task 2), `cells_ge_60s=0` across all 24 cells (Task 3), and score_sum is not more than a
few points below Approach A's **1746.720** (ideally at or above it, per Task 1's prototype
evidence). If it qualifies, this fix is ready to be committed on top of the
`sphere-clearance-corner-lattice-fix` branch in the `ex3-var01-corner-fix` worktree (or
re-applied to whatever branch Approach A ultimately lands on) — actually committing there is a
follow-up action requiring the user's go-ahead, not part of this plan's execution.

- [ ] **Step 3: Confirm the worktree is clean**

```bash
git -C ../ex3-var01-corner-fix status --short
```

Expected: only this plan's Task 1/4 commits show in `git log`; no stray uncommitted
instrumentation, no leftover `DIAG_PERF`/`DIAG_LOCAL_CAP` env-gated code (that was this
investigation's throwaway scaffolding, already reverted before this plan was written — Task 1
introduces no env-var-gated code, only the permanent capped-search change).
