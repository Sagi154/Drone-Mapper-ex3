# VAR-01 Sphere-Clearance Fix — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `verify-independent-component-variants` VAR-01 (`check_foreign_host.sh`) pass
(`HOST_ILLEGAL_MOVE_ATTEMPTS=0`) by prototyping two candidate fixes to the mapping algorithm's
sphere-clearance geometry — each in its own isolated git worktree — measuring both against
VAR-01, VAR-02/03, the 24-cell `verify-cell-runtime` score/wall-clock suite, and the
`Algorithm/tests` unit suite, then reporting a comparison so the user can pick a winner to land
on a real feature branch.

**Architecture:** Two parallel, isolated worktrees (`../ex3-var01-corner-fix`,
`../ex3-var01-center-lattice`), each branched from `known-issues-fixes`. Approach A
(corner-fix) changes one geometry function. Approach B (center-lattice) re-anchors the
navigation lattice and its dependent call sites. Both are measured with the same three
harnesses, then compared and reported — no merge back to `known-issues-fixes` happens in this
plan; that is a follow-up decision by the user.

**Tech Stack:** C++20, CMake presets (`default` = Debug, `opt` = Release), Docker image
`drone-mapper-ex3-dev`, GoogleTest, Python 3 (`time_each_cell.py`), bash harness scripts under
`Simulator/tests/manual/`.

## Global Constraints

(From `docs/superpowers/specs/2026-09-05-var01-sphere-clearance-fix-design.md` §3, §4 — copied
verbatim, apply to every task below.)

- Never touch `common/`, `Simulator/common_simulator/`, `MissionControl/common_mission_control/`.
- No wall-clock abort in `Algorithm/` or `MissionControl/`. `max_steps` is the only
  mission-length limit (`.cursor/skills/verify-cell-runtime/SKILL.md`).
- 24-cell score sum on `inputs/sim_compose.yaml` must stay close to baseline **1769.835**
  (`docs/benchmarks/2026-09-03-*`); treat an aggregate drop beyond ~50-90 points as a flag to
  discuss with the user, not an auto-reject. Individual cell drift is acceptable.
- No cell may newly exceed the ~60s per-cell wall-clock budget.
- VAR-02/03/04 must remain at their current status; VAR-04 is out of scope for this plan (the
  spec only requires VAR-02/03 re-checked).
- Never silently withhold a movement on a clearance refusal (spec §4 lesson 1) — this plan's
  approaches fix the planner's belief about the world, so no execution-time "refuse and
  no-op" gate is introduced by any task below.
- No binary search / iterative refinement on the per-tick hot path (spec §4 lesson 2).
- Any predicate change must keep "drone one full step above the floor it just mapped remains
  passable" true — every new/changed unit test in this plan checks that explicitly.
- One mechanism per commit, one benchmark run per commit (spec §4 lesson 5). Do not squash
  Approach A and Approach B changes, or multiple compensating mechanisms, into one commit.
- Git workflow (`.cursor/rules/git-workflow.mdc`): branch names are kebab-case and name the
  component/fix, never an item code or owner. Human approval required before every
  `git commit` — present the diff and message, wait for explicit approval, in both worktrees.
- `Algorithm/`, `UserCommon/` code style: `.cursor/rules/adv-cpp-standards.mdc` (no magic
  numbers — name constants; `-Wall -Wextra -Werror -pedantic` must stay clean) and
  `.cursor/rules/mp-units-strong-types.mdc` (strong types at API boundaries; unwrap to
  `double` only inside a `.cpp` for hot-path math, exactly as the existing code already does
  in `MappingAlgorithmFrontier.cpp`).
- Explicitly out of scope (spec §7): the `house_lower` spawn/offset bug and
  `Simulator/src/MapsComparison.cpp`'s empty-universe-scores-100 behavior. Do not fix these
  even if encountered — note them and move on.

---

## Task 0: Set up both worktrees

**Files:**
- None modified — this task only creates worktrees and verifies the baseline builds in each.

**Interfaces:** N/A (setup task).

- [ ] **Step 1: Confirm the current branch is committed and clean**

```bash
cd c:\Users\sagi1\Projects\DroneMapper\Drone-Mapper-ex3
git status --short
```

Expected: no output (clean tree). If there is output, stop and ask the user before proceeding
— do not create worktrees from a dirty tree.

- [ ] **Step 2: Create the two worktrees**

```bash
git worktree add ../ex3-var01-corner-fix known-issues-fixes
git worktree add ../ex3-var01-center-lattice known-issues-fixes
git worktree list
```

Expected: `git worktree list` shows three entries — the original checkout plus the two new
ones, each on a detached-from-`known-issues-fixes` state. In each new worktree, immediately
create its own branch (do not stay on `known-issues-fixes` in a second worktree — git forbids
checking out the same branch twice, and this also matches `git-workflow.mdc`'s "one feature
branch per logical change"):

```bash
cd ../ex3-var01-corner-fix
git checkout -b sphere-clearance-corner-lattice-fix
cd ../ex3-var01-center-lattice
git checkout -b sphere-clearance-voxel-center-lattice
```

- [ ] **Step 3: Verify each worktree builds clean before any change**

Run in **both** worktrees (substitute the path):

```bash
cd ../ex3-var01-corner-fix
docker run --rm -e VCPKG_ROOT=/usr/local/vcpkg -v "${PWD}:/work" -w /work drone-mapper-ex3-dev bash -lc '
  set -euo pipefail
  cmake --preset default
  cmake --build --preset default --target skeleton_host simulator_207190406_209543255 Algorithm_207190406_209543255 MissionControl_207190406_209543255 foreign_hits_only_mission_control_plugin adversarial_throw_algorithm_plugin adversarial_never_finish_algorithm_plugin adversarial_into_occupied_algorithm_plugin adversarial_bad_scan_orientation_algorithm_plugin adversarial_throw_mission_control_plugin adversarial_empty_mission_control_plugin adversarial_implausible_steps_mission_control_plugin baseline_lawnmower_algorithm_plugin
  ctest --preset default --output-on-failure -R Algorithm
'
```

Expected: build succeeds, and the `Algorithm` ctest suite passes (this is the pre-change
baseline — every test must be green before any geometry change).

Repeat identically in `../ex3-var01-center-lattice`.

- [ ] **Step 4: Copy the diagnostic instrumentation into each worktree (already on `known-issues-fixes`, so it's already present via the branch — just confirm)**

```bash
git -C ../ex3-var01-corner-fix log --oneline -1 -- Simulator/tests/hosts/skeleton_host/src/HostMovement.cpp
git -C ../ex3-var01-center-lattice log --oneline -1 -- Simulator/tests/hosts/skeleton_host/src/HostMovement.cpp
```

Expected: both show commit `d6ff6ae` ("test: add HOST_DIAG_ILLEGAL diagnostic...") as the most
recent touch, confirming the instrumentation carried over via the branch point. No action
needed if so.

- [ ] **Step 5: Copy the measurement scripts into each worktree**

The scratch scripts `tmp/diag_var01.sh` and `tmp/measure.sh` from the original checkout are
git-ignored (`tmp/` is not tracked) and won't exist in the new worktrees. Copy them:

```bash
Copy-Item "c:\Users\sagi1\Projects\DroneMapper\Drone-Mapper-ex3\tmp\diag_var01.sh" "..\ex3-var01-corner-fix\tmp\diag_var01.sh"
Copy-Item "c:\Users\sagi1\Projects\DroneMapper\Drone-Mapper-ex3\tmp\measure.sh" "..\ex3-var01-corner-fix\tmp\measure.sh"
Copy-Item "c:\Users\sagi1\Projects\DroneMapper\Drone-Mapper-ex3\tmp\diag_var01.sh" "..\ex3-var01-center-lattice\tmp\diag_var01.sh"
Copy-Item "c:\Users\sagi1\Projects\DroneMapper\Drone-Mapper-ex3\tmp\measure.sh" "..\ex3-var01-center-lattice\tmp\measure.sh"
```

(If those files no longer exist because `tmp/` was cleaned, recreate `tmp/diag_var01.sh` with
the content from Task 2 Step 4 of this plan's Approach-A measurement task below, and
`tmp/measure.sh` per the same pattern — both are simple bash wrappers, not graded artifacts.)

---

## Approach A: host-exact corner-anchored clearance (worktree `../ex3-var01-corner-fix`)

### Task 1: Write failing regression tests for the corner-clearance geometry

**Files:**
- Modify: `Algorithm/tests/test_mapping_algorithm_frontier.cpp:307-393` (three existing tests
  whose expectations encode the old, wrong center-box geometry)
- Modify: `Algorithm/tests/test_mapping_algorithm_frontier.cpp` (add new tests, insert after
  line 393, before `TEST(MappingAlgorithm, ExploreReachableFindsFrontierAdjacentCandidates)`)

**Interfaces:**
- Consumes: `detail::MappingAlgorithmFrontier::exploreReachable(map, start, radius, blocked,
  max_expansions) -> ReachabilityResult` (existing, unchanged signature); `ReachabilityResult`
  has bool member `start_passable`. `detail::hasNotMappedInSphere(map, centre, radius) ->
  bool` (existing, unchanged signature). `AlgorithmTest::FakeMap3D` constructor
  `FakeMap3D{dims, config, default_val}`; `map.set(pos, occupancy)`.
- Produces: nothing new consumed by later tasks — this task only adds/fixes test assertions.

Before writing code, understand exactly why the old tests are wrong. On a 10 cm grid, a
lattice point is a voxel **corner** (§2.1 of the spec): the point where up to 8 voxels meet.
`isSpherePassable`'s neighbour loop probes candidate voxels at
`centre + (dx,dy,dz)·step_cm` for `dx,dy,dz ∈ {-1,0,1}` (`radius ≤ step` in every drone
config, so `ceil(radius/step) == 1`). Two physically distinct cases:

- **`dx,dy,dz ∈ {-1,0}` (any combination, not all zero):** the probed voxel is one of the 7
  *other* voxels sharing the corner with the centre voxel — it **touches the corner at
  distance 0**. Any radius `> 0` overlaps it if it is `Occupied`.
- **Any axis `= +1`:** the probed voxel starts a **full `step_cm` away** from the corner (its
  near face is at `+step_cm`, not touching the corner at all). It only overlaps the sphere if
  `radius ≥ step_cm`.

This is what the corrected `sphereIntersectsCellBox` computes (Task 2). The **existing** tests
below assert the *old, wrong* midpoint-based distance (`step_cm/2`, i.e. 5 cm on a 10 cm grid)
for the `+1` case, which is neither the touching-corner distance (0) nor the true far-face
distance (`step_cm`, i.e. 10 cm) — fix each one to the correct value.

- [ ] **Step 1: Update `FrontierRejectsOccupiedFaceNeighbourOnCm10Grid` (line 307-317)**

Replace:

```cpp
// What: 10 cm grid, radius 7.5 cm, Occupied face neighbour (nearest box dist 5 ≤ 7.5).
// Expected: start not passable — the old centre-distance gate silently skipped this probe.
TEST(MappingAlgorithm, FrontierRejectsOccupiedFaceNeighbourOnCm10Grid) {
    const ct::MapConfig config = makeCm10Config();
    Map map{{11, 11, 11}, config};
    const Position3D centre = pointCm(50, 50, 50);
    const Position3D face   = pointCm(60, 50, 50);
    map.set(centre, ct::VoxelOccupancy::Empty);
    map.set(face, ct::VoxelOccupancy::Occupied);

    const detail::MappingAlgorithmFrontier frontier;
    EXPECT_FALSE(frontier.exploreReachable(map, centre, 7.5 * cm, {}, 1).start_passable);
}
```

with:

```cpp
// What: 10 cm grid. `face` at +1 step is a corner-anchored voxel spanning [60,70) — its
// near face is a full 10 cm from `centre` (50,50,50), not 5 cm (that was the old, wrong
// midpoint-box model). A sphere reaches it only once radius >= step_cm (10 cm here).
// Expected: start not passable exactly at radius == step_cm (touching, inclusive).
TEST(MappingAlgorithm, FrontierRejectsOccupiedNeighbourAtFullStepWhenRadiusReachesIt) {
    const ct::MapConfig config = makeCm10Config();
    Map map{{11, 11, 11}, config};
    const Position3D centre = pointCm(50, 50, 50);
    const Position3D face   = pointCm(60, 50, 50);
    map.set(centre, ct::VoxelOccupancy::Empty);
    map.set(face, ct::VoxelOccupancy::Occupied);

    const detail::MappingAlgorithmFrontier frontier;
    EXPECT_FALSE(frontier.exploreReachable(map, centre, 10.0 * cm, {}, 1).start_passable);
}
```

- [ ] **Step 2: Update the comment on `FrontierAllowsOccupiedFaceNeighbourWhenRadiusTooSmall` (line 321-322) — assertion is unchanged, only the derivation comment is wrong**

Replace:

```cpp
// What: same Occupied face neighbour but radius 4 cm (nearest 5 > 4).
// Expected: still passable — sphere does not reach the neighbour box.
```

with:

```cpp
// What: same Occupied face neighbour (full step away, near face at 10 cm), radius 4 cm.
// Expected: still passable — sphere (radius 4) does not reach a face 10 cm away.
```

(The body of `TEST(MappingAlgorithm, FrontierAllowsOccupiedFaceNeighbourWhenRadiusTooSmall)`
itself does not change — `4.0 * cm < 10.0 * cm` was already true under both models.)

- [ ] **Step 3: Update `FrontierHasUnmappedFaceNeighbourOnCm10Grid` (line 335-346)**

Replace:

```cpp
// What: face neighbour Unmapped, centre Empty, radius 7.5 on 10 cm grid.
// Expected: hasNotMappedInSphere sees the face cell (same geometry as passability).
TEST(MappingAlgorithm, FrontierHasUnmappedFaceNeighbourOnCm10Grid) {
    const ct::MapConfig config = makeCm10Config();
    Map map{{11, 11, 11}, config, ct::VoxelOccupancy::Empty};
    const Position3D centre = pointCm(50, 50, 50);
    const Position3D face   = pointCm(60, 50, 50);
    map.set(centre, ct::VoxelOccupancy::Empty);
    map.set(face, ct::VoxelOccupancy::Unmapped);

    EXPECT_TRUE(detail::hasNotMappedInSphere(map, centre, 7.5 * cm));
    EXPECT_FALSE(detail::hasNotMappedInSphere(map, centre, 4.0 * cm));
}
```

with:

```cpp
// What: face neighbour Unmapped, centre Empty, full step (10 cm) away on a 10 cm grid.
// Expected: hasNotMappedInSphere sees the face cell once radius >= step_cm (same corrected
// geometry as passability); a smaller radius (9.9 or 4.0) does not reach it.
TEST(MappingAlgorithm, FrontierHasUnmappedNeighbourOnCm10GridAtFullStep) {
    const ct::MapConfig config = makeCm10Config();
    Map map{{11, 11, 11}, config, ct::VoxelOccupancy::Empty};
    const Position3D centre = pointCm(50, 50, 50);
    const Position3D face   = pointCm(60, 50, 50);
    map.set(centre, ct::VoxelOccupancy::Empty);
    map.set(face, ct::VoxelOccupancy::Unmapped);

    EXPECT_TRUE(detail::hasNotMappedInSphere(map, centre, 10.0 * cm));
    EXPECT_FALSE(detail::hasNotMappedInSphere(map, centre, 9.9 * cm));
    EXPECT_FALSE(detail::hasNotMappedInSphere(map, centre, 4.0 * cm));
}
```

- [ ] **Step 4: Update `ExploreReachableReportsStartPassabilityWithCapOfOne` (line 381-393)**

Replace:

```cpp
TEST(MappingAlgorithm, ExploreReachableReportsStartPassabilityWithCapOfOne) {
    // A cap of 1 makes this an O(1) start-passability probe — the replacement for
    // diagnose().start_passable, which task 6 deletes.
    const ct::MapConfig config = makeCm10Config();
    Map map{{11, 11, 11}, config};
    map.set(pointCm(50, 50, 50), ct::VoxelOccupancy::Empty);
    map.set(pointCm(60, 50, 50), ct::VoxelOccupancy::Occupied);

    const detail::MappingAlgorithmFrontier frontier;
    EXPECT_FALSE(frontier.exploreReachable(map, pointCm(50, 50, 50), 7.5 * cm, {}, 1)
                     .start_passable);
    EXPECT_TRUE(frontier.exploreReachable(map, pointCm(50, 50, 50), 4.0 * cm, {}, 1)
                    .start_passable);
}
```

with:

```cpp
TEST(MappingAlgorithm, ExploreReachableReportsStartPassabilityWithCapOfOne) {
    // A cap of 1 makes this an O(1) start-passability probe — the replacement for
    // diagnose().start_passable, which task 6 deletes.
    // Occupied neighbour is a full step (10 cm) away (corner-anchored voxel [60,70)):
    // radius 10 touches it, radius 4 does not.
    const ct::MapConfig config = makeCm10Config();
    Map map{{11, 11, 11}, config};
    map.set(pointCm(50, 50, 50), ct::VoxelOccupancy::Empty);
    map.set(pointCm(60, 50, 50), ct::VoxelOccupancy::Occupied);

    const detail::MappingAlgorithmFrontier frontier;
    EXPECT_FALSE(frontier.exploreReachable(map, pointCm(50, 50, 50), 10.0 * cm, {}, 1)
                     .start_passable);
    EXPECT_TRUE(frontier.exploreReachable(map, pointCm(50, 50, 50), 4.0 * cm, {}, 1)
                    .start_passable);
}
```

- [ ] **Step 5: Add the direct VAR-01 regression test — drone standing exactly on top of an Occupied floor is NOT passable**

Insert after Step 4's test (still before `ExploreReachableFindsFrontierAdjacentCandidates`):

```cpp
// What: reproduces the VAR-01 small_room failure directly. Floor voxel [0,10) is Occupied;
// centre sits exactly on the floor's top face (z=10, a lattice corner shared with the floor
// voxel below). Any radius > 0 overlaps a corner-touching Occupied voxel at distance 0.
// Expected: not passable, for both drone radii used in inputs/drone/*.yaml.
TEST(MappingAlgorithm, FrontierRejectsStandingExactlyOnOccupiedFloor) {
    const ct::MapConfig config = makeCm10Config();
    Map map{{11, 11, 11}, config, ct::VoxelOccupancy::Empty};
    map.set(pointCm(50, 50, 0), ct::VoxelOccupancy::Occupied);  // floor voxel [0,10) in z

    const detail::MappingAlgorithmFrontier frontier;
    EXPECT_FALSE(frontier.exploreReachable(map, pointCm(50, 50, 10), 4.0 * cm, {}, 1)
                     .start_passable);
    EXPECT_FALSE(frontier.exploreReachable(map, pointCm(50, 50, 10), 7.5 * cm, {}, 1)
                     .start_passable);
}

// What: same floor, but centre one full step higher (z=20, i.e. 10 cm clearance above the
// floor's top face). Neither drone radius (max 7.5 cm) reaches back down to the floor.
// Expected: passable — this is the "still flies one voxel above a mapped floor" guardrail
// (do-not-repeat lesson 3 in the design spec: a correct fix must not turn the floor a drone
// is actually flying above into an obstacle).
TEST(MappingAlgorithm, FrontierAllowsFlyingOneStepAboveOccupiedFloor) {
    const ct::MapConfig config = makeCm10Config();
    Map map{{11, 11, 11}, config, ct::VoxelOccupancy::Empty};
    map.set(pointCm(50, 50, 0), ct::VoxelOccupancy::Occupied);  // floor voxel [0,10) in z

    const detail::MappingAlgorithmFrontier frontier;
    EXPECT_TRUE(frontier.exploreReachable(map, pointCm(50, 50, 20), 4.0 * cm, {}, 1)
                    .start_passable);
    EXPECT_TRUE(frontier.exploreReachable(map, pointCm(50, 50, 20), 7.5 * cm, {}, 1)
                    .start_passable);
}
```

- [ ] **Step 6: Run the suite and confirm the expected failures**

```bash
docker run --rm -e VCPKG_ROOT=/usr/local/vcpkg -v "${PWD}:/work" -w /work drone-mapper-ex3-dev bash -lc '
  cmake --build --preset default --target algorithm_test
  ./build/default/Algorithm/algorithm_test --gtest_filter="MappingAlgorithm.*"
'
```

Expected: `FrontierRejectsOccupiedNeighbourAtFullStepWhenRadiusReachesIt`,
`FrontierHasUnmappedNeighbourOnCm10GridAtFullStep`,
`ExploreReachableReportsStartPassabilityWithCapOfOne` (now asserting radius 10, not 7.5),
`FrontierRejectsStandingExactlyOnOccupiedFloor`, and
`FrontierAllowsFlyingOneStepAboveOccupiedFloor` all **FAIL** against the current (unfixed)
`sphereIntersectsCellBox`. This confirms the tests actually exercise the bug before Task 2
fixes it. If any of these unexpectedly pass already, stop and re-derive — do not proceed with
a test that isn't red.

- [ ] **Step 7: Commit the failing tests**

```bash
git add Algorithm/tests/test_mapping_algorithm_frontier.cpp
git commit -m "test: add corner-anchored clearance regression tests for VAR-01"
```

(Wait for human approval per `git-workflow.mdc` before running this commit.)

### Task 2: Fix `sphereIntersectsCellBox` to the corner-anchored model

**Files:**
- Modify: `Algorithm/src/MappingAlgorithmFrontier.cpp:85-111`

**Interfaces:**
- Consumes: nothing new.
- Produces: `sphereIntersectsCellBox(int dx, int dy, int dz, double step_cm, double
  radius_cm) -> bool` — same signature, corrected geometry. Used by `isSpherePassable`
  (`:113-163`) and `sphereContainsNotMapped` (`:165-...`), both unchanged callers.

- [ ] **Step 1: Replace the function body**

```cpp
// True iff neighbour voxel (dx,dy,dz) intersects the closed sphere of the given radius
// centred at a lattice point. A lattice point is the LOW CORNER of its own voxel
// (Map3DImpl indexes floor((pos - offset) / resolution), matched by skeleton_host's
// HostMap3D), so neighbour d spans [d*step, (d+1)*step) relative to that point. The voxel
// on the "behind" side of the centre (any nonzero offset with all axes in {-1,0}) shares
// the corner with the centre voxel and touches it at distance 0; the voxel a full step
// "ahead" (any axis == +1) only touches once radius >= step_cm. See
// docs/superpowers/specs/2026-09-05-var01-sphere-clearance-fix-design.md §2.1/§5.1.
[[nodiscard]] bool sphereIntersectsCellBox(int dx, int dy, int dz, double step_cm,
                                           double radius_cm) {
    if (dx == 0 && dy == 0 && dz == 0) {
        return true;
    }
    const auto nearest1d = [step_cm](int d) {
        const double lo = static_cast<double>(d) * step_cm;
        const double hi = lo + step_cm;
        if (lo > 0.0) {
            return lo;
        }
        if (hi < 0.0) {
            return hi;
        }
        return 0.0;
    };
    const double nx = nearest1d(dx);
    const double ny = nearest1d(dy);
    const double nz = nearest1d(dz);
    return (nx * nx + ny * ny + nz * nz) <= (radius_cm * radius_cm);
}
```

- [ ] **Step 2: Build and run the `Algorithm` test suite**

```bash
docker run --rm -e VCPKG_ROOT=/usr/local/vcpkg -v "${PWD}:/work" -w /work drone-mapper-ex3-dev bash -lc '
  cmake --build --preset default --target algorithm_test
  ./build/default/Algorithm/algorithm_test
'
```

Expected: **all** tests pass, including the five from Task 1 and every pre-existing test in
`test_mapping_algorithm.cpp`, `test_wavefront_planner.cpp`, `test_scan_planning.cpp`,
`test_path_shaping.cpp`, `test_cone_template.cpp`, `test_lidar_cone.cpp`. If any pre-existing
test outside `test_mapping_algorithm_frontier.cpp` fails, read its assertion, determine
whether it also encodes the old center-box geometry (same derivation method as Task 1), and
either fix it with the same reasoning or — if it's unrelated to clearance geometry — stop and
investigate before continuing; do not paper over an unrelated failure.

- [ ] **Step 3: Commit**

```bash
git add Algorithm/src/MappingAlgorithmFrontier.cpp
git commit -m "fix: anchor sphere-clearance neighbour boxes to voxel corners"
```

(Wait for human approval before running.)

### Task 3: Measure VAR-01 / VAR-02 / VAR-03 in this worktree

**Files:** none modified — measurement only.

**Interfaces:** N/A.

- [ ] **Step 1: Rebuild everything VAR-01/02/03 need**

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

Expected: `PASS: check_foreign_host` and `HOST_ILLEGAL_MOVE_ATTEMPTS=0` for both `small_room`
and `house_lower`. If `small_room` still fails, run the diagnostic
(`HOST_DIAG_ILLEGAL=1` env var on a direct `skeleton_host` invocation, per
`tmp/diag_var01.sh`) and re-derive — do not proceed to Task 4/5 with VAR-01 still failing.

- [ ] **Step 3: Run VAR-02 and VAR-03**

```bash
docker run --rm -e VCPKG_ROOT=/usr/local/vcpkg -v "${PWD}:/work" -w /work drone-mapper-ex3-dev bash -lc '
  chmod +x Simulator/tests/manual/check_foreign_mission_control.sh Simulator/tests/manual/check_adversarial_plugins.sh
  bash Simulator/tests/manual/check_foreign_mission_control.sh build/default
  bash Simulator/tests/manual/check_adversarial_plugins.sh build/default
'
```

Expected: both scripts exit 0 (VAR-02 is diagnostic — record its findings summary but do not
fail on low score; VAR-03 must PASS per the `bad_scan` timeout canary, Known Issue #21).

- [ ] **Step 4: Record results**

Write the three scripts' PASS/FAIL and any relevant numbers (illegal move counts, VAR-02
findings path) into a scratch note — this feeds Task 6's comparison table. No file format
mandated; a plain text or markdown snippet in `tmp/approach-a-var-results.md` (git-ignored) is
sufficient.

### Task 4: Measure the 24-cell suite in this worktree

**Files:** none modified — measurement only.

**Interfaces:** N/A.

- [ ] **Step 1: Build Release and run `verify-cell-runtime`**

```bash
docker run --rm -e PYTHONUNBUFFERED=1 -e VCPKG_ROOT=/usr/local/vcpkg -v "${PWD}:/work" -w /work drone-mapper-ex3-dev bash -lc '
  set -euo pipefail
  if [ ! -f build/opt/CMakeCache.txt ]; then
    cmake -S . -B build/opt -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE="$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake"
  fi
  rm -f build/opt/Algorithm/CMakeFiles/Algorithm_207190406_209543255.dir/src/*.o
  rm -f build/opt/Algorithm/Algorithm_207190406_209543255.so
  cmake --build build/opt -j$(nproc) --target Algorithm_207190406_209543255 simulator_207190406_209543255 MissionControl_207190406_209543255
  python3 .cursor/skills/verify-cell-runtime/scripts/time_each_cell.py --build-dir /work/build/opt
'
```

- [ ] **Step 2: Record the CSV**

```bash
cp tmp/bench-out/per-cell-wall.csv tmp/approach-a-per-cell-wall.csv
```

Expected per Global Constraints: score sum close to **1769.835**, no cell newly over ~60s. Note
the printed `overall=`, `wall_sum=`, and `cells_ge_60s=` summary line for Task 6.

### Task 5: Verify frontier/wavefront/scan-planning tests once more, full suite

**Files:** none modified.

**Interfaces:** N/A.

- [ ] **Step 1: Run the complete Algorithm ctest target and the full manual smoke check**

```bash
docker run --rm -e VCPKG_ROOT=/usr/local/vcpkg -v "${PWD}:/work" -w /work drone-mapper-ex3-dev bash -lc '
  ctest --preset default --output-on-failure -R Algorithm
'
```

Expected: 100% pass. This is a final confirmation that Task 2's fix did not regress anything
beyond what Task 1 already re-derived.

---

## Approach B: voxel-center lattice (worktree `../ex3-var01-center-lattice`)

This approach is exploratory per the design spec (§6.3 lists unresolved hazards). Its tasks
therefore include investigation steps with a required, concrete verification method at each
decision point — not "figure it out," but a specific test to write and run before proceeding.

### Task 6: Resolve the `Map3DImpl` top-layer bounds hazard BEFORE writing the lattice change

**Files:**
- Create: `Algorithm/tests/test_center_lattice_bounds_hazard.cpp` (throwaway investigation
  test — delete or fold into a real test file at the end of Task 8, per Task 8 Step 4)
- Modify: `Algorithm/CMakeLists.txt` (add the new test file to the `algorithm_test` target's
  sources, following the existing pattern for the other `test_*.cpp` files already listed
  there)

**Interfaces:**
- Consumes: `common::IMap3D::isInBounds(Position3D) -> bool` (existing, frozen interface,
  `common/include/Common/IMap3D.h`).
- Produces: a boolean answer to hazard §6.3.1 — does a voxel-center lattice point at the
  outermost layer read `isInBounds() == true` under the *production* `Map3DImpl`? — that Task
  7 depends on.

- [ ] **Step 1: Write the hazard probe against the real `Map3DImpl`, not `FakeMap3D`**

`FakeMap3D` (used by most `Algorithm/tests`) has its own bounds logic
(`Algorithm/tests/FakeMap3D.h:53-58`) that may not match production `Map3DImpl`. This hazard is
specifically about the production simulator's map, so probe `Map3DImpl` directly. Find its
constructor and header:

```bash
grep -n "class Map3DImpl" Simulator/include/*.h Simulator/src/*.h 2>/dev/null
```

Read the located header to find the exact constructor signature (it takes an `NpyArray`-like
shape and a `MapConfig`), then write:

```cpp
// test_center_lattice_bounds_hazard.cpp — throwaway probe for design spec §6.3 hazard 1.
// Delete this file (or fold its surviving assertion into a permanent test) once Task 7
// has committed the resolution this test discovers.
#include <Common/Units.h>
#include <Simulator/Map3DImpl.h>  // adjust include path to match what Step 1's grep found

#include <gtest/gtest.h>

namespace {
using common::Position3D;
using common::cm;
using common::x_extent;
using common::y_extent;
using common::z_extent;
}  // namespace

// What: a 5-voxel-per-axis map at 10 cm resolution, offset 0, spans indices 0..4
// (world [0,50) per axis under corner/floor indexing). A voxel-CENTER lattice point for
// the last voxel (index 4) is at 4*10 + 5 = 45 cm. Does Map3DImpl consider that in bounds?
TEST(CenterLatticeBoundsHazard, LastVoxelCenterIsInBounds) {
    // Construct exactly as SimulationRunFactoryImpl does for the output map (5x5x5 shape,
    // resolution 10 cm, offset zero) — copy the real construction call found by Step 1's
    // grep instead of guessing a constructor signature here.
    // Fill in using the actual Map3DImpl constructor signature.
    // ... construct `map` ...
    const Position3D last_voxel_center{45.0 * x_extent[cm], 45.0 * y_extent[cm],
                                        45.0 * z_extent[cm]};
    EXPECT_TRUE(map.isInBounds(last_voxel_center));
}
```

- [ ] **Step 2: Run it and record the answer**

```bash
docker run --rm -e VCPKG_ROOT=/usr/local/vcpkg -v "${PWD}:/work" -w /work drone-mapper-ex3-dev bash -lc '
  cmake --build --preset default --target algorithm_test
  ./build/default/Algorithm/algorithm_test --gtest_filter="CenterLatticeBoundsHazard.*"
'
```

Two possible outcomes, each with a required next action:

- **If it passes (last voxel center is in bounds):** hazard 1 is a non-issue. Proceed to Task
  7 using centers `offset + (k+0.5)*step` for `k` in `[0, n-1]` unchanged.
- **If it fails (`OutOfBounds`):** the spec's concern is confirmed. Task 7's `quantizePosition`
  / `keyToPoint` must clamp the usable key range to whatever `k` values keep
  `offset + (k+0.5)*step` inside `isInBounds` — determine the exact boundary empirically (try
  `k = n-1` vs `k = n-2` in a follow-up assertion in this same test file) before writing Task
  7's code, and document the discovered boundary rule in a comment at the top of
  `quantizePosition`.

- [ ] **Step 3: Do not commit yet** — this test's file is provisional pending Task 7/8's
  resolution; it gets committed together with Task 7 (Step 1 will already require running it
  again as evidence the chosen implementation is correct).

### Task 7: Re-anchor the core lattice functions to voxel centers

**Files:**
- Modify: `Algorithm/src/MappingAlgorithmFrontier.cpp:55-64` (`keyToPoint`)
- Modify: `Algorithm/src/MappingAlgorithmFrontier.cpp:403-416` (`quantizePosition`)
- Modify: `Algorithm/src/MappingAlgorithmFrontier.cpp:376-399` (`forEachInBoundsVoxel`)
- Modify: `Algorithm/tests/test_center_lattice_bounds_hazard.cpp` (finalize per Task 6 Step 2's
  outcome)

**Interfaces:**
- Consumes: Task 6's hazard resolution (whether/how to clamp the top layer).
- Produces: `keyToPoint(GridKey, MapConfig) -> Position3D` and `quantizePosition(Position3D,
  MapConfig) -> GridKey` — same signatures, new anchor. These are called from
  `isSpherePassable`, `sphereContainsNotMapped`, `findPathTo`, `findUnstickPath`,
  `exploreReachable`, and (via the duplicate in `ScanPlanning.cpp`) the scan-planning module —
  Task 8 updates those call sites' *own* duplicated logic, not these functions directly.

- [ ] **Step 1: Change `quantizePosition` from `lround` to `floor`, matching `Map3DImpl`'s
  own indexing convention, then add the `+0.5` center offset in `keyToPoint`**

```cpp
GridKey quantizePosition(const Position3D& pos, const types::MapConfig& config) {
    const double step = gridStepCm(config);
    const double ox = config.offset.x.force_numerical_value_in(cm);
    const double oy = config.offset.y.force_numerical_value_in(cm);
    const double oz = config.offset.z.force_numerical_value_in(cm);
    const double px = pos.x.force_numerical_value_in(cm);
    const double py = pos.y.force_numerical_value_in(cm);
    const double pz = pos.z.force_numerical_value_in(cm);
    // floor, not lround: a GridKey now names the VOXEL a position falls inside (matching
    // Map3DImpl::toIndex), not the nearest corner lattice point. See design spec §6.1.
    return GridKey{
        static_cast<int>(std::floor((px - ox) / step)),
        static_cast<int>(std::floor((py - oy) / step)),
        static_cast<int>(std::floor((pz - oz) / step)),
    };
}
```

```cpp
[[nodiscard]] Position3D keyToPoint(const GridKey& key, const types::MapConfig& config) {
    const double step = gridStepCm(config);
    const double ox = config.offset.x.force_numerical_value_in(cm);
    const double oy = config.offset.y.force_numerical_value_in(cm);
    const double oz = config.offset.z.force_numerical_value_in(cm);
    // +0.5*step: a lattice point is now the CENTER of voxel `key`, not its low corner.
    return Position3D{
        (ox + (static_cast<double>(key.qx) + 0.5) * step) * x_extent[cm],
        (oy + (static_cast<double>(key.qy) + 0.5) * step) * y_extent[cm],
        (oz + (static_cast<double>(key.qz) + 0.5) * step) * z_extent[cm],
    };
}
```

- [ ] **Step 2: Write the round-trip test that catches quantize/keyToPoint disagreement (spec
  §6.3 hazard 3)**

Add to `Algorithm/tests/test_mapping_algorithm_frontier.cpp` (near the other `quantizePosition`
coverage — search the file for existing `quantizePosition` tests first and place this
alongside them):

```cpp
// What: a center-anchored lattice must round-trip: quantizing a voxel's own centre must
// return that voxel's key, and re-expanding that key must return the same centre exactly.
// Guards against quantize (floor) and keyToPoint (+0.5*step) disagreeing (design spec
// §6.3 hazard 3).
TEST(MappingAlgorithm, CenterLatticeQuantizeKeyToPointRoundTrip) {
    const ct::MapConfig config = makeCm10Config();
    const detail::GridKey key{3, 4, 5};
    const Position3D centre = detail::keyToPoint(key, config);
    EXPECT_EQ(detail::quantizePosition(centre, config), key);
    // A point 1 cm off-center (still inside the same voxel) must quantize to the same key.
    const Position3D nudged{centre.x + 1.0 * x_extent[cm], centre.y, centre.z};
    EXPECT_EQ(detail::quantizePosition(nudged, config), key);
}
```

- [ ] **Step 3: Fix `forEachInBoundsVoxel` to enumerate centers**

```cpp
template <typename Fn>
void forEachInBoundsVoxel(const IMap3D& map, Fn&& fn) {
    const types::MapConfig config = map.getMapConfig();
    const double step = gridStepCm(config);
    if (step <= 0.0) {
        return;
    }
    const types::MappingBounds& bounds = config.boundaries;
    const double min_x = bounds.min_x.force_numerical_value_in(cm) + step * 0.5;
    const double max_x = bounds.max_x.force_numerical_value_in(cm);
    const double min_y = bounds.min_y.force_numerical_value_in(cm) + step * 0.5;
    const double max_y = bounds.max_y.force_numerical_value_in(cm);
    const double min_z = bounds.min_height.force_numerical_value_in(cm) + step * 0.5;
    const double max_z = bounds.max_height.force_numerical_value_in(cm);
    for (double x = min_x; x <= max_x + 1e-9; x += step) {
        for (double y = min_y; y <= max_y + 1e-9; y += step) {
            for (double z = min_z; z <= max_z + 1e-9; z += step) {
                if (!fn(Position3D{x * x_extent[cm], y * y_extent[cm], z * z_extent[cm]})) {
                    return;
                }
            }
        }
    }
}
```

If Task 6 discovered the top-layer-out-of-bounds hazard is real, replace the `<= max_x +
1e-9` bound in each axis with whatever cutoff Task 6 Step 2 determined keeps every enumerated
center in-bounds (document the exact value and cite the failing/passing hazard-probe
assertion in a comment here).

- [ ] **Step 4: Resolve Task 6's provisional test**

Move `CenterLatticeBoundsHazard.LastVoxelCenterIsInBounds` (finalized per whichever outcome
occurred) from the throwaway file into `test_mapping_algorithm_frontier.cpp` as a permanent
regression test, and delete `Algorithm/tests/test_center_lattice_bounds_hazard.cpp` and its
`Algorithm/CMakeLists.txt` entry.

- [ ] **Step 5: Build and observe how many tests now fail**

```bash
docker run --rm -e VCPKG_ROOT=/usr/local/vcpkg -v "${PWD}:/work" -w /work drone-mapper-ex3-dev bash -lc '
  cmake --build --preset default --target algorithm_test
  ./build/default/Algorithm/algorithm_test 2>&1 | tail -80
'
```

Expected: many failures across `test_mapping_algorithm.cpp`, `test_mapping_algorithm_frontier.cpp`,
`test_wavefront_planner.cpp`, `test_scan_planning.cpp`, `test_path_shaping.cpp` — this is
expected and is exactly the "~100+ hard-coded corner-lattice coordinates" the design spec (§6.2)
flagged. Record the full failing-test list; Task 8 works through it file by file.

- [ ] **Step 6: Commit the core lattice-anchor change on its own, even though tests are red**

```bash
git add Algorithm/src/MappingAlgorithmFrontier.cpp Algorithm/tests/test_mapping_algorithm_frontier.cpp Algorithm/tests/test_center_lattice_bounds_hazard.cpp Algorithm/CMakeLists.txt
git commit -m "refactor: re-anchor navigation lattice keys to voxel centers"
```

Note in the commit body (not a separate commit type — `wip` is not an allowed
`git-workflow.mdc` type) that dependent call sites are intentionally not yet updated and the
suite is expected red until the next commit. Wait for human approval. A red-test intermediate
commit is acceptable here specifically because the spec requires isolating "one mechanism" —
the anchor change — from "propagate to dependents" as separately reviewable/bisectable steps,
per Global Constraints' lesson 5. Do not proceed to Task 8 without this checkpoint.

### Task 8: Propagate the center anchor to every dependent call site

**Files:**
- Modify: `Algorithm/src/ScanPlanning.cpp:56-65` (duplicate `keyToPoint`) and its
  `quantizePosition` call sites (search for `detail::quantizePosition(` in this file — the
  design spec cites approximate lines `:143, :260, :318`, re-locate exactly since Task 7 may
  have shifted line numbers elsewhere in the file)
- Modify: `Algorithm/src/WavefrontPlanner.cpp` (search for `detail::quantizePosition(` — spec
  cites `:358`)
- Modify: `UserCommon/src/ConeTemplate.cpp:73-77` (`VoxelStamp::quant`)
- Modify: `UserCommon/src/LidarCone.cpp:96-98` (`voxelKey`)
- Investigate (may or may not need changes): `Simulator/src/MapsComparison.cpp:48-84`
  (`quantizePosition` / `forEachGridCenter`) — per spec §6.2, verify whether this scorer
  already treats grid points as centers before touching it.

**Interfaces:**
- Consumes: Task 7's corrected `keyToPoint`/`quantizePosition` semantics (center-anchored,
  `floor`-quantized).
- Produces: nothing new — this task only makes existing call sites consistent with Task 7.

- [ ] **Step 1: Fix `ScanPlanning.cpp`'s duplicate `keyToPoint`**

Apply the identical `+0.5*step` change from Task 7 Step 1 to the copy in this file:

```cpp
[[nodiscard]] Position3D keyToPoint(const GridKey& key, const types::MapConfig& config) {
    const double step = config.resolution.force_numerical_value_in(cm);
    const double ox = config.offset.x.force_numerical_value_in(cm);
    const double oy = config.offset.y.force_numerical_value_in(cm);
    const double oz = config.offset.z.force_numerical_value_in(cm);
    return Position3D{
        (ox + (static_cast<double>(key.qx) + 0.5) * step) * x_extent[cm],
        (oy + (static_cast<double>(key.qy) + 0.5) * step) * y_extent[cm],
        (oz + (static_cast<double>(key.qz) + 0.5) * step) * z_extent[cm],
    };
}
```

Consider (do not silently skip): should this duplicate be deleted and the file changed to
`#include` `MappingAlgorithmFrontier.h`'s exported `keyToPoint` instead, removing the
duplication (adv-cpp-standards.mdc e10, "no duplicate logic across components")? Check
`MappingAlgorithmFrontier.h` to see whether `keyToPoint` is already exposed outside the `.cpp`
(it is currently a file-local anonymous-namespace function per the earlier research — if so,
exporting it is a larger change than this task's scope; leave the duplication as-is and note
it as a pre-existing issue, do not expand scope here).

- [ ] **Step 2: Fix `UserCommon/src/ConeTemplate.cpp`'s `VoxelStamp::quant`**

Read the current implementation first (it was `llround` per research):

```cpp
int VoxelStamp::quant(double value, double origin, double step) {
    if (!(step > 0.0)) {
        return 0;
    }
    return static_cast<int>(std::llround((value - origin) / step));
```

Change to match the `floor` convention (center-anchored, same reasoning as Task 7 Step 1):

```cpp
int VoxelStamp::quant(double value, double origin, double step) {
    if (!(step > 0.0)) {
        return 0;
    }
    return static_cast<int>(std::floor((value - origin) / step));
}
```

Before committing this specific change, check every caller of `VoxelStamp::quant` in
`UserCommon/` and `Algorithm/` (`grep -rn "\.quant(\|->quant(" UserCommon/ Algorithm/`) and
confirm each caller's own coordinate origin is already center-anchored after Task 7/this
task's other steps — if a caller still passes a corner-anchored `origin`, this change alone
will misalign it; fix the caller's origin argument in the same step, don't leave it
inconsistent.

- [ ] **Step 3: Fix `UserCommon/src/LidarCone.cpp`'s `voxelKey`**

Same `llround` → `floor` change, same caller-origin verification as Step 2:

```cpp
const auto quant = [step](double value, double origin) {
    return static_cast<std::int64_t>(std::floor((value - origin) / step));
};
```

- [ ] **Step 4: Investigate `MapsComparison.cpp` — do not change blindly**

Read `Simulator/src/MapsComparison.cpp:48-84` (`quantizePosition`, `forEachGridCenter`). This
function's own name ("GridCenter") suggests it may already assume centers. Write a small
standalone check: for a known `MapConfig` (offset 0, resolution 10), call
`forEachGridCenter` and print/assert the first few positions it visits. If they are already at
`step*(k+0.5)` — i.e., the scorer was already center-correct and only the *algorithm's own*
lattice was corner-anchored — **do not change this file**; the mismatch was one-sided and is
now resolved by Task 7 alone. If it visits `step*k` (corner-anchored), that means the scorer's
grid and the algorithm's map share the same `Map3DImpl` voxel convention already (which is
correct — the scorer walks *voxels*, not the algorithm's *navigation lattice*, and those are
different concerns), and this file almost certainly does **not** need to change — the
scorer operates on voxel indices via `Map3DImpl`, independent of how the algorithm chooses to
plan paths between them. Record which case applies and why in a comment or commit message; do
not edit this file unless the investigation proves it depends on the algorithm's internal
lattice anchor (unlikely, since it's in `Simulator/src/`, a different component, and reads
finished output maps by voxel, not by the algorithm's `GridKey`).

- [ ] **Step 5: Rebuild and get the failing test count down to a specific, itemized list**

```bash
docker run --rm -e VCPKG_ROOT=/usr/local/vcpkg -v "${PWD}:/work" -w /work drone-mapper-ex3-dev bash -lc '
  cmake --build --preset default --target algorithm_test
  ./build/default/Algorithm/algorithm_test 2>&1 | tail -120
'
```

- [ ] **Step 6: Commit the propagation**

```bash
git add Algorithm/src/ScanPlanning.cpp Algorithm/src/WavefrontPlanner.cpp UserCommon/src/ConeTemplate.cpp UserCommon/src/LidarCone.cpp
git commit -m "fix: propagate voxel-center lattice anchor to scan planning and cone templates"
```

(Wait for human approval.)

### Task 9: Rebase test fixtures file by file

**Files:**
- Modify: `Algorithm/tests/test_mapping_algorithm.cpp` (`gridPoint` helper, ~line 75-84 per
  research — re-locate exactly)
- Modify: `Algorithm/tests/test_mapping_algorithm_frontier.cpp` (`pointCm` helper, already
  shown at line 56-58 of the current file)
- Modify: `Algorithm/tests/test_wavefront_planner.cpp` (`at` helper, ~line 54-55)
- Modify: `Algorithm/tests/test_scan_planning.cpp`
- Modify: `Algorithm/tests/test_path_shaping.cpp` (`at` helper, ~line 54-90)
- Modify: `Algorithm/tests/test_cone_template.cpp`, `Algorithm/tests/test_lidar_cone.cpp`

**Interfaces:**
- Consumes: Task 7/8's corrected `keyToPoint`/`quantizePosition`/`VoxelStamp::quant`/
  `voxelKey`.
- Produces: a fully green `algorithm_test` target.

Do this file by file, not all at once, so a failure in one file's rebase doesn't hide a
different failure in another (Global Constraints: bisectable changes).

- [ ] **Step 1: `test_mapping_algorithm_frontier.cpp` — try the mechanical shift first**

Most of this file's positions are built via `pointCm(x, y, z)` which returns a raw
`Position3D` at those exact cm coordinates — **not** through `keyToPoint`, so they are
absolute world positions, not lattice keys. Whether a test needs to change depends on what it
is asserting, not on a blanket "+5cm to every call." Work through each `TEST` in this file:

1. If the test sets occupancy with `map.set(pointCm(...), ...)` and separately probes
   passability at a **different** `pointCm(...)` used as the algorithm's `centre` argument to
   `exploreReachable`/`hasNotMappedInSphere` — these are still just absolute positions fed
   into `occupancyAt`/`atVoxel`, which is unaffected by the lattice anchor (that call goes
   straight to `FakeMap3D`, not through `quantizePosition`/`keyToPoint`). **These tests need
   no coordinate change** — re-run them after Task 7/8 and confirm they still pass unmodified
   (they exercise `isSpherePassable`'s geometry directly, which Task 7 didn't touch — only
   Task 2, in the *other* worktree, touched that function. Confirm this file's tests were
   never red after Task 7/8, or if some are, they must be using `quantizePosition`/
   `keyToPoint` somewhere — grep for those symbols in the specific failing test).
2. Any test that IS red: grep it for `quantizePosition` or `keyToPoint` calls, and re-derive
   its expected result the same way Approach A's Task 1 did — compute the actual center point
   involved and check the real distance, don't guess.

- [ ] **Step 2: `test_mapping_algorithm.cpp`, `test_wavefront_planner.cpp`,
  `test_scan_planning.cpp`, `test_path_shaping.cpp` — same method**

For each file: build+run it in isolation (`--gtest_filter` scoped to that file's test names),
read every failure's assertion and the code path it exercises, determine whether the failure
is because the test calls `quantizePosition`/`keyToPoint` (needs re-derivation) or because
`WavefrontPlanner`'s descend/attic logic (`WavefrontPlanner.cpp:146-165, 247-260`) now behaves
differently now that standing exactly on a just-mapped floor is a different lattice cell than
before (re-derive the *intended* behavior from the mission logic, not just the old numbers —
if a test asserted a specific descend target position, recompute what the correct center-anchored
equivalent position is, don't add 5 blindly without checking it's still the intended voxel).

- [ ] **Step 3: `test_cone_template.cpp`, `test_lidar_cone.cpp`**

These test `VoxelStamp::quant`/`voxelKey` directly (Task 8 Steps 2-3's targets). Any hardcoded
expected integer keys computed from corner-anchored origins need recomputation with `floor`
against center-anchored origins — same method as Step 1.

- [ ] **Step 4: Full suite green**

```bash
docker run --rm -e VCPKG_ROOT=/usr/local/vcpkg -v "${PWD}:/work" -w /work drone-mapper-ex3-dev bash -lc '
  cmake --build --preset default --target algorithm_test
  ctest --preset default --output-on-failure -R Algorithm
'
```

Expected: 100% pass. Do not proceed to Task 10 with any red test.

- [ ] **Step 5: Commit per file (or per small logical group) — do not squash all fixture rebases into one commit**

```bash
git add Algorithm/tests/test_mapping_algorithm_frontier.cpp
git commit -m "test: rebase frontier fixtures to voxel-center lattice"
# repeat per file
```

(Wait for human approval before each commit.)

### Task 10: Measure VAR-01/02/03 and the 24-cell suite in this worktree

**Files:** none modified — measurement only.

**Interfaces:** N/A.

- [ ] **Step 1-4:** Identical procedure to Approach A's Task 3 and Task 4 (same commands,
  same expected-value bar from Global Constraints), run in `../ex3-var01-center-lattice`
  instead. Save results as `tmp/approach-b-var-results.md` and
  `tmp/approach-b-per-cell-wall.csv`.

Pay particular attention to the 4 `house_full` cells (small/large × long/short lidar) — the
design spec (§6.3 hazard 2) flags these as the ones affected by `gps_resolution_cm: 10`
snapping center-lattice targets 5 cm off. Compare their scores specifically against baseline,
not just the aggregate sum.

---

## Task 11: Build the comparison and report to the user

**Files:**
- Create: `tmp/var01-approach-comparison.md` (scratch, git-ignored — this is a report to paste
  into chat, not a committed artifact; if the user wants it kept, copy it into `docs/` as a
  follow-up, not as part of this task)

**Interfaces:** N/A — this is a reporting task, not a code task.

- [ ] **Step 1: Build the table specified in the design spec §8.3**

Columns: VAR-01 status, VAR-02/03 status, 24-cell score sum (vs baseline 1769.835), notable
per-cell score deltas (especially `house_full` for Approach B), wall-clock sum and any newly
->60s cells, `Algorithm/tests` suite status, lines of production code changed (`git diff
known-issues-fixes --stat` in each worktree, excluding test files), lines of test code
changed.

- [ ] **Step 2: Apply the spec's decision rule (§8.3) and present it to the user**

State which approach(es) qualify (VAR-01 PASS, no test regression, no cell >60s, score sum
within ~50-90 points of baseline), and if both qualify, name the tie-break recommendation
(prefer Approach A's smaller diff unless Approach B is measurably better). Do not pick for the
user — present the numbers and the recommendation, and wait for their decision per the design
spec's explicit instruction: "Report findings back to the user before implementing either on a
real feature branch."

- [ ] **Step 3: Do not merge or clean up yet**

Leave both worktrees in place until the user has seen the comparison and decided. Cleanup
(`git worktree remove`, re-implementing the winner on a fresh feature branch per
`git-workflow.mdc`) is a follow-up action after the user's decision, not part of this plan.
