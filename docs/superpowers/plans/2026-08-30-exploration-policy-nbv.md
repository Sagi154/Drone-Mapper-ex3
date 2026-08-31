# Exploration Policy (NBV) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace `Algorithm/`'s action policy with a budget-aware, frontier-anchored next-best-view policy that stops quitting early, carries a scan on every movement step, and picks viewpoints by expected information per step.

**Architecture:** A new `NbvPlanner` owns the policy and returns an `ExplorationPlan`; `MappingAlgorithmImpl` shrinks to plan execution and command emission; `MappingAlgorithmFrontier` keeps only reachability, path reconstruction, line-of-sight and unstick, with every search expansion-bounded; `UserCommon/LidarCone.h` gains one beam walk that both the boolean gate and the new voxel counter share.

**Tech Stack:** C++20, `mp-units` strong types (`common/include/Common/Units.h`), GoogleTest, CMake, the Python harness in `scripts/benchmark/`.

**Spec:** `docs/superpowers/specs/2026-08-29-exploration-policy-nbv-design.md`. Read it before task 1.

## Global Constraints

- Human approval before every `git commit` (`.cursor/rules/git-workflow.mdc`). Stage and propose; do not commit unprompted.
- Never edit `common/` or any published header. `IMappingAlgorithm`, `MappingStepCommand`, the plugin class name and `REGISTER_MAPPING_ALGORITHM` stay exactly as they are.
- `Algorithm_207190406_209543255.so` must stay independently loadable: no link-time dependency on MissionControl symbols. `UserCommon/` is included as sources per plugin.
- New code uses `mp-units` strong types. `force_numerical_value_in(cm)` is allowed only at the boundary with the existing double-based frontier substrate; no new raw-double geometry APIs. Never write a per-function unit alias (e17s) — use `common`'s `cm` / `deg`.
- No RNG anywhere. Determinism is a hard requirement of the benchmark harness.
- `Unmapped` stays traversable at soft cost 4 vs 1 for `Empty`. The safety invariant is: never command a move whose drone-sphere footprint contains `Occupied` or `OutOfBounds`.
- Constants, verbatim: `kCandidateStrideCells = 3`, `kScoredCandidates = 16`, `kReplanIntervalSteps = 10`, `kBlockedTtlSteps = 50`, `kRecoveryAttempts = 3`, `kTravelScanProbes = 6`, `kMaxMovingStallTicks = 2` (unchanged).
- Build and test from the repo root: `cmake -S . -B build && cmake --build build -j` then `ctest --test-dir build`. In Docker, `apt-get install -y python3-venv python3-pip` may be needed once per container for the harness.
- One known-slow test exists today: `MappingAlgorithm.FrontierStartPassableWhenSphereHasUnmapped`, slow because `diagnose` sweeps the 101³ 1 cm grid. Task 6 converts it to a capped `exploreReachable` probe and it becomes fast. Until then, filter it out while iterating: `./build/Algorithm/algorithm_test --gtest_filter=-MappingAlgorithm.FrontierStartPassableWhenSphereHasUnmapped`.

---

## File Structure

| File | Responsibility | Task |
|------|----------------|------|
| `UserCommon/include/user_common_207190406_209543255/LidarCone.h` | One cone beam walk; boolean gate + unresolved-voxel counter built on it | 1 |
| `Algorithm/tests/test_lidar_cone.cpp` | Cone walk and counter tests | 1 |
| `Algorithm/src/MappingAlgorithmFrontier.h/.cpp` | Add `exploreReachable`, `reconstructPathTo`, `hasClearLineOfSight`, `maxExpansionsForMap`; expansion-bound existing searches | 2 |
| `Algorithm/tests/test_mapping_algorithm_frontier.cpp` | Reachability, LOS, ALG28 bound tests | 2, 6 |
| `Algorithm/src/PathShaping.h/.cpp` (new) | Constant-altitude string-pulling and the real step-cost model | 3 |
| `Algorithm/tests/test_path_shaping.cpp` (new) | Smoothing and step-cost tests | 3 |
| `Algorithm/src/NbvPlanner.h/.cpp` (new) | The policy: candidates, prefilter, gain, utility, feasibility, plan | 4 |
| `Algorithm/tests/test_nbv_planner.cpp` (new) | Objective and plan tests | 4 |
| `Algorithm/src/MappingAlgorithmImpl.cpp`, `Algorithm/include/Algorithm/MappingAlgorithmImpl.h` | Plan execution, co-emission, replan cadence, blocked TTL, termination | 5 |
| `Algorithm/tests/test_mapping_algorithm.cpp` | Execution, co-emission, termination tests | 5 |
| `Algorithm/CMakeLists.txt` | New sources in both targets | 3, 4 |
| `docs/*` + `docs/benchmarks/2026-08-30-post_d_honest.{csv,md}` | Documentation and measurement | 7 |

---

### Task 1: Cone beam walk and unresolved-voxel counter

`LidarCone.h` today has the ring geometry inlined inside `coneCoversUnresolved`, which answers only "is anything unresolved". The planner needs a *count*. Extract the walk once and build both on it, so there is no second copy of the ring math (e10).

Two behaviours are easy to get wrong and are pinned by tests here. Our MissionControl carves from distance **0**, not `z_min` (`MissionControl/src/ScanResultToVoxels.cpp:78-115`), so the walk starts at the first sample — do not "fix" it to start at `z_min`. And `PotentiallyOccupied` is a *resolved* state; counting it as gain makes the drone re-scan near-field walls forever.

**Files:**
- Modify: `UserCommon/include/user_common_207190406_209543255/LidarCone.h`
- Test: `Algorithm/tests/test_lidar_cone.cpp`

**Interfaces:**
- Consumes: `beam_math::{pointAlongBeam, absoluteBeamOrientation, normalizeOrientation}`; `common::IMap3D`; `common::types::LidarConfigData`.
- Produces, all in `namespace user_common_207190406_209543255::lidar_cone`:
  - `std::int64_t voxelKey(const common::types::MapConfig&, const Position3D&)`
  - `template <typename Fn> void forEachConeBeam(const common::types::LidarConfigData&, const Orientation& center_abs, Fn&& on_beam)` — `on_beam(const Orientation&) -> bool`, `false` stops the walk.
  - `std::size_t countUnresolvedVoxels(const common::IMap3D&, const Position3D& origin, const Orientation& drone_heading, const Orientation& relative_scan, const common::types::LidarConfigData&, std::unordered_set<std::int64_t>& seen)`
  - `bool coneCoversUnresolved(...)` — unchanged signature, reimplemented over `forEachConeBeam`.

- [x] **Step 1: Write the failing tests**

Append to `Algorithm/tests/test_lidar_cone.cpp`:

```cpp
TEST(LidarCone, CountsUnresolvedVoxelsAndDeduplicatesAcrossCalls) {
    const ct::MapConfig config = makeSmallMapConfig();
    Map map{{11, 11, 11}, config, ct::VoxelOccupancy::Empty};
    const Position3D origin{50.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]};
    map.set(Position3D{70.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]},
            ct::VoxelOccupancy::Unmapped);
    map.set(Position3D{80.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]},
            ct::VoxelOccupancy::Unmapped);

    ct::LidarConfigData cfg = makeShortLidar();
    cfg.z_max = 40.0 * cm;
    cfg.fov_circles = 1;

    std::unordered_set<std::int64_t> seen;
    const std::size_t first =
        lc::countUnresolvedVoxels(map, origin, Orientation{}, Orientation{}, cfg, seen);
    EXPECT_EQ(first, 2u);

    // Same cone again with the same `seen` set adds nothing.
    const std::size_t second =
        lc::countUnresolvedVoxels(map, origin, Orientation{}, Orientation{}, cfg, seen);
    EXPECT_EQ(second, 0u);
}

TEST(LidarCone, CountStopsAtOccludingVoxel) {
    const ct::MapConfig config = makeSmallMapConfig();
    Map map{{11, 11, 11}, config, ct::VoxelOccupancy::Empty};
    const Position3D origin{50.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]};
    map.set(Position3D{60.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]},
            ct::VoxelOccupancy::Occupied);
    map.set(Position3D{70.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]},
            ct::VoxelOccupancy::Unmapped);

    ct::LidarConfigData cfg = makeShortLidar();
    cfg.z_max = 40.0 * cm;
    cfg.fov_circles = 1;

    std::unordered_set<std::int64_t> seen;
    EXPECT_EQ(lc::countUnresolvedVoxels(map, origin, Orientation{}, Orientation{}, cfg, seen), 0u);
}

TEST(LidarCone, PotentiallyOccupiedCountsAsResolved) {
    const ct::MapConfig config = makeSmallMapConfig();
    Map map{{11, 11, 11}, config, ct::VoxelOccupancy::Empty};
    const Position3D origin{50.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]};
    map.set(Position3D{60.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]},
            ct::VoxelOccupancy::PotentiallyOccupied);

    ct::LidarConfigData cfg = makeShortLidar();
    cfg.z_max = 40.0 * cm;
    cfg.fov_circles = 1;

    std::unordered_set<std::int64_t> seen;
    EXPECT_EQ(lc::countUnresolvedVoxels(map, origin, Orientation{}, Orientation{}, cfg, seen), 0u);
    EXPECT_FALSE(lc::coneCoversUnresolved(map, origin, Orientation{}, Orientation{}, cfg));
}

TEST(LidarCone, NearFieldInsideZMinIsCounted) {
    // Our MissionControl carves from distance 0, so a voxel inside z_min is
    // resolvable and must contribute gain.
    const ct::MapConfig config = makeSmallMapConfig();
    Map map{{11, 11, 11}, config, ct::VoxelOccupancy::Empty};
    const Position3D origin{50.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]};
    map.set(Position3D{60.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]},
            ct::VoxelOccupancy::Unmapped);  // 10 cm away, inside z_min = 20 cm

    ct::LidarConfigData cfg = makeShortLidar();
    cfg.z_max = 40.0 * cm;
    cfg.fov_circles = 1;

    std::unordered_set<std::int64_t> seen;
    EXPECT_EQ(lc::countUnresolvedVoxels(map, origin, Orientation{}, Orientation{}, cfg, seen), 1u);
}
```

Add `#include <cstdint>` and `#include <unordered_set>` to the test file's include block.

- [x] **Step 2: Run the tests to verify they fail**

Run: `cmake --build build -j && ./build/Algorithm/algorithm_test --gtest_filter='LidarCone.*'`
Expected: compile error — `countUnresolvedVoxels` is not a member of `lidar_cone`.

- [x] **Step 3: Implement the shared walk**

In `LidarCone.h`, add `#include <cstdint>` and `#include <unordered_set>`. Replace the `detail::beamHitsUnresolved` helper and the body of `coneCoversUnresolved` with the following, keeping `beamsOnCircle` as it is:

```cpp
[[nodiscard]] inline std::int64_t voxelKey(const common::types::MapConfig& config,
                                           const Position3D& p) {
    const double step = config.resolution.force_numerical_value_in(cm);
    if (!(step > 0.0)) {
        return 0;
    }
    const auto quant = [step](double value, double origin) {
        return static_cast<std::int64_t>(std::llround((value - origin) / step));
    };
    const std::int64_t qx = quant(p.x.force_numerical_value_in(cm),
                                  config.offset.x.force_numerical_value_in(cm));
    const std::int64_t qy = quant(p.y.force_numerical_value_in(cm),
                                  config.offset.y.force_numerical_value_in(cm));
    const std::int64_t qz = quant(p.z.force_numerical_value_in(cm),
                                  config.offset.z.force_numerical_value_in(cm));
    constexpr std::int64_t kBias = 1 << 20;
    constexpr std::int64_t kSpan = 1 << 21;
    return ((qx + kBias) * kSpan + (qy + kBias)) * kSpan + (qz + kBias);
}

/// Invokes on_beam with each absolute beam orientation in the cone (centre first).
/// on_beam returns false to stop the walk early.
template <typename Fn>
inline void forEachConeBeam(const common::types::LidarConfigData& cfg,
                            const Orientation& center_abs,
                            Fn&& on_beam) {
    if (cfg.fov_circles == 0) {
        return;
    }
    if (!on_beam(center_abs)) {
        return;
    }

    const double z_min_cm = cfg.z_min.force_numerical_value_in(cm);
    const double d_cm = cfg.d.force_numerical_value_in(cm);
    const double z_min_safe = (z_min_cm > 1e-9) ? z_min_cm : 1.0;

    // Orthonormal basis around center_abs, same construction as HostLidar.
    const double ch = center_abs.horizontal.numerical_value_in(deg) * (std::numbers::pi / 180.0);
    const double ca = center_abs.altitude.numerical_value_in(deg) * (std::numbers::pi / 180.0);
    const double fx = std::cos(ca) * std::cos(ch);
    const double fy = std::cos(ca) * std::sin(ch);
    const double fz = std::sin(ca);
    double rx = -fy;
    double ry = fx;
    const double rz = 0.0;
    const double rlen = std::hypot(rx, ry);
    if (rlen < 1e-9) {
        rx = 1.0;
        ry = 0.0;
    } else {
        rx /= rlen;
        ry /= rlen;
    }
    const double ux = ry * fz - rz * fy;
    const double uy = rz * fx - rx * fz;
    const double uz = rx * fy - ry * fx;

    for (std::size_t circle = 1; circle < cfg.fov_circles; ++circle) {
        const std::size_t beam_count = detail::beamsOnCircle(circle);
        const double polar = std::atan2(static_cast<double>(circle) * d_cm, z_min_safe);
        const double cp = std::cos(polar);
        const double sp = std::sin(polar);

        for (std::size_t j = 0; j < beam_count; ++j) {
            const double phi = (beam_count == 1)
                                   ? 0.0
                                   : (2.0 * std::numbers::pi * static_cast<double>(j) /
                                      static_cast<double>(beam_count));
            const double ax = ux * std::cos(phi) + rx * std::sin(phi);
            const double ay = uy * std::cos(phi) + ry * std::sin(phi);
            const double az = uz * std::cos(phi) + rz * std::sin(phi);
            double dx = fx * cp + ax * sp;
            double dy = fy * cp + ay * sp;
            double dz = fz * cp + az * sp;
            const double len = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (len < 1e-12) {
                continue;
            }
            dx /= len;
            dy /= len;
            dz /= len;
            const double az_deg = std::atan2(dy, dx) * (180.0 / std::numbers::pi);
            const double el_deg = std::atan2(dz, std::hypot(dx, dy)) * (180.0 / std::numbers::pi);
            if (!on_beam(bm::normalizeOrientation(Orientation{az_deg * deg, el_deg * deg}))) {
                return;
            }
        }
    }
}

namespace detail {

/// Walks one beam from the first sample out to z_max. Calls on_unresolved for each
/// Unmapped sample; stops the beam at Occupied / OutOfBounds. Carving starts at 0 in
/// ScanResultToVoxels, so the walk deliberately does NOT skip the sub-z_min region.
/// PotentiallyOccupied is a resolved state and is skipped, not counted.
template <typename Fn>
inline bool walkBeam(const common::IMap3D& map,
                     const Position3D& origin,
                     const Orientation& absolute_beam,
                     double z_max_cm,
                     double step_cm,
                     Fn&& on_unresolved) {
    if (!(z_max_cm > 0.0) || !(step_cm > 0.0)) {
        return true;
    }
    for (double dist = step_cm; dist <= z_max_cm + 1e-9; dist += step_cm) {
        const Position3D p = bm::pointAlongBeam(origin, absolute_beam, dist * cm);
        const auto occ = map.atVoxel(p);
        if (occ == common::types::VoxelOccupancy::Occupied ||
            occ == common::types::VoxelOccupancy::OutOfBounds) {
            return true;
        }
        if (occ == common::types::VoxelOccupancy::Unmapped) {
            if (!on_unresolved(p)) {
                return false;
            }
        }
    }
    return true;
}

} // namespace detail

[[nodiscard]] inline std::size_t countUnresolvedVoxels(
    const common::IMap3D& map,
    const Position3D& origin,
    const Orientation& drone_heading,
    const Orientation& relative_scan,
    const common::types::LidarConfigData& cfg,
    std::unordered_set<std::int64_t>& seen) {
    const double z_max_cm = cfg.z_max.force_numerical_value_in(cm);
    const double step_cm = 0.5 * map.getMapConfig().resolution.force_numerical_value_in(cm);
    const common::types::MapConfig config = map.getMapConfig();
    const Orientation center_abs =
        bm::normalizeOrientation(bm::absoluteBeamOrientation(drone_heading, relative_scan));

    std::size_t added = 0;
    forEachConeBeam(cfg, center_abs, [&](const Orientation& beam) {
        detail::walkBeam(map, origin, beam, z_max_cm, step_cm, [&](const Position3D& p) {
            if (seen.insert(voxelKey(config, p)).second) {
                ++added;
            }
            return true;
        });
        return true;
    });
    return added;
}

/// True if any beam in the cone still sees Unmapped voxels.
[[nodiscard]] inline bool coneCoversUnresolved(const common::IMap3D& map,
                                               const Position3D& origin,
                                               const Orientation& drone_heading,
                                               const Orientation& relative_scan,
                                               const common::types::LidarConfigData& cfg) {
    const double z_max_cm = cfg.z_max.force_numerical_value_in(cm);
    const double step_cm = 0.5 * map.getMapConfig().resolution.force_numerical_value_in(cm);
    const Orientation center_abs =
        bm::normalizeOrientation(bm::absoluteBeamOrientation(drone_heading, relative_scan));

    bool found = false;
    forEachConeBeam(cfg, center_abs, [&](const Orientation& beam) {
        detail::walkBeam(map, origin, beam, z_max_cm, step_cm, [&](const Position3D&) {
            found = true;
            return false;  // stop this beam
        });
        return !found;     // stop the cone once anything unresolved is seen
    });
    return found;
}
```

Note the ordering constraint: `detail::beamsOnCircle` must be declared before `forEachConeBeam`, and `detail::walkBeam` before the two public functions. Move the existing `namespace detail { beamsOnCircle }` block above `forEachConeBeam` if it is not already.

- [x] **Step 4: Run the tests to verify they pass**

Run: `cmake --build build -j && ./build/Algorithm/algorithm_test --gtest_filter='LidarCone.*'`
Expected: PASS, all cone tests including the four pre-existing ones.

- [x] **Step 5: Verify the C gate did not change behaviour**

Run: `./build/Algorithm/algorithm_test --gtest_filter='-MappingAlgorithm.FrontierStartPassableWhenSphereHasUnmapped'`
Expected: PASS. `coneCoversUnresolved` is a refactor with identical semantics; any failure here is a regression in the extracted walk, not an intended change.

- [x] **Step 6: Stage and propose the commit**

```bash
git add UserCommon/include/user_common_207190406_209543255/LidarCone.h Algorithm/tests/test_lidar_cone.cpp
# propose: "refactor: share one cone beam walk and add unresolved-voxel counter"
```

---

### Task 2: Reachability enumeration, line-of-sight, and the ALG28 expansion bound

Additive only — nothing is deleted yet, so the existing algorithm keeps working while the substrate grows what the planner needs. The expansion bound is the ex2 grading lesson (`docs/ex2-grading-handoff.md:160-166`): today the visited set is bounded only by `isSpherePassable` returning false, so forcing passability true walks an unbounded integer grid and hangs.

**Files:**
- Modify: `Algorithm/src/MappingAlgorithmFrontier.h`, `Algorithm/src/MappingAlgorithmFrontier.cpp`
- Test: `Algorithm/tests/test_mapping_algorithm_frontier.cpp`

**Interfaces:**
- Consumes: existing file-local `isSpherePassable`, `occupancyAt`, `traversalCost`, `keyToPoint`, `quantizePosition`, `kOffsets`, `reconstructPath`.
- Produces, in `namespace algorithm_207190406_209543255::detail`:
  - `struct ReachableCell { GridKey key; common::Position3D position; int cost; int unmapped_neighbours; };`
  - `struct ReachabilityResult { bool start_passable; bool truncated; GridKey start_key; ParentMap parent_of; std::vector<ReachableCell> candidates; };`
  - `ReachabilityResult MappingAlgorithmFrontier::exploreReachable(const common::IMap3D&, const common::Position3D& start, common::PhysicalLength drone_radius, const BlockedCells&, int stride_cells, std::size_t max_expansions) const`
  - `FrontierPathResult reconstructPathTo(const ParentMap&, const GridKey& start_key, const GridKey& goal_key, const common::types::MapConfig&)`
  - `bool hasClearLineOfSight(const common::IMap3D&, const common::Position3D& from, const common::Position3D& to, common::PhysicalLength drone_radius)`
  - `std::size_t maxExpansionsForMap(const common::IMap3D&)`

- [x] **Step 1: Write the failing tests**

Append to `Algorithm/tests/test_mapping_algorithm_frontier.cpp`. That file already provides `detail`, `Map`, `pointCm(x, y, z)`, `makeCm10Config()` (0..100 cm at 10 cm) and `ct` — reuse them; do not add parallel helpers.

```cpp
TEST(MappingAlgorithm, ExploreReachableFindsFrontierAdjacentCandidates) {
    const ct::MapConfig config = makeCm10Config();
    Map map{{11, 11, 11}, config, ct::VoxelOccupancy::Empty};
    // A single Unmapped pocket makes exactly its neighbours frontier-adjacent.
    map.set(pointCm(50, 50, 50), ct::VoxelOccupancy::Unmapped);

    const detail::MappingAlgorithmFrontier frontier;
    const Position3D start = pointCm(0, 0, 0);
    const detail::ReachabilityResult result = frontier.exploreReachable(
        map, start, 4.0 * cm, {}, 1, detail::maxExpansionsForMap(map));

    EXPECT_TRUE(result.start_passable);
    EXPECT_FALSE(result.truncated);
    ASSERT_FALSE(result.candidates.empty());
    for (const detail::ReachableCell& cell : result.candidates) {
        EXPECT_GT(cell.unmapped_neighbours, 0);
        EXPECT_GT(cell.cost, 0);
    }
}

TEST(MappingAlgorithm, ExploreReachableStrideDeduplicatesCandidates) {
    const ct::MapConfig config = makeCm10Config();
    Map map{{11, 11, 11}, config, ct::VoxelOccupancy::Unmapped};
    // Everything Unmapped: every reachable cell is frontier-adjacent, so stride
    // is the only thing bounding the candidate count.
    const detail::MappingAlgorithmFrontier frontier;
    const Position3D start = pointCm(50, 50, 50);

    const std::size_t dense = frontier.exploreReachable(
        map, start, 4.0 * cm, {}, 1, detail::maxExpansionsForMap(map)).candidates.size();
    const std::size_t strided = frontier.exploreReachable(
        map, start, 4.0 * cm, {}, 3, detail::maxExpansionsForMap(map)).candidates.size();

    EXPECT_GT(dense, strided);
    EXPECT_GT(strided, 0u);
}

TEST(MappingAlgorithm, ExploreReachableRespectsExpansionCap) {
    const ct::MapConfig config = makeCm10Config();
    Map map{{11, 11, 11}, config, ct::VoxelOccupancy::Empty};
    const detail::MappingAlgorithmFrontier frontier;
    const Position3D start = pointCm(50, 50, 50);

    const detail::ReachabilityResult result =
        frontier.exploreReachable(map, start, 4.0 * cm, {}, 1, 5);

    EXPECT_TRUE(result.truncated);
}

TEST(MappingAlgorithm, ExploreReachableReportsStartPassabilityWithCapOfOne) {
    // A cap of 1 makes this an O(1) start-passability probe — the replacement for
    // diagnose().start_passable, which task 6 deletes.
    const ct::MapConfig config = makeCm10Config();
    Map map{{11, 11, 11}, config};
    map.set(pointCm(50, 50, 50), ct::VoxelOccupancy::Empty);
    map.set(pointCm(60, 50, 50), ct::VoxelOccupancy::Occupied);

    const detail::MappingAlgorithmFrontier frontier;
    EXPECT_FALSE(frontier.exploreReachable(map, pointCm(50, 50, 50), 7.5 * cm, {}, 1, 1)
                     .start_passable);
    EXPECT_TRUE(frontier.exploreReachable(map, pointCm(50, 50, 50), 4.0 * cm, {}, 1, 1)
                    .start_passable);
}

TEST(MappingAlgorithm, LineOfSightBlockedByOccupiedVoxel) {
    const ct::MapConfig config = makeCm10Config();
    Map map{{11, 11, 11}, config, ct::VoxelOccupancy::Empty};
    const Position3D from = pointCm(0, 0, 0);
    const Position3D to = pointCm(50, 0, 0);

    EXPECT_TRUE(detail::hasClearLineOfSight(map, from, to, 4.0 * cm));

    map.set(pointCm(30, 0, 0), ct::VoxelOccupancy::Occupied);
    EXPECT_FALSE(detail::hasClearLineOfSight(map, from, to, 4.0 * cm));
}

TEST(MappingAlgorithm, MaxExpansionsCoversMapVolume) {
    const ct::MapConfig config = makeCm10Config();
    Map map{{11, 11, 11}, config, ct::VoxelOccupancy::Empty};
    // 0..100 cm inclusive at 10 cm on three axes = 11^3.
    EXPECT_GE(detail::maxExpansionsForMap(map), 11u * 11u * 11u);
}
```

If `makeCm10Config()` bounds differ from 0..100 cm, adjust the last assertion to that config's volume rather than changing the config — other tests depend on it.

- [x] **Step 2: Run the tests to verify they fail**

Run: `cmake --build build -j && ./build/Algorithm/algorithm_test --gtest_filter='MappingAlgorithm.Explore*:MappingAlgorithm.LineOfSight*:MappingAlgorithm.MaxExpansions*'`
Expected: compile error — `exploreReachable`, `hasClearLineOfSight`, `maxExpansionsForMap` do not exist.

- [x] **Step 3: Declare the new API**

In `MappingAlgorithmFrontier.h`, after `FrontierPathResult`:

```cpp
/// A reachable cell that borders unresolved space — an NBV viewpoint candidate.
struct ReachableCell {
    GridKey key{};
    common::Position3D position{};
    int cost = 0;                 ///< Dijkstra traversal cost from start.
    int unmapped_neighbours = 0;  ///< Unmapped cells in the 26-neighbourhood (cheap prefilter).
};

struct ReachabilityResult {
    bool start_passable = false;
    bool truncated = false;  ///< Expansion cap hit; treat the result as partial.
    GridKey start_key{};
    ParentMap parent_of{};
    std::vector<ReachableCell> candidates{};
};
```

Inside the class, next to `findPathTo`:

```cpp
    /// Dijkstra over the passable component with a FIXED edge set, collecting
    /// frontier-adjacent cells deduplicated onto a stride lattice (lowest cost per
    /// bucket wins). Bounded by max_expansions so a broken passability check yields
    /// a truncated result instead of an unbounded walk (ALG28).
    [[nodiscard]] ReachabilityResult exploreReachable(
        const common::IMap3D& map,
        const common::Position3D& start,
        common::PhysicalLength drone_radius,
        const BlockedCells& blocked_cells,
        int stride_cells,
        std::size_t max_expansions) const;
```

And after the class, next to the other free functions:

```cpp
/// Walks parent links from goal back to start. Empty result when goal is unreachable.
[[nodiscard]] FrontierPathResult reconstructPathTo(const ParentMap& parent_of,
                                                   const GridKey& start_key,
                                                   const GridKey& goal_key,
                                                   const common::types::MapConfig& config);

/// True when the straight segment from..to is sphere-passable at every half-resolution sample.
[[nodiscard]] bool hasClearLineOfSight(const common::IMap3D& map,
                                       const common::Position3D& from,
                                       const common::Position3D& to,
                                       common::PhysicalLength drone_radius);

/// Voxel count of the mission bounds — the expansion cap for every search.
[[nodiscard]] std::size_t maxExpansionsForMap(const common::IMap3D& map);
```

- [x] **Step 4: Implement**

In `MappingAlgorithmFrontier.cpp`, add `#include <cstddef>` and `#include <unordered_map>` if absent, then append these definitions inside `namespace algorithm_207190406_209543255::detail`:

```cpp
std::size_t maxExpansionsForMap(const IMap3D& map) {
    const types::MapConfig config = map.getMapConfig();
    const double step = gridStepCm(config);
    if (!(step > 0.0)) {
        return 1;
    }
    const types::MappingBounds& b = config.boundaries;
    const auto span = [step](double lo, double hi) {
        return static_cast<std::size_t>(std::max(0.0, std::floor((hi - lo) / step))) + 1U;
    };
    const std::size_t nx = span(b.min_x.force_numerical_value_in(cm),
                                b.max_x.force_numerical_value_in(cm));
    const std::size_t ny = span(b.min_y.force_numerical_value_in(cm),
                                b.max_y.force_numerical_value_in(cm));
    const std::size_t nz = span(b.min_height.force_numerical_value_in(cm),
                                b.max_height.force_numerical_value_in(cm));
    return nx * ny * nz;
}

bool hasClearLineOfSight(const IMap3D& map,
                         const Position3D& from,
                         const Position3D& to,
                         PhysicalLength drone_radius) {
    const types::MapConfig config = map.getMapConfig();
    const double step = gridStepCm(config);
    if (!(step > 0.0)) {
        return false;
    }
    const double radius_cm = drone_radius.force_numerical_value_in(cm);
    const double dx = to.x.force_numerical_value_in(cm) - from.x.force_numerical_value_in(cm);
    const double dy = to.y.force_numerical_value_in(cm) - from.y.force_numerical_value_in(cm);
    const double dz = to.z.force_numerical_value_in(cm) - from.z.force_numerical_value_in(cm);
    const double length = std::sqrt(dx * dx + dy * dy + dz * dz);
    const double sample = step * 0.5;
    const int samples = static_cast<int>(std::ceil(length / sample));

    for (int i = 0; i <= samples; ++i) {
        const double t = (samples == 0) ? 0.0 : static_cast<double>(i) / samples;
        const Position3D probe{
            (from.x.force_numerical_value_in(cm) + dx * t) * x_extent[cm],
            (from.y.force_numerical_value_in(cm) + dy * t) * y_extent[cm],
            (from.z.force_numerical_value_in(cm) + dz * t) * z_extent[cm],
        };
        if (!isSpherePassable(map, probe, radius_cm, step, {})) {
            return false;
        }
    }
    return true;
}

FrontierPathResult reconstructPathTo(const ParentMap& parent_of,
                                     const GridKey& start_key,
                                     const GridKey& goal_key,
                                     const types::MapConfig& config) {
    if (!parent_of.contains(goal_key)) {
        return {};
    }
    FrontierPathResult result = reconstructPath(start_key, goal_key, parent_of, config);
    result.frontier_key = goal_key;
    return result;
}

ReachabilityResult MappingAlgorithmFrontier::exploreReachable(
    const IMap3D& map,
    const Position3D& start,
    PhysicalLength drone_radius,
    const BlockedCells& blocked_cells,
    int stride_cells,
    std::size_t max_expansions) const {
    const types::MapConfig config = map.getMapConfig();
    const double step = gridStepCm(config);
    const double radius_cm = drone_radius.force_numerical_value_in(cm);
    const int stride = std::max(1, stride_cells);

    ReachabilityResult out;
    out.start_key = quantizePosition(start, config);
    const Position3D start_pt = keyToPoint(out.start_key, config);
    if (!isSpherePassable(map, start_pt, radius_cm, step, blocked_cells)) {
        return out;
    }
    out.start_passable = true;

    GridIntMap cost_of;
    CostQueue queue;
    out.parent_of[out.start_key] = out.start_key;
    cost_of[out.start_key] = 0;
    queue.push({0, out.start_key});

    // bucket key -> index into out.candidates, lowest cost per bucket wins.
    std::unordered_map<GridKey, std::size_t, GridKeyHash> bucket_of;
    std::size_t expansions = 0;

    while (!queue.empty()) {
        if (++expansions > max_expansions) {
            out.truncated = true;
            break;
        }
        const auto [current_cost, current] = queue.top();
        queue.pop();
        if (current_cost > cost_of.at(current)) {
            continue;
        }
        const Position3D current_pt = keyToPoint(current, config);

        if (!(current == out.start_key)) {
            int unmapped = 0;
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dz = -1; dz <= 1; ++dz) {
                        if (dx == 0 && dy == 0 && dz == 0) {
                            continue;
                        }
                        const Position3D nb = keyToPoint(
                            GridKey{current.qx + dx, current.qy + dy, current.qz + dz}, config);
                        if (occupancyAt(map, nb) == types::VoxelOccupancy::Unmapped) {
                            ++unmapped;
                        }
                    }
                }
            }
            if (unmapped > 0) {
                const auto floor_div = [stride](int v) {
                    return (v >= 0) ? (v / stride) : -(((-v) + stride - 1) / stride);
                };
                const GridKey bucket{floor_div(current.qx), floor_div(current.qy),
                                     floor_div(current.qz)};
                const ReachableCell cell{current, current_pt, current_cost, unmapped};
                const auto it = bucket_of.find(bucket);
                if (it == bucket_of.end()) {
                    bucket_of[bucket] = out.candidates.size();
                    out.candidates.push_back(cell);
                } else if (current_cost < out.candidates[it->second].cost) {
                    out.candidates[it->second] = cell;
                }
            }
        }

        for (const Offset& off : kOffsets) {
            const GridKey neighbour{current.qx + off.dx, current.qy + off.dy, current.qz + off.dz};
            const Position3D neighbour_pt = keyToPoint(neighbour, config);
            if (!isSpherePassable(map, neighbour_pt, radius_cm, step, blocked_cells)) {
                continue;
            }
            const int new_cost = current_cost + traversalCost(map, neighbour_pt);
            if (cost_of.contains(neighbour) && new_cost >= cost_of.at(neighbour)) {
                continue;
            }
            out.parent_of[neighbour] = current;
            cost_of[neighbour] = new_cost;
            queue.push({new_cost, neighbour});
        }
    }

    return out;
}
```

There is deliberately **no** mid-search edge-set change here — unlike `findPath:456-460`, the edge set is fixed, so the Dijkstra costs are real shortest-path costs. Frontier preference lives in the NBV objective, not in the search.

- [x] **Step 5: Run the tests to verify they pass**

Run: `cmake --build build -j && ./build/Algorithm/algorithm_test --gtest_filter='MappingAlgorithm.Explore*:MappingAlgorithm.LineOfSight*:MappingAlgorithm.MaxExpansions*'`
Expected: PASS (5 tests).

- [x] **Step 6: Add the ALG28 regression test**

The mutation the ex2 grader used was `isSpherePassable` → `return true`. We cannot mutate a static function from a test, so assert the property that makes the mutation survivable: a search on an all-`Unmapped` map with no occupancy anywhere terminates and reports truncation rather than running forever.

```cpp
TEST(MappingAlgorithm, ExploreReachableTerminatesWithoutOccupancyBound) {
    ct::MapConfig config = makeCm10Config();
    // Huge bounds, nothing Occupied: only the expansion cap can stop the walk.
    config.boundaries.max_x = 100000.0 * x_extent[cm];
    config.boundaries.max_y = 100000.0 * y_extent[cm];
    config.boundaries.max_height = 100000.0 * z_extent[cm];
    Map map{{10000, 10000, 10000}, config, ct::VoxelOccupancy::Unmapped};

    const detail::MappingAlgorithmFrontier frontier;
    const Position3D start = pointCm(0, 0, 0);
    const detail::ReachabilityResult result =
        frontier.exploreReachable(map, start, 4.0 * cm, {}, 3, 20000);

    EXPECT_TRUE(result.truncated);
}
```

Run: `./build/Algorithm/algorithm_test --gtest_filter='MappingAlgorithm.ExploreReachableTerminatesWithoutOccupancyBound'`
Expected: PASS in well under a second.

- [x] **Step 7: Stage and propose the commit**

```bash
git add Algorithm/src/MappingAlgorithmFrontier.h Algorithm/src/MappingAlgorithmFrontier.cpp Algorithm/tests/test_mapping_algorithm_frontier.cpp
# propose: "feat: add bounded reachability enumeration and line-of-sight to frontier substrate"
```

---

### Task 3: Path shaping — constant-altitude string-pulling and the real step-cost model

Two pure functions with no policy in them. The step-cost model is what makes the NBV objective honest: `drone_large` rotates 45° per step, so a right-angle turn costs it two steps, and Dijkstra cell cost cannot see that.

Smoothing merges only across segments at constant altitude, because `movementToward` emits `Elevate` before any horizontal motion — the drone flies an L through a mixed segment, so a 3D diagonal line-of-sight test would validate a trajectory it never flies.

**Files:**
- Create: `Algorithm/src/PathShaping.h`, `Algorithm/src/PathShaping.cpp`
- Create: `Algorithm/tests/test_path_shaping.cpp`
- Modify: `Algorithm/CMakeLists.txt`

**Interfaces:**
- Consumes: `detail::hasClearLineOfSight` (task 2).
- Produces, in `namespace algorithm_207190406_209543255::detail`:
  - `struct MovementLimits { common::PhysicalLength max_advance; common::PhysicalLength max_elevate; common::HorizontalAngle max_rotate; };`
  - `std::vector<common::Position3D> stringPullConstantAltitude(const common::IMap3D&, const std::vector<common::Position3D>& path, common::PhysicalLength drone_radius)`
  - `std::size_t stepCostForPath(const std::vector<common::Position3D>& waypoints, const common::Position3D& start_position, const common::Orientation& start_heading, const MovementLimits&)`

- [x] **Step 1: Write the failing tests**

Create `Algorithm/tests/test_path_shaping.cpp`:

```cpp
// test_path_shaping.cpp — string-pulling and the command-level step-cost model.

#include "FakeMap3D.h"
#include "PathShaping.h"

#include <gtest/gtest.h>

namespace detail = algorithm_207190406_209543255::detail;
using Map = AlgorithmTest::FakeMap3D;

namespace {

using common::Orientation;
using common::Position3D;
using common::cm;
using common::deg;
using common::x_extent;
using common::y_extent;
using common::z_extent;
namespace ct = common::types;

[[nodiscard]] ct::MapConfig makeConfig() {
    ct::MapConfig config{};
    config.resolution = 10.0 * cm;
    config.offset = Position3D{};
    config.boundaries.min_x = 0.0 * x_extent[cm];
    config.boundaries.max_x = 100.0 * x_extent[cm];
    config.boundaries.min_y = 0.0 * y_extent[cm];
    config.boundaries.max_y = 100.0 * y_extent[cm];
    config.boundaries.min_height = 0.0 * z_extent[cm];
    config.boundaries.max_height = 100.0 * z_extent[cm];
    return config;
}

[[nodiscard]] Position3D at(double x, double y, double z) {
    return Position3D{x * x_extent[cm], y * y_extent[cm], z * z_extent[cm]};
}

// drone_small: 30 cm advance, 20 cm elevate, 90 deg rotate.
[[nodiscard]] detail::MovementLimits smallDrone() {
    return {30.0 * cm, 20.0 * cm, 90.0 * deg};
}

// drone_large: 50 cm advance, 40 cm elevate, 45 deg rotate.
[[nodiscard]] detail::MovementLimits largeDrone() {
    return {50.0 * cm, 40.0 * cm, 45.0 * deg};
}

} // namespace

TEST(PathShaping, StringPullCollapsesStaircaseAtConstantAltitude) {
    Map map{{11, 11, 11}, makeConfig(), ct::VoxelOccupancy::Empty};
    const std::vector<Position3D> staircase{
        at(0, 0, 0), at(10, 0, 0), at(10, 10, 0), at(20, 10, 0), at(20, 20, 0),
    };

    const std::vector<Position3D> pulled =
        detail::stringPullConstantAltitude(map, staircase, 4.0 * cm);

    EXPECT_LT(pulled.size(), staircase.size());
    EXPECT_EQ(pulled.front().x.force_numerical_value_in(cm), 0.0);
    EXPECT_EQ(pulled.back().x.force_numerical_value_in(cm), 20.0);
    EXPECT_EQ(pulled.back().y.force_numerical_value_in(cm), 20.0);
}

TEST(PathShaping, StringPullDoesNotCutThroughOccupiedVoxel) {
    Map map{{11, 11, 11}, makeConfig(), ct::VoxelOccupancy::Empty};
    map.set(at(10, 10, 0), ct::VoxelOccupancy::Occupied);
    const std::vector<Position3D> around{
        at(0, 0, 0), at(0, 10, 0), at(0, 20, 0), at(10, 20, 0), at(20, 20, 0),
    };

    const std::vector<Position3D> pulled =
        detail::stringPullConstantAltitude(map, around, 4.0 * cm);

    for (const Position3D& wp : pulled) {
        EXPECT_NE(map.atVoxel(wp), ct::VoxelOccupancy::Occupied);
    }
    // The diagonal shortcut passes through the occupied cell, so the corner survives.
    EXPECT_GE(pulled.size(), 3u);
}

TEST(PathShaping, StringPullKeepsAltitudeChangesSplit) {
    Map map{{11, 11, 11}, makeConfig(), ct::VoxelOccupancy::Empty};
    const std::vector<Position3D> climb{
        at(0, 0, 0), at(10, 0, 0), at(10, 0, 10), at(20, 0, 10),
    };

    const std::vector<Position3D> pulled =
        detail::stringPullConstantAltitude(map, climb, 4.0 * cm);

    // The altitude change at index 2 must remain its own waypoint.
    bool has_transition = false;
    for (const Position3D& wp : pulled) {
        if (wp.z.force_numerical_value_in(cm) == 10.0 &&
            wp.x.force_numerical_value_in(cm) == 10.0) {
            has_transition = true;
        }
    }
    EXPECT_TRUE(has_transition);
}

TEST(PathShaping, StepCostChargesTwoRotationsForRightAngleOnLargeDrone) {
    // Facing +X, target straight ahead: advance only, no rotation.
    const std::size_t ahead = detail::stepCostForPath(
        {at(50, 0, 0)}, at(0, 0, 0), Orientation{0.0 * deg, 0.0 * deg}, largeDrone());
    EXPECT_EQ(ahead, 1u);  // 50 cm at 50 cm per advance

    // Facing +X, target to the left: 90 deg turn at 45 deg per step = 2 rotations.
    const std::size_t sideways = detail::stepCostForPath(
        {at(0, 50, 0)}, at(0, 0, 0), Orientation{0.0 * deg, 0.0 * deg}, largeDrone());
    EXPECT_EQ(sideways, 3u);  // 2 rotate + 1 advance
}

TEST(PathShaping, StepCostChargesOneRotationForRightAngleOnSmallDrone) {
    const std::size_t sideways = detail::stepCostForPath(
        {at(0, 30, 0)}, at(0, 0, 0), Orientation{0.0 * deg, 0.0 * deg}, smallDrone());
    EXPECT_EQ(sideways, 2u);  // 1 rotate (90 deg) + 1 advance (30 cm)
}

TEST(PathShaping, StepCostChargesElevationSeparately) {
    const std::size_t climb = detail::stepCostForPath(
        {at(0, 0, 60)}, at(0, 0, 0), Orientation{0.0 * deg, 0.0 * deg}, smallDrone());
    EXPECT_EQ(climb, 3u);  // 60 cm at 20 cm per elevate, no rotation or advance
}

TEST(PathShaping, StepCostOfEmptyPathIsZero) {
    EXPECT_EQ(detail::stepCostForPath({}, at(0, 0, 0), Orientation{}, smallDrone()), 0u);
}
```

- [x] **Step 2: Run the tests to verify they fail**

Run: `cmake --build build -j`
Expected: FAIL — `PathShaping.h` does not exist and the test target does not know the file.

- [x] **Step 3: Create the header**

`Algorithm/src/PathShaping.h`:

```cpp
#pragma once

// Path shaping for the NBV policy: string-pulling and the command-level step cost.
// Internal to Algorithm — not part of the public API.

#include <Common/IMap3D.h>
#include <Common/Units.h>

#include <cstddef>
#include <vector>

namespace algorithm_207190406_209543255::detail {

/// Per-command movement limits from DroneConfigData.
struct MovementLimits {
    common::PhysicalLength max_advance{};
    common::PhysicalLength max_elevate{};
    common::HorizontalAngle max_rotate{};
};

/// Removes intermediate waypoints whose straight-line shortcut is clear, merging only
/// across runs at constant altitude. Elevation changes stay as their own waypoints
/// because movement emits Elevate before horizontal motion, so a mixed segment is
/// flown as an L and a 3D diagonal would not describe the real trajectory.
[[nodiscard]] std::vector<common::Position3D> stringPullConstantAltitude(
    const common::IMap3D& map,
    const std::vector<common::Position3D>& path,
    common::PhysicalLength drone_radius);

/// Mission steps to fly `waypoints` from (start_position, start_heading), charged
/// against the real command set: ceil(dz/max_elevate) elevates, ceil(turn/max_rotate)
/// rotations per heading change, ceil(run/max_advance) advances.
[[nodiscard]] std::size_t stepCostForPath(const std::vector<common::Position3D>& waypoints,
                                          const common::Position3D& start_position,
                                          const common::Orientation& start_heading,
                                          const MovementLimits& limits);

} // namespace algorithm_207190406_209543255::detail
```

- [x] **Step 4: Implement**

`Algorithm/src/PathShaping.cpp`:

```cpp
// PathShaping.cpp — string-pulling and the command-level step-cost model.

#include "PathShaping.h"

#include "MappingAlgorithmFrontier.h"

#include <cmath>
#include <numbers>

namespace algorithm_207190406_209543255::detail {

namespace {

using common::Orientation;
using common::Position3D;
using common::cm;
using common::deg;

constexpr double kSameAltitudeEpsilonCm = 1e-6;
constexpr double kSameAxisEpsilonCm = 1e-6;

[[nodiscard]] double xCm(const Position3D& p) { return p.x.force_numerical_value_in(cm); }
[[nodiscard]] double yCm(const Position3D& p) { return p.y.force_numerical_value_in(cm); }
[[nodiscard]] double zCm(const Position3D& p) { return p.z.force_numerical_value_in(cm); }

[[nodiscard]] bool sameAltitude(const Position3D& a, const Position3D& b) {
    return std::abs(zCm(a) - zCm(b)) <= kSameAltitudeEpsilonCm;
}

[[nodiscard]] std::size_t ceilDiv(double amount, double per_step) {
    if (!(per_step > 0.0) || amount <= 0.0) {
        return 0;
    }
    return static_cast<std::size_t>(std::ceil(amount / per_step - 1e-9));
}

[[nodiscard]] double wrapDeg(double degrees) {
    double x = std::fmod(degrees, 360.0);
    if (x <= -180.0) {
        x += 360.0;
    } else if (x > 180.0) {
        x -= 360.0;
    }
    return x;
}

} // namespace

std::vector<Position3D> stringPullConstantAltitude(const common::IMap3D& map,
                                                   const std::vector<Position3D>& path,
                                                   common::PhysicalLength drone_radius) {
    if (path.size() <= 2) {
        return path;
    }

    std::vector<Position3D> out;
    out.reserve(path.size());
    out.push_back(path.front());

    std::size_t anchor = 0;
    while (anchor + 1 < path.size()) {
        std::size_t best = anchor + 1;
        for (std::size_t probe = anchor + 2; probe < path.size(); ++probe) {
            if (!sameAltitude(path[anchor], path[probe])) {
                break;  // an altitude change ends the mergeable run
            }
            if (!hasClearLineOfSight(map, path[anchor], path[probe], drone_radius)) {
                break;
            }
            best = probe;
        }
        out.push_back(path[best]);
        anchor = best;
    }

    return out;
}

std::size_t stepCostForPath(const std::vector<Position3D>& waypoints,
                            const Position3D& start_position,
                            const Orientation& start_heading,
                            const MovementLimits& limits) {
    const double advance_cm = limits.max_advance.force_numerical_value_in(cm);
    const double elevate_cm = limits.max_elevate.force_numerical_value_in(cm);
    const double rotate_deg = limits.max_rotate.force_numerical_value_in(deg);

    std::size_t steps = 0;
    Position3D from = start_position;
    double heading_deg = start_heading.horizontal.force_numerical_value_in(deg);

    for (const Position3D& to : waypoints) {
        const double dz = zCm(to) - zCm(from);
        if (std::abs(dz) > kSameAxisEpsilonCm) {
            steps += ceilDiv(std::abs(dz), elevate_cm);
        }

        const double dx = xCm(to) - xCm(from);
        const double dy = yCm(to) - yCm(from);
        const double planar = std::sqrt(dx * dx + dy * dy);
        if (planar > kSameAxisEpsilonCm) {
            const double target_deg = std::atan2(dy, dx) * (180.0 / std::numbers::pi);
            const double turn = std::abs(wrapDeg(target_deg - heading_deg));
            if (turn > 1e-9) {
                steps += ceilDiv(turn, rotate_deg);
                heading_deg = target_deg;
            }
            steps += ceilDiv(planar, advance_cm);
        }

        from = to;
    }

    return steps;
}

} // namespace algorithm_207190406_209543255::detail
```

- [x] **Step 5: Register the new sources**

In `Algorithm/CMakeLists.txt`, add `src/PathShaping.cpp` to both the `Algorithm_207190406_209543255` source list and the `algorithm_test` source list, and add `tests/test_path_shaping.cpp` to `algorithm_test`:

```cmake
add_library(Algorithm_207190406_209543255 SHARED
    src/MappingAlgorithmFrontier.cpp
    src/MappingAlgorithmImpl.cpp
    src/PathShaping.cpp
)
```

```cmake
add_executable(algorithm_test
    tests/test_mapping_algorithm.cpp
    tests/test_mapping_algorithm_frontier.cpp
    tests/test_lidar_cone.cpp
    tests/test_path_shaping.cpp
    src/MappingAlgorithmFrontier.cpp
    src/MappingAlgorithmImpl.cpp
    src/PathShaping.cpp
    tests/StubPluginRegistration.cpp
)
```

- [x] **Step 6: Run the tests to verify they pass**

Run: `cmake -S . -B build && cmake --build build -j && ./build/Algorithm/algorithm_test --gtest_filter='PathShaping.*'`
Expected: PASS (7 tests).

- [x] **Step 7: Stage and propose the commit**

```bash
git add Algorithm/src/PathShaping.h Algorithm/src/PathShaping.cpp Algorithm/tests/test_path_shaping.cpp Algorithm/CMakeLists.txt
# propose: "feat: add constant-altitude string-pulling and command-level step cost"
```

---

### Task 4: `NbvPlanner` — the policy

Everything decided in the spec lands here: enumerate candidates, cheap-prefilter to 16, score by unresolved voxels per step, discard anything the remaining budget cannot afford, return a plan.

The current pose is always a candidate with an empty path — that is what makes the drone sweep in place at spawn, when nothing else is reachable yet.

**Files:**
- Create: `Algorithm/src/NbvPlanner.h`, `Algorithm/src/NbvPlanner.cpp`
- Create: `Algorithm/tests/test_nbv_planner.cpp`
- Modify: `Algorithm/CMakeLists.txt`

**Interfaces:**
- Consumes: `detail::MappingAlgorithmFrontier::exploreReachable`, `detail::reconstructPathTo`, `detail::maxExpansionsForMap`, `detail::stringPullConstantAltitude`, `detail::stepCostForPath`, `lidar_cone::{coneHalfAngleRad, directionCountForHalfAngle, fibonacciSphereOrientations, countUnresolvedVoxels}`.
- Produces, in `namespace algorithm_207190406_209543255::detail`:
  - `struct ExplorationPlan { std::vector<common::Position3D> waypoints; std::vector<common::Orientation> terminal_scans; double expected_gain; bool valid; };`
  - `class NbvPlanner` with `ExplorationPlan plan(const NbvInputs&) const;`
  - `struct NbvInputs { const common::IMap3D& map; const common::types::DroneState& state; const common::types::LidarConfigData& lidar; const common::types::DroneConfigData& drone; std::size_t remaining_steps; const BlockedCells& blocked; bool ignore_blocked; };`

- [x] **Step 1: Write the failing tests**

Create `Algorithm/tests/test_nbv_planner.cpp`:

```cpp
// test_nbv_planner.cpp — NBV objective, feasibility filter, determinism.

#include "FakeMap3D.h"
#include "NbvPlanner.h"

#include <gtest/gtest.h>

namespace detail = algorithm_207190406_209543255::detail;
using Map = AlgorithmTest::FakeMap3D;

namespace {

using common::Orientation;
using common::Position3D;
using common::cm;
using common::deg;
using common::x_extent;
using common::y_extent;
using common::z_extent;
namespace ct = common::types;

[[nodiscard]] ct::MapConfig makeConfig() {
    ct::MapConfig config{};
    config.resolution = 10.0 * cm;
    config.offset = Position3D{};
    config.boundaries.min_x = 0.0 * x_extent[cm];
    config.boundaries.max_x = 200.0 * x_extent[cm];
    config.boundaries.min_y = 0.0 * y_extent[cm];
    config.boundaries.max_y = 200.0 * y_extent[cm];
    config.boundaries.min_height = 0.0 * z_extent[cm];
    config.boundaries.max_height = 200.0 * z_extent[cm];
    return config;
}

[[nodiscard]] ct::LidarConfigData makeLidar() {
    ct::LidarConfigData cfg{};
    cfg.z_min = 20.0 * cm;
    cfg.z_max = 80.0 * cm;
    cfg.d = 2.5 * cm;
    cfg.fov_circles = 4;
    return cfg;
}

[[nodiscard]] ct::DroneConfigData makeDrone() {
    ct::DroneConfigData cfg{};
    cfg.radius = 4.0 * cm;
    cfg.max_rotate = 90.0 * deg;
    cfg.max_advance = 30.0 * cm;
    cfg.max_elevate = 20.0 * cm;
    return cfg;
}

[[nodiscard]] Position3D at(double x, double y, double z) {
    return Position3D{x * x_extent[cm], y * y_extent[cm], z * z_extent[cm]};
}

[[nodiscard]] ct::DroneState stateAt(const Position3D& p, std::size_t step_index = 0) {
    return ct::DroneState{p, Orientation{0.0 * deg, 0.0 * deg}, step_index};
}

} // namespace

TEST(NbvPlanner, PlansTowardTheUnexploredSideOfTheMap) {
    Map map{{21, 21, 21}, makeConfig(), ct::VoxelOccupancy::Empty};
    // Unmapped pocket far along +X only.
    for (int z = 0; z <= 2; ++z) {
        for (int y = 8; y <= 12; ++y) {
            map.set(at(180.0, y * 10.0, z * 10.0), ct::VoxelOccupancy::Unmapped);
        }
    }

    const detail::NbvPlanner planner;
    const ct::DroneState state = stateAt(at(50.0, 100.0, 0.0));
    const detail::BlockedCells blocked;
    const detail::ExplorationPlan plan = planner.plan(
        {map, state, makeLidar(), makeDrone(), 1000, blocked, false});

    ASSERT_TRUE(plan.valid);
    ASSERT_FALSE(plan.waypoints.empty());
    EXPECT_GT(plan.expected_gain, 0.0);
    // Movement is toward +X, where the only unresolved space is.
    EXPECT_GT(plan.waypoints.back().x.force_numerical_value_in(cm), 50.0);
}

TEST(NbvPlanner, ScansInPlaceWhenTheCurrentPoseHasGain) {
    Map map{{21, 21, 21}, makeConfig(), ct::VoxelOccupancy::Unmapped};
    // Everything unknown, as at spawn: the best viewpoint is right here.
    const detail::NbvPlanner planner;
    const ct::DroneState state = stateAt(at(100.0, 100.0, 100.0));
    const detail::BlockedCells blocked;
    const detail::ExplorationPlan plan = planner.plan(
        {map, state, makeLidar(), makeDrone(), 1000, blocked, false});

    ASSERT_TRUE(plan.valid);
    EXPECT_TRUE(plan.waypoints.empty());
    EXPECT_FALSE(plan.terminal_scans.empty());
    EXPECT_GT(plan.expected_gain, 0.0);
}

TEST(NbvPlanner, DiscardsCandidatesTheRemainingBudgetCannotAfford) {
    Map map{{21, 21, 21}, makeConfig(), ct::VoxelOccupancy::Empty};
    for (int y = 8; y <= 12; ++y) {
        map.set(at(200.0, y * 10.0, 0.0), ct::VoxelOccupancy::Unmapped);
    }

    const detail::NbvPlanner planner;
    const ct::DroneState state = stateAt(at(0.0, 100.0, 0.0));
    const detail::BlockedCells blocked;

    const detail::ExplorationPlan rich = planner.plan(
        {map, state, makeLidar(), makeDrone(), 1000, blocked, false});
    const detail::ExplorationPlan poor = planner.plan(
        {map, state, makeLidar(), makeDrone(), 2, blocked, false});

    EXPECT_TRUE(rich.valid);
    // Two steps cannot reach 200 cm away, so no travelling plan survives the filter.
    if (poor.valid) {
        EXPECT_TRUE(poor.waypoints.empty());
    }
}

TEST(NbvPlanner, IsDeterministicAcrossIdenticalCalls) {
    Map map{{21, 21, 21}, makeConfig(), ct::VoxelOccupancy::Empty};
    for (int y = 8; y <= 12; ++y) {
        map.set(at(180.0, y * 10.0, 0.0), ct::VoxelOccupancy::Unmapped);
    }

    const detail::NbvPlanner planner;
    const ct::DroneState state = stateAt(at(50.0, 100.0, 0.0));
    const detail::BlockedCells blocked;
    const detail::ExplorationPlan a = planner.plan(
        {map, state, makeLidar(), makeDrone(), 1000, blocked, false});
    const detail::ExplorationPlan b = planner.plan(
        {map, state, makeLidar(), makeDrone(), 1000, blocked, false});

    ASSERT_EQ(a.waypoints.size(), b.waypoints.size());
    for (std::size_t i = 0; i < a.waypoints.size(); ++i) {
        EXPECT_DOUBLE_EQ(a.waypoints[i].x.force_numerical_value_in(cm),
                         b.waypoints[i].x.force_numerical_value_in(cm));
        EXPECT_DOUBLE_EQ(a.waypoints[i].y.force_numerical_value_in(cm),
                         b.waypoints[i].y.force_numerical_value_in(cm));
        EXPECT_DOUBLE_EQ(a.waypoints[i].z.force_numerical_value_in(cm),
                         b.waypoints[i].z.force_numerical_value_in(cm));
    }
    EXPECT_DOUBLE_EQ(a.expected_gain, b.expected_gain);
}

TEST(NbvPlanner, ReportsInvalidWhenNothingIsUnresolved) {
    Map map{{21, 21, 21}, makeConfig(), ct::VoxelOccupancy::Empty};
    const detail::NbvPlanner planner;
    const ct::DroneState state = stateAt(at(100.0, 100.0, 100.0));
    const detail::BlockedCells blocked;
    const detail::ExplorationPlan plan = planner.plan(
        {map, state, makeLidar(), makeDrone(), 1000, blocked, false});

    EXPECT_FALSE(plan.valid);
    EXPECT_DOUBLE_EQ(plan.expected_gain, 0.0);
}

TEST(NbvPlanner, IgnoreBlockedRecoversWhenTheBlockedSetSealsTheDrone) {
    Map map{{21, 21, 21}, makeConfig(), ct::VoxelOccupancy::Empty};
    for (int y = 8; y <= 12; ++y) {
        map.set(at(180.0, y * 10.0, 0.0), ct::VoxelOccupancy::Unmapped);
    }
    const Position3D start = at(100.0, 100.0, 0.0);

    // Block every 6-neighbour of the start cell.
    detail::BlockedCells blocked;
    const ct::MapConfig config = makeConfig();
    for (const Position3D& nb : {at(110.0, 100.0, 0.0), at(90.0, 100.0, 0.0),
                                 at(100.0, 110.0, 0.0), at(100.0, 90.0, 0.0),
                                 at(100.0, 100.0, 10.0), at(100.0, 100.0, -10.0)}) {
        blocked.insert(detail::quantizePosition(nb, config));
    }

    const detail::NbvPlanner planner;
    const ct::DroneState state = stateAt(start);
    const detail::ExplorationPlan sealed = planner.plan(
        {map, state, makeLidar(), makeDrone(), 1000, blocked, false});
    const detail::ExplorationPlan recovered = planner.plan(
        {map, state, makeLidar(), makeDrone(), 1000, blocked, true});

    EXPECT_TRUE(sealed.waypoints.empty());
    EXPECT_TRUE(recovered.valid);
}
```

- [x] **Step 2: Run the tests to verify they fail**

Run: `cmake --build build -j`
Expected: FAIL — `NbvPlanner.h` does not exist.

- [x] **Step 3: Create the header**

`Algorithm/src/NbvPlanner.h`:

```cpp
#pragma once

// Frontier-anchored next-best-view policy. Internal to Algorithm.
//
// Objective: utility(v) = unresolved voxels observable from v / mission steps to get
// there and scan. Budget-awareness is a feasibility filter (a candidate costing more
// than the remaining budget is discarded), which makes the policy anytime without any
// tuned regime switch. No RNG: candidates are enumerated, so plans are reproducible.

#include "MappingAlgorithmFrontier.h"
#include "PathShaping.h"

#include <Common/IMap3D.h>
#include <Common/Units.h>
#include <Common/types/DroneTypes.h>
#include <Common/types/LidarTypes.h>

#include <cstddef>
#include <vector>

namespace algorithm_207190406_209543255::detail {

struct ExplorationPlan {
    /// Smoothed waypoints; empty means "the best viewpoint is the current pose".
    std::vector<common::Position3D> waypoints{};
    /// World-frame directions worth scanning at the viewpoint, best gain first.
    /// Converted to the drone frame at emission time.
    std::vector<common::Orientation> terminal_scans{};
    double expected_gain = 0.0;
    bool valid = false;
};

struct NbvInputs {
    const common::IMap3D& map;
    const common::types::DroneState& state;
    const common::types::LidarConfigData& lidar;
    const common::types::DroneConfigData& drone;
    std::size_t remaining_steps = 0;
    const BlockedCells& blocked;
    /// Recovery mode: plan as if the blocked set were empty.
    bool ignore_blocked = false;
};

class NbvPlanner {
public:
    [[nodiscard]] ExplorationPlan plan(const NbvInputs& in) const;

    /// World-frame scan directions for this lidar — the set the drone can actually scan.
    [[nodiscard]] static std::vector<common::Orientation> scanDirections(
        const common::types::LidarConfigData& lidar);

    /// Unresolved voxels observable from `origin` across all scan directions, plus the
    /// per-direction breakdown (positive-gain directions only, best first).
    [[nodiscard]] static double gainAt(const common::IMap3D& map,
                                       const common::Position3D& origin,
                                       const common::types::LidarConfigData& lidar,
                                       std::vector<common::Orientation>* directions_out);

private:
    static constexpr int kCandidateStrideCells = 3;
    static constexpr std::size_t kScoredCandidates = 16;

    MappingAlgorithmFrontier frontier_{};
};

} // namespace algorithm_207190406_209543255::detail
```

- [x] **Step 4: Implement**

`Algorithm/src/NbvPlanner.cpp`:

```cpp
// NbvPlanner.cpp — frontier-anchored next-best-view policy.

#include "NbvPlanner.h"

#include <user_common_207190406_209543255/LidarCone.h>

#include <algorithm>
#include <cstdint>
#include <unordered_set>

namespace algorithm_207190406_209543255::detail {

namespace lc = user_common_207190406_209543255::lidar_cone;
namespace types = common::types;

using common::Orientation;
using common::Position3D;
using common::cm;

namespace {

struct ScoredCandidate {
    const ReachableCell* cell = nullptr;
    double prefilter = 0.0;
};

[[nodiscard]] MovementLimits limitsFrom(const types::DroneConfigData& drone) {
    return MovementLimits{drone.max_advance, drone.max_elevate, drone.max_rotate};
}

/// Unique Unmapped voxels of `dirs` in listed order, with a fresh `seen` set.
/// Do not reuse per-direction `added` from `gainAt`: those were counted in
/// Fibonacci order, then the list was sorted.
[[nodiscard]] double uniqueGainOf(const common::IMap3D& map,
                                  const Position3D& origin,
                                  const types::LidarConfigData& lidar,
                                  const std::vector<Orientation>& dirs) {
    const Orientation world_heading{};
    std::unordered_set<std::int64_t> seen;
    double total = 0.0;
    for (const Orientation& dir : dirs) {
        total += static_cast<double>(
            lc::countUnresolvedVoxels(map, origin, world_heading, dir, lidar, seen));
    }
    return total;
}

} // namespace

std::vector<Orientation> NbvPlanner::scanDirections(const types::LidarConfigData& lidar) {
    const double alpha = lc::coneHalfAngleRad(lidar);
    if (!(alpha > 0.0)) {
        return {};
    }
    return lc::fibonacciSphereOrientations(lc::directionCountForHalfAngle(alpha));
}

double NbvPlanner::gainAt(const common::IMap3D& map,
                          const Position3D& origin,
                          const types::LidarConfigData& lidar,
                          std::vector<Orientation>* directions_out) {
    const std::vector<Orientation> directions = scanDirections(lidar);
    // Directions are world-frame, so pass a zero heading and treat them as relative.
    const Orientation world_heading{};

    std::vector<std::pair<double, Orientation>> per_direction;
    per_direction.reserve(directions.size());

    std::unordered_set<std::int64_t> seen;
    double total = 0.0;
    for (const Orientation& dir : directions) {
        const std::size_t added =
            lc::countUnresolvedVoxels(map, origin, world_heading, dir, lidar, seen);
        if (added > 0) {
            total += static_cast<double>(added);
            per_direction.emplace_back(static_cast<double>(added), dir);
        }
    }

    if (directions_out != nullptr) {
        // Stable sort on gain descending keeps the order reproducible for equal gains.
        std::stable_sort(per_direction.begin(), per_direction.end(),
                         [](const auto& a, const auto& b) { return a.first > b.first; });
        directions_out->clear();
        directions_out->reserve(per_direction.size());
        for (const auto& [gain, dir] : per_direction) {
            directions_out->push_back(dir);
        }
    }

    return total;
}

ExplorationPlan NbvPlanner::plan(const NbvInputs& in) const {
    const types::MapConfig config = in.map.getMapConfig();
    const MovementLimits limits = limitsFrom(in.drone);
    const BlockedCells empty_blocked;
    const BlockedCells& blocked = in.ignore_blocked ? empty_blocked : in.blocked;

    ExplorationPlan best;

    // Candidate 0: stay where we are. Costs only the scans it performs.
    {
        std::vector<Orientation> here_dirs;
        (void)gainAt(in.map, in.state.position, in.lidar, &here_dirs);
        const std::size_t scan_budget = in.remaining_steps;
        if (scan_budget > 0 && !here_dirs.empty()) {
            const std::size_t scan_steps = std::min(here_dirs.size(), scan_budget);
            std::vector<Orientation> prefix(
                here_dirs.begin(),
                here_dirs.begin() + static_cast<std::ptrdiff_t>(scan_steps));
            const double prefix_gain =
                uniqueGainOf(in.map, in.state.position, in.lidar, prefix);
            if (prefix_gain > 0.0) {
                best.valid = true;
                best.waypoints.clear();
                best.terminal_scans = std::move(prefix);
                best.expected_gain = prefix_gain;
            }
        }
    }
    double best_utility = best.valid ? best.expected_gain /
                                           static_cast<double>(best.terminal_scans.size())
                                     : 0.0;

    const ReachabilityResult reach = frontier_.exploreReachable(
        in.map, in.state.position, in.drone.radius, blocked, kCandidateStrideCells,
        maxExpansionsForMap(in.map));
    if (!reach.start_passable || reach.candidates.empty()) {
        return best;
    }

    // Cheap prefilter: unmapped neighbours per unit path cost, constant work per cell.
    std::vector<ScoredCandidate> prefiltered;
    prefiltered.reserve(reach.candidates.size());
    for (const ReachableCell& cell : reach.candidates) {
        const double cost = std::max(1, cell.cost);
        prefiltered.push_back({&cell, static_cast<double>(cell.unmapped_neighbours) / cost});
    }
    std::stable_sort(prefiltered.begin(), prefiltered.end(),
                     [](const ScoredCandidate& a, const ScoredCandidate& b) {
                         if (a.prefilter != b.prefilter) {
                             return a.prefilter > b.prefilter;
                         }
                         return a.cell->cost < b.cell->cost;  // deterministic tiebreak
                     });
    if (prefiltered.size() > kScoredCandidates) {
        prefiltered.resize(kScoredCandidates);
    }

    for (const ScoredCandidate& scored : prefiltered) {
        const ReachableCell& cell = *scored.cell;

        FrontierPathResult raw =
            reconstructPathTo(reach.parent_of, reach.start_key, cell.key, config);
        if (!raw.found || raw.path.empty()) {
            continue;
        }
        std::vector<Position3D> waypoints =
            stringPullConstantAltitude(in.map, raw.path, in.drone.radius);

        std::vector<Orientation> dirs;
        const double gain = gainAt(in.map, cell.position, in.lidar, &dirs);
        if (!(gain > 0.0) || dirs.empty()) {
            continue;
        }

        const std::size_t travel =
            stepCostForPath(waypoints, in.state.position, in.state.heading, limits);
        if (travel > in.remaining_steps) {
            continue;  // budget feasibility filter
        }
        const std::size_t scan_budget = in.remaining_steps - travel;
        if (scan_budget == 0) {
            continue;
        }

        const std::size_t scan_steps = std::min(dirs.size(), scan_budget);
        std::vector<Orientation> prefix(
            dirs.begin(), dirs.begin() + static_cast<std::ptrdiff_t>(scan_steps));
        const double prefix_gain = uniqueGainOf(in.map, cell.position, in.lidar, prefix);
        if (!(prefix_gain > 0.0)) {
            continue;
        }

        const double utility = prefix_gain / static_cast<double>(travel + prefix.size());
        if (utility > best_utility) {
            best_utility = utility;
            best.valid = true;
            best.waypoints = std::move(waypoints);
            best.terminal_scans = std::move(prefix);
            best.expected_gain = prefix_gain;
        }
    }

    return best;
}

} // namespace algorithm_207190406_209543255::detail
```

- [x] **Step 5: Register the new sources**

In `Algorithm/CMakeLists.txt`, add `src/NbvPlanner.cpp` to both target source lists and `tests/test_nbv_planner.cpp` to `algorithm_test`, mirroring what task 3 did for `PathShaping`.

- [x] **Step 6: Run the tests to verify they pass**

Run: `cmake -S . -B build && cmake --build build -j && ./build/Algorithm/algorithm_test --gtest_filter='NbvPlanner.*'`
Expected: PASS (6 tests).

If `NbvPlanner.PlansTowardTheUnexploredSideOfTheMap` fails because the in-place candidate wins, check that the far pocket is outside `z_max` of the start pose — the test intends travel to be the only way to see it. Adjust the pocket distance in the test, not the objective.

- [x] **Step 7: Stage and propose the commit**

```bash
git add Algorithm/src/NbvPlanner.h Algorithm/src/NbvPlanner.cpp Algorithm/tests/test_nbv_planner.cpp Algorithm/CMakeLists.txt
# propose: "feat: add frontier-anchored NBV planner with budget feasibility filter"
```

---

### Task 5: Rewrite `MappingAlgorithmImpl` as a plan executor

The phase machine and its mutual recursion go away. What is left: hold a plan, emit one command per `nextStep`, replan on a clear trigger, and never quit while there is still gain to be had.

Co-emission is the point of this task. MissionControl moves first and scans at the *resulting* pose (`MissionControl/src/DroneControlImpl.cpp:190-228`), so the emitted orientation must be relative to the predicted post-move heading — a `Rotate` changes it.

During travel, evaluating all ~26 directions every step would be too expensive, so probe only the six axis-aligned ones (`kTravelScanProbes = 6`), which `fibonacciSphereOrientations` always emits first. At the viewpoint, use the plan's `terminal_scans`, which were scored across the full set.

**Files:**
- Modify: `Algorithm/include/Algorithm/MappingAlgorithmImpl.h`, `Algorithm/src/MappingAlgorithmImpl.cpp`
- Test: `Algorithm/tests/test_mapping_algorithm.cpp`

**Interfaces:**
- Consumes: `detail::NbvPlanner`, `detail::ExplorationPlan`, `detail::NbvInputs`, `detail::hasAnyNotMappedInBounds`, `detail::quantizePosition`, `lidar_cone::countUnresolvedVoxels`.
- Produces: no new public API. The class name, constructor, `nextStep` signature and registration macro are unchanged.

- [x] **Step 1: Write the failing tests**

Add to `Algorithm/tests/test_mapping_algorithm.cpp`. That file's existing helpers are `makeCorridorConfig()`, `makeDroneConfig()` (radius 5 cm, rotate 45°, advance 10 cm, elevate 10 cm), `makeLidarConfig()` (`z_min` 1 cm, `z_max` 200 cm, `d` 1 cm, `fov_circles` 1), `makeMissionConfig()` (`max_steps = 10000`), `gridPoint(x, y, z, config)` and `fillEmptyBox(...)`. Algorithms are constructed inline as `Impl algorithm{common::MappingAlgorithmDependencies{mc, lc, dc, output_map}}`. Follow that; do not introduce `makeConfig`/`makeAlgorithm`/`makeState` helpers.

```cpp
// What: a travel step should carry a scan as well as a movement.
// Expected: at least one emitted command has both fields set.
TEST(MappingAlgorithm, EmitsMovementAndScanInTheSameCommand) {
    const ct::MapConfig config = makeCorridorConfig();
    Map output_map{{11, 11, 11}, config};
    fillEmptyBox(output_map, 0, 8, 0, 10, 0, 10, config);  // unknown wall beyond x=8

    const auto mc = makeMissionConfig();
    const auto lc = makeLidarConfig();
    const auto dc = makeDroneConfig();
    Impl algorithm{common::MappingAlgorithmDependencies{mc, lc, dc, output_map}};

    const ct::DroneState state{gridPoint(2, 5, 5, config), Orientation{0.0 * deg, 0.0 * deg}, 0};

    const auto cmd = firstCommandMatching(
        algorithm, state, 500, [](const ct::MappingStepCommand& step) {
            return step.movement.has_value() && step.scan_orientation.has_value();
        });

    ASSERT_TRUE(cmd.has_value()) << "no command carried both a movement and a scan";
    EXPECT_EQ(cmd->status, ct::AlgorithmStatus::Working);
}

// What: unresolved space remains and the budget is nearly untouched.
// Expected: the algorithm keeps working instead of declaring itself finished.
//
// This is the house_full failure as a unit test: the retired policy set finished=true
// on the first planning cycle whose fallbacks all missed.
TEST(MappingAlgorithm, DoesNotFinishWhileUnresolvedSpaceRemainsAndBudgetIsLarge) {
    const ct::MapConfig config = makeCorridorConfig();
    Map output_map{{11, 11, 11}, config};
    fillEmptyBox(output_map, 0, 8, 0, 10, 0, 10, config);

    const auto mc = makeMissionConfig();
    const auto lc = makeLidarConfig();
    const auto dc = makeDroneConfig();
    Impl algorithm{common::MappingAlgorithmDependencies{mc, lc, dc, output_map}};

    // The map never changes here (no MissionControl fusing scans), so this also proves
    // the policy does not need observed progress to keep trying.
    for (int step = 0; step < 50; ++step) {
        const ct::DroneState state{gridPoint(2, 5, 5, config),
                                   Orientation{0.0 * deg, 0.0 * deg},
                                   static_cast<std::size_t>(step)};
        const ct::MappingStepCommand cmd = algorithm.nextStep(state, nullptr);
        ASSERT_EQ(cmd.status, ct::AlgorithmStatus::Working) << "gave up at step " << step;
    }
}

// What: nothing is Unmapped anywhere in bounds.
// Expected: Finished (true completion), not FinishedWithUnmappableVoxels.
TEST(MappingAlgorithm, FinishesCleanlyWhenNothingIsUnmapped) {
    const ct::MapConfig config = makeCorridorConfig();
    Map output_map{{3, 3, 3}, config};
    fillEmptyBox(output_map, 0, 2, 0, 2, 0, 2, config);

    const auto mc = makeMissionConfig();
    const auto lc = makeLidarConfig();
    const auto dc = makeDroneConfig();
    Impl algorithm{common::MappingAlgorithmDependencies{mc, lc, dc, output_map}};

    const ct::DroneState state{gridPoint(1, 1, 1, config), Orientation{0.0 * deg, 0.0 * deg}, 0};

    EXPECT_EQ(runUntilTerminal(algorithm, state, 5000), ct::AlgorithmStatus::Finished);
}

// What: budget exhausted (step_index == max_steps), unresolved space still present.
// Expected: no candidate is affordable, so the policy terminates instead of spinning.
TEST(MappingAlgorithm, TerminatesWhenBudgetIsExhausted) {
    const ct::MapConfig config = makeCorridorConfig();
    Map output_map{{11, 11, 11}, config};
    fillEmptyBox(output_map, 0, 8, 0, 10, 0, 10, config);

    const auto mc = makeMissionConfig();  // max_steps = 10000
    const auto lc = makeLidarConfig();
    const auto dc = makeDroneConfig();
    Impl algorithm{common::MappingAlgorithmDependencies{mc, lc, dc, output_map}};

    const ct::DroneState state{gridPoint(2, 5, 5, config),
                              Orientation{0.0 * deg, 0.0 * deg},
                              10000};

    ct::AlgorithmStatus status = ct::AlgorithmStatus::Working;
    for (int step = 0; step < 20 && status == ct::AlgorithmStatus::Working; ++step) {
        status = algorithm.nextStep(state, nullptr).status;
    }
    EXPECT_NE(status, ct::AlgorithmStatus::Working);
}
```

Note `makeLidarConfig()` has `fov_circles = 1`, so `coneHalfAngleRad` takes the single-circle branch C added (`atan2(d, z_min)`) and directions are still produced. Do not change that fixture to make a test pass — other tests depend on it.

Delete `ScanSweepEmitsMultipleOrientationsBeforeMovement` and `ScanPassZeroIncludesAxisAlignedOrientations`: both assert the retired phase machine's sweep shape, and the co-emission test replaces their intent. Keep every test that asserts contract-level behaviour — `FirstStepRequestsScanWithNullLatestScan`, `FinishesWhenNoFrontierRemains`, `EmitsMovementTowardFrontier`, status monotonicity after finish, non-null-scan tolerance, and the no-crash cases. If `FirstStepRequestsScanWithNullLatestScan` now fails because the first command also carries a movement, relax its `EXPECT_FALSE(cmd.movement.has_value())` to allow co-emission rather than suppressing co-emission on the first step.

- [x] **Step 2: Run the tests to verify they fail**

Run: `cmake --build build -j && ./build/Algorithm/algorithm_test --gtest_filter='MappingAlgorithm.EmitsMovementAndScan*:MappingAlgorithm.DoesNotFinishWhile*:MappingAlgorithm.FinishesCleanly*:MappingAlgorithm.TerminatesWhenBudget*'`
Expected: FAIL — the current policy emits either movement or scan and gives up on the first failed planning cycle.

- [x] **Step 3: Rewrite the header's private section**

In `Algorithm/include/Algorithm/MappingAlgorithmImpl.h`, update the doc comment and replace the private members. The public section does not change.

```cpp
/// Budget-aware next-best-view exploration over a frontier reachability substrate.
/// Each nextStep emits a movement and, when the resulting pose would observe something
/// new, a scan in the same command.
```

```cpp
private:
    struct Impl;

    void ensurePlanningReady();
    [[nodiscard]] std::size_t remainingSteps(const common::types::DroneState& state) const;
    void pruneExpiredBlockedCells(std::size_t step_index);
    [[nodiscard]] bool replan(const common::types::DroneState& state, bool ignore_blocked);
    [[nodiscard]] std::optional<common::types::MovementCommand> movementToward(
        const common::types::DroneState& state, const common::Position3D& target) const;
    [[nodiscard]] common::types::DroneState predictPose(
        const common::types::DroneState& state,
        const common::types::MovementCommand& movement) const;
    [[nodiscard]] std::optional<common::Orientation> bestTravelScan(
        const common::types::DroneState& predicted) const;
    [[nodiscard]] bool reachedWaypoint(const common::types::DroneState& state,
                                       const common::Position3D& target) const;
    [[nodiscard]] bool samePosition(const common::Position3D& a,
                                    const common::Position3D& b) const;

    std::unique_ptr<Impl> impl_;

    static constexpr int kMaxMovingStallTicks = 2;
    static constexpr std::size_t kReplanIntervalSteps = 10;
    static constexpr std::size_t kBlockedTtlSteps = 50;
    static constexpr int kRecoveryAttempts = 3;
    static constexpr std::size_t kTravelScanProbes = 6;
};
```

The `Phase` enum, the four phase handlers, and `buildScanOrientations` are gone.

- [x] **Step 4: Rewrite the implementation**

In `Algorithm/src/MappingAlgorithmImpl.cpp`: keep `gridStepCm`, `axisSign`, `movementToward`, `reachedWaypoint`, `samePosition` as they are; delete `areCollinearSteps`/`compressPath` (string-pulling replaces them) and `buildScanOrientations`. Replace `struct Impl` and every handler with:

```cpp
#include "NbvPlanner.h"

struct MappingAlgorithmImpl_207190406_209543255::Impl {
    detail::NbvPlanner planner{};
    detail::ExplorationPlan plan{};
    std::size_t waypoint_index = 0;
    std::size_t terminal_scan_index = 0;
    std::size_t steps_since_replan = 0;
    bool has_plan = false;

    int moving_stall_ticks = 0;
    Position3D last_position{};
    bool has_last_position = false;

    detail::BlockedCells blocked_cells{};
    detail::GridIntMap blocked_since{};  ///< cell -> step_index of insertion (TTL)
    int recovery_attempts = 0;
    bool finished = false;
    bool planning_initialized = false;
};
```

```cpp
std::size_t MappingAlgorithmImpl_207190406_209543255::remainingSteps(
    const types::DroneState& state) const {
    const std::size_t budget = mission_config_.max_steps;
    if (budget == 0) {
        return 0;
    }
    return (state.step_index >= budget) ? 0 : (budget - state.step_index);
}

void MappingAlgorithmImpl_207190406_209543255::pruneExpiredBlockedCells(std::size_t step_index) {
    for (auto it = impl_->blocked_since.begin(); it != impl_->blocked_since.end();) {
        const auto inserted = static_cast<std::size_t>(it->second);
        if (step_index >= inserted + kBlockedTtlSteps) {
            impl_->blocked_cells.erase(it->first);
            it = impl_->blocked_since.erase(it);
        } else {
            ++it;
        }
    }
}

bool MappingAlgorithmImpl_207190406_209543255::replan(const types::DroneState& state,
                                                      bool ignore_blocked) {
    const detail::NbvInputs inputs{
        output_map_, state, lidar_config_, drone_config_,
        remainingSteps(state), impl_->blocked_cells, ignore_blocked,
    };
    impl_->plan = impl_->planner.plan(inputs);
    impl_->waypoint_index = 0;
    impl_->terminal_scan_index = 0;
    impl_->steps_since_replan = 0;
    impl_->has_plan = impl_->plan.valid;
    return impl_->has_plan;
}

types::DroneState MappingAlgorithmImpl_207190406_209543255::predictPose(
    const types::DroneState& state, const types::MovementCommand& movement) const {
    types::DroneState next = state;
    switch (movement.type) {
    case types::MovementCommandType::Hover:
        break;
    case types::MovementCommandType::Rotate: {
        const double delta = movement.angle.force_numerical_value_in(deg) *
                             ((movement.rotation == types::RotationDirection::Left) ? 1.0 : -1.0);
        next.heading.horizontal =
            (state.heading.horizontal.force_numerical_value_in(deg) + delta) * deg;
        break;
    }
    case types::MovementCommandType::Advance: {
        const double dist = movement.distance.force_numerical_value_in(cm);
        const double heading_rad =
            state.heading.horizontal.force_numerical_value_in(deg) * (std::numbers::pi / 180.0);
        next.position = Position3D{
            (state.position.x.force_numerical_value_in(cm) + dist * std::cos(heading_rad)) *
                common::x_extent[cm],
            (state.position.y.force_numerical_value_in(cm) + dist * std::sin(heading_rad)) *
                common::y_extent[cm],
            state.position.z,
        };
        break;
    }
    case types::MovementCommandType::Elevate:
        next.position = Position3D{
            state.position.x, state.position.y,
            (state.position.z.force_numerical_value_in(cm) +
             movement.distance.force_numerical_value_in(cm)) * common::z_extent[cm],
        };
        break;
    }
    return next;
}

std::optional<Orientation> MappingAlgorithmImpl_207190406_209543255::bestTravelScan(
    const types::DroneState& predicted) const {
    namespace lc = user_common_207190406_209543255::lidar_cone;
    // Probe only the axis-aligned directions (always the first six emitted) so a
    // per-step scan choice stays cheap; the viewpoint's scans were scored over the
    // full direction set at plan time.
    const std::vector<Orientation> world = detail::NbvPlanner::scanDirections(lidar_config_);
    const std::size_t probes = std::min(kTravelScanProbes, world.size());

    std::optional<Orientation> best;
    std::size_t best_gain = 0;
    for (std::size_t i = 0; i < probes; ++i) {
        std::unordered_set<std::int64_t> seen;
        const std::size_t gain = lc::countUnresolvedVoxels(
            output_map_, predicted.position, Orientation{}, world[i], lidar_config_, seen);
        if (gain > best_gain) {
            best_gain = gain;
            best = world[i];
        }
    }
    if (!best.has_value()) {
        return std::nullopt;
    }
    // Emit relative to the predicted heading: MissionControl scans after moving.
    return Orientation{best->horizontal - predicted.heading.horizontal,
                       best->altitude - predicted.heading.altitude};
}

types::MappingStepCommand MappingAlgorithmImpl_207190406_209543255::nextStep(
    const types::DroneState& state, const types::LidarScanResult* latest_scan) {
    [[maybe_unused]] const types::LidarScanResult* unused_scan = latest_scan;

    if (impl_->finished) {
        types::MappingStepCommand cmd{};
        cmd.status = types::AlgorithmStatus::Finished;
        return cmd;
    }

    ensurePlanningReady();
    pruneExpiredBlockedCells(state.step_index);

    // Stall detection: the waypoint we are driving at is not reachable in practice.
    if (impl_->has_plan && impl_->waypoint_index < impl_->plan.waypoints.size() &&
        impl_->has_last_position && samePosition(impl_->last_position, state.position)) {
        if (++impl_->moving_stall_ticks >= kMaxMovingStallTicks) {
            const auto key = detail::quantizePosition(
                impl_->plan.waypoints[impl_->waypoint_index], output_map_.getMapConfig());
            impl_->blocked_cells.insert(key);
            impl_->blocked_since[key] = static_cast<int>(state.step_index);
            impl_->moving_stall_ticks = 0;
            impl_->has_plan = false;
        }
    } else {
        impl_->moving_stall_ticks = 0;
    }
    impl_->last_position = state.position;
    impl_->has_last_position = true;

    // Advance past waypoints already reached.
    while (impl_->has_plan && impl_->waypoint_index < impl_->plan.waypoints.size() &&
           reachedWaypoint(state, impl_->plan.waypoints[impl_->waypoint_index])) {
        ++impl_->waypoint_index;
    }

    const bool plan_exhausted =
        !impl_->has_plan ||
        (impl_->waypoint_index >= impl_->plan.waypoints.size() &&
         impl_->terminal_scan_index >= impl_->plan.terminal_scans.size());
    const bool interval_elapsed = impl_->steps_since_replan >= kReplanIntervalSteps;

    if (plan_exhausted || interval_elapsed) {
        if (!replan(state, false)) {
            // Nothing feasible with the blocked set honoured: try recovery, then decide.
            if (impl_->recovery_attempts < kRecoveryAttempts && replan(state, true)) {
                ++impl_->recovery_attempts;
            } else {
                impl_->finished = true;
                types::MappingStepCommand cmd{};
                cmd.status = detail::hasAnyNotMappedInBounds(output_map_)
                                 ? types::AlgorithmStatus::FinishedWithUnmappableVoxels
                                 : types::AlgorithmStatus::Finished;
                return cmd;
            }
        } else {
            impl_->recovery_attempts = 0;
        }
    }

    ++impl_->steps_since_replan;

    types::MappingStepCommand cmd{};
    cmd.status = types::AlgorithmStatus::Working;

    if (impl_->waypoint_index < impl_->plan.waypoints.size()) {
        const Position3D& target = impl_->plan.waypoints[impl_->waypoint_index];
        cmd.movement = movementToward(state, target);
        if (cmd.movement.has_value()) {
            cmd.scan_orientation = bestTravelScan(predictPose(state, *cmd.movement));
        }
        return cmd;
    }

    if (impl_->terminal_scan_index < impl_->plan.terminal_scans.size()) {
        const Orientation& world = impl_->plan.terminal_scans[impl_->terminal_scan_index++];
        cmd.scan_orientation = Orientation{world.horizontal - state.heading.horizontal,
                                           world.altitude - state.heading.altitude};
        return cmd;
    }

    cmd.movement = types::MovementCommand{};  // Hover; the next call replans.
    impl_->has_plan = false;
    return cmd;
}
```

Add `#include <cstdint>`, `#include <unordered_set>` and `#include <vector>` to the file's include block, and drop the now-unused `<limits>` if nothing else needs it. `ensurePlanningReady` keeps only the `planning_initialized` flag (the `spacing_cells` computation is gone with travel-scan spacing).

- [x] **Step 5: Run the tests to verify they pass**

Run: `cmake --build build -j && ./build/Algorithm/algorithm_test --gtest_filter='MappingAlgorithm.*:-MappingAlgorithm.FrontierStartPassableWhenSphereHasUnmapped'`
Expected: PASS. Fix any test that asserted the retired phase machine's shape rather than a contract. (A single `--gtest_filter` with `:-` excludes; passing the flag twice silently drops the first.)

- [x] **Step 6: Run the whole suite**

Run: `ctest --test-dir build --output-on-failure -E drone_mapper_algorithm_tests; ./build/Algorithm/algorithm_test --gtest_filter=-MappingAlgorithm.FrontierStartPassableWhenSphereHasUnmapped`
Expected: everything green. MissionControl and Simulator tests must be untouched by this task.

- [x] **Step 7: Stage and propose the commit**

```bash
git add Algorithm/include/Algorithm/MappingAlgorithmImpl.h Algorithm/src/MappingAlgorithmImpl.cpp Algorithm/tests/test_mapping_algorithm.cpp
# propose: "feat: replace phase machine with NBV plan execution and move+scan co-emission"
```

**Review fix:** `adoptTravelPlan` and the extra `hasNotMappedInSphere` finish branches were removed so `nextStep` matches this skeleton. Frozen-map travel / co-emission / stall tests use a local short lidar; `makeLidarConfig()` is unchanged.

---

### Task 6: Delete the superseded search machinery

Now that nothing calls them, remove the policy-shaped helpers. This is where the never-cleared blacklist parameter, the mid-search edge-set change, the stale distance cache, the dead code, and the slow `diagnose` test all go.

**Files:**
- Modify: `Algorithm/src/MappingAlgorithmFrontier.h`, `Algorithm/src/MappingAlgorithmFrontier.cpp`
- Modify: `Algorithm/tests/test_mapping_algorithm_frontier.cpp`

- [x] **Step 1: Confirm there are no remaining callers**

Run: `grep -rn "findPath\|findExplorePath\|findAnyPassableNeighbor\|findFarthestPath\|findGreedyUnknownStep\|diagnose\|buildUnknownDistanceField\|frontier_visits\|explore_dist_cache" Algorithm/ MissionControl/ Simulator/`
Expected: hits only inside `MappingAlgorithmFrontier.{h,cpp}` and `Algorithm/tests/test_mapping_algorithm_frontier.cpp`. If anything else appears, stop and reconcile before deleting.

- [x] **Step 2: Delete from the header**

Remove these declarations from `MappingAlgorithmFrontier.h`: `findPath`, `findExplorePath`, `findFarthestPath`, `findGreedyUnknownStep`, `findAnyPassableNeighbor`, `diagnose`, and `struct PlanningDiagnostics`. Keep `findPathTo`, `findUnstickPath`, `exploreReachable`, `reconstructPathTo`, `hasClearLineOfSight`, `maxExpansionsForMap`, `quantizePosition`, `hasNotMappedInSphere`, `hasAnyNotMappedInBounds`, `countUnmappedInBounds`, and the `GridKey`/`BlockedCells`/`GridIntMap`/`ParentMap`/`FrontierPathResult`/`ReachableCell`/`ReachabilityResult` types.

- [x] **Step 3: Delete from the implementation**

Remove the corresponding definitions plus the now-unused file-local `buildUnknownDistanceField` and `canTraverseForUnknownDistance`. Add the expansion bound to the two searches that survive, so every search in the file is bounded:

```cpp
    std::size_t expansions = 0;
    const std::size_t max_expansions = maxExpansionsForMap(map);
    while (!queue.empty()) {
        if (++expansions > max_expansions) {
            return {};
        }
```

Apply that to `findPathTo` and `findUnstickPath` (in `findUnstickPath` the loop is a BFS over `std::queue`; the same counter and early `return {}` applies).

- [x] **Step 4: Update the tests**

Project C's clearance tests assert passability through `frontier.diagnose(...).start_passable`, so deleting `diagnose` breaks them. **Convert, don't delete** — `exploreReachable` with `max_expansions = 1` is an O(1) start-passability probe, which also fixes the known-slow `FrontierStartPassableWhenSphereHasUnmapped` (it was slow precisely because `diagnose` swept the 101³ 1 cm grid).

Rewrite each `diagnose` call site in this pattern:

```cpp
// before
const detail::PlanningDiagnostics diag = frontier.diagnose(map, centre, 7.5 * cm);
EXPECT_FALSE(diag.start_passable);

// after
EXPECT_FALSE(frontier.exploreReachable(map, centre, 7.5 * cm, {}, 1, 1).start_passable);
```

Call sites to convert: `FrontierStartPassableWhenSphereHasUnmapped`, `FrontierStartNotPassableWhenSphereOverlapsOccupied`, `FrontierStartNotPassableWhenCentreOccupied`, `FrontierRejectsOccupiedFaceNeighbourOnCm10Grid`, `FrontierAllowsOccupiedFaceNeighbourWhenRadiusTooSmall`. Delete `FrontierDiagnoseReportsConnectivityMetrics` (it asserts `PlanningDiagnostics` fields that no longer exist) along with the tests for the other removed functions: `FrontierFindsPathAlongEmptyCorridor`, `FrontierFindsFrontierInsideEmptyCube`, `FrontierFindExplorePathMovesTowardUnknown`, `FrontierPrefersEmptyOverUnmappedPath` — unless a test can be re-pointed at `findPathTo` or `exploreReachable` while keeping its original intent, which is preferable to losing the coverage.

Keep unchanged: `FrontierHasUnmappedInSphereWhenUnknownExists`, `FrontierHasNoUnmappedInSphereWhenFullyKnown`, `FrontierHasUnmappedFaceNeighbourOnCm10Grid`, `FrontierDetectsUnmappedCellsInBounds`, `FrontierNoUnmappedWhenFullyMappedEmpty`, and the task 2 additions.

Add a bound test for the survivor:

```cpp
TEST(MappingAlgorithm, FindPathToIsExpansionBounded) {
    ct::MapConfig config = makeCm10Config();
    config.boundaries.max_x = 100000.0 * x_extent[cm];
    config.boundaries.max_y = 100000.0 * y_extent[cm];
    config.boundaries.max_height = 100000.0 * z_extent[cm];
    Map map{{10000, 10000, 10000}, config, ct::VoxelOccupancy::Unmapped};

    const detail::MappingAlgorithmFrontier frontier;
    const Position3D start{0.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]};
    const Position3D unreachable{99000.0 * x_extent[cm], 99000.0 * y_extent[cm],
                                 99000.0 * z_extent[cm]};

    // Must return, not hang. Result may be found or not; termination is the assertion.
    const detail::FrontierPathResult result =
        frontier.findPathTo(map, start, unreachable, 4.0 * cm, {});
    (void)result;
    SUCCEED();
}
```

- [x] **Step 5: Run the full suite and check timing**

Run: `ctest --test-dir build --output-on-failure`
Expected: all green, and `algorithm_test` completes without the previously slow case. Note the wall-clock time; it should drop.

- [x] **Step 6: Confirm the deletions landed**

Run: `grep -rn "findPath\b\|findExplorePath\|findFarthestPath\|findGreedyUnknownStep\|findAnyPassableNeighbor\|diagnose\|PlanningDiagnostics\|frontier_visit_counts\|explore_dist_cache\|kMaxFrontierVisits\|kNoProgressLimit\|kNoFrontierStuckLimit" Algorithm/`
Expected: no output.

- [x] **Step 7: Stage and propose the commit**

```bash
git add Algorithm/src/MappingAlgorithmFrontier.h Algorithm/src/MappingAlgorithmFrontier.cpp Algorithm/tests/test_mapping_algorithm_frontier.cpp
# propose: "refactor: retire policy-shaped frontier helpers, blacklists and stale cache"
```

---

### Task 7: Measure and document

**Files:**
- Create: `docs/benchmarks/2026-08-30-post_d_honest.csv`, `docs/benchmarks/2026-08-30-post_d_honest.md`
- Modify: `docs/mapping-algorithm-analysis.md`, `docs/HLD.md`, `docs/superpowers/specs/2026-08-29-mapping-algorithm-roadmap.md`, `docs/mapping-algorithm-rewrite-pickup.md`, `docs/known-issues.md`

- [ ] **Step 1: Run the honest column**

Follow `scripts/benchmark/README.md`. In Docker, `apt-get install -y python3-venv python3-pip` may be needed once. Expect this to take longer than post-C's 2–5 minutes per column, because missions now use their budget instead of quitting.

Expected: 24 cells, 0 errored. Save the CSV as `docs/benchmarks/2026-08-30-post_d_honest.csv`.

- [ ] **Step 2: Run the adversarial column**

Same harness, adversarial (hits-only foreign MC) column. This is the robustness check from roadmap decision 1: confirm no cell reports `ERROR`. Record the totals in the summary file; a regression here means the clearance invariant is being violated somewhere in the emitted movements.

- [ ] **Step 3: Write the summary**

Create `docs/benchmarks/2026-08-30-post_d_honest.md` in the same shape as `2026-08-29-post_c_honest.md`: the post-C → post-D deltas for score sum, total steps, `MAX_STEPS` cells and errors, then the per-group ex2 band verdicts. Explain the deltas rather than only reporting them — in particular whether `house_full` moved off its 6–13% floor and what the step-count rise bought.

- [ ] **Step 4: Update the analysis doc**

In `docs/mapping-algorithm-analysis.md`, mark resolved with a short "Resolution (project D, 2026-08-30)" note under each, following the format C used:
- "1. It never combines movement and scanning in one step"
- "3. It's blind to its own step budget"
- "A cache that goes stale exactly when it's used"
- "Dijkstra's optimality assumption is broken mid-search"
- "Two blacklists that only grow"
- "Dead code in a private helper"

Leave "Raw doubles for all geometry" open and point it at project E.

- [ ] **Step 5: Update the HLD**

In `docs/HLD.md` around line 89-91, replace "Uses an internal BFS frontier planner (`MappingAlgorithmFrontier`) to choose scan directions" with a description of the NBV policy over a frontier reachability substrate, noting that a command may carry both a movement and a scan.

- [ ] **Step 6: Update the roadmap and pickup docs**

In the roadmap's Project D section: status implemented, a measured-impact table (post-C vs post-D), a "Resolved at implement time" list, and a short "Notes for project E" section. In `docs/mapping-algorithm-rewrite-pickup.md`: mark D done with its artifacts, replace the measured-scores section's primary column with post-D, and set the queue to project E plus the PR and packaging tracks.

- [ ] **Step 7: Revisit known issue #20**

`docs/known-issues.md` #20 describes foreign-MC step inflation. Re-read it against the post-D adversarial numbers and either update the figures or note that the step profile changed and why.

- [ ] **Step 8: Stage and propose the commit**

```bash
git add docs/
# propose: "docs: record post-D benchmark and mark policy findings resolved"
```

---

## Self-Review

**Spec coverage.** Objective, candidates, gain, budget filter, co-emission, smoothing → tasks 1, 3, 4, 5. Termination and retired state → tasks 5, 6. ALG28 bound → tasks 2, 6. `UserCommon` counter → task 1. mp-units in new code → global constraints, enforced in tasks 3–5. Determinism → task 4 test. Success criteria and docs → task 7. Risks are inherited by task 7's measurement steps. Every spec section maps to a task.

**Type consistency.** `ExplorationPlan`, `NbvInputs`, `ReachableCell`, `ReachabilityResult`, `MovementLimits` are declared once and used with the same field names throughout. `exploreReachable` takes `(map, start, drone_radius, blocked, stride_cells, max_expansions)` in tasks 2 and 4 alike. `countUnresolvedVoxels(map, origin, heading, relative, cfg, seen)` matches between tasks 1, 4 and 5. `stepCostForPath(waypoints, start_position, start_heading, limits)` matches between tasks 3 and 4.

**Known deviation to watch.** `NbvPlanner::gainAt` treats the direction set as world-frame by passing a zero heading, and `MappingAlgorithmImpl` converts to the drone frame at emission. That convention has to hold in both places or scans will be aimed wrong; the co-emission test in task 5 is what catches a mismatch.
