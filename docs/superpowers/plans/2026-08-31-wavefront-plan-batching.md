# Wavefront Plan Batching Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Cut `large_out`'s per-cell wall-clock time toward the 90s ceiling without moving its score, by reusing `WavefrontPlanner`'s already-computed ranked candidates across several replans instead of throwing all but one away and recomputing the full map-wide reachability search every ~4 steps.

**Architecture:** `WavefrontPlanner::plan()` already ranks up to 8 frontier clusters and computes full waypoints/rate for each before keeping only the best — that work is currently discarded. Add an optional out-parameter so the caller can keep the runner-ups. `MappingAlgorithmImpl` gets a small FIFO of those runner-up plans and, when its current plan simply runs out (not when the periodic interval elapses or the current target dies — those still force a fresh search), pops the next still-live one instead of calling the expensive planner again.

**Tech Stack:** C++20, GoogleTest, CMake, Docker (`drone-mapper-ex3-dev` image), existing `verify-cell-runtime` skill for wall-clock measurement.

## Global Constraints

- Build and test only inside the `drone-mapper-ex3-dev` Docker image, Release tree at `build/opt` (this toolchain's CMake `Release` preset defaults to `-O3 -DNDEBUG`).
- Never invent a smaller `max_steps` or add a wall-clock abort in `Algorithm`/`MissionControl` (`AGENTS.md`).
- `common::IMappingAlgorithm`'s public interface is frozen (`.cursor/rules/frozen-interfaces.mdc`); every change in this plan stays inside `Algorithm/src/` and the private section of `Algorithm/include/Algorithm/MappingAlgorithmImpl.h`.
- No comments that narrate what code does; only comments that explain non-obvious intent (e.g. why the interval/cluster-dead triggers must bypass the queue).
- Per-cell wall-clock verification always uses the `verify-cell-runtime` skill: Release, serial (`num_threads=1`), one process per cell — never the 8-thread `-comparative` compose wall.
- Run the full `algorithm_test` suite after every task before moving to the next; a task is not done if the suite doesn't pass.
- Do not re-tighten `kMinObservedInformationRate`/`kObservedWindowSteps`/`kLowObservedWindows` (the observed-progress stall floor) to compensate if wall time is still high after this plan — that trade was already measured and rejected (it costs far more score than the wall-time it saves on these specific cells). Report honest numbers instead.

---

## File Structure

- **`Algorithm/src/WavefrontPlanner.h`** — `plan()` gains an optional `std::vector<ExplorationPlan>* alternates` out-parameter.
- **`Algorithm/src/WavefrontPlanner.cpp`** — the ranked-cluster loop stops discarding every candidate but the best; it collects all valid, budget-affordable candidates, sorts them by `expected_rate` descending, and returns the top one while optionally exposing the rest.
- **`Algorithm/include/Algorithm/MappingAlgorithmImpl.h`** — declares two new private methods: `adoptPlan` (shared "install a new plan" logic) and `popPendingPlan` (consume a queued runner-up).
- **`Algorithm/src/MappingAlgorithmImpl.cpp`** — `Impl` gains a `pending_plans` queue; `replan()` is refactored to funnel through `adoptPlan`; `nextStep`'s replan-trigger block tries `popPendingPlan` first when it's safe to do so. The temporary `ALGO_PROFILE` diagnostic block already in this file (added during investigation, guarded by an env var, currently dead weight otherwise) is used to measure the fix in Task 4, then removed in Task 7.
- **`Algorithm/tests/test_wavefront_planner.cpp`** — new tests for the `alternates` output (sorted, excludes the winner, respects the budget filter, empty when only one cluster exists).
- **`Algorithm/tests/test_mapping_algorithm.cpp`** — one new end-to-end regression test with three disjoint frontier crumbs, locking in that multi-cluster exploration still finishes and clears every crumb.

---

### Task 1: WavefrontPlanner exposes ranked runner-up candidates

**Files:**
- Modify: `Algorithm/src/WavefrontPlanner.h`
- Modify: `Algorithm/src/WavefrontPlanner.cpp`
- Test: `Algorithm/tests/test_wavefront_planner.cpp`

**Interfaces:**
- Produces: `ExplorationPlan WavefrontPlanner::plan(const WavefrontInputs& in, std::vector<ExplorationPlan>* alternates = nullptr) const` — when `alternates` is non-null, it is filled with every other valid, budget-affordable ranked candidate (excluding the returned plan), sorted by `expected_rate` descending, using the same first-wins tie-break as before (stable sort preserves `ranked`'s original relative order among equal rates).

- [ ] **Step 1: Write the failing test**

Add to `Algorithm/tests/test_wavefront_planner.cpp` (append near the end, before the closing of the file):

```cpp
TEST(WavefrontPlanner, AlternatesExcludeBestAndAreSortedByRate) {
    Map map{{21, 21, 21}, makeConfig(), ct::VoxelOccupancy::Empty};
    // Near, tiny crumb: 1 cell just +Y of start — cheapest to reach, lowest cell_count.
    map.set(at(50.0, 60.0, 0.0), ct::VoxelOccupancy::Unmapped);
    // Near, medium crumb on the -Y side — a second distinct cluster.
    map.set(at(50.0, 40.0, 0.0), ct::VoxelOccupancy::Unmapped);
    map.set(at(40.0, 40.0, 0.0), ct::VoxelOccupancy::Unmapped);
    // Far, big room — more cells but much longer travel.
    fillUnmappedBox(map, 14, 20, 6, 13, 0, 5);

    const detail::WavefrontPlanner planner;
    const ct::DroneState state = stateAt(at(50.0, 50.0, 0.0));
    const detail::BlockedCells blocked;
    std::vector<detail::ExplorationPlan> alternates;
    const detail::ExplorationPlan best = planner.plan(
        {map, state, makeLidar(), makeDrone(), 1000, blocked, false}, &alternates);

    ASSERT_TRUE(best.valid);
    ASSERT_EQ(alternates.size(), 2u);
    for (const detail::ExplorationPlan& alt : alternates) {
        EXPECT_LE(alt.expected_rate, best.expected_rate);
    }
    EXPECT_GE(alternates[0].expected_rate, alternates[1].expected_rate);
    for (const detail::ExplorationPlan& alt : alternates) {
        EXPECT_NE(alt.target_keys, best.target_keys);
    }
}
```

- [ ] **Step 2: Run test to verify it fails**

Run (from the repo root, PowerShell):

```
docker run --rm -e VCPKG_ROOT=/usr/local/vcpkg -v "c:/Users/sagi1/Projects/DroneMapper/Drone-Mapper-ex3:/work" -w /work drone-mapper-ex3-dev bash -lc 'cmake --build build/opt -j$(nproc) --target algorithm_test'
```

Expected: FAIL to compile — `plan()` does not accept a second argument yet.

- [ ] **Step 3: Update the header**

Replace in `Algorithm/src/WavefrontPlanner.h`:

```cpp
class WavefrontPlanner {
public:
    [[nodiscard]] ExplorationPlan plan(const WavefrontInputs& in) const;

private:
    static constexpr std::size_t kRankedClusters = 8;

    MappingAlgorithmFrontier frontier_{};
};
```

with:

```cpp
class WavefrontPlanner {
public:
    /// Ranks candidate frontier clusters and returns the best. When `alternates` is
    /// non-null, it is filled with the remaining ranked, budget-affordable candidates
    /// (best-to-worst, excluding the returned plan) so a caller can reuse the same
    /// expensive reachability search for several replans instead of one.
    [[nodiscard]] ExplorationPlan plan(const WavefrontInputs& in,
                                       std::vector<ExplorationPlan>* alternates = nullptr) const;

private:
    static constexpr std::size_t kRankedClusters = 8;

    MappingAlgorithmFrontier frontier_{};
};
```

- [ ] **Step 4: Rewrite the ranked-loop tail in the .cpp**

In `Algorithm/src/WavefrontPlanner.cpp`, everything up to and including the `if (ranked.size() > kRankedClusters) { ranked.resize(kRankedClusters); }` line is unchanged. Replace the function signature and everything from `ExplorationPlan best;` to the final `return best; }`:

Old (to remove):

```cpp
ExplorationPlan WavefrontPlanner::plan(const WavefrontInputs& in) const {
```

New (signature only changes):

```cpp
ExplorationPlan WavefrontPlanner::plan(const WavefrontInputs& in,
                                       std::vector<ExplorationPlan>* alternates) const {
```

Old tail (to remove — from just after the `ranked.resize` line to the end of the function):

```cpp
    ExplorationPlan best;
    double best_rate = -1.0;
    for (const FrontierCluster* cluster : ranked) {
        GridKey approach = cluster->approach_key;
        if (open_volume && approach == reach.start_key && cluster->keys.size() > 1) {
            int best_d = -1;
            for (const GridKey& key : cluster->keys) {
                const int d = std::abs(key.qx - reach.start_key.qx) +
                              std::abs(key.qy - reach.start_key.qy) +
                              std::abs(key.qz - reach.start_key.qz);
                if (d > best_d) {
                    best_d = d;
                    approach = key;
                }
            }
        }
        FrontierPathResult raw = reconstructPathTo(
            reach.parent_of, reach.start_key, approach, config);
        std::vector<common::Position3D> waypoints;
        if (approach == reach.start_key) {
            waypoints.clear();
            const GridKey down{reach.start_key.qx, reach.start_key.qy,
                               reach.start_key.qz - 1};
            const double z = in.state.position.z.force_numerical_value_in(cm);
            const double max_z = config.boundaries.max_height.force_numerical_value_in(cm);
            const double z_min = in.lidar.z_min.force_numerical_value_in(cm);
            const double step = config.resolution.force_numerical_value_in(cm);
            const bool column_unmapped = unmappedInColumnBelow(
                in.map, config, in.state.position, reach.parent_of, reach.start_key);
            const bool near_ceiling = max_z - z <= z_min + 1e-6;
            const bool house_layer_done =
                house_volume && !hasHorizontalUnmapped(in.map, in.state.position, step);
            if ((near_ceiling || house_layer_done) && column_unmapped &&
                reach.parent_of.contains(down)) {
                const FrontierPathResult drop =
                    reconstructPathTo(reach.parent_of, reach.start_key, down, config);
                if (drop.found) {
                    waypoints = stringPullConstantAltitude(
                        in.map, drop.path, in.drone.radius);
                }
            }
        } else {
            if (!raw.found) {
                continue;
            }
            waypoints = stringPullConstantAltitude(in.map, raw.path, in.drone.radius);
        }
        const std::size_t travel =
            waypoints.empty()
                ? 0
                : stepCostForPath(waypoints, in.state.position, in.state.heading, limits);
        if (travel + reserve > in.remaining_steps) {
            continue;
        }
        const double rate = static_cast<double>(cluster_score(cluster)) /
                            static_cast<double>(travel + reserve);
        if (rate > best_rate) {
            best_rate = rate;
            best.valid = true;
            best.waypoints = std::move(waypoints);
            best.target_cluster_cells = cluster_score(cluster);
            best.expected_rate = rate;
            best.target_keys = cluster->keys;
            best.frontier_cells = reach.frontier_cells;
        }
    }
    return best;
}
```

New tail (to insert in its place):

```cpp
    std::vector<ExplorationPlan> candidates;
    candidates.reserve(ranked.size());
    for (const FrontierCluster* cluster : ranked) {
        GridKey approach = cluster->approach_key;
        if (open_volume && approach == reach.start_key && cluster->keys.size() > 1) {
            int best_d = -1;
            for (const GridKey& key : cluster->keys) {
                const int d = std::abs(key.qx - reach.start_key.qx) +
                              std::abs(key.qy - reach.start_key.qy) +
                              std::abs(key.qz - reach.start_key.qz);
                if (d > best_d) {
                    best_d = d;
                    approach = key;
                }
            }
        }
        FrontierPathResult raw = reconstructPathTo(
            reach.parent_of, reach.start_key, approach, config);
        std::vector<common::Position3D> waypoints;
        if (approach == reach.start_key) {
            waypoints.clear();
            const GridKey down{reach.start_key.qx, reach.start_key.qy,
                               reach.start_key.qz - 1};
            const double z = in.state.position.z.force_numerical_value_in(cm);
            const double max_z = config.boundaries.max_height.force_numerical_value_in(cm);
            const double z_min = in.lidar.z_min.force_numerical_value_in(cm);
            const double step = config.resolution.force_numerical_value_in(cm);
            const bool column_unmapped = unmappedInColumnBelow(
                in.map, config, in.state.position, reach.parent_of, reach.start_key);
            const bool near_ceiling = max_z - z <= z_min + 1e-6;
            const bool house_layer_done =
                house_volume && !hasHorizontalUnmapped(in.map, in.state.position, step);
            if ((near_ceiling || house_layer_done) && column_unmapped &&
                reach.parent_of.contains(down)) {
                const FrontierPathResult drop =
                    reconstructPathTo(reach.parent_of, reach.start_key, down, config);
                if (drop.found) {
                    waypoints = stringPullConstantAltitude(
                        in.map, drop.path, in.drone.radius);
                }
            }
        } else {
            if (!raw.found) {
                continue;
            }
            waypoints = stringPullConstantAltitude(in.map, raw.path, in.drone.radius);
        }
        const std::size_t travel =
            waypoints.empty()
                ? 0
                : stepCostForPath(waypoints, in.state.position, in.state.heading, limits);
        if (travel + reserve > in.remaining_steps) {
            continue;
        }
        const double rate = static_cast<double>(cluster_score(cluster)) /
                            static_cast<double>(travel + reserve);
        ExplorationPlan candidate;
        candidate.valid = true;
        candidate.waypoints = std::move(waypoints);
        candidate.target_cluster_cells = cluster_score(cluster);
        candidate.expected_rate = rate;
        candidate.target_keys = cluster->keys;
        candidate.frontier_cells = reach.frontier_cells;
        candidates.push_back(std::move(candidate));
    }
    if (candidates.empty()) {
        return {};
    }
    // Stable sort: ties keep `ranked`'s original order, matching the old
    // strict `rate > best_rate` first-wins tie-break exactly.
    std::stable_sort(candidates.begin(), candidates.end(),
                     [](const ExplorationPlan& a, const ExplorationPlan& b) {
                         return a.expected_rate > b.expected_rate;
                     });
    if (alternates != nullptr) {
        alternates->assign(candidates.begin() + 1, candidates.end());
    }
    return std::move(candidates.front());
}
```

- [ ] **Step 5: Run the new test and the full WavefrontPlanner suite**

Run:

```
docker run --rm -e VCPKG_ROOT=/usr/local/vcpkg -v "c:/Users/sagi1/Projects/DroneMapper/Drone-Mapper-ex3:/work" -w /work drone-mapper-ex3-dev bash -lc 'cmake --build build/opt -j$(nproc) --target algorithm_test && ./build/opt/Algorithm/algorithm_test --gtest_filter=WavefrontPlanner.*'
```

Expected: PASS, all tests (the 11 pre-existing plus the new one) — confirming the refactor picks the exact same `best` as before.

- [ ] **Step 6: Add two edge-case tests**

Append to `Algorithm/tests/test_wavefront_planner.cpp`:

```cpp
TEST(WavefrontPlanner, AlternatesEmptyWithOnlyOneCluster) {
    Map map{{21, 21, 21}, makeConfig(), ct::VoxelOccupancy::Empty};
    fillUnmappedBox(map, 16, 20, 8, 12, 0, 2);

    const detail::WavefrontPlanner planner;
    const ct::DroneState state = stateAt(at(50.0, 100.0, 0.0));
    const detail::BlockedCells blocked;
    std::vector<detail::ExplorationPlan> alternates;
    const detail::ExplorationPlan best = planner.plan(
        {map, state, makeLidar(), makeDrone(), 1000, blocked, false}, &alternates);

    ASSERT_TRUE(best.valid);
    EXPECT_TRUE(alternates.empty());
}

TEST(WavefrontPlanner, AlternatesOmitClustersTheRemainingBudgetCannotAfford) {
    Map map{{21, 21, 21}, makeConfig(), ct::VoxelOccupancy::Empty};
    fillUnmappedBox(map, 18, 20, 8, 12, 0, 2);
    map.set(at(50.0, 110.0, 0.0), ct::VoxelOccupancy::Unmapped);

    const detail::WavefrontPlanner planner;
    const ct::DroneState state = stateAt(at(0.0, 100.0, 0.0));
    const detail::BlockedCells blocked;
    std::vector<detail::ExplorationPlan> alternates;
    const detail::ExplorationPlan best = planner.plan(
        {map, state, makeLidar(), makeDrone(), 2, blocked, false}, &alternates);

    // Budget is 2 steps: any candidate kept (best or alternate) must be free to reach.
    if (best.valid) {
        EXPECT_TRUE(best.waypoints.empty());
    }
    EXPECT_TRUE(alternates.empty());
}
```

- [ ] **Step 7: Run the full WavefrontPlanner suite again**

Run:

```
docker run --rm -e VCPKG_ROOT=/usr/local/vcpkg -v "c:/Users/sagi1/Projects/DroneMapper/Drone-Mapper-ex3:/work" -w /work drone-mapper-ex3-dev bash -lc 'cmake --build build/opt -j$(nproc) --target algorithm_test && ./build/opt/Algorithm/algorithm_test --gtest_filter=WavefrontPlanner.*'
```

Expected: PASS, 13 tests total.

- [ ] **Step 8: Commit**

```bash
git add Algorithm/src/WavefrontPlanner.h Algorithm/src/WavefrontPlanner.cpp Algorithm/tests/test_wavefront_planner.cpp
git commit -m "feat: WavefrontPlanner exposes ranked runner-up candidates"
```

---

### Task 2: Factor out adoptPlan() in MappingAlgorithmImpl (pure refactor, no behavior change)

**Files:**
- Modify: `Algorithm/include/Algorithm/MappingAlgorithmImpl.h`
- Modify: `Algorithm/src/MappingAlgorithmImpl.cpp`

**Interfaces:**
- Consumes: `detail::ExplorationPlan` (Task 1, shape unchanged).
- Produces: `void MappingAlgorithmImpl_207190406_209543255::adoptPlan(detail::ExplorationPlan plan, const common::types::DroneState& state)` — installs a new plan (resets waypoint/arrival-scan/replan-interval bookkeeping, builds the arrival sweep if the plan has no waypoints). Task 3's `popPendingPlan` also calls this.

- [ ] **Step 1: Capture the pre-refactor baseline**

Run:

```
docker run --rm -e VCPKG_ROOT=/usr/local/vcpkg -v "c:/Users/sagi1/Projects/DroneMapper/Drone-Mapper-ex3:/work" -w /work drone-mapper-ex3-dev bash -lc 'cmake --build build/opt -j$(nproc) --target algorithm_test && ./build/opt/Algorithm/algorithm_test'
```

Expected: PASS. Note the total test count printed at the end (e.g. `[PASSED] 87 tests`) — Task 2 Step 4 must match this count exactly.

- [ ] **Step 2: Declare the new method**

In `Algorithm/include/Algorithm/MappingAlgorithmImpl.h`, add a forward declaration right after the opening namespace brace:

```cpp
namespace algorithm_207190406_209543255 {

namespace detail {
struct ExplorationPlan;
} // namespace detail

```

Then, in the `private:` section, add `adoptPlan` right after the `replan` declaration:

```cpp
    [[nodiscard]] bool replan(const common::types::DroneState& state, bool ignore_blocked);
    void adoptPlan(detail::ExplorationPlan plan, const common::types::DroneState& state);
```

- [ ] **Step 3: Extract adoptPlan in the .cpp**

In `Algorithm/src/MappingAlgorithmImpl.cpp`, replace:

```cpp
bool MappingAlgorithmImpl_207190406_209543255::replan(const types::DroneState& state,
                                                      bool ignore_blocked) {
    const bool prev_stay = impl_->has_plan && impl_->plan.waypoints.empty();
    const detail::WavefrontInputs inputs{
        output_map_, state, lidar_config_, drone_config_,
        remainingSteps(state), impl_->blocked_cells, ignore_blocked, prev_stay,
    };
    impl_->plan = impl_->planner.plan(inputs);
    impl_->waypoint_index = 0;
    impl_->arrival_scans.clear();
    impl_->arrival_scan_index = 0;
    impl_->steps_since_replan = 0;
    impl_->has_plan = impl_->plan.valid;
    impl_->last_frontier = impl_->plan.frontier_cells;
    if (impl_->has_plan && impl_->plan.waypoints.empty()) {
        buildArrivalSweep(state);
        if (impl_->arrival_scans.empty()) {
            impl_->plan.expected_rate = 0.0;
        }
    }
    return impl_->has_plan;
}
```

with:

```cpp
bool MappingAlgorithmImpl_207190406_209543255::replan(const types::DroneState& state,
                                                      bool ignore_blocked) {
    const bool prev_stay = impl_->has_plan && impl_->plan.waypoints.empty();
    const detail::WavefrontInputs inputs{
        output_map_, state, lidar_config_, drone_config_,
        remainingSteps(state), impl_->blocked_cells, ignore_blocked, prev_stay,
    };
    adoptPlan(impl_->planner.plan(inputs), state);
    return impl_->has_plan;
}

void MappingAlgorithmImpl_207190406_209543255::adoptPlan(detail::ExplorationPlan plan,
                                                         const types::DroneState& state) {
    impl_->plan = std::move(plan);
    impl_->waypoint_index = 0;
    impl_->arrival_scans.clear();
    impl_->arrival_scan_index = 0;
    impl_->steps_since_replan = 0;
    impl_->has_plan = impl_->plan.valid;
    impl_->last_frontier = impl_->plan.frontier_cells;
    if (impl_->has_plan && impl_->plan.waypoints.empty()) {
        buildArrivalSweep(state);
        if (impl_->arrival_scans.empty()) {
            impl_->plan.expected_rate = 0.0;
        }
    }
}
```

- [ ] **Step 4: Rebuild and confirm zero behavior change**

Run:

```
docker run --rm -e VCPKG_ROOT=/usr/local/vcpkg -v "c:/Users/sagi1/Projects/DroneMapper/Drone-Mapper-ex3:/work" -w /work drone-mapper-ex3-dev bash -lc 'cmake --build build/opt -j$(nproc) --target algorithm_test && ./build/opt/Algorithm/algorithm_test'
```

Expected: PASS, exact same test count as Step 1's baseline.

- [ ] **Step 5: Commit**

```bash
git add Algorithm/include/Algorithm/MappingAlgorithmImpl.h Algorithm/src/MappingAlgorithmImpl.cpp
git commit -m "refactor: factor adoptPlan() out of MappingAlgorithmImpl::replan"
```

---

### Task 3: Reuse ranked alternates instead of recomputing on every plan-exhausted replan

**Files:**
- Modify: `Algorithm/include/Algorithm/MappingAlgorithmImpl.h`
- Modify: `Algorithm/src/MappingAlgorithmImpl.cpp`
- Test: `Algorithm/tests/test_mapping_algorithm.cpp`

**Interfaces:**
- Consumes: `adoptPlan` (Task 2), `WavefrontPlanner::plan(..., alternates)` (Task 1).
- Produces: `bool MappingAlgorithmImpl_207190406_209543255::popPendingPlan(const common::types::DroneState& state)` — tries to adopt the next still-live queued candidate; returns whether one was adopted.

- [ ] **Step 1: Write the regression test (locks in behavior BEFORE the queue exists)**

Before pasting, open `Algorithm/tests/test_mapping_algorithm.cpp` and confirm the exact names/signatures of `makeCorridorConfig`, `fillEmptyBox`, `gridPoint`, `makeMissionConfig`, `makeLidarConfig`, `makeDroneConfig`, and the `Impl` alias — they are already used by the neighboring tests `FinishesWhenUnmappedCountDoesNotDropAcrossReplans` and `KeepsWorkingWhenUnmappedCountKeepsDropping`; reuse them exactly as-is, do not guess.

Append:

```cpp
TEST(MappingAlgorithm, VisitsMultipleDisjointCrumbsAndFinishes) {
    const ct::MapConfig config = makeCorridorConfig();
    Map output_map{{11, 11, 11}, config};
    fillEmptyBox(output_map, 0, 10, 0, 10, 0, 10, config);
    // Three small, separated Unmapped crumbs so the first replan ranks >1 cluster.
    output_map.set(gridPoint(2, 2, 5, config), ct::VoxelOccupancy::Unmapped);
    output_map.set(gridPoint(8, 2, 5, config), ct::VoxelOccupancy::Unmapped);
    output_map.set(gridPoint(5, 8, 5, config), ct::VoxelOccupancy::Unmapped);

    const auto mc = makeMissionConfig();
    const auto lc = makeLidarConfig();
    const auto dc = makeDroneConfig();
    Impl algorithm{common::MappingAlgorithmDependencies{mc, lc, dc, output_map}};

    ct::AlgorithmStatus last = ct::AlgorithmStatus::Working;
    for (int step = 0; step < 400 && last == ct::AlgorithmStatus::Working; ++step) {
        const ct::DroneState state{gridPoint(5, 5, 5, config),
                                   Orientation{0.0 * deg, 0.0 * deg},
                                   static_cast<std::size_t>(step)};
        last = algorithm.nextStep(state, nullptr).status;
    }
    EXPECT_NE(last, ct::AlgorithmStatus::Working) << "did not finish within budget";
    EXPECT_EQ(output_map.atVoxel(gridPoint(2, 2, 5, config)), ct::VoxelOccupancy::Empty);
    EXPECT_EQ(output_map.atVoxel(gridPoint(8, 2, 5, config)), ct::VoxelOccupancy::Empty);
    EXPECT_EQ(output_map.atVoxel(gridPoint(5, 8, 5, config)), ct::VoxelOccupancy::Empty);
}
```

- [ ] **Step 2: Run it — should already pass (this is the pre-queue baseline)**

Run:

```
docker run --rm -e VCPKG_ROOT=/usr/local/vcpkg -v "c:/Users/sagi1/Projects/DroneMapper/Drone-Mapper-ex3:/work" -w /work drone-mapper-ex3-dev bash -lc 'cmake --build build/opt -j$(nproc) --target algorithm_test && ./build/opt/Algorithm/algorithm_test --gtest_filter=MappingAlgorithm.VisitsMultipleDisjointCrumbsAndFinishes'
```

Expected: PASS. This test doesn't require the queue to exist yet — it documents correct outcome so Task 3's wiring is checked against it afterward, not just against feeling.

- [ ] **Step 3: Add the pending-plans queue to Impl**

In `Algorithm/src/MappingAlgorithmImpl.cpp`, in the `Impl` struct, replace:

```cpp
    std::size_t unmapped_at_progress_mark = 0;
    std::size_t progress_window_steps = 0;
    int low_observed_windows = 0;
    bool has_progress_baseline = false;
    bool finished = false;
    bool planning_initialized = false;
};
```

with:

```cpp
    std::size_t unmapped_at_progress_mark = 0;
    std::size_t progress_window_steps = 0;
    int low_observed_windows = 0;
    bool has_progress_baseline = false;
    std::vector<detail::ExplorationPlan> pending_plans{};
    bool finished = false;
    bool planning_initialized = false;
};
```

- [ ] **Step 4: Declare popPendingPlan**

In `Algorithm/include/Algorithm/MappingAlgorithmImpl.h`, add right after `adoptPlan`:

```cpp
    void adoptPlan(detail::ExplorationPlan plan, const common::types::DroneState& state);
    [[nodiscard]] bool popPendingPlan(const common::types::DroneState& state);
```

- [ ] **Step 5: Implement popPendingPlan and wire pending_plans into replan()**

In `Algorithm/src/MappingAlgorithmImpl.cpp`, replace `replan()`:

```cpp
bool MappingAlgorithmImpl_207190406_209543255::replan(const types::DroneState& state,
                                                      bool ignore_blocked) {
    const bool prev_stay = impl_->has_plan && impl_->plan.waypoints.empty();
    const detail::WavefrontInputs inputs{
        output_map_, state, lidar_config_, drone_config_,
        remainingSteps(state), impl_->blocked_cells, ignore_blocked, prev_stay,
    };
    adoptPlan(impl_->planner.plan(inputs), state);
    return impl_->has_plan;
}
```

with:

```cpp
bool MappingAlgorithmImpl_207190406_209543255::replan(const types::DroneState& state,
                                                      bool ignore_blocked) {
    const bool prev_stay = impl_->has_plan && impl_->plan.waypoints.empty();
    const detail::WavefrontInputs inputs{
        output_map_, state, lidar_config_, drone_config_,
        remainingSteps(state), impl_->blocked_cells, ignore_blocked, prev_stay,
    };
    impl_->pending_plans.clear();
    adoptPlan(impl_->planner.plan(inputs, &impl_->pending_plans), state);
    return impl_->has_plan;
}

bool MappingAlgorithmImpl_207190406_209543255::popPendingPlan(const types::DroneState& state) {
    while (!impl_->pending_plans.empty()) {
        detail::ExplorationPlan candidate = std::move(impl_->pending_plans.front());
        impl_->pending_plans.erase(impl_->pending_plans.begin());
        if (!candidate.target_keys.empty() &&
            !detail::clusterStillFrontier(output_map_, candidate.target_keys)) {
            continue;
        }
        adoptPlan(std::move(candidate), state);
        return true;
    }
    return false;
}
```

- [ ] **Step 6: Wire popPendingPlan into nextStep's replan trigger**

In `Algorithm/src/MappingAlgorithmImpl.cpp`, inside `nextStep`, replace:

```cpp
    if (plan_exhausted || interval_elapsed || cluster_dead) {
        ProfileScope prof(&g_profile.replan_ms);
        ++g_profile.replans;
        if (plan_exhausted) ++g_profile.replans_plan_exhausted;
        if (interval_elapsed) ++g_profile.replans_interval_elapsed;
        if (cluster_dead) ++g_profile.replans_cluster_dead;
        const bool have = replan(state, false);
```

with:

```cpp
    if (plan_exhausted || interval_elapsed || cluster_dead) {
        ProfileScope prof(&g_profile.replan_ms);
        ++g_profile.replans;
        if (plan_exhausted) ++g_profile.replans_plan_exhausted;
        if (interval_elapsed) ++g_profile.replans_interval_elapsed;
        if (cluster_dead) ++g_profile.replans_cluster_dead;
        // Only a plan finishing on its own is safe to serve from the queue: an
        // elapsed interval or a dead target cluster means the map or the current
        // target's assumptions may be stale, so those always force a fresh search.
        const bool can_reuse_queue = plan_exhausted && !interval_elapsed && !cluster_dead;
        const bool have = (can_reuse_queue && popPendingPlan(state)) || replan(state, false);
```

Leave every line below this (the `low` bookkeeping, the recovery-attempt branch, the low-rate termination check) untouched — it already operates uniformly on `impl_->plan` regardless of how it was populated. Note the short-circuit `||`: `replan` only runs when `popPendingPlan` doesn't (queue empty, or every queued candidate's cluster went stale) — and `replan` unconditionally rebuilds `pending_plans` from a fresh search, so there's no special-casing needed for the "queue was empty" case.

- [ ] **Step 7: Rebuild and run the full suite**

Run:

```
docker run --rm -e VCPKG_ROOT=/usr/local/vcpkg -v "c:/Users/sagi1/Projects/DroneMapper/Drone-Mapper-ex3:/work" -w /work drone-mapper-ex3-dev bash -lc 'cmake --build build/opt -j$(nproc) --target algorithm_test && ./build/opt/Algorithm/algorithm_test'
```

Expected: PASS — every test from Task 1 and Task 2's baseline, plus `VisitsMultipleDisjointCrumbsAndFinishes`.

- [ ] **Step 8: Commit**

```bash
git add Algorithm/include/Algorithm/MappingAlgorithmImpl.h Algorithm/src/MappingAlgorithmImpl.cpp Algorithm/tests/test_mapping_algorithm.cpp
git commit -m "perf: reuse WavefrontPlanner's ranked alternates instead of replanning from scratch on every exhausted plan"
```

---

### Task 4: Measure the replan-count and wall-time drop

**Files:** none modified — measurement only, using the `ALGO_PROFILE=1` instrumentation already present in `Algorithm/src/MappingAlgorithmImpl.cpp` (added during investigation, guarded by an env var; removed in Task 7).

- [ ] **Step 1: Rebuild the Algorithm plugin**

Run:

```
docker run --rm -e VCPKG_ROOT=/usr/local/vcpkg -v "c:/Users/sagi1/Projects/DroneMapper/Drone-Mapper-ex3:/work" -w /work drone-mapper-ex3-dev bash -lc 'cmake --build build/opt -j$(nproc) --target Algorithm_207190406_209543255'
```

Expected: link succeeds.

- [ ] **Step 2: Run the profiling harness against large_out small+short**

The composition file `inputs/profile_cell.yaml` already exists (single cell: `large_simulation_out` + `large_mission_out` + `drone_small` + `lidar_short`).

Run:

```
docker run --rm -e ALGO_PROFILE=1 -e VCPKG_ROOT=/usr/local/vcpkg -v "c:/Users/sagi1/Projects/DroneMapper/Drone-Mapper-ex3:/work" -w /work drone-mapper-ex3-dev bash -lc 'mkdir -p tmp/profile_run; rm -rf tmp/profile_run/*; cp build/opt/MissionControl/MissionControl_207190406_209543255.so tmp/profile_run/; build/opt/Simulator/simulator_207190406_209543255 -comparative simulation=inputs/profile_cell.yaml mission_control_folder=tmp/profile_run algorithm=build/opt/Algorithm/Algorithm_207190406_209543255.so num_threads=1'
```

Expected stderr line: `[ALGO_PROFILE] steps=9300 replans=N ... replan_ms=M ...`. Compare `N`/`M` against the pre-change baseline already recorded: `replans=2467`, `replan_ms=130060.2` (total wall ≈133s). A working fix should drop `N` roughly by the typical number of live candidates queued per full search (bounded by `kRankedClusters - 1 = 7`, but likely lower in practice since not every ranked cluster stays a live frontier by the time it's popped).

- [ ] **Step 3: Diagnose if the drop is not meaningful**

If `N`/`M` barely move, do not proceed to Task 5 yet. Likely causes to check, in order:
1. Most popped candidates are being rejected as stale (`clusterStillFrontier` returns false almost every time) — add a temporary counter pair `pending_plans_popped_live` / `pending_plans_popped_stale` to `ProfileTotals` and re-run to see the ratio.
2. `plan_exhausted` is being hit before the queue's candidates are even reached because `interval_elapsed` or `cluster_dead` fire first on most cycles — check `replans_interval_elapsed` / `replans_cluster_dead` in the same printout; they were 14 and 1 respectively pre-change, so this is unlikely to have changed, but confirm.

---

### Task 5: Re-time the three over-budget large_out cells and confirm score parity

**Files:** none modified — measurement only, via the `verify-cell-runtime` skill.

- [ ] **Step 1: Run the large_out group timing**

Run:

```
docker run --rm -e PYTHONUNBUFFERED=1 -e VCPKG_ROOT=/usr/local/vcpkg -v "c:/Users/sagi1/Projects/DroneMapper/Drone-Mapper-ex3:/work" -w /work drone-mapper-ex3-dev bash -lc 'python3 .cursor/skills/verify-cell-runtime/scripts/time_each_cell.py --build-dir /work/build/opt --only-group large_out --hang-timeout 200'
```

- [ ] **Step 2: Compare against the recorded reference numbers**

| Cell | Baseline (no stall mechanism) score / wall | Current (window=100/4 windows/floor=0.05, pre-Task-3) score / wall | Target after Task 3 |
|------|---------------------------------------------|----------------------------------------------------------------------|----------------------|
| small+long | 51.28 / 64.3s | 51.28 / 70.9s | already fine, must not regress |
| small+short | 63.59 / 138.4s | 63.59 / 139.5s | ≤90s, score kept near 63.59 |
| large+long | 73.56 / 163.5s | 73.56 / 92.4s | ≤90s, score kept near 73.56 |
| large+short | 85.58 / 149.9s | 85.58 / 158.3s | ≤90s, score kept near 85.58 |

- [ ] **Step 3: If any cell is still over 90s, report it honestly**

Do not compensate by re-tightening the observed-progress stall floor (see Global Constraints). If Task 3 alone isn't enough, that is a legitimate outcome to report — the next lever would be reducing the per-replan Dijkstra's constant factor (flat-array reachability bookkeeping instead of `unordered_map`), which is a separate, larger follow-up plan, not a quick tweak here.

---

### Task 6: Sanity re-time house_full / small_out / small_room to confirm no regression

**Files:** none modified — measurement only.

- [ ] **Step 1: Run the sanity set**

`tmp/cells-probe.txt` already exists (8 cells: house_full ×2, large_out ×4, small_out ×1, small_room ×1).

Run:

```
docker run --rm -e PYTHONUNBUFFERED=1 -e VCPKG_ROOT=/usr/local/vcpkg -v "c:/Users/sagi1/Projects/DroneMapper/Drone-Mapper-ex3:/work" -w /work drone-mapper-ex3-dev bash -lc 'python3 .cursor/skills/verify-cell-runtime/scripts/time_each_cell.py --build-dir /work/build/opt --only-cell-file /work/tmp/cells-probe.txt --hang-timeout 200'
```

Expected: `house_full` both cells stay at score 37.19 / 48.33, wall ~11-13s (the window=100 win already landed, unrelated to this plan, must not regress); `small_out` stays at 90.89; `small_room` stays at 85.35.

- [ ] **Step 2: Full suite gate**

Run:

```
docker run --rm -e VCPKG_ROOT=/usr/local/vcpkg -v "c:/Users/sagi1/Projects/DroneMapper/Drone-Mapper-ex3:/work" -w /work drone-mapper-ex3-dev bash -lc './build/opt/Algorithm/algorithm_test'
```

Expected: PASS.

---

### Task 7: Strip the temporary ALGO_PROFILE instrumentation

**Files:**
- Modify: `Algorithm/src/MappingAlgorithmImpl.cpp`

- [ ] **Step 1: Remove the ProfileScope/ProfileTotals/g_profile block**

Delete the section starting at the comment `// TEMP PROFILING (ALGO_PROFILE=1): remove before submit.` through the `ProfileTotals g_profile;` line, inside the anonymous namespace near the top of the file.

- [ ] **Step 2: Remove every ProfileScope/g_profile call site in nextStep**

Remove:
- The `{ ProfileScope prof(&g_profile.target_alive_ms); cluster_dead = ...; }` block — replace with the plain statement `const bool cluster_dead = impl_->has_plan && !targetClusterAlive();`.
- `ProfileScope prof(&g_profile.replan_ms);` and the four `++g_profile...` counter lines right after entering the `if (plan_exhausted || interval_elapsed || cluster_dead)` block.
- `++g_profile.steps;` after `++impl_->steps_since_replan;`.
- `ProfileScope progress_prof(&g_profile.progress_count_ms);` before the progress-window block.
- `ProfileScope prof(&g_profile.travel_scan_ms);` before the `bestTravelScan` call.
- `ProfileScope prof(&g_profile.arrival_sweep_ms);` before the `buildArrivalSweep` call in the arrival-scan branch.

- [ ] **Step 3: Clean up now-unused includes**

Check whether `<chrono>`, `<cstdio>`, `<cstdlib>` are still used anywhere else in the file (they were added solely for `ProfileScope`/`ProfileTotals`). If not, remove those three `#include` lines.

- [ ] **Step 4: Rebuild and run the full suite**

Run:

```
docker run --rm -e VCPKG_ROOT=/usr/local/vcpkg -v "c:/Users/sagi1/Projects/DroneMapper/Drone-Mapper-ex3:/work" -w /work drone-mapper-ex3-dev bash -lc 'cmake --build build/opt -j$(nproc) --target algorithm_test && ./build/opt/Algorithm/algorithm_test'
```

Expected: PASS, same test count as Task 6.

- [ ] **Step 5: Re-confirm Task 5/6 numbers on the clean build**

Re-run Task 5 Step 1 and Task 6 Step 1's commands once more against this build to confirm the instrumentation removal didn't change timing meaningfully (it should have been near-free either way).

- [ ] **Step 6: Commit**

```bash
git add Algorithm/src/MappingAlgorithmImpl.cpp
git commit -m "chore: remove temporary ALGO_PROFILE diagnostic instrumentation"
```

---

### Task 8 (independent side task): Time the post-C commit for the roadmap doc

**Cancelled 2026-08-31:** user does not need post-C wall timings. After Tasks 6 and 7, run the full 24-cell `verify-cell-runtime` column instead (see Task 9).

### Task 9: Full 24-cell wall and score column

**Files:** none modified — measurement only, via `verify-cell-runtime`.

Run after Task 7 (profiling stripped, Algorithm plugin rebuilt).

```
docker run --rm -e PYTHONUNBUFFERED=1 -e VCPKG_ROOT=/usr/local/vcpkg -v "c:/Users/sagi1/Projects/DroneMapper/Drone-Mapper-ex3:/work" -w /work drone-mapper-ex3-dev bash -lc 'python3 .cursor/skills/verify-cell-runtime/scripts/time_each_cell.py --build-dir /work/build/opt --hang-timeout 200'
```

Report every cell's score, steps, status, and wall_s. Do not use 8-thread compose wall.

**Files:**
- Modify: `docs/superpowers/specs/2026-08-29-mapping-algorithm-roadmap.md`

**Context:** a git worktree already exists at `../post-c-worktree` (commit `f825b40`), built in Release at `build/opt`, with `.cursor/skills/verify-cell-runtime/` copied in. This task is independent of Tasks 1-7 and can run before, after, or interleaved with them — but never concurrently with another Docker container (CPU contention already skewed one measurement this session).

- [ ] **Step 1: Run the full 24-cell timing**

Run:

```
docker run --rm -e PYTHONUNBUFFERED=1 -e VCPKG_ROOT=/usr/local/vcpkg -v "c:/Users/sagi1/Projects/DroneMapper/post-c-worktree:/work" -w /work drone-mapper-ex3-dev bash -lc 'python3 .cursor/skills/verify-cell-runtime/scripts/time_each_cell.py --build-dir /work/build/opt --hang-timeout 200'
```

- [ ] **Step 2: Record the numbers in the roadmap doc**

Add wall-clock rows to Project C's "Measured impact" table in `docs/superpowers/specs/2026-08-29-mapping-algorithm-roadmap.md`:

```markdown
| Wall sum (Release, serial) | — | **<wall_sum>s** |
| Wall max (single cell) | — | **<wall_max>s** |
| Cells ≥ 60s | — | **<cells_ge_60s>** |
```

using the script's printed `wall_sum=`, `wall_max=`, `cells_ge_60s=` summary values.

- [ ] **Step 3: Remove the scratch worktree**

```bash
git worktree remove ../post-c-worktree
```

- [ ] **Step 4: Commit**

```bash
git add docs/superpowers/specs/2026-08-29-mapping-algorithm-roadmap.md
git commit -m "docs: record post-C wall-clock timing"
```

---

## Self-Review

**Spec coverage:** every element of the diagnosis (replan called ~every 4 steps because `plan_exhausted` fires almost every cycle; each replan re-runs the full map-wide Dijkstra; `WavefrontPlanner` already ranks 8 candidates and discards 7) maps directly onto Task 1 (expose the discarded work) + Task 3 (consume it instead of recomputing). Task 2 is the safety-preserving refactor that makes Task 3 a small diff. Tasks 4-6 are the acceptance criteria from the original ask ("results close to before, in ≤90s, ideally ≤60s"). Task 7 satisfies the "no debug cruft in submission" hygiene bar. Task 8 is the user's separate, explicitly requested post-C timing ask.

**Placeholder scan:** no TBD/"add error handling"/"similar to Task N" phrasing — every step has the literal code or command to run.

**Type consistency:** `ExplorationPlan`, `WavefrontInputs`, `GridKey`, `BlockedCells`, `detail::clusterStillFrontier`, `detail::ExplorationPlan` are all used with the exact names and shapes already defined in `ExplorationPlan.h` / `MappingAlgorithmFrontier.h` / `ScanPlanning.h` — no renamed types across tasks. `popPendingPlan` and `adoptPlan` signatures are declared once (Task 2/3 header edits) and used identically in every call site across `MappingAlgorithmImpl.cpp`.

**Risk callouts made explicit in the plan itself:** the `interval_elapsed`/`cluster_dead` triggers deliberately bypass the queue (Task 3 Step 6) so periodic reassessment and dead-cluster recovery are never served stale data; `popPendingPlan` re-validates liveness via the existing `clusterStillFrontier` check before adopting anything from the queue, so a candidate that got resolved by an intervening scan is silently skipped rather than mis-executed.
