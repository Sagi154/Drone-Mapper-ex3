# Mapping algorithm walkthrough

First-principles description of the current `Algorithm/` implementation in
Drone-Mapper-ex3. Ctrl+click a path to open the file (line numbers are in the
link text).

---

## The job

A drone is dropped in a 3D world it cannot see. There is a **hidden** map (the
truth, owned by the Simulator) and an **output** map that starts as all
`Unmapped`. The algorithm never writes that map. It only **looks** at it and,
each tick, says “do this movement and/or this scan.” MissionControl then moves
the drone, fires the lidar, and paints voxels on the output map. Next tick the
algorithm sees a slightly more complete map and decides again.

The published contract is one virtual method:

[`common/include/Common/IMappingAlgorithm.h`](../common/include/Common/IMappingAlgorithm.h) (24–26)

```cpp
    [[nodiscard]] virtual types::MappingStepCommand nextStep(
        const types::DroneState& state,
        const types::LidarScanResult* latest_scan) = 0;
```

It receives:

- `state` — GPS position, heading, and `step_index` (how many host steps have
  already happened)
- `latest_scan` — the previous lidar result, or `nullptr` on the first call
- `output_map_` — read-only map built so far (stored on the base class)

It returns a `MappingStepCommand`:

[`common/include/Common/types/DroneTypes.h`](../common/include/Common/types/DroneTypes.h) (30–34)

```cpp
struct MappingStepCommand {
    std::optional<MovementCommand> movement{};
    std::optional<Orientation> scan_orientation{};
    AlgorithmStatus status = AlgorithmStatus::Working;
};
```

Three pieces, all optional except status:

| Field | Meaning |
|--------|---------|
| `movement` | Hover / Rotate / Advance / Elevate — or omit |
| `scan_orientation` | Where to point the lidar this step — or omit |
| `status` | Still working, done, or done but some voxels could not be mapped |

The implementation is `MappingAlgorithmImpl_207190406_209543255`. It **ignores**
`latest_scan`. Belief is only `output_map_`. The host fuses hits; the algorithm
just reads the result.

[`Algorithm/src/MappingAlgorithmImpl.cpp`](../Algorithm/src/MappingAlgorithmImpl.cpp) (282–284)

```cpp
types::MappingStepCommand MappingAlgorithmImpl_207190406_209543255::nextStep(
    const types::DroneState& state, const types::LidarScanResult* latest_scan) {
    [[maybe_unused]] const types::LidarScanResult* unused_scan = latest_scan;
```

---

## The idea in one sentence

Find the boundary between “I know this is empty” and “I have never seen this,”
fly to the most useful patch of that boundary, scan, repeat, until going
anywhere else is not worth the remaining steps.

That boundary is a **frontier**. Patches of it are **clusters**. A chosen
cluster plus a path to it is an **ExplorationPlan**. `nextStep` does not replan
every tick. It **holds** a plan and executes it, one command at a time.

[`Algorithm/src/ExplorationPlan.h`](../Algorithm/src/ExplorationPlan.h) (15–22)

```cpp
struct ExplorationPlan {
    std::vector<common::Position3D> waypoints{};
    std::size_t target_cluster_cells = 0;
    double expected_rate = 0.0;
    std::vector<GridKey> target_keys{};
    FrontierCells frontier_cells{};
    bool valid = false;
};
```

- `waypoints` — cells to fly through (empty means “scan in place”)
- `expected_rate` — predicted new cells per remaining command
- `target_keys` — the cluster we are going after
- `valid` — whether this is a real plan

The files split by job:

| File | Job |
|------|-----|
| [`MappingAlgorithmImpl.cpp`](../Algorithm/src/MappingAlgorithmImpl.cpp) | Tick loop: stall, replan, emit one command |
| [`WavefrontPlanner.cpp`](../Algorithm/src/WavefrontPlanner.cpp) | Pick the next cluster and path |
| [`MappingAlgorithmFrontier.cpp`](../Algorithm/src/MappingAlgorithmFrontier.cpp) | “What can I reach?” and “where is the unknown?” |
| [`PathShaping.cpp`](../Algorithm/src/PathShaping.cpp) | Shorten the path; count real Rotate/Advance/Elevate steps |
| [`ScanPlanning.cpp`](../Algorithm/src/ScanPlanning.cpp) | Which way is worth pointing the lidar |

---

## One tick, in order

`nextStep` is a loop you can read top to bottom.

### 1. Already finished?

If a previous tick decided we are done, keep saying `Finished`.

[`Algorithm/src/MappingAlgorithmImpl.cpp`](../Algorithm/src/MappingAlgorithmImpl.cpp) (286–290)

```cpp
    if (impl_->finished) {
        types::MappingStepCommand cmd{};
        cmd.status = types::AlgorithmStatus::Finished;
        return cmd;
    }
```

The immediate caller is [`DroneControlImpl::step()`](../MissionControl/src/DroneControlImpl.cpp)
(around line 180). `Finished` / `FinishedWithUnmappableVoxels` becomes
`DroneStepStatus::Completed`. [`MissionControlImpl::runMission()`](../MissionControl/src/MissionControlImpl.cpp)
then ends the mission and returns to the Simulator.

### 2. Forget old blocked cells

If the drone stalled on a cell, that cell is blacklisted so the planner stops
retrying it immediately. After 50 steps it is allowed again
(`kBlockedTtlSteps`).

[`Algorithm/src/MappingAlgorithmImpl.cpp`](../Algorithm/src/MappingAlgorithmImpl.cpp) (168–177)

```cpp
void MappingAlgorithmImpl_207190406_209543255::pruneExpiredBlockedCells(std::size_t step_index) {
    for (auto it = impl_->blocked_since.begin(); it != impl_->blocked_since.end();) {
        const auto inserted = static_cast<std::size_t>(it->second);
        if (step_index >= inserted + kBlockedTtlSteps) {
            impl_->blocked_cells.erase(it->first);
            it = impl_->blocked_since.erase(it);
```

### 3. Detect stall

A **waypoint** is one 3D point on the current plan’s path
(`plan.waypoints`). The executor flies toward the current index, one command
per tick.

A **tick** is one `nextStep` call (one host step).

A **stall** means: we still have an unfinished waypoint, but GPS
`state.position` is the same as last tick (within `1e-6` cm —
[`samePosition`](../Algorithm/src/MappingAlgorithmImpl.cpp)). Heading-only
changes (rotating in place) do **not** count. After 2 such ticks
(`kMaxMovingStallTicks`), that waypoint cell is blacklisted and the plan is
dropped.

[`Algorithm/src/MappingAlgorithmImpl.cpp`](../Algorithm/src/MappingAlgorithmImpl.cpp) (295–304)

```cpp
    if (impl_->has_plan && impl_->waypoint_index < impl_->plan.waypoints.size() &&
        impl_->has_last_position && samePosition(impl_->last_position, state.position)) {
        if (++impl_->moving_stall_ticks >= kMaxMovingStallTicks) {
            const auto key = detail::quantizePosition(
                impl_->plan.waypoints[impl_->waypoint_index], output_map_.getMapConfig());
            impl_->blocked_cells.insert(key);
            impl_->blocked_since[key] = static_cast<int>(state.step_index);
            impl_->moving_stall_ticks = 0;
            impl_->has_plan = false;
```

Why? The planner treats `Unmapped` as walkable (explained below). At execution
time that cell may be a wall. Stall is how we find out.

### 4. Advance past waypoints we already reached

[`Algorithm/src/MappingAlgorithmImpl.cpp`](../Algorithm/src/MappingAlgorithmImpl.cpp) (311–314)

```cpp
    while (impl_->has_plan && impl_->waypoint_index < impl_->plan.waypoints.size() &&
           reachedWaypoint(state, impl_->plan.waypoints[impl_->waypoint_index])) {
        ++impl_->waypoint_index;
    }
```

### 5. Maybe replan

Replan if any of these is true:

- no plan, or the path **and** the arrival scans are done
- 25 steps since the last plan (`kReplanIntervalSteps`) — the map has changed
- the target cluster is no longer a frontier (it got mapped)

[`Algorithm/src/MappingAlgorithmImpl.cpp`](../Algorithm/src/MappingAlgorithmImpl.cpp) (325–336)

```cpp
    const bool plan_exhausted =
        !impl_->has_plan || (waypoints_done && scans_done);
    const bool interval_elapsed = impl_->steps_since_replan >= kReplanIntervalSteps;
    const bool cluster_dead = impl_->has_plan && !targetClusterAlive();

    if (plan_exhausted || interval_elapsed || cluster_dead) {
        const bool can_reuse_queue = plan_exhausted && !interval_elapsed && !cluster_dead;
        const bool reused_queue = can_reuse_queue && popPendingPlan(state);
        const bool have = reused_queue || replan(state, false);
```

If the plan simply finished, try the **runner-up plans** from the last search
(`popPendingPlan`) before doing another expensive Dijkstra. If the map may be
stale (interval / dead cluster), always search again.

### 6. Emit one command

Still have waypoints → move toward the next one, and maybe scan at the same
time:

[`Algorithm/src/MappingAlgorithmImpl.cpp`](../Algorithm/src/MappingAlgorithmImpl.cpp) (403–419)

```cpp
    if (impl_->waypoint_index < impl_->plan.waypoints.size()) {
        const Position3D& target = impl_->plan.waypoints[impl_->waypoint_index];
        cmd.movement = movementToward(state, target);
        if (cmd.movement.has_value()) {
            const types::DroneState predicted = predictPose(state, *cmd.movement);
            const auto& templates = impl_->templates.get(
                lidar_config_, output_map_.getMapConfig().resolution);
            const std::optional<Orientation> world = detail::bestTravelScan(
                output_map_, predicted, target, lidar_config_, impl_->last_frontier,
                templates, impl_->stamp);
            if (world.has_value()) {
                cmd.scan_orientation =
                    Orientation{world->horizontal - predicted.heading.horizontal,
                                world->altitude - predicted.heading.altitude};
            }
        }
        return cmd;
    }
```

Arrived → fire the next direction of a gain-gated sweep, no movement:

[`Algorithm/src/MappingAlgorithmImpl.cpp`](../Algorithm/src/MappingAlgorithmImpl.cpp) (425–429)

```cpp
    if (impl_->arrival_scan_index < impl_->arrival_scans.size()) {
        const Orientation& world = impl_->arrival_scans[impl_->arrival_scan_index++];
        cmd.scan_orientation = Orientation{world.horizontal - state.heading.horizontal,
                                           world.altitude - state.heading.altitude};
        return cmd;
    }
```

That is the whole executor. The interesting part is how `replan` picks the plan.

---

## How a plan is made

`replan` packages the current map, pose, lidar, drone limits, remaining step
budget, and blocked cells, then calls [`WavefrontPlanner::plan`](../Algorithm/src/WavefrontPlanner.cpp).

[`Algorithm/src/MappingAlgorithmImpl.cpp`](../Algorithm/src/MappingAlgorithmImpl.cpp) (180–189)

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
```

Budget is literally `max_steps - step_index`:

[`Algorithm/src/MappingAlgorithmImpl.cpp`](../Algorithm/src/MappingAlgorithmImpl.cpp) (159–166)

```cpp
std::size_t MappingAlgorithmImpl_207190406_209543255::remainingSteps(
    const types::DroneState& state) const {
    const std::size_t budget = mission_config_.max_steps;
    if (budget == 0) {
        return 0;
    }
    return (state.step_index >= budget) ? 0 : (budget - state.step_index);
}
```

`WavefrontPlanner::plan` then does four things.

### A. Expand everything reachable (Dijkstra)

[`exploreReachable`](../Algorithm/src/MappingAlgorithmFrontier.cpp) walks the
output map from the drone. A cell is walkable unless the drone **sphere** hits
`Occupied` or `OutOfBounds`, or the cell is on the stall blacklist.
**`Unmapped` is walkable.** That is deliberate: otherwise the drone would never
leave the first empty bubble.

[`Algorithm/src/MappingAlgorithmFrontier.cpp`](../Algorithm/src/MappingAlgorithmFrontier.cpp) (121–128)

```cpp
    // Treat Unmapped as passable: only confirmed Occupied or OutOfBounds blocks navigation.
    // This allows the planner to route through unexplored territory; stall detection handles
    // the case where an Unmapped cell turns out to be solid at execution time.
    const types::VoxelOccupancy centre_occ = occupancyAt(map, centre);
    if (centre_occ == types::VoxelOccupancy::Occupied ||
        centre_occ == types::VoxelOccupancy::OutOfBounds) {
        return false;
    }
```

Walking through `Empty` costs 1. Walking through `Unmapped` costs 4, so
known-empty corridors are preferred.

[`Algorithm/src/MappingAlgorithmFrontier.cpp`](../Algorithm/src/MappingAlgorithmFrontier.cpp) (26–27)

```cpp
constexpr int kEmptyTraversalCost = 1;
constexpr int kUnmappedTraversalCost = 4;
```

Neighbors are 6-connected (faces only, no diagonals). Expansion is capped at
the map’s voxel count so a bug cannot hang forever.

### B. Mark frontiers and glue them into clusters

A cell is a frontier if it has an `Unmapped` face-neighbor, **or** it is itself
`Unmapped`:

[`Algorithm/src/MappingAlgorithmFrontier.cpp`](../Algorithm/src/MappingAlgorithmFrontier.cpp) (498–514)

```cpp
        int unmapped = 0;
        for (const Offset& off : kOffsets) {
            const Position3D nb = keyToPoint(
                GridKey{current.qx + off.dx, current.qy + off.dy, current.qz + off.dz}, config);
            if (occupancyAt(map, nb) == types::VoxelOccupancy::Unmapped) {
                ++unmapped;
            }
        }
        const bool self_unmapped = current_occ == types::VoxelOccupancy::Unmapped;
        if (unmapped > 0 || self_unmapped) {
            if (out.frontier_cells.insert(current).second) {
                frontier_list.push_back(current);
            }
        }
```

Then a BFS glues adjacent frontier cells into `FrontierCluster`s (see
[`MappingAlgorithmFrontier.h`](../Algorithm/src/MappingAlgorithmFrontier.h)).
Each cluster remembers:

- `cell_count` — Empty surface size
- `volume_count` — whole blob
- `approach_key` — cheapest cell in the cluster to fly to
- `keys` — all members

Think: “unknown room on the left” vs “tiny crumb on the right” as two clusters.

If the start sphere is blocked (drone overlapping a wall), planning never gets
here. It tries [`findUnstickPath`](../Algorithm/src/MappingAlgorithmFrontier.cpp)
— one step to a passable neighbor — and returns that as an escape plan.

### C. Filter by remaining budget, score cells-per-step

Keep at most 8 clusters (`kRankedClusters`). For each, rebuild the path to the
approach cell, shorten it, count **real** command steps:

[`Algorithm/src/WavefrontPlanner.cpp`](../Algorithm/src/WavefrontPlanner.cpp) (211–219)

```cpp
        const std::size_t travel =
            waypoints.empty()
                ? 0
                : stepCostForPath(waypoints, in.state.position, in.state.heading, limits);
        if (travel + reserve > in.remaining_steps) {
            continue;
        }
        const double rate = static_cast<double>(cluster_score(cluster)) /
                            static_cast<double>(travel + reserve);
```

`travel + reserve > remaining_steps` → skip. You cannot finish flying there
and still have reserved scan steps.

`reserve` is a small number of scan steps (min of the lidar’s direction count
and 8). `cluster_score` is `cell_count` usually, or `volume_count` on open
outdoor maps with long lidar.

[`stepCostForPath`](../Algorithm/src/PathShaping.cpp) is not “how many voxels.”
It is how many **host steps** the drone will spend: ceil(height /
`max_elevate`) + ceil(turn / `max_rotate`) + ceil(distance / `max_advance`).

[`Algorithm/src/PathShaping.cpp`](../Algorithm/src/PathShaping.cpp) (91–107)

```cpp
    for (const Position3D& to : waypoints) {
        const double dz = zCm(to) - zCm(from);
        if (std::abs(dz) > kSameAxisEpsilonCm) {
            steps += ceilDiv(std::abs(dz), elevate_cm);
        }
        // ...
            if (turn > 1e-9) {
                steps += ceilDiv(turn, rotate_deg);
                heading_deg = target_deg;
            }
            steps += ceilDiv(planar, advance_cm);
```

Before that, [`stringPullConstantAltitude`](../Algorithm/src/PathShaping.cpp)
drops intermediate waypoints if a straight line at the **same Z** is still
passable. Height changes stay as their own waypoints, because the drone
elevates first, then moves horizontally — a 3D diagonal is not how it actually
flies.

Then sort candidates by `expected_rate` descending. Best plan is returned. The
rest go into `pending_plans`.

[`Algorithm/src/WavefrontPlanner.cpp`](../Algorithm/src/WavefrontPlanner.cpp) (234–241)

```cpp
    std::stable_sort(candidates.begin(), candidates.end(),
                     [](const ExplorationPlan& a, const ExplorationPlan& b) {
                         return a.expected_rate > b.expected_rate;
                     });
    if (alternates != nullptr) {
        alternates->assign(candidates.begin() + 1, candidates.end());
    }
    return std::move(candidates.front());
```

Numeric picture: 40 steps left, reserve 8.

| Cluster | size S | travel | affordable? | rate |
|---------|--------|--------|-------------|------|
| crumb | 3 | 4 | yes | 3/12 = 0.25 |
| room | 20 | 15 | yes | 20/23 ≈ 0.87 |
| far hall | 80 | 50 | no | skipped |

The hall is illegal. The room wins. The crumb is a runner-up.

---

## How movement is chosen (one command, not the whole path)

The plan is a list of waypoints. Each tick only produces **one**
`MovementCommand`. Priority is height, then heading, then forward:

[`Algorithm/src/MappingAlgorithmImpl.cpp`](../Algorithm/src/MappingAlgorithmImpl.cpp) (109–156)

```cpp
    if (std::abs(dh) > 1e-6) {
        // Elevate, clamped to max_elevate
        ...
    }
    // ... if already at XY: Hover
    if (std::abs(delta) > 1e-6) {
        // Rotate Left/Right, clamped to max_rotate
        ...
    }
    // Advance, clamped to max_advance
```

So a 90° turn plus a long corridor is many ticks: rotate (maybe twice if
`max_rotate` is 45°), then several advances. That is why `stepCostForPath`
charges those the same way — ranking and execution agree.

If the predicted pose after that move would see Unmapped the current frontier
cares about, [`bestTravelScan`](../Algorithm/src/ScanPlanning.cpp) attaches a
scan to the **same** command. The host will move, then scan, in one step.

---

## How scanning is chosen

Directions are not a hardcoded 26-way sphere. They come from the real lidar
cone ([`LidarCone.h`](../UserCommon/include/user_common_207190406_209543255/LidarCone.h)
/ [`ConeTemplate.h`](../UserCommon/include/user_common_207190406_209543255/ConeTemplate.h)):
half-angle from `z_min`, `d`, `fov_circles`, laid on a Fibonacci sphere.

A direction is used only if walking that cone still hits Unmapped voxels that
are “gain-masked” to the current frontier (roughly: unknown we can actually see
from here, not a sealed room far away).
[`buildSweepDirections`](../Algorithm/src/ScanPlanning.cpp) ranks those by
gain. On small outdoor maps the arrival sweep is capped at 4 directions.

House / outdoor classifiers in [`ScanPlanning.h`](../Algorithm/src/ScanPlanning.h)
(`isHouseVolumeMission`, `isOpenVolumeMission`, `isSmallOutdoorMission`) tweak
whether downward or volume-carve scans are allowed — e.g. don’t stare at the
floor on a house, do count empty outdoor volume.

---

## When it stops

Two independent “this is no longer paying off” checks.

**Predicted rate.** If several **fresh** replans in a row have
`expected_rate < 0.25` (`kMinInformationRate`) or no valid plan, finish.
Runner-up plans from the queue do **not** count toward that — they are
leftovers from the same search.

[`Algorithm/src/MappingAlgorithmImpl.cpp`](../Algorithm/src/MappingAlgorithmImpl.cpp) (343–358)

```cpp
            const bool low = !have || impl_->plan.expected_rate < kMinInformationRate;
            if (low) {
                ++impl_->low_rate_replans;
                // ... maybe replan(ignore_blocked=true) as recovery ...
                } else if (impl_->low_rate_replans >= kLowRateReplans) {
                    impl_->finished = true;
                    types::MappingStepCommand cmd{};
                    cmd.status = detail::hasAnyNotMappedInBounds(output_map_)
                                     ? types::AlgorithmStatus::FinishedWithUnmappableVoxels
                                     : types::AlgorithmStatus::Finished;
```

`replan(..., true)` is a recovery: ignore the stall blacklist once, in case we
boxed ourselves in.

**Observed rate.** Every 100 steps, count Unmapped voxels in bounds. If the
drop is below 0.05 cells/step for 4 windows, the map is not actually
improving — finish even if the planner is still optimistic.

If anything Unmapped remains, status is `FinishedWithUnmappableVoxels`. If the
map is fully known, `Finished`.

---

## How the pieces sit together

```text
host calls nextStep(state, scan)
        │
        ├─ stall / TTL / skip reached waypoints
        ├─ need a new plan?
        │      └─ WavefrontPlanner::plan
        │            ├─ MappingAlgorithmFrontier::exploreReachable  (Dijkstra)
        │            ├─ cluster frontiers
        │            ├─ PathShaping (shorten + count real steps)
        │            └─ keep best rate among clusters we can still afford
        │
        ├─ have waypoints?  movementToward + optional bestTravelScan
        └─ arrived?         next buildSweepDirections orientation
```

The only public class is [`MappingAlgorithmImpl`](../Algorithm/include/Algorithm/MappingAlgorithmImpl.h).
Everything else is `detail::` inside [`Algorithm/src/`](../Algorithm/src/).
The plugin registers itself at the bottom of the `.cpp`:

[`Algorithm/src/MappingAlgorithmImpl.cpp`](../Algorithm/src/MappingAlgorithmImpl.cpp) (440–442)

```cpp
using MappingAlgorithmImpl_207190406_209543255 =
    algorithm_207190406_209543255::MappingAlgorithmImpl_207190406_209543255;
REGISTER_MAPPING_ALGORITHM(MappingAlgorithmImpl_207190406_209543255);
```
