# Algorithm component — deep dive (idea, code, and tests)

Everything in this doc is about [`Algorithm/`](../Algorithm/) only — the `.so` that implements
[`common::IMappingAlgorithm`](../common/include/Common/IMappingAlgorithm.h) and decides, every tick, where the drone should fly and
where it should point the lidar. It does not move the drone or fire the lidar itself
(`MissionControl` does that); it only reads the shared output map and returns a command.

Ctrl+click a path to open the file (line numbers are in the link text).

This doc has three parts:

1. [Part 1 — The idea](#part-1--the-idea) — what problem the algorithm solves and the mental
   model to hold in your head.
2. [Part 2 — The code, file by file](#part-2--the-code-file-by-file) — every file in
   [`Algorithm/src/`](../Algorithm/src/) and [`Algorithm/include/`](../Algorithm/include/),
   with the key lines quoted and explained.
3. [Part 3 — The tests](#part-3--the-tests) — every test file, what each test actually
   checks, and which function/constant in the code it is protecting. Use this section if
   asked "break this test."

Companion reading, if you want a second pass at the same material: [`docs/mapping-algorithm-walkthrough.md`](mapping-algorithm-walkthrough.md)
(narrative, less exhaustive) and [`tmp/notes/codebase-study-guide.md`](../tmp/notes/codebase-study-guide.md) Part 5 (one-page summary,
whole-codebase context).

---

## Part 1 — The idea

### 1.1 The contract

The drone starts in an unknown 3D world. There is a hidden ground-truth map (owned by the
Simulator, never seen by the algorithm) and an **output map** that starts fully `Unmapped`.
Each tick, the host asks the algorithm one question and paints the map based on the answer:

[`IMappingAlgorithm.h`](../common/include/Common/IMappingAlgorithm.h) (24–26)

```cpp
[[nodiscard]] virtual types::MappingStepCommand nextStep(
    const types::DroneState& state,
    const types::LidarScanResult* latest_scan) = 0;
```

- `state` — GPS position, heading, and `step_index` (how many ticks have already happened).
- `latest_scan` — the previous lidar hits, or `nullptr` on the very first call.
  **The algorithm ignores this parameter.** Its only belief about the world is the
  `output_map_` reference stored on the base class ([`common::IMappingAlgorithm`](../common/include/Common/IMappingAlgorithm.h)), which
  `MissionControl` has already fused the previous scan into by the time `nextStep` runs
  again. Reading `latest_scan` would be redundant — see
  [`Algorithm/src/MappingAlgorithmImpl.cpp:283-284`](../Algorithm/src/MappingAlgorithmImpl.cpp).
- Return value, `MappingStepCommand{movement, scan_orientation, status}` — at most one
  movement and one scan orientation, both optional, plus a status
  (`Working` / `Finished` / `FinishedWithUnmappableVoxels`).

### 1.2 The idea in one sentence

Find the boundary between "confirmed empty" and "never seen" (the **frontier**), fly to
the most information-dense patch of that boundary (a **cluster**), scan it, and repeat —
until flying anywhere else stops being worth the steps it would cost.

### 1.3 Vocabulary used everywhere in the code

| Term | Meaning |
|---|---|
| **Frontier cell** | An `Empty` (or itself `Unmapped`) cell that touches an `Unmapped` face-neighbor — i.e. a cell on the edge of the known region. |
| **Frontier cluster** | A face-connected blob of frontier cells (BFS-glued). Represents one "room" or "pocket" of unknown space. |
| **Reachability search** | A Dijkstra from the drone's current cell over the *passable* component of the map (treats `Unmapped` as walkable but expensive — see §1.4). |
| **Plan (`ExplorationPlan`)** | The chosen cluster plus a waypoint path to its cheapest approach cell. Held across many ticks; not recomputed every tick. |
| **Waypoint** | One 3D grid point on the current plan's path. The executor advances through waypoints one at a time. |
| **Stall** | GPS position unchanged for 2 consecutive ticks while a waypoint is still pending — a sign the "walkable" `Unmapped` cell turned out to be solid. |
| **Arrival sweep** | After the last waypoint, a sequence of scan-only ticks pointing the lidar in several directions before replanning. |
| **Tick** | One call to `nextStep` (one host step). |

### 1.4 Why `Unmapped` is walkable during planning

This is the single most important design decision in the frontier search
([`MappingAlgorithmFrontier.cpp:120-127`](../Algorithm/src/MappingAlgorithmFrontier.cpp)):

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

If `Unmapped` were treated as a wall, the drone could never plan a path out of its
starting empty bubble — there would be nothing to route through. So the search is
optimistic: it assumes unknown space is probably open, but it charges a **higher
traversal cost** for walking through it than through confirmed-empty space
(`kEmptyTraversalCost=1` vs `kUnmappedTraversalCost=4`, [`MappingAlgorithmFrontier.cpp:26-27`](../Algorithm/src/MappingAlgorithmFrontier.cpp))
so that known corridors are preferred over guessing. If the optimism is wrong — the
drone tries to advance into a cell that's actually a wall — the drone doesn't move
(GPS position frozen), and the **stall** counter in `MappingAlgorithmImpl` (§2.4) catches
it, blacklists that cell, and forces a replan.

### 1.5 One tick, top to bottom

`nextStep` ([`MappingAlgorithmImpl.cpp:282-434`](../Algorithm/src/MappingAlgorithmImpl.cpp)) reads like a straight-line story:

1. **Already finished?** → keep returning `Finished`.
2. **Prune expired blocked cells** (TTL 50 steps) so a stall-caused blacklist entry
   doesn't permanently exclude a cell that a later scan proved passable.
3. **Detect stall** — position unchanged for 2 ticks while chasing a waypoint → blacklist
   that cell, drop the plan.
4. **Skip past waypoints already reached** (GPS snapped close enough).
5. **Maybe replan** — if the plan ran out, 25 steps passed since the last plan, or the
   target cluster got fully mapped by someone else's scan.
6. **Termination checks** — predicted-rate streak and observed-unmapped-drop streak
   (§1.7).
7. **Emit one command** — movement toward the next waypoint (with an optional co-emitted
   travel scan), or the next direction of the arrival sweep if waypoints are exhausted.

### 1.6 How a plan is made (`WavefrontPlanner::plan`)

Given the map, drone pose, lidar/drone limits, remaining step budget, and the blocklist,
`WavefrontPlanner::plan` ([`WavefrontPlanner.cpp:80-242`](../Algorithm/src/WavefrontPlanner.cpp)) does:

**A. Explore what's reachable.** `MappingAlgorithmFrontier::exploreReachable` runs a
bounded Dijkstra from the drone's cell, collecting every frontier cell it touches and
BFS-clustering them into `FrontierCluster`s (§2.3). If the drone's own sphere is blocked
(boxed in by a wall), planning skips straight to `findUnstickPath` — a one-step escape to
the nearest passable face-neighbor.

**B. Rank clusters.** Keep the best `kRankedClusters=8` clusters by score
(`cell_count` normally; `volume_count` on open outdoor maps with a long lidar —
see `rank_volume` in [`WavefrontPlanner.cpp:111-117`](../Algorithm/src/WavefrontPlanner.cpp)).

**C. Filter by budget, compute rate.** For each candidate cluster: rebuild the path to its
approach cell, string-pull it (§2.4 `PathShaping`), convert to a **real step count**
(`stepCostForPath` — ceil-divided Rotate/Advance/Elevate commands, not voxel count), and
compute `expected_rate = cluster_score / (travel_steps + reserve)`. `reserve` is a small
budget (min of lidar direction count and 8) held back for the arrival sweep. If
`travel + reserve > remaining_steps`, the cluster is unaffordable and is skipped entirely
— **not** just deprioritized.

**D. Pick the best, remember the rest.** Sort candidates by `expected_rate` descending;
return the best, and (if the caller passed an `alternates` pointer) stash the rest as
**runner-up** plans so the caller can reuse them later without rerunning Dijkstra
(`popPendingPlan` in [`MappingAlgorithmImpl.cpp:192-203`](../Algorithm/src/MappingAlgorithmImpl.cpp)).

There are two mission-specific forced-descent overrides ahead of the generic ranking —
house layer-by-layer descent and outdoor volume-carving — described in §2.6/§2.7. They
exist because generic frontier ranking alone under-explores those specific map shapes;
see [`docs/mapping-algorithm-rewrite-pickup.md`](mapping-algorithm-rewrite-pickup.md) for the score-tuning history.

### 1.7 How it decides to stop

Two independent, unrelated "this isn't paying off" checks, both in
[`MappingAlgorithmImpl::nextStep`](../Algorithm/src/MappingAlgorithmImpl.cpp):

- **Predicted rate** ([`MappingAlgorithmImpl.cpp:341-364`](../Algorithm/src/MappingAlgorithmImpl.cpp)): if `kLowRateReplans=3`
  consecutive **fresh** replans (not reused runner-ups — those don't count, see the
  [comment at line 337](../Algorithm/src/MappingAlgorithmImpl.cpp)) come back with `expected_rate < kMinInformationRate=0.25` (or no
  plan at all), finish. Before giving up, it tries `kRecoveryAttempts=3` recovery replans
  with `ignore_blocked=true`, in case the blocklist itself boxed the drone in.
- **Observed rate** ([`MappingAlgorithmImpl.cpp:372-397`](../Algorithm/src/MappingAlgorithmImpl.cpp)): every `kObservedWindowSteps=100`
  ticks, measure how many `Unmapped` voxels actually disappeared. If the drop rate is
  below `kMinObservedInformationRate=0.05` cells/step for `kLowObservedWindows=4`
  consecutive windows, finish — even if the planner is still "optimistic." This catches
  the case where the plan *looks* good (big cluster, cheap travel) but MissionControl
  never actually classifies the promised cells (an adversarial or foreign
  MissionControl, or a modeling bug).

Final status: `FinishedWithUnmappableVoxels` if any `Unmapped` voxel remains in bounds,
else `Finished`.

### 1.8 How a movement command is built

Only **one** `MovementCommand` per tick, chosen by strict priority in `movementToward`
([`MappingAlgorithmImpl.cpp:109-156`](../Algorithm/src/MappingAlgorithmImpl.cpp)): **height first** (Elevate, if `|dz| > 0`), **then
heading** (Rotate, if not already facing the target), **then forward** (Advance). This
means a diagonal move is many ticks: elevate, rotate (maybe twice if `max_rotate` is
small), then one or more advances — which is exactly what `stepCostForPath` charges when
ranking clusters, so the ranking and the actual execution agree.

If the *predicted* pose after that movement would see new frontier-relevant `Unmapped`
voxels, `bestTravelScan` (§2.7) attaches a scan to the **same** command — the drone moves
and scans in one host step.

---

## Part 2 — The code, file by file

Files, in the order you should read them:

| File | Responsibility |
|---|---|
| [`include/Algorithm/MappingAlgorithmImpl.h`](../Algorithm/include/Algorithm/MappingAlgorithmImpl.h) | Public class declaration (the only exported type) |
| [`src/MappingAlgorithmImpl.cpp`](../Algorithm/src/MappingAlgorithmImpl.cpp) | The tick loop: stall/replan/termination/emit-command state machine |
| [`src/ExplorationPlan.h`](../Algorithm/src/ExplorationPlan.h) | The plan/inputs data structures passed between the impl and the planner |
| [`src/MappingAlgorithmFrontier.h`](../Algorithm/src/MappingAlgorithmFrontier.h) / [`.cpp`](../Algorithm/src/MappingAlgorithmFrontier.cpp) | Reachability search, frontier detection, clustering |
| [`src/WavefrontPlanner.h`](../Algorithm/src/WavefrontPlanner.h) / [`.cpp`](../Algorithm/src/WavefrontPlanner.cpp) | Cluster ranking and budget filtering |
| [`src/PathShaping.h`](../Algorithm/src/PathShaping.h) / [`.cpp`](../Algorithm/src/PathShaping.cpp) | String-pulling and the real step-cost model |
| [`src/ScanPlanning.h`](../Algorithm/src/ScanPlanning.h) / [`.cpp`](../Algorithm/src/ScanPlanning.cpp) | Gain-gated scan direction selection |

### 2.1 [`MappingAlgorithmImpl.h`](../Algorithm/include/Algorithm/MappingAlgorithmImpl.h) — the public class

[`MappingAlgorithmImpl.h`](../Algorithm/include/Algorithm/MappingAlgorithmImpl.h) (12–76)

```cpp
namespace algorithm_207190406_209543255 {

namespace detail {
struct ExplorationPlan;
} // namespace detail

/// Wavefront Frontier Detection over the reachability substrate.
/// Each nextStep emits a movement and, when the resulting pose would observe
/// something new, a scan in the same command.
class MappingAlgorithmImpl_207190406_209543255 final : public common::IMappingAlgorithm {
public:
    /// @param dependencies Mission, sensor, drone, and output-map dependencies.
    explicit MappingAlgorithmImpl_207190406_209543255(
        common::MappingAlgorithmDependencies dependencies);

    /// Returns the next scan orientation and/or movement for DroneControl to execute.
    [[nodiscard]] common::types::MappingStepCommand nextStep(
        const common::types::DroneState& state,
        const common::types::LidarScanResult* latest_scan) override;

    ~MappingAlgorithmImpl_207190406_209543255() override;
    // copy/move deleted — owns a unique_ptr<Impl>, singleton-per-mission semantics
```

Notable points:

- **`final`**, deletes copy/move — one instance per mission, created fresh by the plugin
  factory each run (never cached across runs — see [`.cursor/rules/plugin-architecture.mdc`](../.cursor/rules/plugin-architecture.mdc)).
- All the actual mutable state lives in a **`struct Impl`** defined only in the `.cpp`
  (pimpl pattern) — the header only declares the private helper method signatures and the
  ten tuning constants at the bottom (`kMaxMovingStallTicks`, `kReplanIntervalSteps`,
  `kBlockedTtlSteps`, `kRecoveryAttempts`, `kLowRateReplans`, `kObservedWindowSteps`,
  `kLowObservedWindows`, `kMinInformationRate`, `kMinObservedInformationRate`). Every one
  of these constants is exercised by at least one test in Part 3.
- The private methods map 1:1 onto the numbered steps in §1.5:
  `ensurePlanningReady`, `remainingSteps`, `pruneExpiredBlockedCells`, `replan`,
  `adoptPlan`, `popPendingPlan`, `movementToward`, `predictPose`, `buildArrivalSweep`,
  `targetClusterAlive`, `reachedWaypoint`, `samePosition`.

### 2.2 [`ExplorationPlan.h`](../Algorithm/src/ExplorationPlan.h) — the data passed between impl and planner

[`ExplorationPlan.h`](../Algorithm/src/ExplorationPlan.h) (15–34)

```cpp
struct ExplorationPlan {
    std::vector<common::Position3D> waypoints{};
    std::size_t target_cluster_cells = 0;
    double expected_rate = 0.0;
    std::vector<GridKey> target_keys{};
    FrontierCells frontier_cells{};
    bool valid = false;
};

struct WavefrontInputs {
    const common::IMap3D& map;
    const common::types::DroneState& state;
    const common::types::LidarConfigData& lidar;
    const common::types::DroneConfigData& drone;
    std::size_t remaining_steps = 0;
    const BlockedCells& blocked;
    bool ignore_blocked = false;
    bool prefer_descend = false;
};
```

- `ExplorationPlan.valid == false` means "no plan could be made" — either everything is
  already mapped, or nothing affordable was found. An **empty `waypoints` vector with
  `valid == true`** is a legitimate "stay put and scan" plan (the drone's current cell is
  itself the cheapest frontier — see the `StaysPutWhenStartIsTheCheapestFrontier` test).
- `target_keys` — the winning cluster's member cells. Used by `targetClusterAlive()` to
  detect a stale plan (someone else scanned the target away) and by `popPendingPlan` to
  skip a queued runner-up whose cluster died in the meantime.
- `WavefrontInputs.prefer_descend` is set only from `MappingAlgorithmImpl` to force a
  house-mission vertical descent even when the current layer still has some horizontal
  `Unmapped` (§2.6).

### 2.3 `MappingAlgorithmFrontier` — reachability, frontier, clustering

**Grid quantization.** Every 3D `Position3D` is rounded to an integer `GridKey{qx,qy,qz}`
relative to the map's offset and resolution (`quantizePosition`,
[`MappingAlgorithmFrontier.cpp:225-239`](../Algorithm/src/MappingAlgorithmFrontier.cpp)). All planning happens in this integer grid space;
`keyToPoint` converts back to world coordinates for movement commands.

**Sphere passability — the core walkability test.** `isSpherePassable`
([`MappingAlgorithmFrontier.cpp:116-172`](../Algorithm/src/MappingAlgorithmFrontier.cpp)) is not a simple "is this voxel occupied" check —
it samples every grid cell whose box intersects the drone's collision sphere:

[`MappingAlgorithmFrontier.cpp`](../Algorithm/src/MappingAlgorithmFrontier.cpp) (88–114)

```cpp
// True iff the axis-aligned voxel box centered at (dx,dy,dz)*step_cm with half-extent
// step_cm/2 intersects the closed sphere of radius radius_cm at the origin. Using cell
// centres for the distance gate (the old ox²+oy²+oz² > r² test) is a no-op when
// radius_cm < step_cm: every non-zero offset is ≥ step_cm and is skipped. Nearest-point
// in the box restores footprint checks for e.g. radius 7.5 cm on a 10 cm grid.
[[nodiscard]] bool sphereIntersectsCellBox(int dx, int dy, int dz,
                                           double step_cm,
                                           double radius_cm) {
    if (dx == 0 && dy == 0 && dz == 0) {
        return true;
    }
    const double half = step_cm * 0.5;
    const double ox = static_cast<double>(dx) * step_cm;
    ...
```

This box-nearest-point test (rather than a naive cell-*center* distance test) matters
specifically when `radius_cm < step_cm` (e.g. a 7.5 cm drone on a 10 cm grid) — a
center-distance test would never flag any non-zero offset as a collision, silently
disabling footprint checking exactly when it's needed. Two tests exist specifically for
this edge case (`FrontierRejectsOccupiedFaceNeighbourOnCm10Grid` /
`FrontierAllowsOccupiedFaceNeighbourWhenRadiusTooSmall`, §3.2).

Inside the loop, only `Occupied` blocks (`OutOfBounds` at the mission AABB boundary is
tolerated when the *center* is in-bounds — this is the house-scenario spawn-on-max-height
fix, see the comment at [`MappingAlgorithmFrontier.cpp:163-166`](../Algorithm/src/MappingAlgorithmFrontier.cpp) and the
`FrontierAllowsInBoundsStartWhenSphereClipsOutOfBounds` test).

**`exploreReachable`** ([`MappingAlgorithmFrontier.cpp:437-596`](../Algorithm/src/MappingAlgorithmFrontier.cpp)) is the main entry point:
a Dijkstra (priority queue keyed by integer traversal cost) from the start cell, over
every neighbor that's sphere-passable and not on the `blocked_cells` set. While expanding
each cell it:

1. Counts `Unmapped` face-neighbors; marks the cell a **frontier cell** if it has any, or
   is itself `Unmapped` (self-`Unmapped` inclusion is what keeps a single isolated pocket
   voxel as one cluster instead of six disconnected face cells — comment at
   [`MappingAlgorithmFrontier.cpp:507-509`](../Algorithm/src/MappingAlgorithmFrontier.cpp)).
2. Pushes sphere-passable neighbors with cost `+1` (Empty) or `+4` (Unmapped).
3. Caps total expansions at `max_expansions` (from `maxExpansionsForMap` — the mission's
   full voxel volume) so a passability bug can never hang the mission (this closes the
   ex2 "ALG28" unbounded-BFS bug referenced in [`docs/ex2-grading-handoff.md`](ex2-grading-handoff.md)).

After the Dijkstra terminates, a second BFS glues face-adjacent frontier cells into
`FrontierCluster`s. Each cluster's `keys`/`cell_count` is the **Empty surface only**
(cells actually standable on); `volume_count` is the whole blob including `Unmapped`
members — used only for the outdoor volume-ranking heuristic. The cluster's
`approach_key` is its cheapest-to-reach surface member.

**`findUnstickPath`** ([`MappingAlgorithmFrontier.cpp:352-382`](../Algorithm/src/MappingAlgorithmFrontier.cpp)) is the one-step escape used
only when the drone's own sphere is not passable (boxed in) — tries each of the 6 face
neighbors and returns the first passable one, never further than one step (no teleporting
past a wall).

### 2.4 `WavefrontPlanner` — cluster ranking

`plan()` ([`WavefrontPlanner.cpp:80-242`](../Algorithm/src/WavefrontPlanner.cpp)) walks through §1.6's four steps. Two details
worth calling out beyond the summary:

**Mission-shape classifiers drive score choice.** `rank_volume` is true only for open
outdoor missions using a long lidar (`z_max > 90 cm`); otherwise clusters rank by surface
`cell_count`. This is a scoring-band tuning decision — see
[`docs/mapping-algorithm-rewrite-pickup.md`](mapping-algorithm-rewrite-pickup.md) — not a general frontier-search property.

**The house forced-descent override runs before generic ranking**
([`WavefrontPlanner.cpp:118-134`](../Algorithm/src/WavefrontPlanner.cpp)): if this is a house-shaped mission, the caller asked for
`prefer_descend`, and the column directly below the drone still contains `Unmapped`, skip
cluster ranking entirely and just descend one cell. This is what makes the algorithm sweep
a house floor-by-floor instead of trying to see through the ceiling from one height.

**A second, narrower descent path exists inside the per-cluster loop**
([`WavefrontPlanner.cpp:183-204`](../Algorithm/src/WavefrontPlanner.cpp)) for the "stay in place" candidate specifically (when the
best `approach == reach.start_key`): if the drone is near the lidar's near-field ceiling,
or (house-only) the current layer has no horizontal `Unmapped` left, drop one level
instead of just sweeping in place. This is a *different* trigger from the top-level
`prefer_descend` override — it fires reactively once staying and scanning stops finding
anything, whereas `prefer_descend` fires proactively from the tick loop after a full plan
exhausts.

### 2.5 `PathShaping` — string-pulling and the real step-cost model

**`stringPullConstantAltitude`** ([`PathShaping.cpp:49-76`](../Algorithm/src/PathShaping.cpp)) is a greedy line-of-sight
shortcutter: starting from an anchor, keep extending to the farthest waypoint still at the
**same altitude** with a clear line of sight, then re-anchor. Altitude changes are never
merged across — they always survive as their own waypoint, because §1.8's movement
priority always elevates before moving horizontally, so a "3D diagonal" waypoint would
not describe the actual flight path.

**`stepCostForPath`** ([`PathShaping.cpp:78-113`](../Algorithm/src/PathShaping.cpp)) converts a waypoint list into the number
of **host ticks** it will actually cost, by simulating the same priority as
`movementToward`: for each waypoint, `ceil(|dz| / max_elevate)` elevate-steps, then (if
there's a heading change) `ceil(turn / max_rotate)` rotate-steps, then
`ceil(planar_distance / max_advance)` advance-steps. This is what makes cluster ranking
honest — a "close" cluster behind a 90° turn on a slow-rotating drone can cost more ticks
than a "far" cluster straight ahead.

### 2.6 `ScanPlanning` — gain-gated scan direction selection

**Mission-shape classifiers** ([`ScanPlanning.cpp:145-174`](../Algorithm/src/ScanPlanning.cpp)) — `isOpenVolumeMission`,
`isSmallOutdoorMission`, `isHouseVolumeMission` — bucket the mission by its bounding-box
dimensions (all in cm, thresholds are magic-number map fixtures, not physics):

| Classifier | Condition |
|---|---|
| `isOpenVolumeMission` | all three spans ≥ 199 cm |
| `isSmallOutdoorMission` | all three spans in `[199, 250)` cm |
| `isHouseVolumeMission` | XY spans ≥ 249 cm, height in `[99, 199)` cm |

These feed `skipDownwardScan` (never stare at the floor once near the ceiling, or ever on
a house mission — the floor is never useful to look at from above in a low wide box) and
`volumeGainAllowed` (on outdoor cubes, count `Unmapped` volume behind the frontier shell
as gain, not just gain masked to the shell — because in a big open cube the frontier shell
massively under-counts how much is actually visible per scan).

**`isGainMasked`** ([`ScanPlanning.cpp:176-185`](../Algorithm/src/ScanPlanning.cpp)) is the core gain filter: a candidate voxel
only counts if it *is* a frontier cell, or is face-adjacent to one. This stops the sweep
from "seeing through" to a sealed, already-explored room far away — only genuinely
reachable unknown space near the current frontier counts.

**`buildSweepDirections`** ([`ScanPlanning.cpp:187-241`](../Algorithm/src/ScanPlanning.cpp)) scores every direction template by
`countConeGain` (walk the cone, count masked/volume gain), sorts descending by gain (tie-
broken by horizontal then altitude angle for determinism — not enumeration order, see
`SweepOrdersByIndependentGainNotEnumerationOrder`), then does a **second marginal pass**:
walk each direction again and keep it only if it still finds gain *after* the previous
kept directions' coverage has been stamped (`stamp` dedupes voxels across the whole sweep)
— this is what drops directions that only ever repeated a wider one's coverage
(`MarginalPassDropsFullyClaimedDirections`).

**`bestTravelScan`** ([`ScanPlanning.cpp:243-301`](../Algorithm/src/ScanPlanning.cpp)) is the "co-emit a scan with a movement"
helper: given the *predicted* pose after a movement, probes exactly 3 candidate
orientations (toward the next waypoint, straight up, straight down), rejects any whose
near field already contains solid geometry (about to look straight into a wall you're
flying past), and returns the first one with real masked/volume gain.

---

## Part 3 — The tests

Test name → what it checks → what code it protects.

All Algorithm tests compile the actual `.cpp` files (not the `.so`) into one binary,
`algorithm_test`, using `FakeMap3D` ([`Algorithm/tests/FakeMap3D.h`](../Algorithm/tests/FakeMap3D.h)) — an in-memory
`IMutableMap3D` with no file I/O, and [`StubPluginRegistration.cpp`](../Algorithm/tests/StubPluginRegistration.cpp) to satisfy the
registration macro outside the real Simulator. Run with:

```bash
ctest --test-dir build/default -R algorithm --output-on-failure
# or by name:
./build/default/Algorithm/algorithm_test --gtest_filter='*SomeTestName*'
```

### 3.1 [`test_mapping_algorithm.cpp`](../Algorithm/tests/test_mapping_algorithm.cpp) — end-to-end `nextStep` state machine

Exercises the public `MappingAlgorithmImpl_207190406_209543255` through its public
interface only (no `detail::` access). This is the file to read first if you want to see
the algorithm "from the outside."

| Test | What it checks | Breaks if you touch… |
|---|---|---|
| `FirstStepRequestsScanWithNullLatestScan` | First tick returns `Working` + a scan orientation even with `latest_scan == nullptr` | the initial-scan path in `nextStep`, or making the algorithm depend on `latest_scan` |
| `FinishesWhenNoFrontierRemains` | Fully-known small box → eventually `Finished` | the termination checks (§1.7), `hasAnyNotMappedInBounds` |
| `EmitsMovementTowardFrontier` | Corridor with unknown past a short lidar range → some tick emits non-Hover movement | `WavefrontPlanner::plan`, `movementToward` |
| `ReturnsFinishedOnSubsequentCallsAfterCompletion` | After `Finished`, later calls return `Finished` with no movement/scan | the `impl_->finished` early-return at the top of `nextStep` |
| `EmitsRotateWhenHeadingMisalignedWithPath` | Heading 90° off from the path → first movement is `Rotate` | the height→heading→advance priority order in `movementToward` |
| `EmitsAdvanceWhenHeadingAlignedWithPath` | Heading already aligned → first movement is `Advance` | same priority order, the "already aligned" branch |
| `EmitsElevateWhenWaypointHeightDiffers` | Waypoint above current Z → first movement is `Elevate` | the `dh` branch at the top of `movementToward` |
| `FinishesWithUnmappableWhenStartNotSpherePassable` | Drone boxed in by `Occupied` on every side → `FinishedWithUnmappableVoxels`, not a hang | `findUnstickPath`, the recovery-attempt loop, `hasAnyNotMappedInBounds` |
| `DroneNavigatesFromStartWhenAdjacentCellsAreEmpty` | Small local Empty bubble → still finds a frontier and moves | `exploreReachable`'s Dijkstra starting from a non-trivial local bubble |
| `AcceptsNonNullLatestScanWithoutChangingFirstScanRequest` | Non-null `latest_scan` on tick 0 changes nothing | confirms `latest_scan` truly is unused |
| `DoesNotTerminatePrematurely` | Corridor with real unknown space → stays `Working` for at least one tick | any early/aggressive termination check firing too soon |
| `HandlesNonNullScanResultWithoutCrashOrPrematureFinish` | A "miss" scan result (`double::max()`) doesn't crash or finish early | robustness of ignoring `latest_scan` |
| `StallPathDoesNotMutateMap` | Stalling never writes to `output_map_` (algorithm is read-only on the map) | anything in the stall-handling branch that touches `output_map_` instead of only `impl_->blocked_cells`/`blocked_since` |
| `EmitsMovementAndScanInTheSameCommand` | At least one command carries **both** `movement` and `scan_orientation` | the `bestTravelScan` co-emission call inside the waypoint-following branch ([`MappingAlgorithmImpl.cpp:403-419`](../Algorithm/src/MappingAlgorithmImpl.cpp)) |
| `DoesNotFinishWhileUnresolvedSpaceRemainsAndBudgetIsLarge` | With plenty of budget and unresolved space, stays `Working` for 50 ticks | over-eager finish logic (regression test for a retired "quit after first bad plan" policy) |
| `FinishesWhenUnmappedCountDoesNotDropAcrossReplans` | If `Unmapped` count never actually drops (map "frozen"), eventually `FinishedWithUnmappableVoxels` — not a 400-tick burn, not an instant quit | the **observed rate** check (`kObservedWindowSteps`/`kMinObservedInformationRate`/`kLowObservedWindows`) |
| `KeepsWorkingWhenUnmappedCountKeepsDropping` | If `Unmapped` count keeps dropping every tick, stays `Working` past the point the frozen-map test would have finished | same observed-rate check, opposite branch — confirms it doesn't false-positive |
| `FinishesCleanlyWhenNothingIsUnmapped` | Nothing `Unmapped` anywhere → `Finished` (not `FinishedWithUnmappableVoxels`) | the `hasAnyNotMappedInBounds` branch used to pick the final status |
| `TerminatesWhenBudgetIsExhausted` | `state.step_index == max_steps` → some non-`Working` status within a few ticks | `remainingSteps` (§1.6 budget filter) returning 0, causing every cluster to be unaffordable |
| `VisitsMultipleDisjointCrumbsAndFinishes` | Three separated `Unmapped` crumbs get visited and classified one-by-one, algorithm eventually finishes | multi-cluster ranking + replanning across many ticks, cluster-death detection |
| `QueuedLowRateRunnerUpsDoNotFinishEarly` | Adopting queued **low-rate** runner-up plans must not trip `kLowRateReplans` | the `reused_queue` branch resetting `low_rate_replans`/`recovery_attempts` ([`MappingAlgorithmImpl.cpp:339-342`](../Algorithm/src/MappingAlgorithmImpl.cpp)) — this is the test that pins down "runner-ups from the same search aren't a new low-rate replan" |
| `FinishesAfterConsecutiveLowRateReplans` | Isolated far-away leftover keeps `expected_rate` below `kMinInformationRate` → `FinishedWithUnmappableVoxels` | the **predicted rate** check and `kLowRateReplans` counter |
| `AbandonsPlanWhenTargetClusterIsResolved` | If the only unknown voxel gets externally resolved to `Empty` mid-plan, the algorithm notices and finishes instead of chasing a dead target | `targetClusterAlive()` / `clusterStillFrontier` |

### 3.2 [`test_mapping_algorithm_frontier.cpp`](../Algorithm/tests/test_mapping_algorithm_frontier.cpp) — direct tests of `MappingAlgorithmFrontier`

Calls `detail::MappingAlgorithmFrontier` directly — bypasses the tick loop entirely.

| Test | What it checks | Breaks if you touch… |
|---|---|---|
| `FrontierStartPassableWhenSphereHasUnmapped` | `Unmapped` inside the drone sphere does not block start passability | the "only `Occupied`/`OutOfBounds` blocks" rule in `isSpherePassable` |
| `FrontierStartNotPassableWhenSphereOverlapsOccupied` | `Occupied` voxels inside the sphere radius do block it | same function, the `Occupied` branch |
| `FrontierStartNotPassableWhenCentreOccupied` | Centre cell itself `Occupied` → not passable | the early centre-occupancy check in `isSpherePassable` |
| `FrontierFindsPathAlongEmptyCorridor` | `exploreReachable` on a corridor finds a frontier cluster at the empty/unknown interface | the Dijkstra expansion loop, frontier-cell detection |
| `FrontierFindsFrontierInsideEmptyCube` | Cube face frontier is found without walking the whole unmapped volume (bounded `max_expansions=5000`) | expansion-cap enforcement |
| `FrontierHasUnmappedInSphereWhenUnknownExists` / `FrontierHasNoUnmappedInSphereWhenFullyKnown` | `hasNotMappedInSphere` correctness on both sides | `sphereContainsNotMapped` |
| `FrontierFindExplorePathMovesTowardUnknown` | Reachability from a corridor points toward the unknown end, not backward | same Dijkstra, sanity on cost ordering |
| `FrontierDetectsUnmappedCellsInBounds` / `FrontierNoUnmappedWhenFullyMappedEmpty` | `hasAnyNotMappedInBounds` correctness | `countUnmappedInBounds` |
| `FrontierPrefersEmptyOverUnmappedPath` | `findPathTo` chooses a longer **Empty** detour over a shorter **Unmapped** shortcut | `kEmptyTraversalCost`/`kUnmappedTraversalCost` weighting |
| `FrontierRejectsOccupiedFaceNeighbourOnCm10Grid` | Occupied face-neighbor at nearest-box-distance 5 cm blocks a 7.5 cm-radius sphere on a 10 cm grid | `sphereIntersectsCellBox`'s nearest-point-in-box math (the fix described in §2.3) |
| `FrontierAllowsOccupiedFaceNeighbourWhenRadiusTooSmall` | Same geometry, 4 cm radius → sphere doesn't reach it, still passable | same function, negative case |
| `FrontierHasUnmappedFaceNeighbourOnCm10Grid` | `hasNotMappedInSphere` uses the same box-distance geometry as passability | `sphereContainsNotMapped` sharing the geometry helper |
| `ExploreReachableFindsFrontierAdjacentCandidates` | Every returned cluster has `cell_count > 0` and `approach_cost > 0` | cluster construction in `exploreReachable`'s second BFS pass |
| `ExploreReachableRespectsExpansionCap` / `ExploreReachableTerminatesWithoutOccupancyBound` | A tiny/huge cap actually stops the Dijkstra and sets `truncated=true` | the `expansions > max_expansions` check — this is the ALG28 unbounded-BFS regression guard |
| `ExploreReachableReportsStartPassabilityWithCapOfOne` | `max_expansions=1` is a valid O(1) start-passability probe | `exploreReachable` returning early before the queue loop when start isn't passable |
| `LineOfSightBlockedByOccupiedVoxel` | `hasClearLineOfSight` sees an `Occupied` voxel placed mid-segment | `hasClearLineOfSight`'s sampled `isSpherePassable` calls |
| `MaxExpansionsCoversMapVolume` | `maxExpansionsForMap` is at least the voxel count of the mission bounds | `maxExpansionsForMap`'s per-axis `span` math |
| `FindUnstickPathStepsToAdjacentEmpty` / `FindUnstickPathBoxedInByOccupied` / `FindUnstickPathDoesNotTeleportThroughOccupied` | Unstick returns exactly one adjacent passable step, or nothing if fully boxed in, and never a 2-cell "through the wall" jump | `findUnstickPath`'s single face-neighbor loop |
| `FindPathToIsExpansionBounded` | `findPathTo` terminates (doesn't hang) even against an astronomically distant unreachable goal | `findPathTo`'s own `max_expansions` cap |
| `ExploreReachableClustersTwoRoomsSeparatedByAWall` | An `Occupied` wall correctly splits reachable space into one cluster, not two — the far pocket must not join | passability-gated Dijkstra expansion (a wall must actually block, not just get scored higher) |
| `ExploreReachableClusterCountEqualsFrontierSet` | Sum of cluster `cell_count` never exceeds total frontier cell count | cluster/`frontier_cells` bookkeeping consistency |
| `ExploreReachableApproachKeyIsLowestCostMember` | The chosen approach cell truly is on the *near* side of the pocket | the `best_cost`/`best_key` tracking inside the cluster BFS |
| `FrontierFlagIgnoresDiagonalOnlyUnmapped` | A cell whose *only* `Unmapped` neighbor is diagonal is **not** flagged as frontier itself (though its neighbors are) | the 6-face-only `unmapped` counting loop (no diagonal offsets in `kOffsets`) |
| `ExploreReachableIncludesStartWhenItBordersUnmapped` | If the start cell itself borders `Unmapped`, it can be its own cluster's approach cell | the "stay put" plan path, `self_unmapped` frontier inclusion |
| `FrontierAllowsInBoundsStartWhenSphereClipsOutOfBounds` | Spawn sitting exactly on `max_height` (sphere clips `OutOfBounds` above it) stays passable | the `OutOfBounds`-is-not-a-wall carve-out at [`MappingAlgorithmFrontier.cpp:163-166`](../Algorithm/src/MappingAlgorithmFrontier.cpp) (the house-scenario spawn fix) |
| `FrontierRejectsStartWhoseCentreIsOutOfBounds` | If the *centre* itself is out of the mission box, never passable | the early centre-occupancy check catching `OutOfBounds` directly |
| `ExploreReachableClusterCountIsEmptySurfaceNotVolume` | Cluster `cell_count` (surface) stays ≤ 27 while `volume_count` is larger | the surface-vs-volume split in the cluster-building loop ([`MappingAlgorithmFrontier.cpp:562-587`](../Algorithm/src/MappingAlgorithmFrontier.cpp)) |
| `ExploreReachableClusteringIsDeterministic` | Two identical calls produce identical cluster ordering/costs/keys | `std::sort` stability and the `unordered_map` traversal not leaking nondeterminism into results |

### 3.3 [`test_wavefront_planner.cpp`](../Algorithm/tests/test_wavefront_planner.cpp) — `WavefrontPlanner::plan`

| Test | What it checks | Breaks if you touch… |
|---|---|---|
| `PrefersDistantRoomOverNearbyCrumb` | A big distant room outranks a tiny nearby crumb by `expected_rate` | the rate formula / cluster-score comparator in `plan` |
| `DescendsFromCeilingWhenUnmappedIsBelow` | Generic (non-house) top-of-map start descends when the column below is unknown | `unmappedInColumnBelow` + the "stay put" per-cluster descent branch |
| `DescendsThroughHouseWhenUnmappedIsBelowMidLayer` / `DescendsThroughHouseEmptyColumnTowardUnmapped` | House-mission mid-layer / empty-column descent | `isHouseVolumeMission` classification + the same descent branch |
| `HouseStaysToScanWhenHorizontalUnmappedRemains` | House mission with horizontal `Unmapped` on the *current* layer stays and scans instead of descending | the `house_layer_done`/`hasHorizontalUnmapped` gate |
| `HouseDropsOnPreferDescendDespiteHorizontalUnmapped` | Passing `prefer_descend=true` overrides the horizontal-unmapped gate and forces descent | the top-level `in.prefer_descend` branch ([`WavefrontPlanner.cpp:118-134`](../Algorithm/src/WavefrontPlanner.cpp)), distinct from the per-cluster branch above |
| `RoomDoesNotForceDescendWhenUnmappedIsBelow` | A generic small room (not house-shaped) does **not** force-descend | `isHouseVolumeMission`/`isOpenVolumeMission` both returning false correctly excluding the forced-descent paths |
| `StaysPutWhenStartIsTheCheapestFrontier` | Start cell itself is the best frontier → `valid=true`, empty `waypoints` | the `approach == reach.start_key` "stay put" branch |
| `DiscardsClustersTheRemainingBudgetCannotAfford` | A tight budget (2 steps) rejects the same cluster a generous budget (1000) accepts | the `travel + reserve > in.remaining_steps` filter |
| `IsDeterministicAcrossIdenticalCalls` | Two identical `plan()` calls produce byte-identical waypoints/rate/cluster size | overall determinism of sorting/tie-breaking throughout `plan` |
| `UnsticksWhenStartSphereHitsOccupiedFloor` | A boxed-in start (occupied floor) still produces a valid escape plan with `expected_rate >= 0.25` | the `!reach.start_passable` → `findUnstickPath` → synthetic `escape` plan branch |
| `ReportsInvalidWhenNothingIsUnresolved` | Fully-known map → `plan.valid == false` | `reach.clusters.empty()` early return |
| `ClearsStaleAlternatesOnEarlyReturn` | A pre-populated `alternates` vector is cleared even when `plan()` returns early/invalid | the `alternates->clear()` at the very top of `plan()` |
| `IgnoreBlockedRecoversWhenTheBlockedSetSealsTheDrone` | With the blocklist covering every neighbor, `ignore_blocked=false` fails but `true` recovers | the `in.ignore_blocked ? empty_blocked : in.blocked` selection |
| `AlternatesExcludeBestAndAreSortedByRate` | `alternates` never contains the winning plan and is itself rate-sorted descending | the `candidates.begin() + 1` slice after sorting |
| `AlternatesEmptyWithOnlyOneCluster` | Single-cluster map → `alternates` is empty | same slice, degenerate case |
| `AlternatesOmitClustersTheRemainingBudgetCannotAfford` | Alternates never include an unaffordable cluster even under a tight budget | the same budget filter applied before alternates are populated (candidates are filtered, then sorted, then sliced) |

### 3.4 [`test_path_shaping.cpp`](../Algorithm/tests/test_path_shaping.cpp) — `PathShaping`

| Test | What it checks | Breaks if you touch… |
|---|---|---|
| `StringPullCollapsesStaircaseAtConstantAltitude` | A same-altitude staircase collapses to fewer waypoints | the LOS-extend loop in `stringPullConstantAltitude` |
| `StringPullDoesNotCutThroughOccupiedVoxel` | An `Occupied` voxel on the diagonal shortcut prevents collapsing across it | `hasClearLineOfSight` gating the extend loop |
| `StringPullKeepsAltitudeChangesSplit` | A waypoint where Z changes always survives as its own waypoint | the `sameAltitude` check breaking the mergeable run |
| `StepCostChargesTwoRotationsForRightAngleOnLargeDrone` | 90° turn at `max_rotate=45°` costs 2 rotate-steps + 1 advance | `ceilDiv(turn, rotate_deg)` in `stepCostForPath` |
| `StepCostChargesOneRotationForRightAngleOnSmallDrone` | Same turn at `max_rotate=90°` costs only 1 rotate-step | same `ceilDiv` call, different divisor |
| `StepCostChargesElevationSeparately` | A pure-Z waypoint charges only elevate-steps, no rotate/advance | the `dz`/`planar` branches being mutually exclusive per waypoint |
| `StepCostOfEmptyPathIsZero` | Empty waypoint list → 0 steps | the `for` loop over `waypoints` simply not running |

### 3.5 [`test_scan_planning.cpp`](../Algorithm/tests/test_scan_planning.cpp) — `ScanPlanning`

| Test | What it checks | Breaks if you touch… |
|---|---|---|
| `MaskRejectsUnmappedBehindOccupied` | A cell 2 steps from any frontier cell is *not* gain-masked; a face-adjacent one is | `isGainMasked`'s self + 6-face check |
| `SweepIgnoresUnmappedBeyondFrontierShell` | On a generic (non-outdoor) mission, `buildSweepDirections` finds nothing when the only `Unmapped` voxel is outside the frontier mask | `isGainMasked` gating `countConeGain` when `open_volume == false` |
| `SweepCountsUnmappedVolumeOnOpenSkyMission` / `...OnSmallOutdoorEvenWithLongLidar` | Outdoor-cube missions count volume gain even beyond the frontier shell | `volumeGainAllowed` / `isOpenVolumeMission` / `isSmallOutdoorMission` |
| `SweepDoesNotVolumeCarveDownwardOnOpenSkyMission` | Downward directions never get volume-gain credit even outdoors | `pointingDown` gating inside `volumeGainAllowed` |
| `SweepIgnoresHorizontalVolumeOnLongLidarOpenSky` | A large open-sky mission (not "small outdoor") with a long lidar does **not** get horizontal volume credit | the `lidar.z_max <= 90.0` condition in `volumeGainAllowed` |
| `SweepRejectsDownwardOnHouseMissionEvenMidLayer` / `SweepRejectsDownwardWhenOriginIsOnMaxHeight` | Downward scans are always excluded on a house mission, and near the ceiling generically | `skipDownwardScan`'s two branches (`isHouseVolumeMission` unconditional; height-vs-`z_min` for the generic case) |
| `SweepOrdersByIndependentGainNotEnumerationOrder` | The direction with the actual `Unmapped` gain ranks first even if it's not first in the template array | the gain-descending `std::stable_sort` in `buildSweepDirections` |
| `MarginalPassDropsFullyClaimedDirections` | The output is smaller than the full template count once already-claimed voxels are excluded | the second "marginal" walk that re-checks each direction against the shared `stamp` |
| `TravelScanRejectsDirectionWithOccupiedNearField` | A direction whose near field already has an `Occupied` voxel is never chosen, even if other directions would find nothing | the `nearFieldContainsSolid` guard in `bestTravelScan` |

### 3.6 [`test_lidar_cone.cpp`](../Algorithm/tests/test_lidar_cone.cpp) / [`test_cone_template.cpp`](../Algorithm/tests/test_cone_template.cpp) — UserCommon geometry consumed by Algorithm

These test [`LidarCone.h`](../UserCommon/include/user_common_207190406_209543255/LidarCone.h) / [`ConeTemplate.h`](../UserCommon/include/user_common_207190406_209543255/ConeTemplate.h),
not files inside [`Algorithm/`](../Algorithm/), but they're colocated here because `WavefrontPlanner` and
`MappingAlgorithmImpl` are the only consumers of the cone-geometry cache
(`ConeTemplateCache`) that every scan-direction computation in `ScanPlanning` depends on.

| Test | What it checks |
|---|---|
| `HalfAngleMatchesMockLidarShortAndLong` | `coneHalfAngleRad` matches the same formula `MockLidar` uses (`atan2((fov_circles-1)*d, z_min)`) — if these ever diverge, the algorithm's scan-gain estimate stops matching what the real sensor actually sees |
| `DirectionCountsDifferBetweenShortAndLong` | Direction count is derived from angular spacing (`directionCountForHalfAngle`, floor 6/cap 64) — not a hardcoded 26 |
| `FibonacciSizeAndDeterminism` | `fibonacciSphereOrientations` returns exactly N deterministic directions, with axis-aligned directions first |
| `ConeCoversUnresolvedWhenUnmappedAlongBeam` / `ConeDoesNotCoverWhenFullyResolved` | `coneCoversUnresolved` trig-walk sanity: true only when `Unmapped` actually sits inside the cone |
| `CountsUnresolvedVoxelsAndDeduplicatesAcrossCalls` | `countUnresolvedVoxels` dedupes across a shared `seen` set |
| `CountStopsAtOccludingVoxel` | An `Occupied` voxel stops the beam walk before reaching the `Unmapped` voxel behind it |
| `PotentiallyOccupiedCountsAsResolved` | `PotentiallyOccupied` voxels don't count as unresolved gain |
| `NearFieldInsideZMinIsCounted` | A voxel closer than `z_min` still counts (our fusion carves from distance 0, unlike a physical sensor's blind spot) |
| `WalkMatchesTrigVoxelSetOnAlignedOrigin` / `...OnNonAlignedOrigin` | **`ConeTemplate`'s precomputed integer walk visits exactly the same voxels as the naive trig walk** — the equivalence the whole `ConeTemplateCache` optimization depends on |
| `WalkStopsAtOccupied` | Template walk also respects occlusion, matching the trig walk |
| `StampDeduplicatesAcrossBeamsAndGenerations` | `VoxelStamp` correctly dedupes within one `begin()` generation and resets on the next |
| `NearFieldSamplesCoverExactlyInsideZMin` | Precomputed near-field sample count matches the analytic `step`-spaced count under `z_min` |
| `CacheReturnsSameTemplatesForSameLidarAndResolution` | `ConeTemplateCache::get` returns the *same* cached vector (by address) for repeated identical `(lidar, resolution)` keys — the caching contract `ScanPlanning`/`WavefrontPlanner` rely on to avoid recomputing geometry every tick |
| `NearFieldContainsSolidWhenOccupiedInsideZMin` | `nearFieldContainsSolid` — the guard `bestTravelScan` uses to reject a direction already blocked close-up |

---

## Appendix — quick answers for "break this test"

- **Want a frontier/clustering test to fail?** Touch [`MappingAlgorithmFrontier.cpp`](../Algorithm/src/MappingAlgorithmFrontier.cpp):
  `isSpherePassable`, `sphereIntersectsCellBox`, the `kOffsets` 6-face list, the
  frontier-flagging condition (`unmapped > 0 || self_unmapped`), or the traversal costs
  `kEmptyTraversalCost`/`kUnmappedTraversalCost`.
- **Want a planner/ranking test to fail?** Touch [`WavefrontPlanner.cpp`](../Algorithm/src/WavefrontPlanner.cpp): the
  `cluster_score` selector, the budget filter (`travel + reserve > in.remaining_steps`),
  the rate formula, or the house/outdoor descent branches.
- **Want a path-cost test to fail?** Touch [`PathShaping.cpp`](../Algorithm/src/PathShaping.cpp): `ceilDiv`, the
  elevate/rotate/advance ordering in `stepCostForPath`, or the altitude-equality epsilon
  in `stringPullConstantAltitude`.
- **Want a scan test to fail?** Touch [`ScanPlanning.cpp`](../Algorithm/src/ScanPlanning.cpp): `isGainMasked`,
  `volumeGainAllowed`, `skipDownwardScan`, or the mission-shape thresholds
  (`isOpenVolumeMission`/`isSmallOutdoorMission`/`isHouseVolumeMission`).
- **Want an end-to-end `nextStep` test to fail?** Touch [`MappingAlgorithmImpl.cpp`](../Algorithm/src/MappingAlgorithmImpl.cpp): any of
  the 9 named constants at the bottom of [`MappingAlgorithmImpl.h`](../Algorithm/include/Algorithm/MappingAlgorithmImpl.h), the stall detection
  (`samePosition`/`kMaxMovingStallTicks`), the replan trigger conditions
  (`plan_exhausted`/`interval_elapsed`/`cluster_dead`), or the two termination checks
  (predicted-rate streak vs. observed-drop streak).
- After any change, rebuild and re-run just the affected filter:
  `cmake --build --preset default && ./build/default/Algorithm/algorithm_test --gtest_filter='*Name*'`.
