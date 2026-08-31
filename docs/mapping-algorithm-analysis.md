# Mapping Algorithm Analysis

Reference analysis of `Algorithm/` for Assignment 3 — purpose, algorithmic soundness,
implementation quality, and alternatives. Written 2026-08-29 from a read-only review of
the assignment docx, `MappingAlgorithmImpl`, `MappingAlgorithmFrontier`, surrounding
contracts (`IMappingAlgorithm`, `DroneControlImpl`, `MockLidar`, `MapsComparison`), and
shipped `inputs/`. Cleaned up the scratch docx extract afterwards.

**Status at time of writing:** Default composition `inputs/sim_compose.yaml` scores 24/24
`COMPLETED` with `mission_score >= 0` under our MissionControl. Competitive performance
and foreign-host step counts are weaker — see Known Issues #20.

---

## Bottom line

The **idea is the right one** — frontier-based exploration with a cost-scored path to the
best frontier is the textbook approach for "map an unknown volume," and it comfortably
clears the assignment's stated minimum ("don't fly into walls, map the surroundings, try to
be efficient and exact"). But it is a **weak competitor**, and the reason is not the
frontier idea — it's the **action policy** layered on top: the algorithm spends roughly
26 of every ~28 `nextStep` calls scanning, never uses the movement-and-scan-in-one-step
capability the API offers, and never reads `max_steps`. The implementation has one
structural no-op, several full-graph sweeps per planning cycle, and two irreversible
blacklists.

---

## What the assignment actually measures

The docx gives the algorithm three lines of requirement (don't hit walls, map the bounds,
be efficient and exact), plus two things that matter more than they look:

1. **"Each part above may run independently with another team's implementation of the other
   parts."** In `-competition` mode the MissionControl is fixed and the algorithms vary —
   so our algorithm will be graded under someone else's MissionControl.

2. **"There would be a class competition and a bonus would be given for the best
   algorithms."** The competitive report sorts by `total_score` descending, then
   `total_steps` ascending. Coverage is primary; steps are the tiebreaker.

Scoring uses `MapsComparison::compare` — 0–100 BFS/reachability-weighted voxel agreement
against the hidden map, seeded at the drone spawn. Pass 2 then counts **target-known**
voxels **outside** that spawn-reachable set and credits only `Empty`. Resolving wall
interiors or painting `PotentiallyOccupied` there increases the denominator without a
matching credit, so it **lowers** the score.

---

## Current algorithm — purpose and shape

### High-level loop

`MappingAlgorithmImpl` runs a phase machine:

| Phase | Behavior |
|-------|----------|
| **Scanning** | Emit up to 26 scan orientations (6 face + 12 edge + 8 corner directions), then transition to Planning |
| **Planning** | Delegate to frontier cleanup (BFS/Dijkstra-style search) |
| **Moving** | Follow compressed path waypoints; optionally interrupt for mid-travel scans every `spacing_cells` steps |

Each `nextStep` returns **at most one** scan orientation **or** one movement command — never both.

### Frontier planner (`MappingAlgorithmFrontier`)

Ported from ex1 `ExplorationFrontier`, adapted for `IMap3D`:

- **Passability:** Treat `Unmapped` as traversable (soft cost 4 vs 1 for `Empty`); only
  confirmed `Occupied` / `OutOfBounds` block navigation. Stall detection handles cells that
  turn out solid at execution time.
- **Frontier definition:** A cell adjacent to `Unmapped`, or an `Empty` neighbour whose
  sphere contains unmapped voxels.
- **Primary search (`findPath`):** Dijkstra over the passable component; score each
  reachable frontier by **information density** = unmapped neighbours in a radius-2 sphere /
  path cost. Pick the best; tie-break on lower cost.
- **Fallbacks when no frontier:** `findExplorePath` (descend unknown-distance field),
  `findUnstickPath`, `findAnyPassableNeighbor`, then give up with
  `FinishedWithUnmappableVoxels`.
- **Termination guards:** Stop after 100 planning cycles with no decrease in unmapped count,
  or 100 cycles with no reachable frontier but unknown cells remain.

### What works well (algorithmically)

Genuinely good choices in the current design:

1. **Treating `Unmapped` as soft-cost traversable** (cost 4 vs 1 for `Empty`,
   `MappingAlgorithmFrontier.cpp:27-28`) so the planner explores instead of refusing to move.

2. **Scoring frontiers by information density** (value / path_cost, line 407) rather than
   pure nearest-frontier.

3. **Compressing collinear path runs** so one `Advance` covers several voxels.

4. **Stall → blocked-cell blacklist** — when movement doesn't change position for 2 ticks,
   mark the waypoint cell blocked and replan.

5. **Scan-during-travel** — periodic 26-scan batches while moving long paths (maps walls
   before walking into them on high `max_steps` missions).

---

## The idea: sound, with three policy gaps

Three gaps are in the idea itself, not the code:

### 1. It never combines movement and scanning in one step

`MappingStepCommand` has both `movement` and `scan_orientation` as independent optionals,
and the canonical drone-control order is movement → scan → fuse. Our own foreign-host
fixture does exactly that (`foreign_hits_only_mission_control_plugin.cpp:105-111). Every
handler in `MappingAlgorithmImpl.cpp` returns one or the other, never both — so on any
host that honours both, we pay **two steps** for what the API lets you do in one.

### 2. The 26-direction sphere sweep isn't derived from the sensor

**Historical finding:** `buildScanOrientations` hardcoded 6 face + 12 edge + 8 corner
directions regardless of the lidar. Cone half-angle from MockLidar/HostLidar is
`atan2((fov_circles-1)*d, z_min)` — ≈20.6° for `lidar_short`, ≈14° for `lidar_long`.

**Resolution (project C, 2026-08-29):** Directions are derived from that half-angle
(Fibonacci lattice, mandatory ± axes, count from cone spacing), and scans whose cones
are already fully resolved in `output_map_` are gain-gated. Shared helpers live in
`UserCommon/` (`LidarCone.h`, `BeamMath.h`). See
`docs/superpowers/specs/2026-08-29-sensor-model-clearance-belief-design.md`.

### 3. It's blind to its own step budget

`mission_config_` is never read anywhere in `Algorithm/`, so `max_steps` is invisible. The
shipped compositions span 500, 1000, and 10000 steps, and the algorithm behaves identically
in all three. An anytime exploration policy should be thorough when the budget is loose and
greedy about gain-per-step when it's tight.

---

## The portability risk (resolved for step accounting — 2026-08-29)

**Historical finding (at time of writing):** Our own `DroneControlImpl::step()` batched up to 16
consecutive scans into a single mission step (`kMaxScansPerStep=16`). A foreign MissionControl does
one scan per step — VAR-02 ("Batch consecutive scan commands into one step | Exactly one scan per
step"). Known Issues #20 recorded **209 steps vs 42** on `small_simulation_room` under a hits-only
foreign MC (~5×), largely from that asymmetry.

**Resolution (project B):** The batching loop was removed. `step()` now calls `nextStep` once and
runs movement → at most one scan → fuse, matching the documented contract and the foreign scan
rate. Remaining foreign-MC gaps are Empty-carving and `latest_scan` policy, not scan batching.
See `docs/superpowers/specs/2026-08-29-missioncontrol-step-honesty-design.md`.

Foreign MC may also:

- Pass `latest_scan = nullptr` every call (algorithm ignores scan data anyway).
- Never carve `Empty` along beams (planner's soft-cost `Unmapped` traversal is the main
  way to explore).

---

## Implementation findings, by impact

### The drone-radius clearance test is a no-op with the shipped inputs

**Historical finding:** `isSpherePassable` used a centre-distance gate
`ox²+oy²+oz² > radius²` after `rx = ceil(radius/step)`. With radius 4–7.5 cm and step
10 cm, every non-zero offset failed the gate — the sphere sweep reduced to a single
`atVoxel`. Same bug in `sphereContainsNotMapped` / `isFrontier`.

**Consequence (at time of finding):** planner acted as a point drone; foreign MC hard
`Error` on blocked moves could score **-1**.

**Resolution (project C, 2026-08-29):** Keep `ceil` loop bounds (needed for 1 cm test
grids); replace the gate with nearest-point-in-voxel-box vs sphere. Face neighbours on
the shipped 10 cm / 7.5 cm case are now rejected when Occupied. Soft-cost `Unmapped`
traversal is unchanged. Spec:
`docs/superpowers/specs/2026-08-29-sensor-model-clearance-belief-design.md`.

### Every planning cycle pays several full-graph sweeps

`findPath` runs Dijkstra to exhaustion — it drains the whole reachable component because
the density score can't be compared until then — and at each popped node calls `isFrontier`
plus a 33-cell information sphere.

`handleFrontierCleanupPhase` additionally calls `countUnmappedInBounds` (a complete bounds
sweep — 14,880 voxels on `large_mission_out`) purely to detect stagnation, and on the stuck
path calls `diagnose()` (full BFS plus a rebuilt unknown-distance field) and then
`findExplorePath` (which builds that field again).

Three to four O(V) sweeps in one `nextStep`, and `nextStep` is called more than once per
step because of the batching loop.

### A cache that goes stale exactly when it's used

`explore_dist_cache` is cleared only when `findPath` succeeds
(`MappingAlgorithmImpl.cpp:399`). It's populated and reused on the stuck path — where the
drone keeps moving and scanning across many cycles — so the unknown-distance field it steers
by can be arbitrarily out of date. It needs a map-version counter or an unconditional
recompute.

### Dijkstra's optimality assumption is broken mid-search

```cpp
if (best_score >= 0.0 && occupancyAt(map, neighbour_pt) == types::VoxelOccupancy::Unmapped) {
    continue;
}
```

(`MappingAlgorithmFrontier.cpp:428`) changes the edge set during the search depending on
whether a frontier has been found yet. The result is expansion-order dependent, so the
returned path isn't a shortest path — it's a heuristic wearing Dijkstra's clothes. It works,
but no reviewer can tell what it computes.

### Two blacklists that only grow

- **`blocked_cells`** gets a cell inserted after 2 stall ticks and never releases it, even
  after a later scan proves it `Empty` — on a long mission this monotonically shrinks the
  search graph and caps achievable coverage.

- **`frontier_visit_counts`** permanently retires a frontier after 3 visits (5 near the
  floor) regardless of how much unknown space still borders it.

### Policy hidden in constants

`kMaxFrontierVisits=3`, `kMaxNearFloorFrontierVisits=5`, `kNoProgressLimit=100`,
`kNoFrontierStuckLimit=100`, `kFrontierValueRadius=2`, `kUnmappedTraversalCost=4`,
`kMaxMovingStallTicks=2`, `spacing_cells = clamp(z_max/step/2, 2, 15)`.

They're named, so e23 is satisfied, but they encode the entire exploration policy and not
one is derived from the configs. The near-floor special case in particular reads as a fix
for a specific failing scenario.

### Dead code in a private helper

`findFarthestPath` and `findGreedyUnknownStep` have no callers anywhere, including tests;
`findPathTo` is called only from `test_mapping_algorithm_frontier.cpp:268`. That's roughly
120 lines — an e08 (lean API) exposure that also makes the real code path harder to find.

### Raw doubles for all geometry

`force_numerical_value_in(cm)` appears throughout both files; headings, path compression,
sphere probes, and the distance field are all computed in plain `double` and re-wrapped at
the end. `.cursor/rules/mp-units-strong-types.mdc` flags exactly this as e03/e16 and points
at the `Position3D` `operator+`/`operator-` added in ex3. This is the largest single
rubric exposure in `Algorithm/`.

### Minor structure

The four phase handlers are mutually recursive (Scanning → Planning → Cleanup → Moving →
Scanning), there are four handlers but a three-value `Phase` enum, `handlePlanningPhase` is
a one-line forwarder, and `handleFrontierCleanupPhase` keeps a name from a two-stage design
that no longer exists. Also `(void)latest_scan;` should be `[[maybe_unused]]`.

---

## Better known algorithms for this use case

Roughly in order of benefit per unit of effort before Sep 6:

### 1. Emit movement and scan in the same command

Spec-supported, no new algorithm. Caveat: our own MC's batching loop currently discards the
movement when a command carries both (it overwrites `command` with the next `nextStep`), so
this needs the MC loop fixed too — but it's a large win on any host that applies both.

### 2. Gate scan directions on information gain

Derive the cone half-angle from `d`/`z_min`/`fov_circles`, lay directions out on a spherical
Fibonacci lattice sized to that angle, and emit only cones whose solid angle actually
contains `Unmapped` voxels. This kills the "re-scan the ceiling I mapped four stops ago"
steps, which is where most of the step budget goes.

### 3. Promote the multi-source distance field to the primary mechanism — **adopted by project F**

`buildUnknownDistanceField` is already a multi-source BFS from every unknown cell — that's
Wavefront Frontier Detection (Keidar & Kaminka). At the time of this review it was
relegated to the fallback path while a full Dijkstra did the main work.
**Project F** (`docs/superpowers/specs/2026-08-31-wavefront-frontier-exploration-design.md`)
makes WFD the primary exploration policy: cluster reachable frontiers, rank by cells per
step, and scan toward the chosen cluster. Post-F measurement then showed three `house_full`
cells at score 0.06 (start sphere vs mission AABB / Occupied floor, and a ceiling trap).
The score-aware nav follow-up is recorded in `docs/benchmarks/2026-08-31-score_aware_nav.md`.
Outdoor Empty-carve (gated volume gain, no global unmask) is
`docs/benchmarks/2026-08-31-outdoor-empty-carve.md` (honest sum 1793.4).

### 4. Any-angle path smoothing

The BFS is 6-connected, so paths are Manhattan staircases, and since `Rotate` / `Advance` /
`Elevate` are separate commands every turn costs an extra step. Theta* / lazy Theta* (Nash
et al.), or just line-of-sight string-pulling over the existing path, lets one `Advance`
cover a diagonal run.

### 5. Incremental replanning — D* Lite / LPA* (Koenig & Likhachev)

The graph barely changes between cycles; re-searching from scratch each time is precisely
what these exist to avoid. Worth it only if the distance-field change above isn't enough.

### 6. Frontier clustering with tour ordering — FUEL (Zhou et al. 2021) / TARE (Cao et al. 2021)

FUEL keeps an incremental frontier information structure and solves an ATSP over frontier
clusters; TARE splits into a coarse global tour plus fine local planning. Either eliminates
the revisit oscillation by construction instead of blacklisting it. This is the right answer
for a competition metric, but it's a rewrite.

### 7. Receding-horizon Next-Best-View (Bircher et al. 2016)

Sample candidate (position, orientation) viewpoints, score each by expected newly-observed
voxels under the real sensor cone minus travel cost, execute the first edge, re-plan.
Directly optimizes coverage-per-step — literally the competitive metric — and subsumes the
scan-direction problem into the same objective.

### 8. Build our own belief map from `latest_scan`

Raycasting the hits ourselves instead of relying on the host's fusion fixes Known Issues #20
at the root, decouples us from a foreign MC's carving policy, and makes "is this move into
unknown space safe?" answerable locally. It needs the beam math moved from
`MissionControl/src/ScanResultToVoxels` into `UserCommon/` so both projects share one copy
— which is also what the no-duplicate-logic rule (e10) wants.

---

## Recommended pre-deadline stack

Given the deadline, my read is: **don't rewrite.** Take the move-plus-scan combination and
the gain-gated scan directions, then make the clearance check actually clear (or state the
point-drone assumption honestly in the HLD), then fix the cache invalidation and give the
two blacklists an expiry. FUEL/NBV are a different timeline.

| Approach | Benefit | Effort |
|----------|---------|--------|
| **Emit movement + scan in same command** | ~2× fewer steps on foreign hosts; API-supported | Low (may need MC loop fix if batching drops movement) |
| **Gain-gated scan directions** | Fewer redundant scans; cone from `d`/`z_min`/`fov_circles` | Low–medium |
| **Multi-source unknown-distance field as primary** | One O(V) pass vs per-node `isFrontier`; already partially implemented | Medium |
| **Any-angle path smoothing (Theta*, string-pulling)** | Fewer rotate/advance steps on Manhattan paths | Medium |
| **Incremental replanning (D* Lite / LPA*)** | Avoid full re-search each cycle | Medium–high |
| **Frontier clustering + tour (FUEL, TARE)** | Eliminates revisit oscillation by construction | High (rewrite) |
| **Receding-horizon NBV (Bircher et al.)** | Optimizes coverage-per-step directly | High |
| **Own belief map from `latest_scan`** | Fixes #20 root cause; decouples from foreign MC fusion | Medium (beam math → `UserCommon/`) |

---

## Key file references

| File | Role |
|------|------|
| `Algorithm/include/Algorithm/MappingAlgorithmImpl.h` | Phase machine, public API |
| `Algorithm/src/MappingAlgorithmImpl.cpp` | 26-scan batch, planning orchestration, movement |
| `Algorithm/src/MappingAlgorithmFrontier.cpp` | BFS/Dijkstra frontier search, passability, distance field |
| `MissionControl/src/DroneControlImpl.cpp` | Scan batching (`kMaxScansPerStep=16`), movement execution |
| `MissionControl/tests/foreign_hits_only_mission_control_plugin.cpp` | Foreign-host fixture: one scan per step, movement → scan → fuse |
| `Simulator/src/MapsComparison.cpp` | 0–100 mission score |
| `Simulator/src/MockLidar.cpp` | Cone geometry from `fov_circles`, `d`, `z_min` |
| `Simulator/src/SimulationRunFactoryImpl.cpp` | Output map resolution from mission config |
| `docs/known-issues.md` | #20 (foreign MC steps), #21 (scan-batch hang, resolved) |
| `docs/superpowers/specs/2026-08-28-independent-component-variants-design.md` | VAR-02 foreign MC contract |

---

## Related docs

- `docs/ex2-grading-handoff.md` — ex2 ALG28 unbounded-BFS hang lesson
- `docs/superpowers/specs/2026-08-27-wall-collision-recovery-and-planner-design.md` — recent
  planner/recovery fixes
- `docs/assignment3-checklist.md` — condensed assignment requirements
- `docs/open-questions.md` — scoring/grouping ambiguities
