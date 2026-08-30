# Wavefront Frontier Exploration on D's Executor — Design

**Date:** 2026-08-31
**Status:** Accepted 2026-08-31
**Goal:** Keep project D's plan-execution architecture and replace its candidate-scoring policy with
Wavefront Frontier Detection over the reachability sweep, so a replan costs a bounded multiple of one
O(V) pass instead of ~1.6 million cone-raycast map lookups, and so the objective the policy maximises
is the quantity `MapsComparison` actually rewards.

This is project **F**. It supersedes the scoring and candidate-selection halves of
`docs/superpowers/specs/2026-08-29-exploration-policy-nbv-design.md` (project D) and keeps everything
else D built. See `docs/superpowers/specs/2026-08-29-mapping-algorithm-roadmap.md` for the A–E
sequence and the two standing decisions (honest-step accounting as the optimisation target;
beat-ex2 as the success bar).

---

## Problem

D shipped and is measurably worse than the baseline it was meant to beat, on both axes.

### Measured, optimised builds (`-O2`, `CMAKE_BUILD_TYPE=Release`)

| Cell | post-C (`f825b40`) score / wall | D (`8f79eb5`) score / wall |
|------|-------------------------------|----------------------------|
| `house_lower` | 100.0 / ~0 s | 100.0 / ~0 s |
| `small_room` | 82.57 / 2 s (COMPLETED) | 76.37 / 4 s (MAX_STEPS) |
| `large_room` | 92.65 / ~0 s | 93.03 / 7 s (MAX_STEPS) |
| `small_out` | 25.16 / 1 s | 83.49 / 38 s (MAX_STEPS) |
| `house_full` | 6.30 / ~0 s | — / **>120 s, killed** |
| `large_out` | 16.80 / 1 s | — / **>120 s, killed** |

D wins enormously where it finishes (`small_out`: 25 → 83) and cannot be measured at all on the two
largest cells. All earlier D measurements were taken from unoptimised `-O0` builds, so the runtime
problem was understated until this sweep; `-O2` bought roughly 8× and was still not enough.

There are four distinct causes, and they are independent of each other.

### 1. The cost model in D's spec was wrong by two orders of magnitude

D's design bounded a replan at "roughly `16 × directions × (z_max / ray_step)` map lookups — a few
tens of thousands." That treated each scan direction as a single ray. The implementation casts the
whole physical cone per direction. For `lidar_short` (`fov_circles = 4`), `detail::beamsOnCircle` is
`4^circle`, so a cone is `1 + 4 + 16 + 64 = 85` beams; `walkBeam` samples every `resolution/2` = 5 cm
out to `z_max` = 80 cm, i.e. 16 samples per beam. That is **1360 `atVoxel` calls per cone**, and
`directionCountForHalfAngle` yields ~34 directions, so one `gainAt` is ~46,000 lookups.

`NbvPlanner::plan` calls `gainAt` on the current pose plus up to `kScoredCandidates = 16` viewpoints,
and then calls `uniqueGainOf` on each winner's prefix — two full passes per candidate:

```
17 candidates × 2 passes × 46,000 lookups ≈ 1.6 million lookups per replan
```

Each of those lookups is expensive, not cheap. `forEachConeBeam` computes an orthonormal basis and a
`cos`/`sin` pair per beam; `walkBeam` calls `bm::pointAlongBeam` — more trig — at every sample; and
`countUnresolvedVoxels` hashes an `int64_t` voxel key into an `unordered_set` for every `Unmapped`
sample. `IMap3D::atVoxel` is itself a virtual call over coordinate arithmetic.

The reachability sweep adds its own load. `exploreReachable` probes 26 neighbours per popped cell for
the `unmapped_neighbours` prefilter, and calls `isSpherePassable` on each of the 6 face neighbours —
27 probes each at radius 7.5 cm on a 10 cm grid — so a popped cell costs ~188 lookups before any
scoring. Over a ~5,000-cell reachable component that is ~940,000 lookups per replan on its own, and
every cell is probed for passability up to six times because the result is never memoised.

With `kReplanIntervalSteps = 10` and `max_steps = 10000`, `house_full` pays ~1000 replans of ~2.5
million lookups: ~2.5 billion. That is the 120-second wall.

### 2. The prefilter cannot see distant rooms

`plan` ranks candidates by `unmapped_neighbours / dijkstra_cost` and keeps 16. `unmapped_neighbours`
is a count over the candidate's own 26-neighbourhood, capped at 26. A cell one step away with three
unmapped neighbours scores 3.0; the doorway into an entirely unmapped 400-voxel room forty steps away
scores at most 26/40 = 0.65. Nearby crumbs win, always, and the expensive full raycast is then spent
on sixteen crumbs. The gain function that was supposed to fix this never gets to see the good
candidate, because the prefilter discarded it first.

### 3. The objective is not the score

`Simulator/src/MapsComparison.cpp` scores in two passes over the output map's region.

Pass 1 (lines 173-199) walks reference-known voxels, restricted by `reachable_set` — a BFS through
reference-`Empty` cells from the spawn, plus the `Occupied`/`PotentiallyOccupied` cells adjacent to
them. Every such voxel increments `total`, and increments `correct` only on an exact match. Crucially
`total` is incremented **whatever our map says**, so inside the reachable set an `Unmapped` voxel and
a wrong `PotentiallyOccupied` voxel score identically: both are already in the denominator.

Pass 2 (lines 203-225) walks the remainder — voxels outside `reachable_set`, and voxels the reference
does not know — and counts a voxel **only if our map calls it known**, crediting it only when we
call it `Empty`. So for every voxel outside the spawn-reachable set:

| Our belief | Effect on score |
|------------|-----------------|
| `Unmapped` | not counted — free |
| `Empty` | counted, correct |
| `Occupied` / `PotentiallyOccupied` | counted, **wrong** |

Resolving voxels outside the reachable set is therefore score-negative unless they resolve `Empty`.
Wall interiors, sealed-off volumes, and anything the reference maps behind a closed surface all fall
into pass 2, and beams that terminate inside them, or `PotentiallyOccupied` painted from 0 to `z_min`
by a zero-distance hit (`MissionControl/src/ScanResultToVoxels.cpp:81-89`), put wrong answers into a
denominator a drone that never went there gets for free.

D's gain function counts *every* `Unmapped` voxel a cone would touch, out to `z_max`, with no notion
of which side of the reachable boundary it lies on. It therefore rewards exactly the scanning that
pass 2 punishes. This is the most likely mechanism behind `small_room` regressing from 82.57
(COMPLETED at post-C) to 76.37 (MAX_STEPS at D): D flew far more, scanned on every step, and some of
that resolving cost score.

The corollary is the good news in this design: the score-relevant unknown voxels are the ones
reachable through non-`Occupied` belief space — which is precisely the frontier set the wavefront
sweep already produces. **Frontier cluster size is a closer proxy to the score than cone-cast gain
is.** The cheaper policy is also the more correct one.

### 4. Best-first scan order steals credit from itself

`gainAt` walks directions in Fibonacci order with one shared `seen` set, records each direction's
*marginal* contribution under that order, then sorts by it. Direction 30's recorded gain is whatever
was left after directions 1–29 claimed their voxels, so the "best first" ordering is an artefact of
the enumeration order, not of the geometry. `uniqueGainOf` then re-walks the sorted prefix with a
fresh set, so the prefix total is right but the *choice* of prefix is not.

---

## Decisions taken before the design

1. **D's executor, path shaping, and reachability substrate are kept.** The plan/replan/stall/TTL/
   budget structure in `MappingAlgorithmImpl`, `PathShaping`'s `stringPullConstantAltitude` and
   `stepCostForPath`, and `MappingAlgorithmFrontier`'s Dijkstra, `hasClearLineOfSight`, and ALG28
   expansion bound all stay. `NbvPlanner`'s candidate enumeration and scoring are replaced.
2. **`Unmapped` stays soft-cost traversable; the safety invariant remains "never command a move whose
   drone-sphere footprint contains `Occupied` or `OutOfBounds`."** Unchanged from D decision 1, and
   unchanged for the same physical reason.
3. **Score-awareness is in scope for this project**, not deferred. Gain is masked to the frontier set
   rather than counting every touched voxel, and the travel-scan gate accounts for pass 2. Decided in
   the design conversation on 2026-08-31.
4. **No RNG.** Clusters are enumerated from a deterministic sweep; templates are precomputed
   deterministically. Project A's determinism requirement stays structural.
5. **No approximation of the sensor model.** The cone template must reproduce the voxel set that
   today's `countUnresolvedVoxels` visits, exactly, verified by an equivalence test. Speed comes from
   removing redundant work, not from a coarser cone.
6. **mp-units:** new code in strong types; the substrate's internal `force_numerical_value_in(cm)`
   doubles stay for project E, as under D.

---

## Design

### Components

| Unit | Change | Responsibility |
|------|--------|----------------|
| `UserCommon/.../ConeTemplate.h` (new) | new | Precompute per-lidar, per-resolution cone geometry once; walk it with no trig and no hashing |
| `UserCommon/.../LidarCone.h` | extended | `countUnresolvedVoxels` gains a template-backed overload with a caller-supplied voxel predicate; the trig form is retained as the equivalence oracle |
| `Algorithm/src/MappingAlgorithmFrontier.{h,cpp}` | modified | `exploreReachable` returns frontier **clusters** and the frontier cell set instead of strided `ReachableCell` candidates; sphere passability is memoised per sweep |
| `Algorithm/src/WavefrontPlanner.{h,cpp}` (new, replaces `NbvPlanner`) | replace | Rank clusters, reconstruct and shape the path, emit an `ExplorationPlan` |
| `Algorithm/src/MappingAlgorithmImpl.cpp` | modified | Sweep state on arrival; score-aware travel scan; termination on information rate |
| `Algorithm/src/PathShaping.{h,cpp}` | unchanged | — |

### Lever 1 — cone templates

A `ConeTemplate` is the geometry of one scan direction, precomputed for a `(LidarConfigData,
resolution)` pair:

```cpp
namespace user_common_207190406_209543255::cone_template {

struct BeamRun {
    double ux = 0.0, uy = 0.0, uz = 0.0;  ///< unit direction, precomputed once
    std::size_t sample_count = 0;         ///< samples of `step_cm` out to z_max
};

struct ConeTemplate {
    common::Orientation direction{};      ///< world-frame cone axis
    std::vector<BeamRun> beams{};         ///< centre beam first, then rings
    double step_cm = 0.0;                 ///< resolution / 2, as today
    std::size_t near_field_samples = 0;   ///< samples with distance < z_min
};

/// Built once per (lidar, resolution) and cached for the mission's lifetime.
class ConeTemplateCache {
public:
    [[nodiscard]] const std::vector<ConeTemplate>& get(
        const common::types::LidarConfigData& lidar,
        common::PhysicalLength resolution);
};

} // namespace
```

Walking a template is vector addition: start at the origin, add `step_cm * (ux, uy, uz)` per sample,
call `atVoxel`. No basis construction, no `cos`/`sin`, no `pointAlongBeam`. Cross-beam deduplication
uses a **generation-stamped dense buffer** over the cone's bounding box in grid coordinates relative
to the quantised origin, indexed by integer offset — replacing the `unordered_set<int64_t>` and its
hashing. The buffer is owned by the caller and reused across directions and across replans; a
generation counter makes clearing O(1).

Offsets are stored as *unit directions and sample counts*, not as grid offsets, precisely so the walk
starts from the drone's true (possibly non-voxel-aligned) position and visits the same voxels the
current implementation visits. That keeps decision 5 achievable: the template form and the trig form
are tested for exact set equality on randomised maps and poses.

`near_field_samples` records how many leading samples of every beam lie inside `z_min`. That is what
makes the pass-2 gate in Lever 3 cheap.

Expected effect: same lookup count per cone, but each lookup loses two trig calls and a hash
insertion. Combined with Levers 2 and 3 reducing how many cones are cast at all, this is the
constant-factor half of the runtime fix.

### Lever 2 — Wavefront Frontier Detection

`exploreReachable` already performs the Dijkstra pass Keidar & Kaminka's WFD needs. Three changes
turn it into WFD and make it cheaper at the same time:

**Frontier flag from 6 faces, not 26 neighbours.** A popped cell is a frontier cell when any of its
six face neighbours is `Unmapped`. The 26-probe `unmapped_neighbours` count it replaces existed only
to feed the prefilter, and cluster size supersedes that, so 6 probes suffice — a 4.3× cut on the
sweep's scoring work.

**Memoised sphere passability.** `isSpherePassable` results are cached per `GridKey` for the duration
of one sweep. Each cell is currently probed up to six times, once per face neighbour that pops; the
memo makes it once. Roughly 6× on the dominant term of the sweep.

**Clustering.** After the Dijkstra pass, a second BFS over the frontier cell set groups
face-connected frontier cells into clusters. Each cluster records:

```cpp
struct FrontierCluster {
    std::size_t cell_count = 0;              ///< frontier cells — the score-relevant gain proxy
    GridKey approach_key{};                  ///< member with the lowest Dijkstra cost
    common::Position3D approach_position{};
    int approach_cost = 0;
};
```

The clustering BFS visits each frontier cell once and touches only the frontier set, so it is O(F)
with F ≪ V, and it needs no map lookups at all — membership is already known.

`ReachabilityResult` then carries `std::vector<FrontierCluster> clusters` and a
`FrontierCells frontier_cells` set, and drops `candidates`, `ReachableCell`, and the `stride_cells`
parameter. Strided deduplication existed only to bound the candidate count; clusters bound it
naturally and much better, because they aggregate a region instead of sampling a lattice.

**Ranking.** For each of the `kRankedClusters = 8` clusters with the highest `cell_count`:

1. `reconstructPathTo(parent_of, start_key, approach_key, config)`
2. `stringPullConstantAltitude` on the raw path
3. `travel = stepCostForPath(waypoints, position, heading, limits)`
4. discard if `travel + kSweepStepsReserve > remaining_steps` (D's feasibility filter, unchanged in
   spirit; the reserve is the steps needed to actually scan on arrival)
5. `rate = cell_count / (travel + kSweepStepsReserve)`

The best rate wins. The current pose competes as cluster zero with `travel = 0`, which preserves D's
"sweep in place at spawn" behaviour.

Ranking uses `cell_count`, which the sweep already produced, so **a replan casts no cones at all** —
down from seventeen candidates at two full-cone passes each. Cone casting moves entirely to arrival
(Lever 3) and to the cheap per-step travel probe. That is the algorithmic half of the runtime fix.
`kRankedClusters = 8` costs eight path reconstructions and eight string-pulls, all of which are
cheap: `reconstructPathTo` walks parent links, and `stringPullConstantAltitude`'s line-of-sight
checks are bounded by path length.

### Lever 3 — score-aware scanning

**Gain is masked to the frontier set.** When counting a cone's gain, an `Unmapped` voxel counts only
if it is in `frontier_cells` or face-adjacent to a cell in it. Those are the voxels a scan can resolve
that plausibly sit inside the spawn-reachable free space, and therefore the voxels pass 1 is already
charging us for. `Unmapped` voxels deep behind a surface fall outside the mask and contribute nothing,
which stops the policy chasing resolving work that pass 2 penalises. `PotentiallyOccupied` remains a
resolved state and is never gain, as under D.

**Scan selection moves to arrival.** `ExplorationPlan` no longer carries `terminal_scans` computed
against a stale map at plan time. It carries only the waypoints and the target cluster:

```cpp
struct ExplorationPlan {
    std::vector<common::Position3D> waypoints{};
    std::size_t target_cluster_cells = 0;  ///< for the information-rate termination rule
    double expected_rate = 0.0;            ///< cell_count / (travel + reserve), for logging
    bool valid = false;
};
```

When the executor exhausts the waypoints it enters a sweep state and builds the scan list *there*,
against the map as it then is. Building it uses two passes over the ~34 directions, but the ordering
is fixed: pass one scores each direction **independently** with its own generation stamp, giving true
per-direction geometry; the list is sorted on that; pass two walks the sorted list with a shared
stamp to compute marginal contributions and drops directions whose marginal gain is zero. That is the
fix for cause 4, and it is now affordable because it happens once per arrival rather than seventeen
times per replan. The list is rebuilt when exhausted, so late directions are scored against the map
the earlier scans produced.

**The travel scan is gated on pass 2, not just on gain.** Per step, probe three directions — the one
pointing along the next waypoint, plus `±Z` — with an early-exit "any masked-unresolved voxel?" walk
rather than a full count. A direction is rejected when its `near_field_samples` prefix contains an
`Occupied` or `OutOfBounds` voxel, because such a beam returns a zero-distance hit and paints
`PotentiallyOccupied` from 0 to `z_min`; inside the reachable set that is score-neutral, but outside
it, it is a wrong answer entered into the denominator for free. Rejecting it costs one scan and buys
nothing lost, since the mask already told us the direction has no score-relevant gain to offer.

`kTravelScanProbes = 6` becomes `kTravelScanProbes = 3` with a directed probe set, and the early-exit
walk replaces the full count — a direction pointing into unexplored space exits on its first
`Unmapped` sample.

### Termination

D terminated only when no feasible candidate had positive gain after three recovery attempts, which
on `small_room` meant vacuuming crumbs to `max_steps` and losing 6 points to post-C. Given the pass-2
finding, stopping early is no longer merely neutral — it is a positive when the remaining work is
low-value, because continued flying and scanning can lower the score.

`Finished` when no `Unmapped` voxel remains in bounds. Otherwise
`FinishedWithUnmappableVoxels` when, for `kLowRateReplans = 3` consecutive replans, either:

- no feasible cluster exists (after D's `ignore_blocked` recovery attempt), or
- the best feasible cluster's rate is below `kMinInformationRate`.

`kMinInformationRate = 0.25` frontier cells per step is the one genuinely tuned constant in this
design, and the implementation plan sweeps it over {0.10, 0.25, 0.50} on the benchmark rather than
asserting it. A cluster of two cells eight steps away rates 0.25 and survives; a single cell twenty
steps away rates 0.05 and does not.

### Replan cadence

`kReplanIntervalSteps` rises from 10 to 25. The interval exists because `IMap3D` has no version
counter, and at 10 it was buying re-evaluation the candidate policy could not use — the prefilter
returned the same nearby crumbs regardless. With clusters, a plan stays valid much longer, and the
cheap local invalidation covers the case the interval was guarding: before continuing on a plan, check
whether any cell of the target cluster is still a frontier cell, which is O(cluster) map lookups, and
replan immediately if not.

### Retired

- `NbvPlanner` — replaced by `WavefrontPlanner`. Its `scanDirections` moves to the template cache;
  its `gainAt` becomes the arrival-time two-pass scan-list builder in the executor.
- `ReachableCell`, `ReachabilityResult::candidates`, the `stride_cells` parameter, and
  `kCandidateStrideCells` — superseded by clusters.
- `kScoredCandidates` — superseded by `kRankedClusters`.
- `uniqueGainOf` and the shared-`seen` Fibonacci ordering in `gainAt` — superseded by the
  independent-then-marginal two-pass ordering.
- `ExplorationPlan::terminal_scans` and `expected_gain` — superseded by arrival-time selection and
  `target_cluster_cells` / `expected_rate`.

### Constants

Every constant this project introduces or changes, with where its value comes from. Only one is
tuned; the rest are derived or structural, which is the property `.cursor/rules` e23 asks for.

| Constant | Value | Where the value comes from |
|----------|-------|---------------------------|
| `kRankedClusters` | 8 | Bounds path reconstruction per replan. Path work is cheap, so this is generous rather than tight |
| `kSweepStepsReserve` | `min(directions.size(), 8)` | Steps reserved for scanning on arrival, so a cluster is not chosen that can be reached but not observed. Derived from the lidar's direction count, not fixed |
| `kMinInformationRate` | 0.25 | **Tuned.** Swept {0.10, 0.25, 0.50} on the six profiling cells 2026-08-31; all three rates produced identical scores/steps. Kept 0.25. Table: `docs/benchmarks/2026-08-31-post_f_honest.md` |
| `kLowRateReplans` | 3 | Matches D's `kRecoveryAttempts = 3` so termination needs the same amount of corroboration as recovery |
| `kReplanIntervalSteps` | 25 | Raised from D's 10. A backstop only: the target-cluster invalidation check is the real trigger |
| `kTravelScanProbes` | 3 | The waypoint direction plus ±Z. Down from D's 6, since the probe is now early-exit and directed |

### What does not change

- C's clearance geometry and cone half-angle formula; the `UserCommon` placement of both.
- B's `step()` contract and `isRecoverableMovementFailure`.
- The `IMappingAlgorithm` interface, the plugin class name, and registration.
- `Unmapped` soft cost (4) versus `Empty` (1) in traversal.
- `kMaxMovingStallTicks = 2`, `kBlockedTtlSteps = 50`, `kRecoveryAttempts = 3`, the blocked-cell TTL
  mechanism, and the stall-triggered replan.
- Co-emission: every travel step still carries a scan when one is worth taking.

---

## Expected effect on runtime

Per replan, against the ~2.5 million lookups measured above:

| Source | Before | After | Factor |
|--------|--------|-------|--------|
| Sweep scoring probes | 26 per cell | 6 per cell | 4.3× |
| Sweep passability probes | up to 6× per cell | memoised to 1× | ~6× |
| Replans per mission | `max_steps / 10` | `max_steps / 25` backstop | ~2.5× |
| Cones cast per replan | 34 cones × 2 passes × 17 candidates | **zero** | — |
| Cones cast per arrival | — | 34 × 2 passes | new term, once per plan |
| Per-lookup cost inside a cone | 2 trig calls + hash insert | vector add + stamp index | ~3–4× |
| Travel-scan probes per step | 6 full counts | 3 early-exit counts | ~4× or better |

The sweep and the cone casts are separate terms of a sum, so the total is bounded by whichever
survives rather than by their product. The sweep term drops roughly 10× from probes and memoisation
and a further ~2.5× from the interval, while the cone term stops being per-replan altogether: it
becomes one arrival-time evaluation per plan plus a cheap directed probe per step. Conservatively that
is ~15–25× on top of the 8× already gained from `-O2`, which should put `house_full` in single-digit
seconds.

This is an estimate derived from the code, not a measurement. D's equivalent estimate was wrong by two
orders of magnitude, so the implementation plan measures one large cell early rather than at the end,
and success criterion 1 is stated in wall-clock seconds rather than in factors.

---

## Test plan

### Cone templates

| Test | Asserts |
|------|---------|
| Template walk visits exactly the voxel set the trig walk visits | Decision 5, on randomised maps and non-voxel-aligned origins |
| Template walk stops a beam at `Occupied` and at `OutOfBounds` | Occlusion preserved |
| Cross-beam deduplication counts a shared voxel once | Stamp buffer replaces the hash set faithfully |
| Generation counter makes reuse across directions correct | A stale stamp never suppresses a fresh count |
| `near_field_samples` covers exactly the samples inside `z_min` | The pass-2 gate probes the right prefix |
| Cache returns the same templates for the same `(lidar, resolution)` | Built once, not per call |

### Wavefront sweep and clustering

| Test | Asserts |
|------|---------|
| Two unmapped rooms separated by a wall yield two clusters | Face-connectivity clustering |
| A cluster's `cell_count` equals its frontier cell count | The gain proxy is what it claims |
| `approach_key` is the cluster member with the lowest Dijkstra cost | Ranking uses the cheapest entry |
| Frontier flag uses 6 faces, and a diagonal-only unmapped neighbour does not flag | The cheaper probe is the specified one |
| Memoised passability yields the identical sweep to the unmemoised one | Optimisation is behaviour-preserving |
| Sweep is expansion-bounded when passability is forced true | ALG28, retained from D |

### Policy

| Test | Asserts |
|------|---------|
| A 400-cell room forty steps away beats a 3-cell crumb one step away | Cause 2, as a unit test |
| Cluster exceeding `remaining_steps - kSweepStepsReserve` is discarded | Feasibility filter |
| Current pose competes as cluster zero and wins when it should | Spawn sweep behaviour retained from D |
| Identical map and configs produce an identical plan across runs | Project A determinism |
| Plan is abandoned when its target cluster stops being a frontier | Local invalidation, not just the interval |

### Score-aware scanning

| Test | Asserts |
|------|---------|
| An `Unmapped` voxel behind an `Occupied` surface, outside the frontier mask, contributes no gain | Cause 3 |
| Scan direction list is ordered by independent per-direction gain, not by enumeration order | Cause 4 |
| Marginal pass drops a direction whose voxels were all claimed by earlier directions | No wasted scan steps |
| Scan list is rebuilt against the updated map when exhausted | Arrival-time selection is not a one-shot |
| A direction whose `z_min` prefix contains `Occupied` is rejected | Pass-2 gate |
| A travel step still emits movement and scan together when a direction passes the gate | Co-emission survives the gate |

### Termination

| Test | Asserts |
|------|---------|
| Does not finish while a feasible cluster above the rate floor remains | D's `house_full` property, retained |
| Finishes with `Finished` when no `Unmapped` remains in bounds | True completion |
| Finishes after `kLowRateReplans` consecutive sub-floor replans, not on the first | Bounded, not twitchy |
| A blocked cell becomes plannable again after its TTL | Retained from D |

### Harness

Run the `honest` column and record `docs/benchmarks/<date>-post_f_honest.{csv,md}` with per-group band
verdicts against ex2, and against both post-C (1331.0) and the lawnmower fixture (646.4) as reference
points. Run the adversarial column as a robustness check. Sweep `kMinInformationRate` over
{0.10, 0.25, 0.50} on the six profiling cells before fixing it.

---

## Success criteria

1. **Every cell in the `honest` column completes within 60 s wall clock at `-O2`**, including
   `house_full` and `large_out`. This is the gate; without it nothing else is measurable.
2. `honest` score sum exceeds post-C's 1331.0 by a clear margin, and exceeds D's on every cell D
   managed to finish. `small_room` recovers to at least post-C's 82.57.
3. `small_out`'s 25.16 → 83.49 improvement from D is preserved, not traded away for speed.
4. `house_full` moves toward its ex2 band of 56–62 from post-C's 6.30.
5. Zero cells with `ERROR` status in either column; `MAX_STEPS` cells acceptable.
6. The template walk is proven set-equal to the trig walk, so the speedup carries no silent change to
   the sensor model.
7. No cell terminates with substantial unused budget while a feasible cluster above the rate floor
   remains — D's property, retained under the new termination rule.
8. `NbvPlanner`, `ReachableCell`, `kCandidateStrideCells`, `kScoredCandidates`, and
   `ExplorationPlan::terminal_scans` are gone from the tree.
9. New code uses `mp-units` strong types; no new `force_numerical_value_in` outside the boundary with
   the legacy substrate.
10. Full `ctest` green; docs below updated.

---

## Risks

- **The frontier mask may be too aggressive.** Masking gain to frontier-adjacent voxels is a
  heuristic for "inside the reference's reachable set". If it suppresses legitimate long-range
  observation — a scan down a corridor whose far end is not yet a frontier because nothing there is
  reachable-and-known yet — coverage could fall. Mitigation: the mask includes face-adjacency to the
  frontier set, and the benchmark compares masked against unmasked gain on the six profiling cells
  before the mask is locked in.
- **`kMinInformationRate` is a coverage-versus-score trade with no principled value.** Set too high it
  quits on real work; too low it reintroduces D's crumb-vacuuming. It is swept, not guessed, and the
  chosen value is recorded with its measurement.
- **The pass-2 mechanism for `small_room` is inferred, not measured.** The code path is certain; that
  it accounts for the full 6-point gap is not. If the sweep shows the mask and the gate do not recover
  `small_room`, the next thing to instrument is the count of pass-2-counted voxels in the output map,
  not further policy changes.
- **The runtime estimate could be wrong again.** D's was, by two orders of magnitude, because it was
  derived from the spec's mental model rather than from the code. This one is derived from the code
  and states its terms explicitly, but the implementation plan still measures one cell early rather
  than at the end.
- **Clusters can be large and shallow.** A thin frontier shell wrapping a mapped room has a high
  `cell_count` without much genuinely new volume behind it. If ranking oscillates between such shells,
  frontier clustering with tour ordering (FUEL/TARE) is the escalation, and it remains out of scope.

---

## Out of scope

- Frontier clustering with **tour ordering** (FUEL, TARE). This design clusters but does not solve a
  tour; it re-ranks every replan. Revisit only if measurements show revisit oscillation.
- Building an independent belief map by raycasting `latest_scan`. `output_map_` remains the belief.
- Incremental replanning (D* Lite / LPA*). The cheaper sweep is the alternative being tried first.
- Converting the frontier substrate to `mp-units` — project E.
- Any change to `MissionControl/`, `common/`, `Simulator/`, or the harness. In particular the pass-2
  scoring behaviour is a property of the grader and is *adapted to*, not modified.
- The `-O2` build configuration itself, beyond ensuring the harness uses it. Whether `Release` becomes
  the default `CMAKE_BUILD_TYPE` is a separate change.

---

## Related docs

| Doc | Role |
|-----|------|
| `docs/superpowers/specs/2026-08-29-exploration-policy-nbv-design.md` | Project D — the executor this keeps and the scoring this replaces |
| `docs/superpowers/specs/2026-08-29-mapping-algorithm-roadmap.md` | A–E sequence and the standing decisions |
| `docs/superpowers/specs/2026-08-29-sensor-model-clearance-belief-design.md` | C — cone helpers the templates precompute |
| `docs/superpowers/specs/2026-08-29-missioncontrol-step-honesty-design.md` | B — the step contract co-emission depends on |
| `docs/superpowers/specs/2026-08-29-algorithm-benchmark-harness-design.md` | A — harness, ex2 bands, determinism |
| `docs/mapping-algorithm-analysis.md` | Alternative #3 (WFD as primary mechanism) and #4 (any-angle smoothing) |
| `docs/ex2-grading-handoff.md` | ALG28 unbounded-search hang the bound addresses |
| `docs/benchmarks/2026-08-29-post_c_honest.{csv,md}` | The 1331.0 baseline |

## Documentation updates

| Doc | Change |
|-----|--------|
| `docs/mapping-algorithm-analysis.md` | Mark alternative #3 adopted; add the `MapsComparison` pass-2 finding to "What the assignment actually measures", since it changes what "map more" is worth |
| `docs/HLD.md` | Replace the NBV description with WFD over the reachability substrate |
| `docs/superpowers/specs/2026-08-29-mapping-algorithm-roadmap.md` | Add project F, record D's measured outcome honestly, mark the superseded parts of D's spec |
| `docs/mapping-algorithm-rewrite-pickup.md` | F status and next queue item |
| `docs/known-issues.md` | Revisit #20 given the new step profile |
