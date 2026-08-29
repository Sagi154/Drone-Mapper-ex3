# Sensor Model + Clearance Fix — Design

**Date:** 2026-08-29
**Status:** Proposed
**Goal:** Fix the `isSpherePassable` clearance-check no-op (safety) and derive the scan pattern from the
lidar's actual cone geometry instead of a hardcoded 26-direction sweep (efficiency), so the planner
stops treating the drone as a point and stops re-scanning already-covered space.

This is project **C** of four. See `docs/superpowers/specs/2026-08-29-mapping-algorithm-roadmap.md`
for the full sequence, dependencies, and the two decisions (honest-step-accounting scope; beat-ex2 bar)
that constrain this project. Depends on A (harness, to measure before/after) and B (honest MC, the
step-accounting reality this project's efficiency motivation responds to).

---

## Problem

Two independent findings from `docs/mapping-algorithm-analysis.md`, verified directly against
`Algorithm/src/MappingAlgorithmFrontier.cpp` and `MappingAlgorithmImpl.cpp` while writing this spec.

### 1. `isSpherePassable` is a no-op — safety issue

`isSpherePassable` (`MappingAlgorithmFrontier.cpp:87-141`) is supposed to sweep every probe point in a
sphere of `radius_cm` around a candidate cell and reject the cell if any probe lands on `Occupied` or
`OutOfBounds`. Verified the arithmetic directly:

- `radius_cm` comes from `drone_config_.radius` (`MappingAlgorithmImpl.cpp:322` etc.) — 4 cm or 7.5 cm
  in the shipped drone configs.
- `step_cm` is the output map's grid resolution — 10 cm in every shipped mission (no mission sets
  `output_mapping_resolution_factor`).
- `rx = rh = ceil(radius_cm / step_cm) = ceil(4/10) = 1` (or `ceil(7.5/10) = 1`) — so the probe loop
  runs `dx, dy, dz ∈ {-1, 0, 1}`.
- The loop's own gate, `ox² + oy² + oz² > radius_cm²`, then `continue`s past every non-zero offset:
  the smallest non-zero offset is a single axis step, `ox² = step_cm² = 100`, which already exceeds
  `radius_cm² ∈ {16, 56.25}` for both shipped radii. **Every probe point except the centre itself is
  skipped.** The function reduces to a single `occupancyAt(map, centre)` call — the ~55-line sphere
  sweep is dead weight that never executes its intended branch.

Consequences:

- `Unmapped` is deliberately treated as passable (soft cost, not a hard block — see the comment at
  `MappingAlgorithmFrontier.cpp:92-94`), which is intentional: it lets the planner route through
  unexplored territory rather than stalling forever. But combined with the sphere-probe no-op, the
  planner effectively plans **as a point drone through a single-voxel occupancy check**, with no
  margin for the drone's actual footprint and no verification of the cells *around* a waypoint before
  committing to it.
- Our own MissionControl forgives the fallout via `isRecoverableMovementFailure`'s `blocked`/`boundary`
  substring match (`MissionControl/src/DroneControlImpl.cpp:93-96`) — a bump into a wall the planner
  didn't see coming just costs a wasted step, not a run.
- **A foreign host is not guaranteed to forgive it.** `docs/superpowers/specs/2026-08-28-independent-component-variants-design.md`
  documents that a foreign MissionControl may return a hard `Error` on an invalid/blocked movement.
  Under decision 1 (roadmap), our own honest MC is the optimization target and it *does* forgive this
  — so this is not the dominant cost in the `honest` benchmark column today — but it is a correctness
  bug independent of any benchmark number, it is what the adversarial column exists to catch, and any
  competition/foreign host in `-competition` mode is not obligated to be as forgiving as ours.

### 2. The scan pattern ignores the lidar's actual cone — efficiency issue

`buildScanOrientations` (`MappingAlgorithmImpl.cpp:128-199`) hardcodes exactly 26 directions — 6 face +
12 edge + 8 corner — regardless of the lidar config. `lidar_config_` (`common::types::LidarConfigData`:
`z_min`, `z_max`, `d`, `fov_circles`) is read in exactly one place in the whole algorithm,
`ensurePlanningReady` (`MappingAlgorithmImpl.cpp:120`), and only for `z_max` (to size `spacing_cells`).
`d`, `z_min`, and `fov_circles` are never read anywhere in `Algorithm/`.

The 26-direction sweep is a fixed spherical tiling, not derived from the sensor's actual cone
half-angle. Per the earlier analysis (`docs/mapping-algorithm-analysis.md:111-127`): `lidar_short`
(`fov_circles: 4`) roughly matches a ~45° tiling already, but `lidar_long` (`fov_circles: 3`) leaves
~17° wedges unscanned between the 26 fixed directions — worth re-deriving precisely in this project
rather than trusting that number, since it depends on the exact cone half-angle formula (below).

Every one of those 26 scans costs one full mission step under project B's honest accounting (was
amortized under the old batching). Re-scanning a direction whose cone already covers previously-seen
space is now a direct, non-amortized waste of the scarcest resource in the mission. This motivation is
**stronger after B, not weaker** — before B, the cost of an unnecessary scan was hidden inside a batch;
after B it is one whole step, same as an Advance.

---

## What decision 1 changes about this project (do not re-litigate)

Roadmap decision 1: our own MissionControl is the primary optimization target, and it **keeps** carving
`Empty` along beams and **keeps** passing non-null `latest_scan` — it is a good host. The original
framing of this project (written before decision 1) assumed the primary host might hand back
`latest_scan = nullptr` or skip Empty-carving, making "build our own belief map because we can't trust
the host's fusion" the main driver. That is no longer the primary case. This spec is scoped
accordingly:

- **In scope (primary):** fix the clearance/passability no-op; derive scan geometry from the lidar
  config. Both operate against `output_map_` as today — **no new belief-map data structure**, no
  raycasting `latest_scan` ourselves. `output_map_` already reflects our own honest MC's fusion, which
  is the ground truth we're optimizing against.
- **Out of scope (deferred to D, if needed at all):** raycasting `latest_scan` into an independent
  occupancy representation, decoupling from the host's fusion policy, degrading gracefully when scans
  are `nullptr`. This is a robustness concern for the adversarial column / a genuinely foreign
  `-competition` host, not a requirement for beating ex2 under our own honest MC. D's viewpoint-scoring
  will define what minimal belief interface it actually needs (if `output_map_` isn't already enough)
  — building speculative belief-map machinery now, before D exists, would be guessing at that
  interface.

---

## Design

### Part 1 — fix `isSpherePassable`

Two candidate fixes exist; this spec picks one and records why, per the roadmap's "genuinely open"
question.

**Chosen: probe at the *real* fractional radius, not `ceil`-rounded to whole grid steps.** Replace the
"how many whole steps fit in the radius" loop bound with a direct check: for the fixed `3×3×3` (or
`3×3×1`-equivalent, 2D-flattened where relevant) neighbourhood of grid cells around the centre — the
only cells that can possibly overlap a sphere of ≤ 7.5 cm radius against a 10 cm grid — test whether
that neighbour cell's *nearest corner* to the centre lies within `radius_cm`, using the real
(non-rounded) distance. Concretely: keep the `dx, dy, dz ∈ {-1, 0, 1}` neighbour loop (it already
covers every cell that could possibly be within one grid step, which is more than enough at these
radii), but change the inclusion test from `ox² + oy² + oz² > radius_cm²` (comparing the offset to the
*cell centre*, which is always ≥ half a step away and therefore always fails at these radii) to a
nearest-point-in-box-to-sphere test: clamp the offset to the neighbour cell's own half-extent
(`step_cm / 2` per axis) before squaring and comparing. This makes a neighbour cell "in range" exactly
when the sphere of radius `radius_cm` actually intersects that grid cell's box, which is the physically
correct question — not whether the sphere reaches the cell's *centre* point, which is what silently
disabled every case at these radii today.

Why not the alternative (treat `Unmapped` as non-traversable near frontiers until confirmed clear)?
That's a bigger behavioral change — it would make the planner more conservative in a way that trades
coverage for safety broadly, not just fixing the specific arithmetic bug. The nearest-point-in-box fix
makes the existing sphere-sweep *do what its own comment already claims it does* (reject `Occupied`/
`OutOfBounds` within the drone's real footprint) without changing the `Unmapped`-is-soft-cost policy at
all. If harness numbers after this fix still show unsafe-feeling routing through recently-`Unmapped`
cells, that is a signal for D to reconsider, with real data — not a reason to bundle a bigger policy
change into C.

Apply the same fix to `sphereContainsNotMapped` (`MappingAlgorithmFrontier.cpp:143-176`) — it has the
identical `ox² + oy² + oz² > r²` bug and is used by `isFrontier`'s frontier-detection sphere check; an
inconsistent fix between the two functions would leave `isFrontier` and `isSpherePassable` disagreeing
about which cells are "in range" of a point.

### Part 2 — derive scan orientations from the lidar cone

Replace the hardcoded 26-direction list in `buildScanOrientations` with a set of directions sized and
spaced from the actual lidar config:

1. **Cone half-angle from `d` / `z_min` / `fov_circles`.** Derive the formula precisely (this is the
   "genuinely open" item from the roadmap — resolve it here, not by trusting the two example angles
   already sketched in `docs/mapping-algorithm-analysis.md:116-118`). `fov_circles` is the number of
   concentric ring samples the lidar takes within its cone; `d` and `z_min` bound the near/far
   detection geometry. Work out the half-angle as a function of these three fields against the actual
   `ILidar`/`LidarConfigData` semantics (re-read `common/include/Common/ILidar.h` and any simulator-side
   lidar model source for the authoritative geometry — don't guess from field names alone).
2. **Direction count and spacing from the half-angle**, not a fixed 26. Use a Fibonacci-lattice (or
   equivalent low-discrepancy) sampling over the sphere, with an angular spacing tied to the cone's
   full width so adjacent scan cones overlap by a small, deliberate margin (avoid the ~17° gap the
   current 26-direction sweep leaves for `lidar_long`) without over-sampling for `lidar_short`, which
   already has a wide-enough cone that 26 fixed directions may already be close to correct — the point
   is that the number and placement should fall out of the lidar's numbers, not be hardcoded for either
   config.
3. **Gain-gating: skip a candidate scan direction whose cone is already known-`Occupied`/`Empty` at the
   current position.** Before emitting a scan orientation, check whether the region that orientation's
   cone would cover, out to `z_max`, is already resolved in `output_map_` (not `Unmapped`). If so, skip
   it and move to the next orientation instead of spending a step on a scan that can't add information.
   This is the direct fix for "re-scanning already-known space is direct waste of a now-expensive
   resource" — it requires no belief-map addition, just a query against `output_map_` along the
   candidate direction, using the same beam/geometry math as part 4 below.
4. **Shared beam math goes in `UserCommon/`, not duplicated.** `UserCommon/` currently contains no
   geometry/beam code — `TimeFormat`, `SimulationCoordUtil`, `RunErrorLog`, `ConfigParseResult` only
   (verified via directory listing). `MissionControl/src/BeamMath.hpp` and `ScanResultToVoxels` are the
   existing raycasting/geometry implementation to lift from, per the no-duplicate-logic rule (e10) and
   the roadmap's Project C notes. Move (or extract a shared subset of) the beam-stepping math into
   `UserCommon/`, compiled into both `Algorithm/` and `MissionControl/` as separate translation units —
   no cross-`.so` symbol dependency between plugins. Scope the extraction to exactly what part 3's
   gain-gating query needs (a beam/cone-vs-map occupancy walk); don't move unrelated `MissionControl/`
   internals just because they live in the same file.

### What does not change

- `Unmapped`-is-soft-cost traversal policy (`traversalCost`, `MappingAlgorithmFrontier.cpp:80-85`).
- The `Occupied`/`OutOfBounds`-only hard-block rule in `isSpherePassable` — only the arithmetic that
  decides which probe points count changes, not which occupancy states block.
- `MissionControl`'s `isRecoverableMovementFailure` substring match — still needed as defense-in-depth
  for whatever residual risk remains after the sphere-probe fix (e.g. dynamic aspects of a mission this
  static check can't see), not removed by this project.
- Project A's harness, Project B's `step()` contract, and decision 1/2 from the roadmap.
- No new belief-map data structure (see "What decision 1 changes" above).

---

## Test plan

### `isSpherePassable` / `sphereContainsNotMapped`

| Test | Asserts |
|------|---------|
| Centre `Occupied` still rejected | Unchanged behavior for the already-working centre check |
| Neighbour cell `Occupied` within real radius now rejected | A neighbour whose box intersects the sphere at `radius_cm = 7.5`, `step_cm = 10` is detected — this is the case that silently passed before the fix |
| Neighbour cell `Occupied` just outside real radius still passable | Confirms the fix isn't overly conservative — a neighbour whose box does *not* intersect the sphere is not falsely blocked |
| `blocked_cells` override still short-circuits | Unchanged from today |
| `sphereContainsNotMapped` uses the same corrected geometry | Same box-intersection fix applied consistently; a regression test that would have caught the original mismatch between this function and `isSpherePassable` |
| Regression: existing frontier/pathing tests | Re-run full `Algorithm` test suite — some paths that were previously "passable" under the no-op may now correctly reject; expect some path/step-count deltas, not necessarily zero-diff |

### Scan-orientation derivation

| Test | Asserts |
|------|---------|
| Direction count/spacing for `lidar_short` config | Matches the derived formula's output for `fov_circles: 4`'s cone half-angle — not still hardcoded to 26 |
| Direction count/spacing for `lidar_long` config | Matches the derived formula's output for `fov_circles: 3`; verify no unscanned gap wider than the deliberate overlap margin |
| Gain-gating skips a fully-known direction | Given a map where one candidate cone is entirely `Occupied`/`Empty`, that orientation is not emitted; the next un-resolved orientation is |
| Gain-gating still scans when anything in cone is `Unmapped` | Confirms the gate isn't overly aggressive — partial coverage still triggers a scan |
| Determinism | Same map + same lidar config always produces the same ordered orientation list (required by project A's harness) |

### Harness re-run

- Re-run project A's harness `honest` column after C lands; compare against the post-B baseline
  (`docs/benchmarks/2026-08-29-post_b_honest.{csv,md}`) — expect fewer wasted scans (lower step counts
  on cells that were scan-bound) and, if the clearance fix changes routing, a possible shift in step
  counts on cells that were previously bumping into unseen obstacles. Record both deltas; do not assume
  the fix is strictly step-reducing everywhere (a more conservative clearance check can cost a few
  detour steps in exchange for not bumping walls).
- Adversarial column: confirm still completes without new illegal-move disasters; the clearance fix
  should only *improve* this column's safety margin, never regress it.

---

## Documentation updates

| Doc | Change |
|-----|--------|
| `docs/mapping-algorithm-analysis.md` clearance-check finding | Mark resolved; keep as historical record of the bug, note the box-intersection fix and why it was chosen over the "Unmapped non-traversable near frontiers" alternative |
| `docs/mapping-algorithm-analysis.md` scan-geometry finding | Mark resolved; record the derived half-angle formula and direction counts actually used for both shipped lidar configs |
| `docs/superpowers/specs/2026-08-29-mapping-algorithm-roadmap.md` Project C section | Update "Considerations for the next spec" (i.e. for D) with what was actually learned — belief-map interface D will need, if any, and the measured harness delta |
| `docs/component-placement.md` (if it documents `UserCommon/` contents) | Add the new beam-math module |

---

## Out of scope

- The actual exploration policy / frontier selection logic — project D.
- Raycasting `latest_scan` into an independent belief representation — deferred to D if D's
  viewpoint-scoring turns out to need more than `output_map_` provides (see "What decision 1 changes").
- Any change to `DroneControlImpl`'s movement/scan step order (project B, already landed) or to
  `isRecoverableMovementFailure`.
- `mp-units` strong-type conversion of `Algorithm/`'s raw-double geometry — project D explicitly claims
  this (roadmap, Project D "What's already decided").
- Retiring `blocked_cells` / `frontier_visit_counts` / `explore_dist_cache` / the mid-search Dijkstra
  edge-set change — project D.

---

## Success criteria

1. `isSpherePassable` and `sphereContainsNotMapped` reject/accept neighbour cells based on real
   sphere-vs-grid-box intersection, verified by tests that fail against the current `ceil`-based
   arithmetic and pass after the fix.
2. `buildScanOrientations` derives direction count and spacing from `lidar_config_` (`d`, `z_min`,
   `fov_circles`), not a hardcoded 26, and differs measurably between `lidar_short` and `lidar_long`
   configs.
3. Gain-gating skips at least one otherwise-would-scan orientation on a map where that orientation's
   cone is already fully resolved, demonstrated by a test.
4. Shared beam/geometry math used by gain-gating lives in `UserCommon/`, compiled separately into
   `Algorithm/` and `MissionControl/` — no new cross-plugin `.so` symbol dependency.
5. Full `ctest` suite green; project A's harness `honest` column re-measured and recorded in
   `docs/benchmarks/`, with deltas vs. post-B explained (not just reported).
6. Listed docs updated.

---

## Related docs

- `docs/superpowers/specs/2026-08-29-mapping-algorithm-roadmap.md` — full project sequence and the
  decisions this spec inherits
- `docs/superpowers/specs/2026-08-29-missioncontrol-step-honesty-design.md` — project B, whose honest
  step accounting is why the scan-geometry motivation is now first-order
- `docs/mapping-algorithm-analysis.md` — the review that identified both findings this spec fixes
- `docs/superpowers/specs/2026-08-28-independent-component-variants-design.md` — foreign MC hard-`Error`
  contract that makes the clearance fix a correctness issue, not just an efficiency one
- `common/include/Common/types/LidarTypes.h`, `common/include/Common/ILidar.h` — `LidarConfigData`
  fields this project reads for the first time
- `MissionControl/src/BeamMath.hpp` — existing beam/raycasting math to lift into `UserCommon/`
