# VAR-01 sphere-clearance fix — design

Date: 2026-09-05
Branch at time of writing: `known-issues-fixes` @ `84f4816`
Related: `docs/archive/2026-09-05-var01-attempt.md` (parked, failed attempt — read first),
`docs/superpowers/specs/2026-08-28-independent-component-variants-design.md`,
`.cursor/skills/verify-independent-component-variants/SKILL.md`,
`.cursor/skills/verify-cell-runtime/SKILL.md`

## 1. Problem

`verify-independent-component-variants` VAR-01 (`check_foreign_host.sh`) requires
`HOST_ILLEGAL_MOVE_ATTEMPTS=0` when our `Algorithm_207190406_209543255.so` /
`MissionControl_207190406_209543255.so` run under the blind `skeleton_host` test host.
Currently it fails on `small_room`:

```
HOST_STATUS=Completed
HOST_STEPS=600
HOST_ILLEGAL_MOVE_ATTEMPTS=12
```

`house_lower` reports `HOST_ILLEGAL_MOVE_ATTEMPTS=0`, but only because it does nothing for
3 steps and terminates (`FinishedWithUnmappableVoxels`) — see §7, out of scope.

## 2. Root cause (confirmed empirically)

Instrumented `skeleton_host`'s `HostMovement` to classify every rejection and cross-reference
the culprit voxel against our own plugin's output map (temporary diagnostic, gated behind
`HOST_DIAG_ILLEGAL` env var, isolated to `Simulator/tests/hosts/skeleton_host/`, not part of
the graded submission). Result: **all 12 rejections on `small_room` are the same failure
mode, and in every one the culprit voxel was already `Occupied` on our own output map** —
this is a pure geometry bug, not a "the algorithm didn't know" bug.

```
DIAG_ILLEGAL kind=SPHERE_GRAZE at=(150,170,13) r=4 culprit_voxel=[150,170,0]+10 gap=3 plugin_map=Occupied
```

### 2.1 The mechanism

Our navigation lattice is the set of points `offset + k·resolution` (`GridKey`,
`Algorithm/src/MappingAlgorithmFrontier.cpp:403-416` `quantizePosition`,
`:55-64` `keyToPoint`). Both our own `Map3DImpl` and `skeleton_host`'s `HostMap3D` index
voxels identically: voxel `k` covers `[offset + k·res, offset + (k+1)·res)`
(`Simulator/src/Map3DImpl.cpp:86-97`; `Simulator/tests/hosts/skeleton_host/ASSUMPTIONS.md:53-56`).
**Every lattice point is therefore a voxel corner**, the point where up to 8 voxels meet.

The clearance predicate `isSpherePassable` / `sphereIntersectsCellBox`
(`MappingAlgorithmFrontier.cpp:85-163`) treats each neighbour offset `(dx,dy,dz)*step` as a
box **centred** on that offset, with half-extent `step/2`:

```85:97:Algorithm/src/MappingAlgorithmFrontier.cpp
// True iff the axis-aligned voxel box centered at (dx,dy,dz)*step with half-extent
// step/2 intersects the closed sphere of radius at the origin. ...
[[nodiscard]] bool sphereIntersectsCellBox(int dx, int dy, int dz, double step_cm,
                                           double radius_cm) {
    if (dx == 0 && dy == 0 && dz == 0) {
        return true;
    }
    const double half = step_cm * 0.5;
    ...
```

For the immediate neighbour (`dx=0,dy=0,dz=-1`, i.e. straight down) on a 10 cm grid this
models the neighbour's nearest face as **5 cm away** from the lattice point. In truth (corner
convention) the neighbour voxel's near face is **at** the lattice point — distance 0. With a
4 cm drone radius, `half - radius = 1 cm > 0`, so the predicate reports "clear" for a voxel
the drone's sphere already penetrates by 1 cm past its center, hence the 3 cm `gap` at the
grazing point (`t=0.7` along a 10 cm elevate → z=13, floor voxel spans `[0,10]`, sphere
radius 4 reaches to z=9, well past the corner at z=10... actually the arithmetic that matters
is: the model believes the nearest point of the floor box is at `z=10-5=5` below the lattice
point i.e. it undercounts penetration by exactly `step/2 = 5 cm`).

Net effect: **a systematic half-voxel (5 cm on a 10 cm grid) underestimate of how close a
wall/floor is**, for every immediate face/edge/corner neighbour. This is large relative to
both drone radii (4 cm, 7.5 cm) and explains why it reproduces on both drones once the
geometry is exercised (confirmed: `small_room` fails for `drone_small`; the same predicate is
used for `drone_large`, not yet exercised host-side because VAR-01 only runs `drone_small`
per `check_foreign_host.sh`).

### 2.2 Why this didn't previously show up

Our own `MockMovement` (`Simulator/src/MockMovement.cpp`) only checks the **destination**,
using a different, **discrete grid-sample** geometry (`forEachSphereSample`,
`UserCommon/include/user_common_207190406_209543255/SimulationCoordUtil.h:21-57` — samples at
`center ± k·res`, center-distance test) — not a swept path, and not the corner-anchored
continuous AABB `skeleton_host` uses. It also **throws** instead of returning
`{success=false}`, which our own `DroneControlImpl` never lets the planner route through in
the first place (`isSpherePassable` already blocks known-Occupied cell centers). The two
host implementations validate different geometry, and only `skeleton_host`'s stricter,
swept, corner-exact model exposes the bug. This is expected: VAR-01 exists precisely to catch
divergence from "the host we happened to write."

## 3. What must NOT change (hard constraints, from `.cursor/rules/` and `AGENTS.md`)

- `common/`, `Simulator/common_simulator/`, `MissionControl/common_mission_control/` — frozen,
  untouched by either approach below.
- No wall-clock abort in Algorithm/MissionControl (`verify-cell-runtime` skill). `max_steps`
  is the only mission-length limit.
- 24-cell score sum on `inputs/sim_compose.yaml` must stay "pretty similar" to the current
  baseline (**1769.835**, `docs/benchmarks/2026-09-03-*` / reproduced this session) — individual
  cell drift is fine, a large aggregate drop is not. No cell should newly exceed the ~60s
  per-cell budget (`verify-cell-runtime` bar); current baseline already has 2 cells over 60s
  (`large_out` small/short 87.0s in this session's noisy run — needs a clean re-time, see §6)
  that are a pre-existing, separate concern, not something either approach should worsen.
- VAR-02/03/04 must remain PASS (or at their current status) — neither approach touches
  foreign-MissionControl or adversarial-plugin paths, but must be re-verified.

## 4. Do-not-repeat lessons (from the parked attempt, `docs/archive/2026-09-05-var01-attempt.md`
and full archaeology of `backup/var01-2026-09-05` @ `524056a`, commit `5c8c6a2`)

1. **Never silently withhold a movement.** The parked attempt's gate did
   `cmd.movement.reset()` on refusal, leaving the pose unchanged. That is exactly what the
   stall detector (`kMaxMovingStallTicks == 2`, `MappingAlgorithmImpl.cpp:393-402`)
   interprets as "stuck" — it blocklists the waypoint and forces a full bounded
   `exploreReachable` replan. One cell (`house_full` small/long) hit 423 withheld Advances and
   241 blocklist-driven replans this way. **Both approaches below fix the planner's belief
   about the world, so the planner should stop generating infeasible waypoints in the first
   place — no execution-time "refuse and hope" gate should be needed.** If validation is
   still added at emit time as a defense-in-depth (see §5.3), a refusal must trigger an
   immediate in-tick replan attempt, not a bare no-op.
2. **No binary search / iterative refinement on the per-tick hot path.** The parked
   `longestClearSignedDistanceCm` shrink-until-clear ran the swept-clearance check
   O(log2(distance)) times per tick, each doing a 3D voxel-neighbourhood scan. This is the
   direct cause of the `large_out` large/short regression (46.2s → 65.3s at the *same* step
   count — a pure per-tick cost increase). Neither approach below needs this: fixing the
   predicate's geometry (Approach A) or the lattice anchor (Approach B) removes the false
   "clear" reading without needing runtime search for a safe partial move.
3. **Never make a floor/support voxel read as an obstacle for the drone standing on it.**
   The parked fix's naive AABB swap made the floor under a stationary drone geometrically
   overlap its sphere at 1 cm resolution, which then required a special-cased exception
   (`floor_like && cz >= vz1`). Any predicate change must be unit-tested for "drone standing on
   its own floor remains passable" as step zero.
4. **Match sample stride/bounds exactly if mirroring host geometry**, or don't claim to mirror
   it. The parked code used a fixed 0.5 cm stride (stricter than the host's
   `max(0.5, min(res·0.25, 1))`) and started its sweep loop at `i=0` (testing the current pose
   against itself), which made every escape attempt fail. Approach A below fixes the
   *neighbour-box* geometry only, not the sweep sampling, and does not need to replicate the
   host's sampling stride because it operates at plan time on a discrete lattice.
5. **One mechanism per commit, one benchmark per commit.** The parked attempt squashed four
   compensating mechanisms into one commit and one CSV, making individual attribution
   impossible after the fact. Both approaches below are single, self-contained changes; if
   either needs a compensating mechanism during implementation, that is a signal to stop and
   reconsider the approach, not to add a second squashed mechanism.

## 5. Approach A — host-exact corner-anchored clearance, same lattice

### 5.1 Idea

Keep the existing corner-anchored lattice (`offset + k·step`) exactly as is — no change to
`quantizePosition`, `keyToPoint`, `forEachInBoundsVoxel`, blocked-cell keys, waypoint
tolerances, or any test fixture. Fix only `sphereIntersectsCellBox`'s geometry: model
neighbour `(dx,dy,dz)` as the voxel box `[dx·step, (dx+1)·step) × ... ` (a box whose **low
corner**, not center, is at the offset), matching how `Map3DImpl`/`HostMap3D` actually index
voxels relative to a lattice point.

```cpp
// Neighbour d spans [d*step, (d+1)*step) relative to the lattice point (which is the
// voxel's own low corner, per Map3DImpl's floor((pos-offset)/res) indexing).
const auto nearest1d = [step_cm](int d) {
    const double lo = static_cast<double>(d) * step_cm;
    const double hi = lo + step_cm;
    if (lo > 0.0) return lo;
    if (hi < 0.0) return hi;
    return 0.0;
};
```

This is a pure geometry correction confined to one free function
(`MappingAlgorithmFrontier.cpp:89-111`), used only by `isSpherePassable` (navigation) — it
does **not** change `sphereContainsNotMapped` unless that function is found to share the same
predicate (it currently reuses `sphereIntersectsCellBox` too — needs re-check during
implementation whether the "not mapped in sphere" termination check should also tighten, since
that changes when the mission is allowed to declare `FinishedWithUnmappableVoxels`).

### 5.2 What changes for the drone in practice

Neighbour cells that were previously (wrongly) considered non-adjacent enough to ignore at
`radius > step/2` now correctly count. Concretely, for `drone_small` (radius 4 cm, step 10 cm):
previously only the 6 face neighbours were probed at all (since `ceil(4/10)=1` bounds the
loop to `dx,dy,dz ∈ [-1,1]`, unchanged by this fix) — so the fix does not add new probed
neighbours, it corrects the pass/fail verdict for the neighbours already probed. The immediate
effect: the **outermost lattice layer directly touching a wall/floor will now correctly read
as blocked** for face/edge/corner neighbours whose box the sphere actually penetrates.
Concretely this removes the ability to plan the drone's center *exactly on* a lattice point
one full voxel from a wall while grazing it — i.e. the surface-hugging layer shrinks by
however much of the false "clear" margin the old center-model was granting (up to `step/2 -
radius` per axis: 1 cm for `drone_small`, up to `step/2` collapsed since `radius=7.5 >
step/2=5` for `drone_large`, so `drone_large` was already being told every immediate
neighbour blocks navigation regardless — meaning `drone_large`'s cells should see **zero**
geometry change from this fix, since `sphereIntersectsCellBox` for `radius > step/2` already
always returns true for the 6 face neighbours under both the old and new formula... this must
be verified numerically during implementation, not assumed from this spec).

### 5.3 Emit-time defense-in-depth (needs a decision during implementation, not pre-committed here)

Investigate whether a final `isSpherePassable` check against the **predicted** post-move pose
in `emitMovementOrScan` (`MappingAlgorithmImpl.cpp:342-378`), using the corrected predicate, is
needed as a second line of defense — e.g. because the plan was computed several ticks ago
against a since-updated map, or because `movementToward`'s clamped step could overshoot a
waypoint into a cell not itself re-validated. If added, on refusal: **do not silently drop the
movement** (lesson #1). Instead, immediately attempt one bounded local recovery (e.g.
`findUnstickPath` from the current pose, or blacklist just the offending waypoint and let the
existing next-tick replan machinery run) so a refusal costs at most one extra tick, not a
frozen drone. Prototype without this first; add only if empirically needed (see §6 acceptance
criteria — if VAR-01 already passes cleanly with just the predicate fix, skip this).

### 5.4 Estimated blast radius

- **Code:** ~1 function (`sphereIntersectsCellBox`), possibly +1 emit-time check if §5.3 proves
  necessary. No public API, no `MapConfig`/`GridKey` change, no test-fixture coordinate
  changes (existing corner-lattice test helpers are unaffected in *position*, only in
  *pass/fail verdict* for cells adjacent to Occupied/OutOfBounds).
- **Tests:** existing `Algorithm/tests/test_mapping_algorithm_frontier.cpp` passability tests
  must be re-run; `SpherePassableAtRadius7_5On10CmGrid` (`:296-332`) directly encodes the old
  (wrong) geometry for `drone_large` and will need its expected verdict re-derived from the
  correct math, not just updated to "make it pass."
- **Runtime:** no new per-tick work; the predicate is evaluated exactly as often as today, same
  loop bounds (`ceil(radius/step)`), only the inner `nearest1d` formula changes — O(1)
  constant-time difference, not expected to move the needle on any cell's wall-clock.
- **Score risk:** cells with the drone routing close to walls (small rooms, doorways) may see
  the planner take a wider path to the same destination once it correctly refuses the
  surface-hugging layer. Expected to be a minor, localized effect, not a global one, because
  Dijkstra will route around a now-correctly-blocked cell through an adjacent free one at very
  small extra cost (this is a bounded 2D map, not a maze); §6 measures this directly per cell.

## 6. Approach B — voxel-center lattice

### 6.1 Idea

Anchor the navigation lattice at voxel **centers**: `offset + (k + 0.5)·step` instead of
`offset + k·step`. Change `quantizePosition` from `lround((pos-offset)/step)` to
`floor((pos-offset)/step)` (so a position maps to the index of the voxel it's inside, matching
`Map3DImpl`'s own indexing), and `keyToPoint` to add the `+0.5·step` center offset on every
axis. The existing `sphereIntersectsCellBox` centered-box math (`half = step/2` around the
offset) then becomes **exactly correct** without modification, because a lattice point is now
genuinely a voxel center and its neighbour offsets are genuinely voxel-center-to-voxel-center.

### 6.2 What must move with it (full inventory, from research; verify exhaustively during
implementation, this list is not guaranteed complete)

- `Algorithm/src/MappingAlgorithmFrontier.cpp`: `keyToPoint` (`:55-64`), `quantizePosition`
  (`:403-416`), `forEachInBoundsVoxel` (`:390-392`, currently starts enumeration at `min_x`,
  must start at `min_x + 0.5·step` and bound correctly at the new top).
- `Algorithm/src/ScanPlanning.cpp`: duplicate `keyToPoint` (`:56-65`), `quantizePosition` call
  sites (`:143, :260, :318`).
- `Algorithm/src/WavefrontPlanner.cpp`: `hasHorizontalUnmapped` neighbour probes (`:69-73`),
  `quantizePosition` for blocked cells (`:358`).
- `Algorithm/src/PathShaping.cpp`: `hasClearLineOfSight` sample spacing (`:583-584`, currently
  `step*0.5` — re-derive relative to the new anchor, likely unchanged in magnitude).
- `UserCommon/src/ConeTemplate.cpp` `VoxelStamp::quant` (`:73-77`, `llround` → matching
  `floor`), `UserCommon/src/LidarCone.cpp` `voxelKey` (`:96-98`, same).
- `Simulator/src/MapsComparison.cpp` `quantizePosition`/`forEachGridCenter` (`:48-84`) — **only
  if** score parity requires the scorer's own grid-center walk to agree with the new algorithm
  lattice; needs a decision on whether the scorer already treats centers correctly (research
  suggests it does — `forEachGridCenter` already starts at `min`, step `res`, which may already
  BE center-anchored depending on how `min` is defined there; must be re-derived, not assumed).
- **Test fixtures:** ~100+ hard-coded corner-lattice coordinates across
  `test_mapping_algorithm.cpp`, `test_mapping_algorithm_frontier.cpp`,
  `test_wavefront_planner.cpp`, `test_scan_planning.cpp`, `test_path_shaping.cpp`,
  `test_cone_template.cpp`, `test_lidar_cone.cpp` (full file:line inventory already gathered
  this session, available on request) — every one needs its helper (`gridPoint`, `at`,
  `pointCm`) shifted by `+0.5·step`, or the tests need re-deriving from first principles rather
  than blindly offsetting (some tests assert *boundary* behavior that a shift changes
  semantically, not just numerically).

### 6.3 Known hazards specific to this approach (must be resolved during implementation, not
deferred)

1. **Top-layer bounds loss.** `Map3DImpl` derives `bounds.max = offset + (n-1)·step`
   (`Simulator/src/Map3DImpl.cpp:46-62`) — the corner of the last voxel, not its center. A
   center-anchored lattice point at `offset + (n-1+0.5)·step` is **past** that bound and reads
   `OutOfBounds`, meaning **the topmost/outermost lattice layer on every axis becomes
   unreachable** under the current `Map3DImpl` bounds convention. This must be fixed (e.g. by
   generating lattice keys only up to `n-1` inclusive using center points, i.e. `k` ranges
   `0..n-1` and centers are `offset + (k+0.5)·step`, which is *within* `[offset, offset +
   n·step)` — recheck the exact arithmetic empirically against `isInBounds`, this may be a
   non-issue if `MappingBounds.max` is reinterpreted correctly, but the current text describes
   `max` as a reachable *position*, and a center + 0.5 step may or may not exceed it depending
   on whether `max` is defined as `(n-1)*step` (corner of last voxel, i.e. NOT the far face)
   — resolve this ambiguity with a unit test before writing any other code).
2. **GPS-resolution instability on `house_mission_full`.** `MockGPS::setPosition`
   (`Simulator/src/MockGPS.cpp:15-19`) snaps to the nearest multiple of `gps_resolution_cm`
   measured from world 0, independent of map offset. `house_mission_full` uses
   `gps_resolution_cm: 10` (all other 5 of the 6 (simulation, mission) pairs use 5). Center
   lattice points are at `min + 5 + 10k`; with `gps_res=10`, `round((5+10k)/10)*10` snaps every
   center **5 cm off** its intended point (confirmed by calculation this session). This affects
   4 of the 24 `sim_compose.yaml` cells (`house_full` × 2 drones × 2 lidars) **under our own
   Simulator**, not under `skeleton_host` (whose `HostGPS` does not snap at all — VAR-01 itself
   is unaffected). This is a real risk to the "scores stay similar" constraint and must be
   measured directly, not estimated.
3. **`quantizePosition`'s `lround`→`floor` change also affects blocked-cell keys and
   stall/waypoint-arrival tolerance math** (`kHalfStepTolerance` in
   `MappingAlgorithmImpl.cpp:32`) — these are independent of the corner/center question and
   must be re-derived for the new anchor, not left as `lround`, or quantization and
   `keyToPoint` will disagree (a `floor`-quantized center point re-projected by the new
   `keyToPoint` must round-trip exactly).

### 6.4 Estimated blast radius

- **Code:** ~8-10 production sites across `Algorithm/`, `UserCommon/`, possibly
  `Simulator/src/MapsComparison.cpp`. Meaningfully larger and more cross-cutting than Approach
  A.
- **Tests:** full rebase of ~100+ hard-coded lattice coordinates across 7 test files; some
  tests likely need semantic re-derivation, not just coordinate translation.
- **Runtime:** per-node clearance for `drone_small` (radius 4 cm < half-voxel 5 cm) becomes a
  single map lookup instead of the current 6-27 probes, once a free voxel is provably fully
  navigable — likely a *speedup* for cells not near walls. Cells with `drone_large` similarly
  simplify. Not expected to regress runtime; needs to be measured because the bounds-handling
  fix (hazard 1) could add overhead if implemented as extra edge-of-map special-casing.
- **Score risk:** higher and less predictable than Approach A — the GPS-snap hazard alone
  could measurably change 4 of 24 cells' behavior under our Simulator, and the top-layer bounds
  question could make entire boundary rows of every mission's map permanently unreachable if
  resolved incorrectly.

## 7. Explicitly out of scope for this spec

- **`house_lower` 3-step no-op under `skeleton_host`.** Root-caused separately this session:
  the drone spawns 90 cm above the mission's output-map slab because
  `inputs/simulation/house_simulation.yaml`'s `initial_drone_position.height_cm` was hand-edited
  from the staff value `10` to `150` (commit `427edf2`) to compensate for our own Simulator
  double-applying `map_axes_offset` to the spawn position on the way into `Map3DImpl`. Under
  the staff-original value, our own Simulator now fails the identical cell with
  `SPAWN_NOT_PASSABLE`, and `Simulator/src/MapsComparison.cpp` currently scores a fully
  unmapped output map as 100 (empty comparison universe), masking the failure entirely today.
  This is unrelated to the VAR-01 sphere-grazing mechanism (§2) and touches
  `SimulationRunFactoryImpl`, `SimulationCoordUtil`, `MapsComparison`, and a staff input file —
  a separate, larger change that should get its own Known Issues row
  (`populate-known-issues`) and, if pursued, its own spec. Do not fold it into this work.
- **Pre-existing `verify-cell-runtime` `large_out` >60s cells.** Noted as a pre-existing
  condition to not worsen, not something either approach here is meant to fix.

## 8. Validation process (both approaches; answers §"measure, don't decide yet")

Both approaches are prototyped and measured **in isolated git worktrees**, so neither touches
the current branch (`known-issues-fixes`) until one is chosen and re-implemented for real (or
cherry-picked) on a proper feature branch per `.cursor/rules/git-workflow.mdc`.

### 8.1 Setup (per approach)

```bash
git worktree add ../ex3-var01-approach-a known-issues-fixes
git worktree add ../ex3-var01-approach-b known-issues-fixes
```

Each worktree gets its own Docker build (`build/default` for VAR-01 + house_lower harness,
`build/opt` Release for `verify-cell-runtime`) — no shared build directories between worktrees
or with the main tree, to avoid stale-object cache-poisoning across the differing source.

### 8.2 Per-approach measurement loop

1. Implement the approach's minimal code change only (§5.1 for A, §6.2 for B — plus whatever
   hazards in §6.3 are unavoidable for B to even build/run correctly).
2. `verify-independent-component-variants --only-var01` (plus `--only-var02 --only-var03` once
   VAR-01 is green, to confirm no regression there) — record PASS/FAIL and, on FAIL, the
   `HOST_ILLEGAL_MOVE_ATTEMPTS` count and classification (reuse the `HOST_DIAG_ILLEGAL`
   instrumentation already added to `skeleton_host` in this session — it lives outside the
   graded submission and can be copied into each worktree, or upstreamed to the shared
   `skeleton_host` source if kept permanently — decide during implementation whether it's
   worth keeping past this investigation or should be stripped before final commit).
3. `verify-cell-runtime` full 24-cell Release run. Record the per-cell CSV.
4. Run the relevant `Algorithm/tests` suite for the approach (existing suite for A;
   full rebased suite for B) and confirm green.

### 8.3 Comparison and decision

Build a table: VAR-01 status, 24-cell score sum (vs baseline **1769.835**), per-cell score
deltas, wall-clock sum and any newly-over-60s cells, test suite status, lines of production
code changed, lines of test code changed. Decision rule:

- **Disqualifying:** VAR-01 still FAIL, any test suite regression, any cell newly exceeding
  ~60s, or a score-sum drop that isn't "pretty similar" (treat >~3-5% aggregate drop, i.e.
  roughly 50-90 points off 1769.835, as the line to flag and discuss rather than a hard
  auto-reject — this is a judgment call for the user, not an automatic cutoff).
- **Tie-break if both qualify:** prefer the smaller/less cross-cutting change (Approach A) per
  the general engineering bias toward minimal diffs, unless Approach B's numbers are
  meaningfully better (e.g. materially faster or higher-scoring) or Approach A required the
  emit-time gate from §5.3 with its own added complexity.
- Report findings back to the user before implementing either on a real feature branch;
  do not merge speculative worktree work into `known-issues-fixes` directly.

### 8.4 Cleanup

`git worktree remove` both scratch worktrees once a decision is made and the winning approach
has been re-implemented cleanly on its own feature branch per `git-workflow.mdc` (fresh branch
off updated `main`/`known-issues-fixes` as appropriate, not a carry-over of the worktree's
history).
