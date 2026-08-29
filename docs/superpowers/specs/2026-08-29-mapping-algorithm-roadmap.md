# Mapping Algorithm Roadmap — A/B/C/D

**Date:** 2026-08-29
**Status:** Accepted 2026-08-29
**Purpose:** Index and rationale for the four-project sequence that replaces `Algorithm/`'s action
policy, and a standing record of the considerations relevant to each not-yet-specced project — so
that specifying B, C, or D never depends on re-reading this chat.

This doc is living: as each project lands, update its "Considerations for the next spec" section
with what was actually learned (numbers, surprises, scope changes) before moving to the next one.

---

## Why four projects, and why this order

`docs/mapping-algorithm-analysis.md` (2026-08-29 review of `Algorithm/`) concluded the frontier idea
is sound but the action policy on top of it is weak: ~26 of every ~28 `nextStep` calls scan, movement
and scan are never combined, the 26-direction sweep isn't derived from the sensor, `max_steps` is
never read, and there are two never-cleared blacklists plus a Dijkstra search whose edge set changes
mid-search. Patching each finding individually costs about as much as replacing the policy and
leaves code no reviewer can characterize; a principled objective (next-best-view, budget-aware) makes
most of the findings disappear by construction. See that doc's "Better known algorithms" and
"Recommended pre-deadline stack" sections for the full alternatives survey — the deadline framing in
that doc's title section is explicitly not the constraint here; **quality is**.

The work is cut into four independently-specced, independently-landable projects because they are
not actually entangled the way they first look:

| Project | Scope | Depends on |
|---------|-------|------------|
| **A** — benchmark harness | Sweep tool, committed score/step CSVs, ex2 reference bands | Nothing. Spec: `docs/superpowers/specs/2026-08-29-algorithm-benchmark-harness-design.md` |
| **B** — MissionControl honesty | Exactly one scan per step; honor a command carrying both movement and scan | A (to measure the impact) |
| **C** — sensor model + belief map | Cone half-angle from `d`/`z_min`/`fov_circles`; own occupancy belief from `latest_scan`; fix the clearance-check no-op | A, B (to know how bad step inflation actually is) |
| **D** — exploration policy | Receding-horizon next-best-view; any-angle smoothing; retire the blacklists/stale cache/mid-search Dijkstra hack | A, B, C |

**A first:** nothing downstream is verifiable without it. Right now the only recorded fact about
ex3's algorithm is "24/24 `COMPLETED` with `mission_score >= 0`" — a floor, not a score
(`docs/assignment-compliance-pickup.md:29`, `docs/known-issues.md:16`). Ex2, by contrast, has real
per-group numbers (`Drone-Mapper-ex2/docs/HLD.md:387-402`) on byte-identical inputs and the same
scorer. A produces the ex2-comparable snapshot and the tooling to keep measuring.

**B second, deliberately before the policy is fixed:** honest step accounting (one scan per step) is
the correct reading of the contract, but ex2's `DroneControlImpl` batched an entire 26-direction
sweep into one step with no cap, so B will very likely turn today's 24/24 `COMPLETED` into
`MAX_STEPS` on tight-budget missions (`inputs/mission/large_mission_room.yaml`, 500 steps) and keep
it that way until D recovers it. Landing B before C/D means the fall is measured (via A) rather than
theoretical, and it means C and D are designed against the actual contract the algorithm will run
under, not the generous one.

**C third:** the belief map and gain-gated scanning are prerequisites for D's objective (expected
newly-observed voxels under the real sensor cone) to mean anything. Building it before D also
surfaces the clearance-check bug independently — see C's considerations below — which is a
correctness issue, not an efficiency one.

**D last:** the actual policy rewrite, once the ground under it (measurement, honest step contract,
real sensor model) is solid.

---

## Two decisions that constrain every project below

Made explicitly in this design conversation; do not re-derive or second-guess them without raising it
with the user first.

1. **Pessimism scope for our own MissionControl is "honest step accounting only."** Exactly one scan
   per step, and a command carrying both movement and scan is honored, not silently dropped. Our MC
   **keeps** carving `Empty` along beams and **keeps** passing non-null `latest_scan` — those are what
   make it a good host, and it is graded in comparative mode hosting other teams' algorithms. The
   existing hits-only, no-carve, `nullptr`-scan fixture (`foreign_hits_only_mission_control_plugin`)
   remains a robustness regression check, never an optimization target.

2. **"Better algorithm" is measured against ex2's recorded scores on the same 24 cells**, not against
   `mission_score >= 0`. See project A's ex2 reference bands
   (`docs/superpowers/specs/2026-08-29-algorithm-benchmark-harness-design.md`, "Ex2 reference").

---

## Project A — status: done (harness + pre-B baseline committed)

Spec: `docs/superpowers/specs/2026-08-29-algorithm-benchmark-harness-design.md`.  
Plan: `docs/superpowers/plans/2026-08-29-algorithm-benchmark-harness.md`.  
Baseline: `docs/benchmarks/2026-08-29-pre_b_baseline.csv` (+ `.md`).

| Group | Ex2 band | Pre-B `ex2_comparable` mean | Verdict |
|-------|----------|-----------------------------|---------|
| house_lower | 100 | 100.00 | inside |
| house_full | 56–62 | **10.49** | **below** |
| large_out | 80–88 | **36.29** | **below** |
| large_room | 92–96 | 93.38 | inside |
| small_out | 75–89 | **37.32** | **below** |
| small_room | 87–90 | **65.35** | **below** |

Totals (`ex2_comparable`): score sum **1371.3** / 24 cells, **4946** steps, 0 errors.  
Adversarial column (hits-only MC): score sum **536.0**, **6982** steps, 4× `MAX_STEPS` — much worse, as expected.

**Implication for C/D:** today's algorithm already fails the beat-ex2 bar on four of six groups under
our own batching MC. Project B's de-batching will make step budgets tighter still; recovering
toward ex2 bands is the job of C+D, not of A.

---

## Project B — MissionControl honesty

**Status:** specced. Spec: `docs/superpowers/specs/2026-08-29-missioncontrol-step-honesty-design.md`.

### What's already decided

- Remove the scan-batching loop in `MissionControlImpl.cpp`... in `DroneControlImpl::step()`
  (`MissionControl/src/DroneControlImpl.cpp:187-214`) so at most one `lidar.scan(...)` happens per
  `step()` call — matching the foreign VAR-02 fixture's contract
  (`docs/superpowers/specs/2026-08-28-independent-component-variants-design.md:109-113`).
- Fix the bug where a command carrying both `movement` and `scan_orientation` silently loses its
  movement: the scan loop runs first (because `scan_orientation` is set) and overwrites `command` via
  `nextStep` before the movement block ever reads it (`DroneControlImpl.cpp:216-248`). This bug is
  currently latent — our algorithm never emits both (`Algorithm/tests/test_mapping_algorithm.cpp:355-370`)
  — but D's rewrite is expected to emit both, per project A's decision that this is the honest-target
  behavior to benchmark against.
- Keep Empty-carving (`ScanResultToVoxels::applyToMap`, `supplementGridAlignedFusion`) and keep
  passing non-null `latest_scan` — decision 1 above.
- Update docs that describe the old batching as current behavior: `docs/HLD.md:251-253`,
  `docs/known-issues.md` #20/#21, `docs/mapping-algorithm-analysis.md:140-151`,
  `docs/superpowers/specs/2026-08-28-independent-component-variants-design.md:109-113`,
  `.cursor/rules/frozen-interfaces.mdc:66-67` (which documents movement → scan → fuse order; actual
  code today is scan-batch → movement — B should make the code match the documented order, or fix the
  doc, whichever the implementation lands on. Resolve this explicitly, don't leave it split.).

### Resolved by the spec (superseding earlier speculation in this roadmap)

- **`nextStep` is called exactly once per `step()` call — the loop is removed entirely, not capped
  at 1.** Earlier speculation in this roadmap recommended keeping the loop shape with a hard cap,
  reasoning it stayed defensive against the VAR-03 hang class. The spec supersedes that: with no loop
  inside `step()`, `nextStep` can't be re-invoked regardless of what the algorithm returns, so the
  hang class is structurally impossible rather than bounded. Stronger fix, not a smaller diff.
- **Movement executes before the scan, and a recoverable movement failure still lets the scan
  happen** (stationary but functional) before returning `Continue`. A hard failure still
  short-circuits before any scan. This makes the code match the documented movement → scan → fuse
  order in `frozen-interfaces.mdc:66-67` — no rule change needed, only the code catching up.
- **No test currently hardcodes `kMaxScansPerStep == 16`** (re-verify by grep at implementation time,
  not by trusting this note) — full details and the new test list are in the spec.

### Considerations for whoever executes B

- Use project A's harness to capture `ex2_comparable` (pre-B) before starting, and to measure the
  `honest` column immediately after landing B, on the unmodified current algorithm — this is the
  "expected fall" measurement referenced in the roadmap's ordering rationale above. Expect and record
  a regression on tight-budget cells; do not treat it as a bug to fix within B's scope. C and D are
  what recovers it.

---

## Project C — sensor model + belief map

**Status:** not yet specced. Depends on A (harness exists) and B (real step-accounting numbers).

### Why this project exists (two distinct motivations — keep them separate)

1. **Gain-gating the scan pattern.** `buildScanOrientations` hardcodes 6 face + 12 edge + 8 corner
   directions regardless of the lidar (`docs/mapping-algorithm-analysis.md:111-127`). The actual cone
   half-angle follows from `d`, `z_min`, `fov_circles`: `lidar_short` (`fov_circles: 4`) roughly tiles
   the sphere at 45° spacing; `lidar_long` (`fov_circles: 3`) leaves ~17° wedges unscanned — a 44 cm
   gap at 150 cm range. `lidar_config_` is read only for `z_max` today
   (`Algorithm/src/MappingAlgorithmImpl.cpp:120`); `d`, `z_min`, `fov_circles` are never touched. Once
   B makes every scan cost a full mission step, re-scanning already-known space is direct waste of a
   now-expensive resource — this motivation gets **stronger** after B, not weaker.

2. **The clearance-check bug — a correctness issue, independent of B.** `isSpherePassable` computes
   `rx = ceil(radius_cm / step_cm)`; with drone radius 4 cm or 7.5 cm and the output map's 10 cm step
   (missions don't set `output_mapping_resolution_factor`), `rx = 1` and every non-zero probe offset
   fails the sphere filter — the ~55-line clearance function collapses to a single `atVoxel`
   (`docs/mapping-algorithm-analysis.md:163-181`). Since `Unmapped` is deliberately traversable (soft
   cost), the planner plans as a point drone and commands moves into unverified space. Our own MC
   forgives this via a `blocked`/`boundary` substring match
   (`MissionControl/src/DroneControlImpl.cpp:93-96`); **a foreign MC returning a hard `Error` scores
   that run `-1`** — a fixed cost far larger than any efficiency gain on the table, and it sits
   directly against the assignment's "do not fly into walls." Decision 1 above keeps *our* MC
   forgiving, but the adversarial/foreign-host benchmark column (project A) will expose this, and any
   competition host might not forgive it.

### What decision 1 changes about this project's original framing

Earlier framing (before decision 1 was made) assumed the primary benchmark host might pass
`latest_scan = nullptr` or skip Empty-carving, which would make "build our own belief map because we
can't trust the host's fusion" the main driver. **That's no longer the primary case** — our honest MC
(the primary benchmark target) always carves Empty and always passes a real scan. The belief-map work
is still worth doing, but its priority ordering shifts:

- **Primary:** fix the clearance/passability no-op (safety, see above) and derive scan geometry from
  the lidar config (efficiency, see above). These do not strictly require a separate belief map — they
  can be done against `output_map_` as today, with the sphere-probe math fixed to actually probe at
  the output map's real resolution.
- **Secondary, worth doing but not blocking D:** raycasting `latest_scan` into our own occupancy
  representation, decoupling from the host's fusion policy, and degrading gracefully when scans are
  `nullptr` (still needed for robustness against the adversarial column and any genuinely foreign
  host in `-competition` mode, where our algorithm runs under *someone else's* MC — see
  `docs/mapping-algorithm-analysis.md`, section "What the assignment actually measures", point 1).
- Shared beam math, if built, goes in `UserCommon/` compiled into each plugin separately (no cross-`.so`
  symbol dependency) — this is what the no-duplicate-logic rule (e10) and
  `docs/mapping-algorithm-analysis.md`'s alternative #8 both point at. `ScanResultToVoxels` and
  `BeamMath.hpp` in `MissionControl/src/` are the existing implementation to share from, not duplicate.

### What's genuinely open (resolve when specced)

- Whether the sphere-probe fix alone (making `isSpherePassable` probe at real resolution) is
  sufficient, or whether the planner needs to additionally treat `Unmapped` as non-traversable near
  frontiers until confirmed clear — these have different coverage/safety tradeoffs and should be
  decided with A's measurements in hand, not guessed.
- Exact cone half-angle formula and how many directions a Fibonacci-lattice layout needs per lidar
  config — needs a short derivation, not just the two example angles already computed
  (`docs/mapping-algorithm-analysis.md:116-118`).
- Whether raycasting `latest_scan` ourselves is in C's scope or deferred to D, once D's actual
  viewpoint-scoring needs are known (D needs *some* occupancy belief to score candidate viewpoints
  against — C should produce whatever minimal form D needs, not more).

---

## Project D — exploration policy (next-best-view)

**Status:** not yet specced. Depends on A, B, C.

### What's already decided

- Algorithm: receding-horizon next-best-view (Bircher et al. 2016) — sample candidate
  (position, orientation) viewpoints, score each by expected newly-observed voxels under the real
  sensor cone minus travel cost, execute the first edge, replan. This directly optimizes
  coverage-per-step, which is exactly the competitive report's sort key: `total_score` descending,
  then `total_steps` ascending (`Simulator/src/io/CompetitiveReportWriter.cpp:32-37`,
  `docs/mapping-algorithm-analysis.md:38-40`).
- Budget-aware: must read `mission_config_.max_steps` (never read today,
  `Algorithm/src/MappingAlgorithmImpl.cpp` — grep confirms zero production reads) and behave as an
  anytime policy — thorough while budget is loose, greedy about gain-per-step when tight.
- Any-angle path smoothing (Theta* / lazy Theta*, or line-of-sight string-pulling over the existing
  path) — the planner is 6-connected today so paths are Manhattan staircases, and `Rotate`/`Advance`/
  `Elevate` are separate commands, so every turn costs a whole step
  (`docs/mapping-algorithm-analysis.md`, alternative #4).
- Hard invariant: no emitted movement may enter a cell the belief map (project C) hasn't cleared —
  this is what actually fixes the clearance-check risk, not just the sphere-probe patch.
- Emit movement and scan in the same command wherever both are beneficial in one step, now that
  project B makes MissionControl honor that instead of discarding the movement.
- Written in `mp-units` strong types from the start (`Position3D` `operator+`/`operator-` from ex3
  are already available) — retires the "raw doubles for all geometry" finding, called the single
  largest rubric exposure in `Algorithm/` (`docs/mapping-algorithm-analysis.md:243-249`,
  `.cursor/rules/mp-units-strong-types.mdc`).
- The existing frontier/Dijkstra machinery (`MappingAlgorithmFrontier`) survives as the
  reachability-and-path substrate under the new objective, not as the objective itself. Concretely
  retired by the rewrite, not patched: `blocked_cells` and `frontier_visit_counts` (never cleared),
  `explore_dist_cache` (stale exactly when reused on the stuck path), the mid-search edge-set change
  that voids Dijkstra's optimality (`MappingAlgorithmFrontier.cpp:428`), and the near-floor visit-count
  special case (`kMaxNearFloorFrontierVisits`) — all catalogued in
  `docs/mapping-algorithm-analysis.md`, "Implementation findings, by impact".
- Dead code with zero callers (`findFarthestPath`, `findGreedyUnknownStep`, ~120 lines,
  `docs/mapping-algorithm-analysis.md:237-241`) is deleted, not carried forward.
- Must be deterministic (fixed seed for any viewpoint sampling) — required by project A's harness to
  remain a valid comparison tool.

### What's genuinely open (resolve when specced)

- Exact viewpoint-sampling strategy (how many candidates, what radius, how travel cost is weighted
  against information gain) — this needs project C's belief map interface to exist first, and ideally
  a couple of harness runs to tune against, not a guess made before either exists.
- Known score ceiling to accept, not chase: `docs/known-issues-guidelines.md:63` and
  `docs/mapping-algorithm-analysis.md` note the house-full floor layer sits below lidar `z_min` and is
  structurally undetectable — ex2 topped out at 56–62% there for the same reason. D should not spend
  effort trying to beat that specific ceiling; it's physical, not a policy gap.
- Whether frontier clustering + tour ordering (FUEL/TARE) is worth a follow-up project after D lands,
  if NBV's local replanning still shows revisit oscillation on the harness — explicitly deferred, not
  in D's initial scope (`docs/mapping-algorithm-analysis.md`, alternative #6).
- ALG28's unbounded-BFS-hang risk: `docs/ex2-grading-handoff.md` notes the ex3 port of
  `MappingAlgorithmFrontier` still has this hang class if `isSpherePassable` is mutated to always
  return true; whatever replaces the passability check in C/D must keep an explicit bound, not
  reintroduce an unbounded walk.

---

## Related docs

- `docs/mapping-algorithm-analysis.md` — the review that this whole roadmap responds to
- `docs/superpowers/specs/2026-08-29-algorithm-benchmark-harness-design.md` — project A's full spec
- `docs/known-issues.md` — #20 (foreign MC step inflation, moves once B lands), #21 (batching cap's
  origin, revisited by B)
- `docs/superpowers/specs/2026-08-28-independent-component-variants-design.md` — VAR-02 foreign MC
  contract that B aligns toward
- `docs/ex2-grading-handoff.md` — ALG28 hang, ex2's official grade context (not a mapping-score source)
- `Drone-Mapper-ex2/docs/HLD.md:387-402` — ex2's per-group score bands, the bar for decision 2
