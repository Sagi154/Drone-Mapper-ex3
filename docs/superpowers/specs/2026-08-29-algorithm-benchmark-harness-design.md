# Algorithm Benchmark Harness — Design

**Date:** 2026-08-29
**Status:** Accepted 2026-08-29
**Goal:** Produce a comparable, committed table of per-cell mission score, step count, and status
across the 24-cell `inputs/sim_compose.yaml` matrix, so that later algorithm work can be judged
against ex2's recorded scores instead of against the `mission_score >= 0` floor.

This is project **A** of four. See "Where this fits" below.

---

## Problem

Ex3 has never recorded a mission score. Every status line in the repo says the same thing —
`inputs/sim_compose.yaml` is "24/24 `COMPLETED` with `mission_score >= 0`"
(`docs/assignment-compliance-pickup.md:29`, `docs/known-issues.md:16`, `docs/workplan.md:4`,
`docs/superpowers/specs/2026-08-27-wall-collision-recovery-and-planner-design.md:88`). That is a
floor, not a measurement: a cell scoring 12 and a cell scoring 96 both satisfy it. No report
artifact is committed anywhere in the tree, and `.gitignore` excludes run output directories.

Ex2, by contrast, recorded real numbers. `Drone-Mapper-ex2/docs/HLD.md:387-402` gives a Jul 5 2026
measurement over all 24 runs:

| Scenario group | Ex2 score band |
|----------------|----------------|
| house_lower × 4 | 100 |
| house_full × 4 | 56–62 |
| large_out × 4 | 80–88 |
| large_room × 4 | 92–96 |
| small_out × 4 | 75–89 |
| small_room × 4 | 87–90 |

That averages roughly 84.6. The inputs are directly comparable: `scenario_small.npy`,
`scenario_big.npy`, and `scenario_house.npy` are byte-identical between the two repos, the
composition matrices are the same 24 combinations, and `Simulator/src/MapsComparison.cpp` is a
port of ex2's scorer with the same two-pass BFS-reachability formula.

Consequently we cannot currently answer the only question that matters for the class competition:
is our algorithm better or worse than the one we submitted last time?

### The step-accounting asymmetry

The comparison is not naively apples-to-apples. Ex2's `DroneControlImpl::step()` batched scans with
**no cap at all**:

```cpp
// ex1 performs a full spherical sweep in one tick; batch consecutive scan
// commands into a single mission step so step budget is not spent on aiming.
while (command.scan_orientation.has_value() &&
       command.status == types::AlgorithmStatus::Working) {
```

(`Drone-Mapper-ex2/src/DroneControlImpl.cpp:243-246`.) A full 26-direction sphere sweep therefore
cost ex2 exactly one mission step. Ex3 capped it at `kMaxScansPerStep = 16`
(`MissionControl/src/DroneControlImpl.cpp:187-214`) as the fix for the VAR-03 hang recorded as
Known Issues #21 — a robustness fix, not a considered policy decision.

Project B will make our MissionControl do exactly one scan per step, which is the honest reading of
the contract and what a foreign MissionControl does. That change costs roughly 26× more steps per
observation stop, against budgets as tight as 500 steps (`inputs/mission/large_mission_room.yaml`).
The harness must therefore keep the two regimes in separate columns, or every future comparison
will silently conflate an algorithm change with a step-accounting change.

---

## Where this fits

Four projects, in dependency order. Each gets its own spec and plan.

| Project | Scope |
|---------|-------|
| **A** (this spec) | Benchmark harness. Touches no production code. |
| **B** | MissionControl honesty: exactly one scan per step, and honor a command carrying both a movement and a scan instead of discarding the movement (`MissionControl/src/DroneControlImpl.cpp:187-214` overwrites `command` inside the scan loop). Plus the HLD, Known Issues, and VAR-02 doc updates. |
| **C** | Sensor model and belief map in the algorithm: cone half-angle derived from `d` / `z_min` / `fov_circles`; raycast `latest_scan` into our own occupancy belief; degrade to movement-derived free space when scans are absent; beam math shared as `UserCommon/` sources compiled into each plugin separately. |
| **D** | Exploration policy: receding-horizon next-best-view over sampled (position, orientation) viewpoints scored by expected newly-observed voxels minus travel cost; budget-aware; any-angle path smoothing; a hard guarantee that no emitted movement enters an uncleared cell. Frontier/Dijkstra code survives as the reachability substrate. Written in `mp-units` strong types. |

B lands second, immediately after this harness, because making our own MissionControl the honest
benchmark host is what gives every later measurement meaning. The consequence is accepted
deliberately: B will very likely turn today's 24/24 `COMPLETED` into `MAX_STEPS` on the tight-budget
missions and keep it that way until D recovers it. That is the point of capturing the
`ex2_comparable` snapshot in project A first — so the fall is on record and the recovery is provable.

Two decisions from the design discussion constrain A:

1. **Optimization target is a pessimistic host, in the sense of honest step accounting only.**
   Exactly one scan per step, and movement plus scan honored in the same step. Our MissionControl
   keeps carving `Empty` along beams and keeps passing `latest_scan`, because those make it a good
   host and it is graded in comparative mode under other teams' algorithms. The adversarial
   hits-only fixture remains a robustness check, not an optimization target.

2. **Success criterion is beating ex2's recorded scores on the same 24 cells.** Not merely
   `mission_score >= 0`.

---

## Architecture

A Python sweep tool that shells out to the existing `simulator_207190406_209543255` CLI, parses the
per-plugin YAML it already writes, and emits a committed CSV plus a markdown summary. It adds no
production code and reimplements no simulator logic.

```text
scripts/benchmark/
  run_benchmark.py        orchestrator + CLI
  report_parser.py        simulation_output.yaml -> labelled rows
  cell_labels.py          config_indices -> human-readable cell name
  summarize.py            totals, ex2-band comparison, baseline diff
  requirements.txt        pinned deps (pyyaml)
  tests/
    test_report_parser.py
    fixtures/sample_simulation_output.yaml

docs/benchmarks/
  ex2-reference.csv       ex2 HLD bands, with citation
  <date>-<label>.csv      committed sweep results
  <date>-<label>.md       committed sweep summary
```

### Columns

| Column | Host | Lifetime | Role |
|--------|------|----------|------|
| `ex2_comparable` | Our shipped MC with production code as at commit `b1e141c` (batches up to 16) | **One-time snapshot**, captured in project A before any production change | The only apples-to-apples comparison with ex2's bands that will ever exist |
| `honest` | Our MC after project B (one scan per step, movement+scan honored) | Ongoing, primary | Competition proxy; the number every later change is judged on |
| `adversarial` | `foreign_hits_only_mission_control_plugin.so` (existing fixture) | Ongoing, secondary | Robustness regression; explicitly not an optimization target |

Project A can populate `ex2_comparable` and `adversarial` only. The `honest` column is defined and
supported by the harness here, but is first measurable when project B lands — which is the event
that makes it the primary column.

Because the harness lives under `scripts/` and touches no production code, the `ex2_comparable`
snapshot may be taken from this branch's tip: the built plugins and simulator are identical to
those at `b1e141c`.

The `ex2_comparable` column is deliberately a historical snapshot rather than a permanent host
fixture. Preserving today's batching behavior as a fixture would mean duplicating
`DroneControlImpl`'s logic, which collides with the no-duplicate-logic rule (e10) and adds a second
MissionControl to maintain. Reproducibility comes from git instead: check out `b1e141c`, build, run.

### Driving the simulator

The existing CLI already expresses both comparisons, so the harness only chooses a mode:

| Harness mode | CLI | Varies | Fixed |
|--------------|-----|--------|-------|
| `hosts` | `-comparative simulation=… mission_control_folder=<staged> algorithm=<algo.so>` | MissionControl plugins | one algorithm |
| `algorithms` | `-competition simulation=… mission_control=<mc.so> algorithms_folder=<staged>` | Algorithm plugins | one host |

`hosts` mode is how the three columns above are produced: staging several MC plugins in one
`mission_control_folder` yields one `<plugin>_simulation_output.yaml` per host from a single
invocation. `algorithms` mode is how a new algorithm is compared against the current one, or later
against an ex2-faithful baseline plugin.

Each invocation writes to a fresh scratch directory; the harness locates the produced
`comparative_results_<utc>` / `competition_<utc>` directory rather than assuming a path.

### Cell labelling

Each run entry in `<plugin>_simulation_output.yaml` carries
`config_indices: {simulation, mission, drone, lidar}`. `cell_labels.py` resolves those against
`inputs/sim_compose.yaml` into a label of the form
`house_simulation+house_mission_full|drone_large|lidar_short`.

**Known risk:** the harness's index enumeration could diverge from the simulator's — specifically
whether `simulation` indexes the `simulations` list with `mission` indexing within that entry's
`mission_configs`, or whether the (simulation, mission) pair is flattened into one index. The
enumeration must be pinned against the C++ (`Simulator/src/io/` composition parsing and the run
matrix construction) during implementation, not guessed.

**Mitigation:** a full sweep asserts exactly 24 rows per column, with every observed index
combination appearing exactly once. If the enumeration changes or the assumption is wrong, that
assert fires rather than producing a quietly mislabelled table.

Parsing uses `pyyaml`, not regular expressions. The `grep`/`re.finditer` snippets in
`docs/superpowers/plans/2026-08-26-default-composition-scoring.md:234` and
`docs/superpowers/plans/2026-08-27-wall-collision-recovery-and-planner.md` were adequate for
counting `>= 0` cells and are not adequate for a labelled table.

---

## Outputs

### Per-sweep CSV — `docs/benchmarks/<date>-<label>.csv`

One row per (column, cell):

```csv
column,cell,score,steps,status
honest,small_simulation_room+small_mission_room|drone_small|lidar_short,88.75,209,COMPLETED
```

Committing this is the point of the project: it makes algorithm progress a reviewable git diff
rather than terminal scrollback.

### Per-sweep summary — `docs/benchmarks/<date>-<label>.md`

Per column: `total_score`, `total_steps`, cells scored, cells at `MAX_STEPS`, cells errored. Then
the ex2 band comparison, then the baseline diff if one was given.

### Ex2 reference — `docs/benchmarks/ex2-reference.csv`

The six scenario groups as `(group, low, high)` bands, carrying a citation to
`Drone-Mapper-ex2/docs/HLD.md:387-402` and the Jul 5 2026 measurement date. The summary reports,
per group, whether the sweep is below, inside, or above the band.

This stays explicit that ex2 recorded **group bands, not per-cell values**, so a per-cell claim
against ex2 is not available and will not be manufactured. Group-level "below / inside / above" is
the strongest honest statement the data supports.

### Baseline diff

`--baseline <csv>` produces per-cell deltas and an explicit regression list (cells whose score
dropped or whose status got worse). A missing baseline file skips the diff rather than failing.

---

## Error handling

| Condition | Behavior |
|-----------|----------|
| Simulator exits non-zero | Record that column as failed with the captured stderr; continue other columns; exit non-zero at the end |
| Expected report directory or per-plugin YAML absent | Same as above |
| A cell scores `-1` | Recorded as `-1` with its status and error text — never dropped or coerced |
| A cell is `MAX_STEPS` | Recorded with its score; counted separately in the summary |
| Row count differs from the composition's expected cell count, or an index combination repeats | Hard failure with the offending indices named — a mislabelled table is worse than no table |
| `--baseline` file missing | Warn, skip the diff, continue |

Determinism: runs pin `num_threads` so scheduling cannot perturb results.
`Simulator/tests/manual/check_threading.sh` already establishes that report contents are
thread-count-independent, so one run per column suffices.

**Constraint this places on project D:** if the new policy samples viewpoints, it must sample from a
fixed seed. A nondeterministic algorithm makes the harness useless as a comparison tool.

---

## Runtime

A full comparative sweep of `inputs/sim_compose.yaml` is roughly 276 s at 8 threads
(`docs/assignment-compliance-pickup.md:85`), so about 14 minutes for three columns.

For fast iteration, `--composition <path>` accepts an alternate composition file and defaults to
`inputs/sim_compose.yaml`; `--quick` is shorthand for the existing
`Simulator/tests/fixtures/tiny_compose.yaml` (one cell). The 24-row invariant applies **only** when
the composition is `inputs/sim_compose.yaml`; on any other composition the harness computes the
expected cell count from the file and asserts against that instead. Quick sweeps are for iteration
and are not committed to `docs/benchmarks/`.

---

## Testing

The harness is a developer tool, so testing targets only the parts that can silently corrupt
results: parsing and labelling. `tests/test_report_parser.py` covers both `report_parser.py` and
`cell_labels.py` against a committed `fixtures/sample_simulation_output.yaml`, asserting the
resulting labelled rows, the 24-row / unique-index invariant, and correct handling of a `-1` cell
and a `MAX_STEPS` cell.

No test drives the simulator. Sweeps are minutes long and belong in a developer's hands, not in
`ctest`.

**Deferred:** once project D's algorithm settles, add a thin `ctest` gate asserting the committed
baseline has not regressed. Hardcoding per-cell score floors now would be actively counterproductive
while scores legitimately move in both directions during the rewrite.

---

## Placement and dependencies

The harness lives in `scripts/benchmark/`, alongside the existing `scripts/render_hld_pdf.sh`.
Because the submission zip has a mandated 5-folder structure, `pre-submission-review` must be run
after the directory is added to confirm it does not upset the `ZIP-*` checks.

Python dependencies (`pyyaml`) go in `scripts/benchmark/requirements.txt` installed into a
project-local environment. No global `pip install`, per the `managing-python-dependencies` skill.

---

## Success criteria

1. A single command produces a labelled 24-row table of score, steps, and status for a given
   (algorithm, host) pair, with a hard failure if the row count or index uniqueness invariant breaks.
2. `docs/benchmarks/` contains a committed `ex2_comparable` snapshot of the full 24-cell matrix,
   taken with production code as at commit `b1e141c` (that is, before project B), together with the
   ex2 reference bands.
3. The summary states, per scenario group, whether that snapshot is below, inside, or above ex2's
   band — the first time ex3's actual scores are on record.
4. `--baseline` produces a per-cell diff and an explicit regression list.
5. `report_parser` tests pass under `pytest`; `pre-submission-review` reports no new `ZIP-*` finding.

---

## Out of scope

- Any change to `Algorithm/`, `MissionControl/`, or `Simulator/` production code (projects B, C, D).
  In particular, this project does **not** change `kMaxScansPerStep` — that is project B.
- A `ctest` regression gate (deferred, see Testing).
- An ex2-faithful baseline algorithm plugin. Attractive later for a per-cell comparison rather than
  a group-band one, but it needs a faithful port of ex2's BFS policy — not today's ex3 port, which
  differs in planner search, scan-during-travel gating, and stall handling — and it would then need
  validating under ex3's `DroneControlImpl`, whose movement semantics changed.
- Plotting or any visualization beyond the markdown summary.

---

## Related docs

- `docs/mapping-algorithm-analysis.md` — the algorithm review that prompted this work
- `docs/known-issues.md` — #20 (foreign MC step inflation), #21 (the batching cap's origin)
- `docs/superpowers/specs/2026-08-28-independent-component-variants-design.md` — VAR-02 foreign MC
- `docs/open-questions.md` — `total_score` / `total_steps` definitions (working assumption)
- `docs/ex2-grading-handoff.md` — ALG28 unbounded-BFS hang, still present in the ex3 planner
