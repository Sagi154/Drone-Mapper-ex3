# Independent component variants — design

**Date:** 2026-08-28  
**Status:** Accepted 2026-08-28 (single-host placement; single-PC workflow; VAR-02 findings gate)  
**Goal:** Verify the assignment's independence requirement — each part must work with another team's implementation of the other parts — by exercising our shipped `.so` files against components authored **without any knowledge of our implementation**, and by exercising our host against plugins that behave legally but differently from ours.

## Problem

Assignment 3 requires that the Simulator, Algorithm, and MissionControl each work with another team's implementations of the other parts. Our current test surface does not prove that:

- Unit tests in `MissionControl/tests/` and `Algorithm/tests/` use in-process fakes (`FakeMap3D`, `ScriptedAlgorithm`, etc.), never a foreign `dlopen`'d host.
- Manual scripts (`check_isolation.sh`, `check_multi_plugin_outputs.sh`) already load distinct fixture `.so`s through our real CLI, but those fixtures are trivial stubs (`Finished` immediately / empty `MissionRunResult`). They prove the loader works, not that our plugins are independent.
- The highest-risk coupling is MissionControl → Algorithm via the **output map**: `MappingAlgorithmDependencies.output_map` is the map the MissionControl fuses. Our planner's soft-cost scheme (Empty preferred over Unmapped) assumes free-space carving that the frozen `IMutableMap3D` never mandates. A foreign MissionControl that only marks lidar hits as `Occupied` is legal and can collapse our coverage under `-competition` (grader MissionControl + our Algorithm).

## Non-goals

- No second real Simulator. There is no submission slot for it; a test-only host gives the same signal.
- No assertion on any specific score value; scoring is unspecified (`docs/open-questions.md` #3/#4). Assert shape, absence of crashes, and absence of collisions only.
- No ex2-style bug-injection ceremony (`.cursor/rules/testing-requirements.mdc`).
- Not displacing mandatory zip work (Known Issues `.xlsx`, pre-submission packaging) or inventing graded requirements the docx never states.

## Constraints (project rules)

- **Frozen interfaces:** no published header signature or folder changes; fixtures and host compile against skeleton headers only.
- **Skeleton-blind authorship:** every variant is authored in a Cursor window opened at `../ex_3_skeleton`, which physically cannot see `Drone-Mapper-ex3` (same isolation as `docs/instructor-test-suite-approach.md` Phase A).
- **Placement / ZIP-13:** simulator-role objects must not appear under `Algorithm/` or `MissionControl/`. Host fakes use distinct names (`HostMap3D`, `HostLidar`, …) so the `ZIP-13` mocks grep stays unambiguous.
- **Plugin build:** each fixture `.so` is `SHARED`, `PREFIX ""`, links `common::common` only, registration ctor left undefined and resolved from the host at `dlopen` (`ENABLE_EXPORTS ON` on the host). Mirror existing `Simulator/CMakeLists.txt` fixture targets.
- **Git:** feature branch; Conventional Commits; human approval before each commit.
- **Budget:** deadline Sep 6; instructor-catalog follow-up roadmap is **finished** (2026-08-28);
  execute VAR-01/VAR-02 next; VAR-03/VAR-04 only after packaging is safe.

## Isolation rule (applies to all four deliverables)

**Blind author may read:** `common/`, `Simulator/common_simulator/`, `MissionControl/common_mission_control/`, root `CMakeLists.txt` / `CMakePresets.json` / `README.md`, `inputs/`, and `context/`.

**Blind author must not read:** any of our sources, our `docs/`, or our `.cursor/rules/`.

Blind-phase deliverable: source files plus a short note listing every assumption the headers forced (e.g. "assumed `atVoxel` outside bounds returns `Unmapped`"). Those notes are the interesting output — each is a place where two teams could legitimately differ.

Phase B (wiring into our CMake, our `check_*.sh` conventions, our ID-suffixed artifact names) happens on this same PC after the blind sources exist. It must not soften any assertion to match what our code currently does. Isolation is preserved by **window**, not by person: blind authoring stays in an `ex_3_skeleton`-only Cursor window; Phase B and all work that reads our sources stay in the `Drone-Mapper-ex3` window. Do not paste our implementation details into the blind window's chat.

**Precondition:** run `verify-interfaces-vs-skeleton` first. If the local skeleton clone is stale relative to upstream, or any header differs from ours, resolve that before authoring. Blind code compiles against skeleton headers but links into our build.

## Why a single host (placement decision)

Three constraints pull in different directions:

1. `.cursor/rules/testing-requirements.mdc` — keep tests next to the project they exercise → suggests `Algorithm/tests/` for a foreign host of our Algorithm.
2. `ZIP-13` / Structuring the project — simulator-role objects must not leak into Algorithm or MissionControl.
3. `e10` — a host that drives our Algorithm and a host that drives our MissionControl need the same fakes; splitting duplicates code, and `UserCommon/` cannot hold them (it compiles into both shipped plugins).

**Resolution (accepted): one host, not two.** A single blind host executable takes both plugin paths on its CLI and can drive any combination (our Algorithm under foreign MissionControl, our MissionControl under foreign Algorithm, or both ours). It lives under `Simulator/tests/` (correct role, no `ZIP-13` risk, reuses fixture CMake). Adversarial plugin variants sit next to existing fixtures. Nothing lands in `Algorithm/tests/` or `MissionControl/tests/`.

## Architecture

```text
Blind phase (ex_3_skeleton Cursor window on this PC)
  → VAR-01 skeleton_host sources + own registration ctor bodies
  → VAR-02 foreign MissionControl .so sources
  → VAR-03 adversarial Algorithm / MissionControl .so sources
  → VAR-04 baseline Algorithm .so sources
  → assumption notes per variant

Phase B (Drone-Mapper-ex3 Cursor window on this PC)
  → copy/wire blind sources into Simulator/tests/...
  → CMake SHARED targets + ENABLE_EXPORTS host
  → check_*.sh black-box scripts
  → run_all.sh inclusion
  → no softened assertions
```

## Deliverables

### VAR-01 — Blind host executable (highest value, do first)

A test-only executable, `skeleton_host`, linking only `common::common` (plus `Simulator/common_simulator/` and `MissionControl/common_mission_control/` headers). Implements from scratch:

- `IMap3D` / `IMutableMap3D` backed by **staff `inputs/` maps** (one or several scenarios — e.g. house / small room / large room from the skeleton `inputs/` tree)
- Minimal loaders for the staff `.npy` maps and the YAML configs the host needs to build `MissionControlDependencies` / `MappingAlgorithmDependencies` (blind-authored; must not copy our `Map3DImpl` / NpyArray / YAML parsers)
- `IGPS`, `ILidar`, `IDroneMovement`
- Its own `MappingAlgorithmRegistration` / `MissionControlRegistration` constructor bodies (do **not** link our `simulator_registration` object library)

It `dlopen`s the plugin paths it is given (`RTLD_NOW | RTLD_LOCAL`), builds `MissionControlDependencies` / `MappingAlgorithmDependencies`, runs one mission per selected staff scenario, prints an observable summary (steps taken, voxels written by occupancy class, whether any movement was refused), and `dlclose`s after destroying every plugin object.

**CLI shape:**

```text
skeleton_host --algorithm=<so> --mission-control=<so> \
    --simulation=<inputs/...yaml> --mission=<inputs/...yaml> \
    [--drone=<inputs/...yaml>] [--lidar=<inputs/...yaml>]
```

Multiple staff scenarios are in scope: run the host once per composition cell of interest (or a small fixed list of staff map+mission pairs) via the Phase B `check_*.sh` script. Do not invent synthetic tiny grids as the primary path — staff maps are the ground truth for independence.

**What it catches:** any dependency our plugins have on our own `Map3DImpl` bounds/rounding, GPS at voxel centers, lidar hit ordering, `MockMovement` throw-vs-failed-result semantics, our registrar's timing, or our output-map offset convention. This is the "cross-team plugin test" that `docs/instructor-test-suite-approach.md` recommended and nobody built.

**Pass criteria:**

- Our `MissionControl_*.so` completes each selected staff scenario under the host without crashing, without an escaping exception, and without a movement into a voxel the host had marked `Occupied`.
- Our `Algorithm_*.so` returns a well-formed `MappingStepCommand` on every call and eventually reports `Finished` or is stopped by the host's step cap.

### VAR-02 — Foreign MissionControl variant

A blind `IMissionControl` (+ optional `IDroneControl`-shaped helper inside the same `.so`) that is legal but deliberately unlike ours:

| Our behavior (contamination risk) | Foreign variant |
|-----------------------------------|-----------------|
| Ray-carve `Empty` along lidar beams | Write only hits as `Occupied`; never carve free space |
| Batch consecutive scan commands into one step | Exactly one scan per step |
| Cache last scan → pass non-null `latest_scan` | Pass `latest_scan = nullptr` every `nextStep` |
| Output resolution from our mission wiring | Honor a coarser output resolution |

**What it catches:** planner soft-cost coupling to MissionControl fusion policy — the `-competition` path a grader would run (their MissionControl + our Algorithm).

**Pass criteria (diagnostic, not binary):** our algorithm must still be collision-free and must not stall at zero coverage.

**Findings gate (accepted):** default disposition is **bug to fix**. Before changing production Algorithm/MissionControl code, present the findings (what failed, which foreign behavior triggered it, observable evidence). We then decide per finding whether to fix or record a `docs/known-issues.md` row. Do not silently skip a finding or auto-write Known Issues without that review.

### VAR-03 — Adversarial plugin variants

Small blind plugins, one behavior each, proving containment black-box rather than unit-only. Fill in the pattern established by `faulty_wall_algorithm_plugin.cpp` and `unregistered_plugin.cpp`.

**Algorithm side:**

- Throws from `nextStep`
- Returns `AlgorithmStatus::Error`
- Requests movement into a voxel the map already reports `Occupied`
- Never reports `Finished` (max-steps path)
- Requests a scan orientation outside the lidar's configured range

**MissionControl side:**

- Throws from `runMission`
- Returns a `MissionRunResult` with implausible step counts
- Returns immediately without writing anything

**Pass criteria:** the affected run scores `-1`; the plugin filename appears in aggregate `errors: [...]` where the failure was a load/run failure; an `error.log` line is written immediately; sibling runs still complete; the process never crashes or calls `exit`.

### VAR-04 — Independent baseline algorithm (last, optional)

A blind, genuinely independent mapping algorithm (lawnmower or frontier-lite). Gives `-competition` two real competitors with different scores/steps so grouping and sort order are proven with real data, and double-checks that our `MissionControlDependencies` wiring works for an algorithm written by someone who never saw our code.

**Lowest priority:** report shape is already unit-tested and the metric is unspecified. Cut first if time is short.

## Placement and build

```text
Simulator/tests/hosts/skeleton_host/              VAR-01 host sources + own registration ctor bodies
Simulator/tests/fixtures/                         VAR-02, VAR-03, VAR-04 plugin sources (existing folder)
Simulator/tests/manual/check_foreign_host.sh
Simulator/tests/manual/check_foreign_mission_control.sh
Simulator/tests/manual/check_adversarial_plugins.sh
```

Each plugin variant gets a `SHARED` target following the existing pattern in `Simulator/CMakeLists.txt`: `PREFIX ""`, output into `PLUGIN_FIXTURES_OUT`, links `common::common` only, registration ctor left undefined and resolved from the host at `dlopen`. The host executable needs `ENABLE_EXPORTS ON`. New scripts get added to `run_all.sh` and to the catalog orchestrator (`.cursor/skills/verify-instructor-test-catalog/SKILL.md` — Point 4 done).

## Single-PC workflow

All work runs on this machine. There is no teammate split by person.

| Step | Where | Rule |
|------|--------|------|
| Blind author each variant | Cursor window rooted at `../ex_3_skeleton` | Do not open or paste from `Drone-Mapper-ex3` |
| Phase B wire + scripts | Cursor window rooted at `Drone-Mapper-ex3` | May read our sources; do not soften assertions |
| VAR-02 findings review | `Drone-Mapper-ex3` chat | Present findings → human fix-vs-Known-Issues decision → then implement |

Order: VAR-01 blind → VAR-01 Phase B → VAR-02 blind → VAR-02 Phase B + findings gate → VAR-03 → VAR-04 (optional). After Phase B for a variant, do not return to that variant's blind window with contaminated context; start a fresh blind window for the next variant if needed.

## Budget vs other work

Deadline **Sep 6**. `docs/assignment-compliance-pickup.md` still lists Known Issues `.xlsx` export and pre-submission packaging as the only mandatory open work. Assignment 3 never asks for tests.

**Prerequisite (cleared 2026-08-28):** `docs/instructor-test-catalog-followup-roadmap.md` Points 1–4 are **done** (Point 2 Tasks 1–8 + Point 4 orchestrator). Task 0 of the implementation plan still requires a skeleton-header verify before blind authoring; the roadmap half of that gate is satisfied.

| Priority | Action |
|----------|--------|
| Done | Instructor-test-catalog-followup-roadmap (Points 1–4) |
| Keep | Packaging + Known Issues excel |
| Next (execution unblocked) | VAR-01, VAR-02 (graded independence claim, currently untested) |
| Only after packaging safe | VAR-03, VAR-04 |

## Open decisions

1. ~~**VAR-02 late findings**~~ — **decided:** default is fix as bugs; present findings first; per-finding human choice of fix vs Known Issues row.
2. ~~**Scenario size for VAR-01**~~ — **decided:** staff `inputs/` maps (one or several scenarios), not a hand-built tiny grid. Blind host includes its own minimal `.npy` / YAML loaders; Phase B scripts exercise multiple staff map+mission pairs.

## Prerequisite / sequencing (not part of this plan)

`docs/instructor-test-catalog-followup-roadmap.md` is **finished** (2026-08-28). The items below were blocked-by-roadmap and are now complete outside this variants plan:

- Closing catalog Point 2 Tasks 4–8 — **done**
- Point 4 catalog orchestrator skill — **done** (`verify-instructor-test-catalog`)

They are not tasks inside the variants plan. Remaining Task 0 work is skeleton-header verification only.

## Out of scope for this spec's implementation plan

- Cross-team physical `.so` swap with another course team (forum-dependent; note as free option if allowed)
- Anything already covered by the finished catalog-followup roadmap (do not re-implement Tasks 4–8 or Point 4 here)

## Related docs

- `docs/instructor-test-suite-approach.md` — Phase A isolation pattern; cross-team plugin test recommendation
- `docs/instructor-test-catalog-followup-roadmap.md` — **done 2026-08-28**; variants plan may execute
- `docs/assignment-compliance-pickup.md` — remaining mandatory zip work
- `docs/open-questions.md` — unspecified scoring / grouping
- `.cursor/rules/plugin-architecture.mdc`, `testing-requirements.mdc`, `error-handling-logging.mdc`
