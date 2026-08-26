# Default Composition Scoring Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Get at least one cell of the provided `inputs/sim_compose.yaml` composition to finish as `Completed` or `MaxSteps` with `mission_score >= 0`, then confirm `-verbose` writes `*.verbose.txt` on that path.

**Architecture:** This is a diagnose → smallest fix → re-measure loop, not a rewrite. The default composition already runs end-to-end (CLI + reports); every cell currently ends `Error` / `mission_score < 0`. Leading evidence: `inputs/simulation/house_simulation.yaml` still has the pre–Jul 4 staff spawn (`height_cm: 10` → world z 160 → occupied NPY layer), while the working ex2 instructor set uses `height_cm: 150`. Fix that first, then chase whatever remains with error-log evidence only.

**Tech Stack:** C++20, gtest, yaml-cpp, TinyNPY, Docker image `drone-mapper-ex3-dev`, existing CLI (`simulator_207190406_209543255`).

## Global Constraints

- Never edit `common/`, `Simulator/common_simulator/`, or `MissionControl/common_mission_control/`.
- No `new`/`delete`; no `exit()`/`abort()`.
- Build and run only inside Docker (`drone-mapper-ex3-dev`), not the Windows host toolchain.
- Branch from updated `main`; kebab-case name with **no** workplan codes / owner names. Propose each commit and wait for human approval (`.cursor/rules/git-workflow.mdc`).
- Do **not** invent a fake “happy” composition to replace `inputs/sim_compose.yaml`. Prefer syncing instructor spawn values already proven in `../Drone-Mapper-ex2/tests/data/instructor/`.
- Do **not** rewrite the mapping algorithm, MockMovement, or scorer on speculation. Change code only when a specific error log / voxel check forces it.
- Out of scope: README rewrite, HLD PDF, Known Issues excel export (pickup items 2–3).
- Success bar (hard): **≥1** cell with status `Completed` or `MaxSteps` and `mission_score >= 0`. Stretch: maximize scored cells on the full 24-cell compose; do not block the plan on 24/24 if one cell is green and remaining failures are documented.

## Frozen surfaces (do not touch)

- Entire `common/`, `Simulator/common_simulator/`, `MissionControl/common_mission_control/`

After every code change:

```bash
git diff --name-status main -- common/ Simulator/common_simulator/ MissionControl/common_mission_control/
git status --porcelain -- common/ Simulator/common_simulator/ MissionControl/common_mission_control/
```

Empty on both = pass.

## How this plan expects debugging chaos

Expect several Docker comparative runs and at least one wrong hypothesis. Treat Task 3 as a **loop**, not a single patch:

```text
measure (error codes per cell)
  → one root cause from evidence
  → smallest fix
  → unit/ctest if the fix is code
  → re-measure
  → stop when success bar met OR remaining failures are listed in known-issues.md
```

Do not open parallel speculative refactors. One cause at a time.

## File map

- Likely modify: `inputs/simulation/house_simulation.yaml` — `height_cm: 10` → `150` (ex2 Jul 4 instructor fix)
- Possibly modify later (only if Task 3 evidence requires): Simulator / MissionControl / Algorithm / UserCommon sources involved in spawn, movement, or mission termination — **not** frozen headers
- Modify tests that encode the stale house z: `Simulator/tests/test_usercommon_simulation_coord_util.cpp` (comment + expected world z if the test hardcodes `10`)
- Add or extend: a small regression that house spawn is passable against `inputs/map/scenario_house.npy` (factory or coord-util test)
- Docs after green: `docs/assignment-compliance-pickup.md`, `docs/known-issues.md` (row 2), `docs/map3d-contract.md`, `docs/api-delta-ex2-to-ex3.md` (stale “inputs match Jul 2026 sync” claim), `docs/workplan.md`, `AGENTS.md`, canvas if open

## Leading hypothesis (verify in Task 1; do not skip Task 1)

| Source | `house_simulation.yaml` `height_cm` | World z (`+ height_offset 150`) | NPY z at 10 cm/voxel |
|--------|--------------------------------------|----------------------------------|----------------------|
| ex3 `inputs/` today | `10` | `160` | `1` — occupied floor (value `3`) |
| ex2 instructor set (Jul 4 fix) | `150` | `300` | `15` — first empty interior layer |

Other four simulation YAMLs already match ex2. If Task 1 shows non-house cells also `Error`, that is a **second** problem (often `MISSION_EXCEPTION` from MockMovement wall throws) — handle in Task 3 after house spawn is fixed.

## Docker commands (baseline)

From `Drone-Mapper-ex3/` on the host (PowerShell-friendly volume):

```bash
docker run --rm -e VCPKG_ROOT=/usr/local/vcpkg -v "${PWD}:/work" -w /work drone-mapper-ex3-dev bash -lc 'cmake --preset default && cmake --build --preset default && ctest --test-dir build/default --output-on-failure'
```

Comparative smoke (after build; copy plugins to a writable scratch like the manual scripts):

```bash
docker run --rm -e VCPKG_ROOT=/usr/local/vcpkg -v "${PWD}:/work" -w /work drone-mapper-ex3-dev bash -lc '
  set -euo pipefail
  BUILD=/work/build/default
  SCRATCH=/tmp/ex3_compose_diag
  rm -rf "$SCRATCH" && mkdir -p "$SCRATCH/mc"
  cp "$BUILD/MissionControl/MissionControl_207190406_209543255.so" "$SCRATCH/mc/"
  "$BUILD/Simulator/simulator_207190406_209543255" -comparative \
    simulation=/work/inputs/sim_compose.yaml \
    mission_control_folder="$SCRATCH/mc" \
    algorithm="$BUILD/Algorithm/Algorithm_207190406_209543255.so"
  REPORT=$(ls -d "$SCRATCH/mc"/comparative_results_*/comparative_report.yaml | head -1)
  echo "REPORT=$REPORT"
  grep -E "mission_status|mission_score|SPAWN_NOT_PASSABLE|MISSION_EXCEPTION" "$REPORT" | head -80
'
```

---

### Task 1: Measure — classify every cell’s failure

**Files:**
- Read only: latest `comparative_report.yaml` + per-run `*_error.log` under the results dir
- No production code changes in this task

**Interfaces:**
- Consumes: built default-preset binaries
- Produces: a short written tally (paste into the PR/commit notes): counts of `SPAWN_NOT_PASSABLE` vs `MISSION_EXCEPTION` vs other, and whether house cells are the only spawn failures

- [ ] **Step 1: Branch from updated main**

```bash
git checkout main
git pull
git checkout -b fix-default-composition-scoring
```

- [ ] **Step 2: Build in Docker**

Run the baseline `cmake --preset default && cmake --build --preset default` command above.

Expected: build succeeds.

- [ ] **Step 3: Run one comparative on `inputs/sim_compose.yaml` and tally**

Use the comparative smoke block above. Also open one house-cell and one non-house-cell `*_error.log`.

Record:

```text
SPAWN_NOT_PASSABLE: N
MISSION_EXCEPTION: N
other Error: N
Completed/MaxSteps with score>=0: N
```

Expected today (approx): `Completed = 0`; many `SPAWN_NOT_PASSABLE` and/or `MISSION_EXCEPTION`.

- [ ] **Step 4: Confirm house voxel math once (optional one-liner, no commit)**

If useful, inside Docker with Python/numpy or a tiny throwaway — world z 160 vs 300 on `inputs/map/scenario_house.npy`. Do not add a committed script unless Task 2 needs a lasting test.

- [ ] **Step 5: Decide Task 2 entry**

If house cells show `SPAWN_NOT_PASSABLE` and the YAML still has `height_cm: 10` → proceed to Task 2 (sync spawn).  
If house already has `150` or spawn is clean and everything is `MISSION_EXCEPTION` → skip Task 2 YAML edit; go to Task 3 with the error-log evidence.

No commit required for Task 1 unless you want a docs-only note (prefer waiting for a real fix).

---

### Task 2: Sync house instructor spawn (`height_cm: 150`)

**Files:**
- Modify: `inputs/simulation/house_simulation.yaml`
- Modify: `Simulator/tests/test_usercommon_simulation_coord_util.cpp` — update the house-offset unit test to use local z `150` → world z `300` (it currently documents `10` → `160`)
- Modify or extend: `Simulator/tests/test_simulation_run_factory.cpp` **or** `Simulator/tests/test_usercommon_simulation_coord_util.cpp` — add a regression that loads `inputs/map/scenario_house.npy` + house YAML spawn and asserts `isDroneSpawnPassable` for `drone_small` radius (`dimensions_cm: 8` → radius 4 cm)

**Interfaces:**
- Consumes: Task 1 confirmation that house spawn is blocked
- Produces: passable house world spawn `(150, 200, 300)` cm

- [ ] **Step 1: Write the failing regression (TDD)**

Add a test that:

1. Parses or hardcodes the **fixed** house pose: local height 150, offset 150 → world z 300.
2. Loads `inputs/map/scenario_house.npy` via the same `Map3DImpl` + `hiddenMapConfig` path the factory uses (offset = `map_offset`, resolution 10 cm).
3. Asserts `isDroneSpawnPassable(map, 4.0 * cm, world_spawn) == true`.

Also assert the **current broken** pose is not required to stay red forever — after the YAML fix, optionally keep a comment that world z 160 lands on occupied NPY z=1 (document in test comment, no need for a permanent failing assertion).

Update existing test comment/values:

```cpp
TEST(SimulationCoordUtil, WorldInitialDronePosition_HouseScenarioOffset) {
    // house_simulation.yaml: height_cm: 150, height_offset: 150 → world z 300
    simulator::types::SimulationConfigData simulation{};
    simulation.initial_drone_position =
        Position3D{150.0 * x_extent[cm], 200.0 * y_extent[cm], 150.0 * z_extent[cm]};
    simulation.map_offset =
        Position3D{0.0 * x_extent[cm], 0.0 * y_extent[cm], 150.0 * z_extent[cm]};

    const Position3D world =
        user_common_207190406_209543255::worldInitialDronePosition(simulation);
    EXPECT_DOUBLE_EQ(world.x.numerical_value_in(cm), 150.0);
    EXPECT_DOUBLE_EQ(world.y.numerical_value_in(cm), 200.0);
    EXPECT_DOUBLE_EQ(world.z.numerical_value_in(cm), 300.0);
}
```

- [ ] **Step 2: Run the new/updated tests — expect fail or stale comments until YAML matches**

```bash
docker run --rm -e VCPKG_ROOT=/usr/local/vcpkg -v "${PWD}:/work" -w /work drone-mapper-ex3-dev bash -lc 'cmake --build --preset default && ctest --test-dir build/default --output-on-failure -R "SimulationCoordUtil|test_simulation_run_factory"'
```

If the passability test loads the YAML from disk and YAML is still `10`, expect FAIL on passable assertion. Good.

- [ ] **Step 3: Apply the YAML fix (match ex2 instructor file)**

In `inputs/simulation/house_simulation.yaml`, change only the spawn height line to:

```yaml
    height_cm: 150        # local coords; world z = 150+150=300 = NPY z=15 (first empty layer)
```

Do **not** change x/y, offset, or other simulation YAMLs (they already match ex2).

- [ ] **Step 4: Re-run the same ctest filter**

Expected: PASS.

- [ ] **Step 5: Re-measure one comparative (full compose or house-only scratch)**

Re-run Task 1’s comparative command. Tally again.

Expected after this task alone:

- House cells: no `SPAWN_NOT_PASSABLE` (may still `MISSION_EXCEPTION` if algorithm hits a wall — Task 3).
- Non-house: unchanged vs Task 1 baseline.

- [ ] **Step 6: Propose commit (wait for human approval)**

```bash
git add inputs/simulation/house_simulation.yaml \
  Simulator/tests/test_usercommon_simulation_coord_util.cpp \
  Simulator/tests/test_simulation_run_factory.cpp
# (only the files you actually touched)

git commit -m "$(cat <<'EOF'
fix: sync house instructor spawn height to empty layer

EOF
)"
```

Message rationale: Jul 4 instructor/forum fix; world z 300 clears occupied NPY z=1.

---

### Task 3: Iterative loop — clear remaining Errors until ≥1 scored cell

**Files:**
- Unknown until evidence. Candidates (edit only what logs implicate):
  - `Simulator/src/SimulationRunFactoryImpl.cpp` / `SimulationRunImpl.cpp`
  - `Simulator/src/MockMovement.cpp`
  - `MissionControl/src/MissionControlImpl.cpp` / `DroneControlImpl.cpp`
  - `Algorithm/src/MappingAlgorithm*.cpp`
  - `UserCommon/src/SimulationCoordUtil.cpp`
- Never: frozen headers

**Interfaces:**
- Consumes: post–Task 2 compose tally
- Produces: ≥1 cell `Completed`/`MaxSteps` with `mission_score >= 0`

- [ ] **Step 1: Gate — check success bar**

If Task 2’s re-measure already has ≥1 scored `Completed`/`MaxSteps` → skip to Task 4.

- [ ] **Step 2: Pick the single most common remaining error code**

From the report / one representative `*_error.log`, classify:

| Evidence | First place to look | Disallowed first move |
|----------|---------------------|------------------------|
| Still `SPAWN_NOT_PASSABLE` on non-house | spawn vs map bounds/occupancy for that YAML; radius = `dimensions_cm / 2` | Rewriting the algorithm |
| `MISSION_EXCEPTION` + MockMovement / wall / occupied message | planned step into known Occupied / map wall; compare to mandatory Common-issues throw+catch path | Disabling MockMovement throws |
| `MISSION_EXCEPTION` other | stack the `ex.what()` string; fix that call site | Broad try/catch swallowing |
| Status `Error` without exception, score `-1` | `SimulationRunImpl` startup_errors / mission status mapping | Changing `MapsComparison` |

- [ ] **Step 3: Smallest fix for that one cause**

Prefer, in order:

1. Config/input sync already known from ex2 (spawn/bounds)  
2. One-line / local logic bug in ported code  
3. Narrow algorithm/planner fix only if logs show illegal step into confirmed Occupied

Re-read `.cursor/rules/frozen-interfaces.mdc` and `docs/error-handling-matrix.md` before changing catch boundaries (`DroneControlImpl` deliberately not catching MockMovement is already a Known Issues “different way” row — do not “fix” that unless it blocks scoring and you have no smaller option).

- [ ] **Step 4: Prove the fix**

- If code changed: add or update a focused unit test that fails without the fix.  
- Rebuild in Docker.  
- Re-run comparative; update the tally.

- [ ] **Step 5: Loop**

Repeat Steps 2–4 until the success bar is met. Cap thrashing: if three consecutive hypotheses fail, stop and re-read the latest error log from scratch (no larger rewrite).

- [ ] **Step 6: Propose commits at natural checkpoints**

One concern per commit (e.g. `fix: …`, `test: …`). Always wait for human approval before `git commit`.

---

### Task 4: Re-check `-verbose` file list on a completing composition

**Files:**
- Run: `Simulator/tests/manual/check_verbose.sh` (may still use full `sim_compose.yaml` — that is fine once at least one cell calls `runMission()` and writes a map)
- No production code expected if verbose wiring from the prior PR is intact

**Interfaces:**
- Consumes: ≥1 completing cell from Task 3
- Produces: with `-verbose`, at least one `*.verbose.txt` under the results tree; without `-verbose`, none

- [ ] **Step 1: Run the manual verbose script in Docker**

```bash
docker run --rm -e VCPKG_ROOT=/usr/local/vcpkg -v "${PWD}:/work" -w /work drone-mapper-ex3-dev bash -lc '
  sed -i "s/\r$//" Simulator/tests/manual/*.sh
  chmod +x Simulator/tests/manual/*.sh
  ./Simulator/tests/manual/check_verbose.sh /work/build/default
'
```

Expected: file list **with** `-verbose` includes `*.verbose.txt`; **without** does not.

If the script still shows no verbose files because Error cells dominate and no `output_map_file` path is usable, run a one-off comparative with `-verbose` and inspect a known completing cell’s plugin output directory instead. Do not weaken the factory verbose wiring.

- [ ] **Step 2: Frozen-interfaces check**

```bash
git diff --name-status main -- common/ Simulator/common_simulator/ MissionControl/common_mission_control/
git status --porcelain -- common/ Simulator/common_simulator/ MissionControl/common_mission_control/
```

Expected: empty.

- [ ] **Step 3: Full `ctest` once before docs**

```bash
docker run --rm -e VCPKG_ROOT=/usr/local/vcpkg -v "${PWD}:/work" -w /work drone-mapper-ex3-dev bash -lc 'ctest --test-dir build/default --output-on-failure'
```

Expected: all green.

---

### Task 5: Update compliance / pickup docs

**Files:**
- Modify: `docs/assignment-compliance-pickup.md` — move item 1 to Fixed; leave README/HLD as next
- Modify: `docs/known-issues.md` — row 2: either remove **FIX BEFORE SUBMIT** / mark resolved, or narrow to remaining unscored cells only
- Modify: `docs/map3d-contract.md` — house spawn note: `height_cm: 150` → world z 300 (delete the stale `10 → 160` line)
- Modify: `docs/api-delta-ex2-to-ex3.md` §7 — correct the claim that ex3 `inputs/` already matched the Jul 2026 spawn sync (house was still `10` until this fix)
- Modify: `docs/workplan.md` — verification / “still open” bullets that say all cells unscored
- Modify: `AGENTS.md` — status line: default-composition scoring progress
- Modify: `docs/integration-verification-report.md` — short note that scoring is no longer universally `-1` (optional one paragraph; do not rewrite the whole 2026-08-25 report)
- Modify: canvas `ex3-assignment-compliance.canvas.tsx` if present in the Cursor canvases folder

- [ ] **Step 1: Edit the docs to match measured results**

Pickup “Next session” should drop item 1 once evidence exists (report snippet: at least one `mission_score >= 0` and status Completed/MaxSteps). Keep items 2–3 (README/HLD, Known Issues excel).

- [ ] **Step 2: Propose docs commit**

```bash
git add docs/assignment-compliance-pickup.md docs/known-issues.md docs/map3d-contract.md \
  docs/api-delta-ex2-to-ex3.md docs/workplan.md AGENTS.md docs/integration-verification-report.md
# plus canvas path if updated

git commit -m "$(cat <<'EOF'
docs: record default composition scoring progress

EOF
)"
```

Wait for human approval.

---

## Self-review (plan author)

1. **Spec coverage:** Pickup item 1 (one completing scored cell) + follow-up `check_verbose.sh` + doc updates are all tasked. README/HLD/excel explicitly out of scope.
2. **Placeholders:** None — YAML values, Docker commands, and the debug loop decision table are concrete.
3. **Rules verified:** frozen interfaces listed; Docker-only; git-workflow human-approve commits; no algorithm rewrite by default; inputs sync preferred over fake compositions; AdvCpp happy-flow addressed without inventing ceremony.
4. **YAGNI:** No new orchestration layer, no custom full composition rewrite, no TSan re-run required for this item.

## Execution Handoff

Plan complete and saved to `docs/superpowers/plans/2026-08-26-default-composition-scoring.md`. Two execution options:

**1. Subagent-Driven (recommended)** — fresh subagent per task, review between tasks, fast iteration  

**2. Inline Execution** — execute tasks in this session using executing-plans, batch with checkpoints  

Which approach?
