# Wall Collision Recovery and Planner Improvements Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make all 24 `inputs/sim_compose.yaml` cells finish with `mission_score >= 0` in minutes-scale wall-clock by catching recoverable wall throws in DroneControl, learning stalls quickly, preferring Empty over Unmapped in path search, and keeping scan-during-travel on for long missions.

**Architecture:** Keep MockMovement throws (Common-issues). DroneControl catches `blocked`/`boundary` and Continues; SimulationRun remains the backstop. Algorithm: lower stall threshold; Dijkstra-style path costs so Unmapped is expensive but still passable; always enable scan-during-travel. House spawn `height_cm: 150` must already be on the branch.

**Tech Stack:** C++20, gtest, Docker `drone-mapper-ex3-dev`.

**Spec:** [docs/superpowers/specs/2026-08-27-wall-collision-recovery-and-planner-design.md](../specs/2026-08-27-wall-collision-recovery-and-planner-design.md)

## Global Constraints

- Never edit `common/`, `Simulator/common_simulator/`, or `MissionControl/common_mission_control/`
- MockMovement must keep throwing on real-map wall collision
- Do not treat Unmapped as fully impassable
- Named constants for stall ticks and traversal costs (AdvCpp e23)
- Docker-only build/test; human approval before each `git commit`
- Branch: continue `fix-default-composition-scoring` (has house spawn + partial DroneControl catch already in the working tree)
- Success: **24/24** scored `Completed`/`MaxSteps` with `mission_score >= 0`; compose finishes in **minutes**, not tens of minutes
- Out of scope: README, HLD PDF, optional Common-issues CI2–CI12

## Already in working tree (do not redo blindly)

- `inputs/simulation/house_simulation.yaml` `height_cm: 150` + coord-util regression
- `DroneControlImpl` try/catch recoverable throws → Continue
- `test_drone_control.cpp`: `CollisionBlockedThrowContinues` + `NonRecoverableThrowEscapesStep`

Task 1 verifies those are green and commits them with related docs/spawn if still uncommitted.

## File map

- Modify: `MissionControl/src/DroneControlImpl.cpp` (verify catch; already largely done)
- Modify: `MissionControl/tests/test_drone_control.cpp` (verify tests)
- Modify: `Algorithm/include/Algorithm/MappingAlgorithmImpl.h` — `kMaxMovingStallTicks` 8 → 2
- Modify: `Algorithm/src/MappingAlgorithmImpl.cpp` — `enable_scan_during_travel = true` always
- Modify: `Algorithm/src/MappingAlgorithmFrontier.cpp` (+ header if needed) — traversal cost helper + Dijkstra for movement path searches
- Modify: `Algorithm/tests/test_mapping_algorithm_frontier.cpp` — Empty preferred over Unmapped
- Docs: pickup, known-issues (2, 17, 18), AGENTS, workplan, integration note, canvas; include design spec path in docs commit if not already committed

---

### Task 1: Land DroneControl recovery + house spawn (verify + commit)

**Files:**
- Verify: `MissionControl/src/DroneControlImpl.cpp`, `MissionControl/tests/test_drone_control.cpp`
- Include already-fixed: `inputs/simulation/house_simulation.yaml`, `Simulator/tests/test_usercommon_simulation_coord_util.cpp`, `Simulator/CMakeLists.txt` if still uncommitted

**Interfaces:**
- Produces: recoverable wall throw → `DroneStepStatus::Continue`; non-recoverable still escapes

- [ ] **Step 1: Confirm catch block matches spec**

In `DroneControlImpl.cpp`, `executeMovement` must be in try/catch; `isRecoverableMovementFailure(ex.what())` → Continue; else `throw`.

- [ ] **Step 2: Docker GREEN for drone_control**

```bash
docker run --rm -e VCPKG_ROOT=/usr/local/vcpkg -v "${PWD}:/work" -w /work drone-mapper-ex3-dev bash -lc 'cmake --build --preset default --target test_drone_control MissionControl_207190406_209543255 && ctest --test-dir build/default --output-on-failure -R test_drone_control'
```

Expected: 100% pass (including `CollisionBlockedThrowContinues`, `NonRecoverableThrowEscapesStep`).

- [ ] **Step 3: Frozen check**

```bash
git diff --name-status main -- common/ Simulator/common_simulator/ MissionControl/common_mission_control/
git status --porcelain -- common/ Simulator/common_simulator/ MissionControl/common_mission_control/
```

Expected: empty.

- [ ] **Step 4: Propose commit (wait for human approval)**

```bash
git add MissionControl/src/DroneControlImpl.cpp \
  MissionControl/tests/test_drone_control.cpp \
  inputs/simulation/house_simulation.yaml \
  Simulator/tests/test_usercommon_simulation_coord_util.cpp \
  Simulator/CMakeLists.txt
git commit -m "$(cat <<'EOF'
fix: catch recoverable wall throws and sync house spawn

EOF
)"
```

(If spawn was already committed separately, omit those files.)

---

### Task 2: Fast stall + scan-during-travel always on

**Files:**
- Modify: `Algorithm/include/Algorithm/MappingAlgorithmImpl.h`
- Modify: `Algorithm/src/MappingAlgorithmImpl.cpp`

**Interfaces:**
- Consumes: DroneControl Continue on wall (Task 1) so position stays put
- Produces: stall after 2 same-position ticks; scan-during-travel enabled for all `max_steps`

- [ ] **Step 1: Change stall constant**

In `MappingAlgorithmImpl.h`:

```cpp
static constexpr int kMaxMovingStallTicks = 2;
```

(was 8)

- [ ] **Step 2: Always enable scan-during-travel**

In `MappingAlgorithmImpl.cpp` constructor / init (replace the `max_steps <= 500` gate):

```cpp
impl_->enable_scan_during_travel = true;
```

- [ ] **Step 3: Rebuild Algorithm tests in Docker**

```bash
docker run --rm -e VCPKG_ROOT=/usr/local/vcpkg -v "${PWD}:/work" -w /work drone-mapper-ex3-dev bash -lc 'cmake --build --preset default --target Algorithm_207190406_209543255 test_mapping_algorithm test_mapping_algorithm_frontier 2>/dev/null; cmake --build --preset default; ctest --test-dir build/default --output-on-failure -R "test_mapping_algorithm"'
```

Expected: existing Algorithm tests still PASS.

- [ ] **Step 4: Propose commit**

```bash
git add Algorithm/include/Algorithm/MappingAlgorithmImpl.h Algorithm/src/MappingAlgorithmImpl.cpp
git commit -m "$(cat <<'EOF'
fix: speed stall learning and always scan during travel

EOF
)"
```

Wait for approval.

---

### Task 3: Soft Unmapped traversal cost in frontier path search

**Files:**
- Modify: `Algorithm/src/MappingAlgorithmFrontier.cpp` (and `.h` only if declaring a helper)
- Modify: `Algorithm/tests/test_mapping_algorithm_frontier.cpp`

**Interfaces:**
- Consumes: `isSpherePassable` unchanged (Unmapped still passable)
- Produces: paths prefer Empty; Unmapped edges cost more

Current `findPath` / `findPathTo` / explore / unstick use **unweighted BFS** (`depth_of`, `queue`). Soft cost requires **integer Dijkstra** (or equivalent):

```cpp
namespace {
constexpr int kEmptyTraversalCost = 1;
constexpr int kUnmappedTraversalCost = 4; // named; tune only if compose needs it

[[nodiscard]] int traversalCost(const IMap3D& map, const Position3D& cell) {
    return occupancyAt(map, cell) == types::VoxelOccupancy::Unmapped
               ? kUnmappedTraversalCost
               : kEmptyTraversalCost;
}
} // namespace
```

For each movement path search that currently does `depth_of[neighbour] = current_depth + 1` and `std::queue`:

- Replace with `cost_of` map + `std::priority_queue` of `(neg_cost or min-heap pair<int,GridKey>)`
- Edge cost into `neighbour` = `traversalCost(map, neighbour_pt)`
- Relax if cheaper path found (standard Dijkstra; allow decrease-key via skip-if-worse on pop)
- Frontier **value density** score that divides by path length must use **path cost** (or step count from reconstructed path), not unweighted depth — keep behavior coherent: `value / path_cost`

Apply to all drone-movement searches in this file that use the BFS pattern (`findPath`, `findPathTo`, explore-to-unknown, unstick). Do **not** change `isSpherePassable` Unmapped policy.

- [ ] **Step 1: Write failing frontier test**

Add a test map: start in Empty; two routes to a frontier — short through Unmapped vs longer through Empty only. After soft cost, chosen path must go through Empty (or have lower Unmapped cell count). Use existing frontier test fixtures as style guide in `test_mapping_algorithm_frontier.cpp`.

- [ ] **Step 2: Run test — expect FAIL (or wrong path) before Dijkstra**

```bash
docker run --rm -e VCPKG_ROOT=/usr/local/vcpkg -v "${PWD}:/work" -w /work drone-mapper-ex3-dev bash -lc 'cmake --build --preset default --target test_mapping_algorithm_frontier && ctest --test-dir build/default --output-on-failure -R test_mapping_algorithm_frontier'
```

- [ ] **Step 3: Implement traversalCost + Dijkstra in path searches**

Minimal change: helper + convert the movement BFS loops; keep reconstructPath.

- [ ] **Step 4: Re-run frontier tests — GREEN**

- [ ] **Step 5: Propose commit**

```bash
git add Algorithm/src/MappingAlgorithmFrontier.cpp Algorithm/src/MappingAlgorithmFrontier.h \
  Algorithm/tests/test_mapping_algorithm_frontier.cpp
git commit -m "$(cat <<'EOF'
feat: prefer empty cells over unmapped in frontier paths

EOF
)"
```

Wait for approval.

---

### Task 4: Full composition re-measure

**Files:** none (verification only)

- [ ] **Step 1: Build all + comparative**

```bash
docker run --rm -e VCPKG_ROOT=/usr/local/vcpkg -v "${PWD}:/work" -w /work drone-mapper-ex3-dev bash -lc '
set -euo pipefail
cmake --build --preset default
BUILD=/work/build/default
SCRATCH=/tmp/ex3_wall_c
rm -rf "$SCRATCH" && mkdir -p "$SCRATCH/mc"
cp "$BUILD/MissionControl/MissionControl_207190406_209543255.so" "$SCRATCH/mc/"
/usr/bin/time -f "elapsed_sec=%e" "$BUILD/Simulator/simulator_207190406_209543255" -comparative \
  simulation=/work/inputs/sim_compose.yaml \
  mission_control_folder="$SCRATCH/mc" \
  algorithm="$BUILD/Algorithm/Algorithm_207190406_209543255.so"
OUT=$(ls -d "$SCRATCH/mc"/comparative_results_* | head -1)
YAML="$OUT/MissionControl_207190406_209543255.so_simulation_output.yaml"
python3 - <<PY
import re, pathlib
from collections import Counter
text = pathlib.Path("'"$YAML"'").read_text()
scores = [float(m.group(1)) for m in re.finditer(r"mission_score:\s*([-\d.]+)", text)]
statuses = re.findall(r"^\s+- status:\s*(\w+)", text, re.M)
print("n", len(scores), "ge0", sum(s>=0 for s in scores), "lt0", sum(s<0 for s in scores))
print("statuses", dict(Counter(statuses)))
print("MISSION_EXCEPTION", text.count("MISSION_EXCEPTION"))
print("scores", scores)
PY
find "$OUT" -name "*_error.log" -size +0c -print -exec head -2 {} \;
'
```

Expected: `ge0 == 24`, `MISSION_EXCEPTION == 0` (or only non-wall anomalies — investigate any non-empty error log), `elapsed_sec` on the order of **minutes** (target under ~10–15 min; if still >>20 min, stop and narrow — do not leave thrashing).

- [ ] **Step 2: Full ctest**

```bash
docker run --rm -e VCPKG_ROOT=/usr/local/vcpkg -v "${PWD}:/work" -w /work drone-mapper-ex3-dev bash -lc 'ctest --test-dir build/default --output-on-failure'
```

Expected: all green.

- [ ] **Step 3: Frozen check again**

Same as Task 1 Step 3 — empty.

If any cell still fails: capture that `*_error.log` and stop for a narrow follow-up (per spec). Do not expand optional CI rows.

---

### Task 5: Docs and Known Issues

**Files:**
- `docs/assignment-compliance-pickup.md`
- `docs/known-issues.md` (row 2 fixed/resolved; row 17 DroneControl now catches; row 18 note soft cost + fast stall)
- `AGENTS.md`, `docs/workplan.md`, `docs/integration-verification-report.md`
- Canvas if present
- `docs/superpowers/specs/2026-08-27-wall-collision-recovery-and-planner-design.md` (status → Accepted; include in commit if untracked)
- Prior pickup docs from house spawn if still uncommitted and accurate

- [ ] **Step 1: Update docs to measured 24/24 (or honest partial if Task 4 fell short)**

- [ ] **Step 2: Propose docs commit**

```bash
git add docs/ AGENTS.md
git commit -m "$(cat <<'EOF'
docs: record wall recovery, planner costs, and composition scores

EOF
)"
```

Wait for approval.

## Self-review (plan author)

1. Spec coverage: recovery, fast stall, soft Unmapped cost, scan-during-travel, verify, docs — all tasked.
2. No placeholders; Dijkstra concrete because current BFS is unweighted.
3. Rules: frozen, throw kept, Unmapped not hard-blocked, Docker, human commits.
4. Partial working-tree state called out in Task 1.

## Execution Handoff

Plan complete and saved to `docs/superpowers/plans/2026-08-27-wall-collision-recovery-and-planner.md`. Two execution options:

**1. Subagent-Driven (recommended)** — fresh subagent per task, review between tasks  

**2. Inline Execution** — execute in this session with checkpoints  

Which approach?
