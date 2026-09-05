# Optional Common-Issues Recovery (CI9, CI2/CI10, CI3, CI8) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.
>
> **Do not** use superpowers:subagent-driven-development. **Do not** dispatch Task/subagents. **Do not** select opus, gpt-5, or other expensive subagent models. Run this plan **inline** in the current session with the same model (`inherit`).

**Goal:** Implement four optional Common-issues recoveries in MissionControl — CI9 (continue after step `Error`), CI2+CI10 (ignore world-OOB moves; clamp to mission bounds), CI3 (retry invalid commands then throw), CI8 (split oversize moves into legal steps) — claim them in `bonus.txt`, drop the Known Issues rows, and prove each slice does not regress 24-cell scores or independence harnesses.

**Architecture:** All behavior lives in our MissionControl plugin. `MissionControlImpl` already owns `DroneControlImpl`. CI9 is a mission-loop change (log `DRONE_STEP_FAILED` and keep `max_steps`). CI2/CI10/CI3/CI8 are `DroneControlImpl::step` / `applyMovement` recoveries around the existing one-command path. `SimulationRunImpl::run` already catches exceptions from `runMission()` (`MISSION_EXCEPTION`) — CI3 relies on that, and must not wrap the new throw in the wall-collision `catch`. Our mapping algorithm is required not to emit illegal moves, so honest 24-cell **scores must stay bit-identical** to the 2026-09-03 baseline; any score change is a stop-the-line regression.

**Tech Stack:** C++20, gtest, mp-units, Docker image `drone-mapper-ex3-dev`, CMake presets `default` (unit tests) and `build/opt` Release (cell timing).

**Specs:** `docs/error-handling-matrix.md` (staff PDF), `docs/known-issues.md` rows 7 / 1 / 8 / 2 / 6, `docs/known-issues-explained.md`.

## Global Constraints

- Never edit `common/`, `Simulator/common_simulator/`, or `MissionControl/common_mission_control/` (frozen `IDroneControl` included).
- No `new`/`delete`; no `exit()`/`abort()`; no magic retry counts (e23 — named `constexpr`).
- Do not change Algorithm, MockMovement wall-throw (mandatory CI5), or Continue-on-`MovementResult::success==false` (that is **not** CI7).
- Do **not** implement CI4 (NOOP retry), CI6 (empty LiDAR retry), CI7 (retry Movement `false`), CI11, CI12.
- Do **not** add a wall-clock abort, alarm, or invented smaller `max_steps` in Algorithm or MissionControl (`verify-cell-runtime`).
- Do **not** globally unmask cone gain or start unrelated mapping-algorithm work.
- Git: confirm toplevel is this project (never `C:\Users\sagi1` home). Branch from updated `main`. Conventional Commits. **No commit without explicit human approval** (show `git status`, `git diff`, message; wait). Never `--no-verify`, never push `main`.
- Docker-only build/test on this Windows host. No `&&` / bash heredocs in PowerShell — one `docker run ... bash -lc '...'`.
- After **each** of the four issue groups: unit tests, then `verify-cell-runtime` (full 24, scores vs 2026-09-03 baseline), then `verify-independent-component-variants` (VAR-01..03; VAR-04 only in the final gate).
- Cell-runtime success: 24/24 COMPLETED, `mission_score >= 0`, scores match `docs/benchmarks/2026-09-03-branch-after-score-fix-per-cell-wall.csv`; expected WARNs only `large_out` × short lidar (~45–46s). Overall FAIL if any FAIL/HANG or score drift.
- Out of scope: Known Issues excel export, zip packaging, lazy `.so` load, `ISimulation` impl.

## File map

- Create: `MissionControl/include/MissionControl/MissionRunLoop.h` — testable mission loop used by CI9.
- Create: `bonus.txt` (zip-root claim file; first create in Task 2, append after each later issue).
- Modify: `MissionControl/src/MissionControlImpl.cpp` — call `runMissionSteps`; keep ctor/wiring.
- Modify: `MissionControl/CMakeLists.txt` — no new TU required if `runMissionSteps` stays in `MissionControlImpl.cpp`.
- Modify: `MissionControl/tests/test_mission_control.cpp` — FakeDroneControl + CI9 tests; later CI3 throw-through-MC.
- Modify: `MissionControl/include/MissionControl/DroneControlImpl.h` — `pending_movements_` only in the CI8 task.
- Modify: `MissionControl/src/DroneControlImpl.cpp` — helpers + CI2/CI10/CI3/CI8.
- Modify: `MissionControl/tests/test_drone_control.cpp` — AABB `FakeMap3D`, recorded advance distance, new tests.
- Modify docs as each issue lands: `docs/known-issues.md` (delete row, compact `#`), `docs/known-issues-explained.md`, `docs/error-handling-matrix.md`, `docs/HLD.md` (e14/e15), `docs/submission-junk-audit.md`, `docs/assignment-compliance-pickup.md`.
- Do **not** edit `AGENTS.md` as part of Known Issues updates.
- Regenerate `HLD.pdf` once at the end of CI8 docs (`scripts/render_hld_pdf.sh`), not after every slice.

## Order and why

1. **CI9** — mission loop only; FakeDroneControl keeps the test alive after later slices remove oversize-`Error`.
2. **CI2+CI10 together** — they share destination prediction; clamp first, then ignore if still world-OOB.
3. **CI3** — retry invalid **type** then throw. Oversize stays `Error` until CI8 (invalid ≠ oversize).
4. **CI8** — pending fragment queue; update the leftover oversize-`Error` integration test.

## Shared Docker helpers (every verification task)

Repo bind: `c:\Users\sagi1\Projects\DroneMapper\Drone-Mapper-ex3` → `/work`.

Unit tests (`build/default`):

```bash
docker run --rm -e VCPKG_ROOT=/usr/local/vcpkg \
  -v "c:\Users\sagi1\Projects\DroneMapper\Drone-Mapper-ex3:/work" \
  -w /work drone-mapper-ex3-dev bash -lc '
  set -euo pipefail
  cmake --preset default
  cmake --build --preset default --target test_drone_control test_mission_control
  ctest --preset default -R "test_(drone|mission)_control" --output-on-failure
'
```

Expected: 100% tests passed.

`verify-cell-runtime` (Release `build/opt`, full 24). Using verify-cell-runtime.

```bash
docker run --rm -e PYTHONUNBUFFERED=1 -e VCPKG_ROOT=/usr/local/vcpkg \
  -v "c:\Users\sagi1\Projects\DroneMapper\Drone-Mapper-ex3:/work" \
  -w /work drone-mapper-ex3-dev bash -lc '
  set -euo pipefail
  if [ ! -f build/opt/CMakeCache.txt ]; then
    cmake -S . -B build/opt -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_TOOLCHAIN_FILE="$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake"
  fi
  rm -f build/opt/Algorithm/CMakeFiles/Algorithm_207190406_209543255.dir/src/*.o
  rm -f build/opt/Algorithm/Algorithm_207190406_209543255.so
  cmake --build build/opt -j$(nproc) --target \
    Algorithm_207190406_209543255 \
    simulator_207190406_209543255 \
    MissionControl_207190406_209543255
  python3 .cursor/skills/verify-cell-runtime/scripts/time_each_cell.py \
    --build-dir /work/build/opt
'
```

Then diff scores against `docs/benchmarks/2026-09-03-branch-after-score-fix-per-cell-wall.csv`. Stop on any score change.

`verify-independent-component-variants` (VAR-01..03). Using verify-independent-component-variants.

```bash
docker run --rm -e VCPKG_ROOT=/usr/local/vcpkg \
  -v "c:\Users\sagi1\Projects\DroneMapper\Drone-Mapper-ex3:/work" \
  -w /work drone-mapper-ex3-dev bash -lc '
  set -euo pipefail
  cmake --preset default
  cmake --build --preset default --target \
    skeleton_host \
    simulator_207190406_209543255 \
    Algorithm_207190406_209543255 \
    MissionControl_207190406_209543255 \
    foreign_hits_only_mission_control_plugin \
    adversarial_throw_algorithm_plugin \
    adversarial_never_finish_algorithm_plugin \
    adversarial_into_occupied_algorithm_plugin \
    adversarial_bad_scan_orientation_algorithm_plugin \
    adversarial_throw_mission_control_plugin \
    adversarial_empty_mission_control_plugin \
    adversarial_implausible_steps_mission_control_plugin
  chmod +x Simulator/tests/manual/check_foreign_host.sh \
            Simulator/tests/manual/check_foreign_mission_control.sh \
            Simulator/tests/manual/check_adversarial_plugins.sh
  BD=build/default
  bash Simulator/tests/manual/check_foreign_host.sh "$BD"
  bash Simulator/tests/manual/check_foreign_mission_control.sh "$BD"
  bash Simulator/tests/manual/check_adversarial_plugins.sh "$BD"
'
```

Fill the VAR table. Overall PASS only if every selected row is PASS. `bad_scan` timeout → FAIL.

---

### Task 0: Feature branch

**Files:** none (git only)

**Interfaces:** none

- [ ] **Step 1: Confirm git toplevel**

Run from `Drone-Mapper-ex3`:

```bash
git rev-parse --show-toplevel
git status
```

Expected: toplevel is `.../Drone-Mapper-ex3` or `.../DroneMapper` — **not** `C:\Users\sagi1`. Working tree either clean or only files you intend to keep off this branch. If toplevel is the home directory, **stop**.

- [ ] **Step 2: Branch from updated main**

```bash
git checkout main
git pull
git checkout -b optional-common-issues-recovery
```

Expected: on `optional-common-issues-recovery` built from current `origin/main`. No workplan codes in the branch name.

---

### Task 1: CI9 — log step Error and keep the max_steps loop

**Files:**
- Create: `MissionControl/include/MissionControl/MissionRunLoop.h`
- Modify: `MissionControl/src/MissionControlImpl.cpp`
- Modify: `MissionControl/tests/test_mission_control.cpp`

**Interfaces:**
- Consumes: `mission_control::IDroneControl::step()`, `MissionConfigData::max_steps`, existing `writeVerboseLog` in `MissionControlImpl.cpp`.
- Produces: `mission_control_207190406_209543255::runMissionSteps(IDroneControl&, std::size_t max_steps, const std::filesystem::path& output_map_file, bool verbose) -> common::types::MissionRunResult`.

Extracting the loop is required so CI9 stays unit-tested after CI8 removes oversize-`Error`. Do **not** add a second public ctor on `MissionControlImpl`.

- [ ] **Step 1: Write the failing FakeDroneControl tests**

Add to `MissionControl/tests/test_mission_control.cpp` (keep existing helpers). Include `<MissionControl/IDroneControl.h>` and `<MissionControl/MissionRunLoop.h>`.

```cpp
class FakeDroneControl final : public mission_control::IDroneControl {
public:
    explicit FakeDroneControl(std::vector<common::types::DroneStepResult> script)
        : script_(std::move(script)) {}

    [[nodiscard]] common::types::DroneStepResult step() override {
        ++step_calls_;
        if (call_index_ >= script_.size()) {
            return {common::types::DroneStepStatus::Completed, {}};
        }
        return script_[call_index_++];
    }

    [[nodiscard]] common::types::DroneState state() const override { return {}; }

    std::size_t step_calls_ = 0;

private:
    std::vector<common::types::DroneStepResult> script_;
    std::size_t call_index_ = 0;
};

TEST(MissionControl, StepErrorIsLoggedAndLoopContinuesUntilFinished) {
    FakeDroneControl drone({
        {common::types::DroneStepStatus::Error, "Movement command exceeds drone limits."},
        {common::types::DroneStepStatus::Completed, {}},
    });
    const auto result = mission_control_207190406_209543255::runMissionSteps(
        drone, 5, {}, false);
    EXPECT_EQ(result.status, common::types::MissionRunStatus::Completed);
    EXPECT_EQ(result.steps, 2U);
    EXPECT_EQ(drone.step_calls_, 2U);
    ASSERT_EQ(result.errors.size(), 1U);
    EXPECT_EQ(result.errors.front().code, "DRONE_STEP_FAILED");
    EXPECT_EQ(result.errors.front().message, "Drone step failed.");
}

TEST(MissionControl, PersistentStepErrorRunsToMaxSteps) {
    FakeDroneControl drone({
        {common::types::DroneStepStatus::Error, "bad"},
        {common::types::DroneStepStatus::Error, "bad"},
        {common::types::DroneStepStatus::Error, "bad"},
        {common::types::DroneStepStatus::Error, "bad"},
    });
    const auto result = mission_control_207190406_209543255::runMissionSteps(
        drone, 3, {}, false);
    EXPECT_EQ(result.status, common::types::MissionRunStatus::MaxSteps);
    EXPECT_EQ(result.steps, 3U);
    EXPECT_EQ(result.errors.size(), 3U);
    EXPECT_EQ(result.errors[0].code, "DRONE_STEP_FAILED");
    EXPECT_EQ(result.errors[1].code, "DRONE_STEP_FAILED");
    EXPECT_EQ(result.errors[2].code, "DRONE_STEP_FAILED");
}
```

Keep `FailedStepReportsStructuredErrorRef` for now (oversize still `Error` until CI8). After CI9 impl it must **not** abort: script is one oversize then `ScriptedAlgorithm` returns `Finished`. Expected after impl:

```cpp
EXPECT_EQ(result.status, common::types::MissionRunStatus::Completed);
EXPECT_EQ(result.steps, 2U);
ASSERT_FALSE(result.errors.empty());
EXPECT_EQ(result.errors.front().code, "DRONE_STEP_FAILED");
```

Rename that test to `FailedOversizeStepLogsAndContinuesUntilFinished` in the same edit.

- [ ] **Step 2: Run tests to verify they fail**

Same docker `ctest` helper, `-R test_mission_control`.

Expected: FAIL — `MissionRunLoop.h` missing and/or `FailedOversizeStep` still `MissionRunStatus::Error` with `steps==1`.

- [ ] **Step 3: Create `MissionRunLoop.h`**

```cpp
#pragma once

#include <MissionControl/IDroneControl.h>

#include <Common/Types.h>

#include <filesystem>

namespace mission_control_207190406_209543255 {

[[nodiscard]] common::types::MissionRunResult runMissionSteps(
    mission_control::IDroneControl& drone_control, std::size_t max_steps,
    const std::filesystem::path& output_map_file, bool verbose);

} // namespace mission_control_207190406_209543255
```

- [ ] **Step 4: Move the loop into `runMissionSteps` and continue on Error**

In `MissionControlImpl.cpp`, `#include <MissionControl/MissionRunLoop.h>`. Keep `finalizeMission` / `writeVerboseLog` in the anonymous namespace. Implement:

```cpp
common::types::MissionRunResult runMissionSteps(
    mission_control::IDroneControl& drone_control, std::size_t max_steps,
    const std::filesystem::path& output_map_file, bool verbose) {
    std::size_t steps = 0;
    common::types::MissionRunStatus status = common::types::MissionRunStatus::MaxSteps;
    std::vector<common::types::ErrorRef> errors;

    while (steps < max_steps) {
        const common::types::DroneStepResult step_result = drone_control.step();
        ++steps;

        if (step_result.status == common::types::DroneStepStatus::Error) {
            errors.push_back(
                common::types::ErrorRef{"DRONE_STEP_FAILED", "Drone step failed."});
            continue;
        }

        if (step_result.status == common::types::DroneStepStatus::Completed) {
            status = common::types::MissionRunStatus::Completed;
            auto result = finalizeMission(status, steps, std::move(errors));
            if (verbose && !output_map_file.empty()) {
                writeVerboseLog(output_map_file, result.status, result.steps);
            }
            return result;
        }
    }

    auto result = finalizeMission(status, steps, std::move(errors));
    if (verbose && !output_map_file.empty()) {
        writeVerboseLog(output_map_file, result.status, result.steps);
    }
    return result;
}
```

`MissionControlImpl::runMission` becomes:

```cpp
return runMissionSteps(*drone_control_, mission_.max_steps, output_map_file_, verbose_);
```

Do not write verbose output on every Error tick — only at Completed / MaxSteps, matching today’s end-of-mission write. `SimulationRunImpl` still logs `mission_result.errors` after `runMission()` returns (existing immediacy at the run boundary). Do not invent a MissionControl `error.log` file.

- [ ] **Step 5: Run tests to verify they pass**

Same `ctest` helper for `test_mission_control` **and** `test_drone_control`.

Expected: PASS. `HitsMaxSteps` / `CompletesWhenAlgorithmFinishes` unchanged.

---

### Task 2: CI9 docs and bonus claim

**Files:**
- Create: `bonus.txt`
- Modify: `docs/known-issues.md`, `docs/known-issues-explained.md`, `docs/error-handling-matrix.md`, `docs/HLD.md`, `docs/submission-junk-audit.md`, `docs/assignment-compliance-pickup.md`

**Interfaces:** none (docs only)

- [ ] **Step 1: Delete KI row 7 and compact `#`**

Remove the CI9 row. Renumber former 8–14 to 7–13. Remaining optional Common-issues rows in the file: CI2, CI3, CI4, CI6, CI7, CI8, CI10, CI11, CI12 (old 1–6 and 8–10).

- [ ] **Step 2: Update `known-issues-explained.md`**

Move CI9 out of the “what you actually do” table into a short “Implemented optional rows” note: log `DRONE_STEP_FAILED` and continue the `max_steps` loop (`runMissionSteps` in `MissionControlImpl.cpp`). Keep #1–#10 numbering in that explained table in sync with `known-issues.md` after compacting (or drop CI9 from the optional table entirely and list it under implemented).

- [ ] **Step 3: Mark CI9 implemented in `error-handling-matrix.md`**

After the Optional table, add:

```markdown
## Implemented optional rows

- **Drone returns `Error` status on a step (CI9):** `runMissionSteps` logs `DRONE_STEP_FAILED` and continues until `Completed` or `max_steps`. See `MissionControl/src/MissionControlImpl.cpp`.
```

Do not rewrite the staff PDF wording in the table.

- [ ] **Step 4: HLD**

In `docs/HLD.md` sequence “DroneControl step loop”:

- Change mermaid `loop until Finished / MaxSteps / Error` to `loop until Finished / MaxSteps`.
- Add one sentence: step `Error` is pushed as `DRONE_STEP_FAILED` and the loop continues; it does not set `MissionRunStatus::Error` by itself.

- [ ] **Step 5: Create `bonus.txt` at the repo root**

```text
Optional Common-issues recoveries (context/Common issues and handling.pdf)

CI9 — Drone step Error: log and continue the max_steps loop
  MissionControl/src/MissionControlImpl.cpp  (runMissionSteps)
  MissionControl/include/MissionControl/MissionRunLoop.h
```

- [ ] **Step 6: Pickup / junk audit**

`docs/submission-junk-audit.md`: change the `bonus.txt` row from “Do not add” to “Include — we claim CI9 (and later CI2/CI10/CI3/CI8)”.

`docs/assignment-compliance-pickup.md` Known Issues paragraph: CI9 is implemented and claimed; remaining optional skips listed in `docs/known-issues.md`.

---

### Task 3: CI9 verification gates

**Files:** none (run only)

- [ ] **Step 1: Unit tests** — shared Docker ctest helper. Expected: PASS.
- [ ] **Step 2: verify-cell-runtime** — shared Release 24-cell command. Fill the skill report table. Scores **must** match the 2026-09-03 baseline (our algorithm should never return `Error`). WARNs only `large_out` short lidar. If any score differs, **stop** and investigate; do not continue to CI2.
- [ ] **Step 3: verify-independent-component-variants** — VAR-01, VAR-02, VAR-03. Skip VAR-04. `bad_scan` must finish inside timeout. Fill:

```markdown
# Independent component variants verification (after CI9)

| Variant | Script | Status | Notes |
|---------|--------|--------|-------|
| VAR-01 | check_foreign_host.sh | PASS/FAIL | |
| VAR-02 | check_foreign_mission_control.sh | PASS/FAIL | findings path |
| VAR-03 | check_adversarial_plugins.sh | PASS/FAIL | bad_scan canary |
| VAR-04 | check_baseline_algorithm.sh | SKIP | deferred to final gate |
```

---

### Task 4: CI9 commit (human approval)

- [ ] **Step 1: Show what would be committed**

```bash
git status
git diff
git log -5 --oneline
```

Proposed message:

```text
feat: continue mission loop after drone step Error

Optional CI9: log DRONE_STEP_FAILED and keep max_steps instead of
aborting the mission.
```

- [ ] **Step 2: Stop and wait for explicit approval** (`yes` / `commit` / `lgtm`). Do not treat this plan as approval.
- [ ] **Step 3: After approval, commit** (PowerShell — no bash HEREDOC):

```powershell
git add MissionControl/include/MissionControl/MissionRunLoop.h `
  MissionControl/src/MissionControlImpl.cpp `
  MissionControl/tests/test_mission_control.cpp `
  bonus.txt `
  docs/known-issues.md docs/known-issues-explained.md `
  docs/error-handling-matrix.md docs/HLD.md `
  docs/submission-junk-audit.md docs/assignment-compliance-pickup.md
git commit -m "feat: continue mission loop after drone step Error"
git status
```

Do not add unrelated files. If a hook rejects, fix and make a **new** commit — do not `--amend` unless the git-workflow amend rules all hold.

---

### Task 5: CI2 + CI10 — ignore world-OOB; clamp to mission bounds

**Files:**
- Modify: `MissionControl/src/DroneControlImpl.cpp`
- Modify: `MissionControl/tests/test_drone_control.cpp`

**Interfaces:**
- Consumes: `output_map_.isInBounds`, `mission_.mission_bounds`, `gps_.position()` / `heading()`, `drone_` limits already used by `movementWithinLimits`.
- Produces: anonymous-namespace helpers `predictedDestination`, `isUnsetMissionBounds`, `isWithinMappingBounds`, `clampMovementToMissionBounds` in `DroneControlImpl.cpp`.

**Policy (lock this):** Hover/Rotate never translate — skip both checks. For Advance/Elevate: (1) if mission bounds are **not** all-zero, clamp the command so the predicted **center** stays inside `mission_bounds` (inclusive AABB). Zero remaining distance → do not call Movement (same as ignore). (2) Then if `!output_map_.isInBounds(predictedDestination(...))`, ignore — do not call Movement. Clamp first so a move that can be shortened to stay in both AABBs is shortened; ignore only if still world-OOB (mission AABB larger than the map). Do **not** use `sphereHitsOccupiedOrOutOfBounds` here — Occupied is mandatory CI5, not CI2. Center-only, no radius inset.

Kinematics must match `Simulator/src/MockMovement.cpp`: heading 0° = +X, 90° = +Y; unwrap to `double` cm/deg inside the `.cpp` for `std::cos`/`std::sin` (same libm path as MockMovement).

- [ ] **Step 1: Make `FakeMap3D::isInBounds` AABB-aware and record advance distance**

Replace the ignore-`pos` fake with:

```cpp
[[nodiscard]] bool isInBounds(const Position3D& pos) const override {
    const auto& b = config_.boundaries;
    return pos.x >= b.min_x && pos.x <= b.max_x && pos.y >= b.min_y &&
           pos.y <= b.max_y && pos.z >= b.min_height && pos.z <= b.max_height;
}
```

Remove unused `setInBounds` / `in_bounds_` if nothing else calls them.

On `FakeMovement`:

```cpp
common::types::MovementResult advance(PhysicalLength distance) override {
    ++advance_count_;
    last_advance_ = distance;
    // existing throw / fail / success branches unchanged
}

PhysicalLength last_advance_{};
```

Existing tests keep GPS at `{}` and map 0–100 cm — still in bounds.

- [ ] **Step 2: Write failing tests**

`defaultMission()` has zero `mission_bounds` (CI10 off). Heading `0 deg` = +X.

```cpp
TEST(DroneControl, WorldOutOfBoundsAdvanceIsIgnored) {
    Fixture fixture;
    fixture.gps.position_ = Position3D{
        95.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]};
    // heading default 0 => +X; dest 115 is outside map 0..100

    ScriptedAlgorithm algorithm{
        common::MappingAlgorithmDependencies{
            defaultMission(), defaultLidar(), defaultDrone(), fixture.stand_in_map},
        {common::types::MappingStepCommand{
            .movement =
                common::types::MovementCommand{
                    .type = common::types::MovementCommandType::Advance,
                    .distance = 20.0 * cm,
                },
            .status = common::types::AlgorithmStatus::Working,
        }},
    };

    mission_control_207190406_209543255::DroneControlImpl control{
        defaultDrone(), defaultMission(), defaultLidar(), fixture.lidar, fixture.gps,
        fixture.movement, fixture.output_map, algorithm,
    };

    const auto result = control.step();
    EXPECT_EQ(result.status, common::types::DroneStepStatus::Continue);
    EXPECT_EQ(fixture.movement.advance_count_, 0);
}

TEST(DroneControl, InBoundsAdvanceStillReachesMovement) {
    Fixture fixture;
    fixture.gps.position_ = Position3D{
        50.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]};
    ScriptedAlgorithm algorithm{
        common::MappingAlgorithmDependencies{
            defaultMission(), defaultLidar(), defaultDrone(), fixture.stand_in_map},
        {common::types::MappingStepCommand{
            .movement =
                common::types::MovementCommand{
                    .type = common::types::MovementCommandType::Advance,
                    .distance = 10.0 * cm,
                },
            .status = common::types::AlgorithmStatus::Working,
        }},
    };
    mission_control_207190406_209543255::DroneControlImpl control{
        defaultDrone(), defaultMission(), defaultLidar(), fixture.lidar, fixture.gps,
        fixture.movement, fixture.output_map, algorithm,
    };
    EXPECT_EQ(control.step().status, common::types::DroneStepStatus::Continue);
    EXPECT_EQ(fixture.movement.advance_count_, 1);
}

TEST(DroneControl, MissionBoundsAdvanceIsClamped) {
    Fixture fixture;
    fixture.gps.position_ = Position3D{
        70.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]};
    auto mission = defaultMission();
    mission.mission_bounds = {
        0.0 * x_extent[cm], 80.0 * x_extent[cm], 0.0 * y_extent[cm], 100.0 * y_extent[cm],
        0.0 * z_extent[cm], 100.0 * z_extent[cm],
    };
    ScriptedAlgorithm algorithm{
        common::MappingAlgorithmDependencies{
            mission, defaultLidar(), defaultDrone(), fixture.stand_in_map},
        {common::types::MappingStepCommand{
            .movement =
                common::types::MovementCommand{
                    .type = common::types::MovementCommandType::Advance,
                    .distance = 20.0 * cm,
                },
            .status = common::types::AlgorithmStatus::Working,
        }},
    };
    mission_control_207190406_209543255::DroneControlImpl control{
        defaultDrone(), mission, defaultLidar(), fixture.lidar, fixture.gps,
        fixture.movement, fixture.output_map, algorithm,
    };
    EXPECT_EQ(control.step().status, common::types::DroneStepStatus::Continue);
    EXPECT_EQ(fixture.movement.advance_count_, 1);
    EXPECT_DOUBLE_EQ(fixture.movement.last_advance_.numerical_value_in(cm), 10.0);
}

TEST(DroneControl, MissionLargerThanMapStillIgnoresWorldOob) {
    Fixture fixture;
    fixture.gps.position_ = Position3D{
        95.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]};
    auto mission = defaultMission();
    mission.mission_bounds = {
        0.0 * x_extent[cm], 200.0 * x_extent[cm], 0.0 * y_extent[cm], 100.0 * y_extent[cm],
        0.0 * z_extent[cm], 100.0 * z_extent[cm],
    };
    ScriptedAlgorithm algorithm{
        common::MappingAlgorithmDependencies{
            mission, defaultLidar(), defaultDrone(), fixture.stand_in_map},
        {common::types::MappingStepCommand{
            .movement =
                common::types::MovementCommand{
                    .type = common::types::MovementCommandType::Advance,
                    .distance = 20.0 * cm,
                },
            .status = common::types::AlgorithmStatus::Working,
        }},
    };
    mission_control_207190406_209543255::DroneControlImpl control{
        defaultDrone(), mission, defaultLidar(), fixture.lidar, fixture.gps,
        fixture.movement, fixture.output_map, algorithm,
    };
    EXPECT_EQ(control.step().status, common::types::DroneStepStatus::Continue);
    EXPECT_EQ(fixture.movement.advance_count_, 0);
}

TEST(DroneControl, ElevateIsClampedToMissionHeight) {
    Fixture fixture;
    fixture.gps.position_ = Position3D{
        50.0 * x_extent[cm], 50.0 * y_extent[cm], 90.0 * z_extent[cm]};
    auto mission = defaultMission();
    mission.mission_bounds = {
        0.0 * x_extent[cm], 100.0 * x_extent[cm], 0.0 * y_extent[cm], 100.0 * y_extent[cm],
        0.0 * z_extent[cm], 100.0 * z_extent[cm],
    };
    ScriptedAlgorithm algorithm{
        common::MappingAlgorithmDependencies{
            mission, defaultLidar(), defaultDrone(), fixture.stand_in_map},
        {common::types::MappingStepCommand{
            .movement =
                common::types::MovementCommand{
                    .type = common::types::MovementCommandType::Elevate,
                    .distance = 20.0 * cm,
                },
            .status = common::types::AlgorithmStatus::Working,
        }},
    };
    // Extend FakeMovement to record last_elevate_ the same way as last_advance_.
    mission_control_207190406_209543255::DroneControlImpl control{
        defaultDrone(), mission, defaultLidar(), fixture.lidar, fixture.gps,
        fixture.movement, fixture.output_map, algorithm,
    };
    EXPECT_EQ(control.step().status, common::types::DroneStepStatus::Continue);
    EXPECT_DOUBLE_EQ(fixture.movement.last_elevate_.numerical_value_in(cm), 10.0);
}
```

Add `last_elevate_` on `FakeMovement::elevate`.

- [ ] **Step 3: Run tests to verify they fail**

`ctest -R test_drone_control`. Expected: FAIL — world-OOB still calls `advance` (`advance_count_ == 1`).

- [ ] **Step 4: Implement helpers and wire `applyMovement`**

In `DroneControlImpl.cpp` anonymous namespace (includes already have `<mp-units/math.h>`; add `<cmath>` and `<numbers>`):

```cpp
[[nodiscard]] bool isUnsetMissionBounds(const common::types::MappingBounds& bounds) {
    using common::cm;
    using common::x_extent;
    using common::y_extent;
    using common::z_extent;
    return bounds.min_x == 0.0 * x_extent[cm] && bounds.max_x == 0.0 * x_extent[cm] &&
           bounds.min_y == 0.0 * y_extent[cm] && bounds.max_y == 0.0 * y_extent[cm] &&
           bounds.min_height == 0.0 * z_extent[cm] &&
           bounds.max_height == 0.0 * z_extent[cm];
}

[[nodiscard]] bool isWithinMappingBounds(const Position3D& pos,
                                         const common::types::MappingBounds& bounds) {
    return pos.x >= bounds.min_x && pos.x <= bounds.max_x && pos.y >= bounds.min_y &&
           pos.y <= bounds.max_y && pos.z >= bounds.min_height &&
           pos.z <= bounds.max_height;
}

[[nodiscard]] Position3D predictedDestination(const Position3D& pos,
                                              const common::Orientation& heading,
                                              const common::types::MovementCommand& command) {
    using common::cm;
    using common::deg;
    using common::x_extent;
    using common::y_extent;
    using common::z_extent;
    if (command.type == common::types::MovementCommandType::Elevate) {
        const double dist_cm = command.distance.numerical_value_in(cm);
        return Position3D{pos.x, pos.y, pos.z + dist_cm * z_extent[cm]};
    }
    if (command.type != common::types::MovementCommandType::Advance) {
        return pos;
    }
    const double dist_cm = command.distance.numerical_value_in(cm);
    const double angle_rad =
        heading.horizontal.numerical_value_in(deg) * (std::numbers::pi / 180.0);
    const double dx = std::cos(angle_rad);
    const double dy = std::sin(angle_rad);
    return Position3D{
        pos.x + (dist_cm * dx) * x_extent[cm],
        pos.y + (dist_cm * dy) * y_extent[cm],
        pos.z,
    };
}

[[nodiscard]] common::types::MovementCommand clampMovementToMissionBounds(
    common::types::MovementCommand command, const Position3D& pos,
    const common::Orientation& heading, const common::types::MappingBounds& bounds) {
    using common::cm;
    using common::deg;
    if (command.type == common::types::MovementCommandType::Elevate) {
        const double z0 = pos.z.numerical_value_in(cm);
        const double dz = command.distance.numerical_value_in(cm);
        const double z1 = z0 + dz;
        const double zmin = bounds.min_height.numerical_value_in(cm);
        const double zmax = bounds.max_height.numerical_value_in(cm);
        double clamped_z = z1;
        if (clamped_z > zmax) {
            clamped_z = zmax;
        }
        if (clamped_z < zmin) {
            clamped_z = zmin;
        }
        command.distance = (clamped_z - z0) * cm;
        return command;
    }
    if (command.type != common::types::MovementCommandType::Advance) {
        return command;
    }
    const double dist = command.distance.numerical_value_in(cm);
    if (dist <= 0.0) {
        return command;
    }
    const double rad =
        heading.horizontal.numerical_value_in(deg) * (std::numbers::pi / 180.0);
    const double dirx = std::cos(rad);
    const double diry = std::sin(rad);
    const double px = pos.x.numerical_value_in(cm);
    const double py = pos.y.numerical_value_in(cm);
    double t = dist;
    constexpr double kEps = 1e-9;
    const double xmin = bounds.min_x.numerical_value_in(cm);
    const double xmax = bounds.max_x.numerical_value_in(cm);
    const double ymin = bounds.min_y.numerical_value_in(cm);
    const double ymax = bounds.max_y.numerical_value_in(cm);
    if (dirx > kEps) {
        t = std::min(t, (xmax - px) / dirx);
    } else if (dirx < -kEps) {
        t = std::min(t, (xmin - px) / dirx);
    }
    if (diry > kEps) {
        t = std::min(t, (ymax - py) / diry);
    } else if (diry < -kEps) {
        t = std::min(t, (ymin - py) / diry);
    }
    if (t < 0.0) {
        t = 0.0;
    }
    command.distance = t * cm;
    return command;
}

[[nodiscard]] bool isZeroLengthMove(const common::types::MovementCommand& command) {
    using common::cm;
    using common::deg;
    switch (command.type) {
    case common::types::MovementCommandType::Advance:
    case common::types::MovementCommandType::Elevate:
        return mp_units::abs(command.distance) <= 0.0 * cm;
    case common::types::MovementCommandType::Rotate:
        return command.angle <= 0.0 * common::horizontal_angle[deg];
    case common::types::MovementCommandType::Hover:
        return false;
    }
    return false;
}
```

In `applyMovement`, after `movementWithinLimits` and **before** `executeMovement`:

```cpp
common::types::MovementCommand move = *command.movement;
const Position3D here = gps_.position();
const common::Orientation heading = gps_.heading();
if (!isUnsetMissionBounds(mission_.mission_bounds)) {
    move = clampMovementToMissionBounds(move, here, heading, mission_.mission_bounds);
}
if (isZeroLengthMove(move)) {
    return {common::types::DroneStepStatus::Continue, {}};
}
if (move.type == common::types::MovementCommandType::Advance ||
    move.type == common::types::MovementCommandType::Elevate) {
    const Position3D dest = predictedDestination(here, heading, move);
    if (!output_map_.isInBounds(dest)) {
        return {common::types::DroneStepStatus::Continue, {}};
    }
}
try {
    const common::types::MovementResult movement_result = executeMovement(movement_, move);
    // unchanged success/catch handling
}
```

- [ ] **Step 5: Run tests to verify they pass**

`ctest -R "test_(drone|mission)_control"`. Expected: PASS. Collision/Continue tests unchanged.

---

### Task 6: CI2/CI10 docs

**Files:** `bonus.txt`, `docs/known-issues.md`, `docs/known-issues-explained.md`, `docs/error-handling-matrix.md`, `docs/HLD.md`, `docs/assignment-compliance-pickup.md`

- [ ] **Step 1:** Delete the CI2 and CI10 rows from `known-issues.md` (whatever `#` they have after CI9 compact). Compact remaining `#` to `1..n`.
- [ ] **Step 2:** Explained doc: those two are implemented (ignore world OOB via `output_map_.isInBounds`; clamp Advance/Elevate to `mission_bounds` when set). Mention clamp-then-ignore order.
- [ ] **Step 3:** Append to “Implemented optional rows” in `error-handling-matrix.md` with `DroneControlImpl.cpp` helpers named above.
- [ ] **Step 4:** HLD: in the step-loop `alt`, add ignore-OOB (no Movement call) and amend-to-mission-bounds.
- [ ] **Step 5:** Append `bonus.txt`:

```text
CI2 — ignore movement that would leave the world/map
  MissionControl/src/DroneControlImpl.cpp  (applyMovement + predictedDestination)
CI10 — amend movement to stay inside mission bounds
  MissionControl/src/DroneControlImpl.cpp  (clampMovementToMissionBounds)
```

---

### Task 7: CI2/CI10 verification gates

Same three steps as Task 3 (unit tests, full 24-cell runtime, VAR-01..03). Scores must still match 2026-09-03. Our algorithm should not command world-OOB or mission-OOB moves; if a score changes, **stop** — likely a false-positive ignore/clamp on a legal outdoor step.

---

### Task 8: CI2/CI10 commit (human approval)

Show status/diff. Proposed message:

```text
feat: ignore world-OOB moves and clamp to mission bounds

Optional CI2/CI10: drop map-leaving commands before Movement, and
shorten Advance/Elevate so the center stays in mission_bounds.
```

Wait for approval, then commit only the Task 5–6 files.

---

### Task 9: CI3 — retry invalid commands, throw after N

**Files:**
- Modify: `MissionControl/src/DroneControlImpl.cpp`
- Modify: `MissionControl/tests/test_drone_control.cpp`
- Modify: `MissionControl/tests/test_mission_control.cpp`

**Interfaces:**
- Consumes: `mapping_algorithm_.nextStep`, `isSupportedMovementType`.
- Produces: `constexpr int kMaxInvalidCommandRetries = 3` in the `DroneControlImpl.cpp` anonymous namespace. Throws `std::runtime_error` **out of** `step()` (do not catch it in the Movement `try`). `SimulationRunImpl` already maps that to `MISSION_EXCEPTION`.

**Invalid means:** `movement.has_value() && !isSupportedMovementType(type)` (including `static_cast<MovementCommandType>(99)`). Oversize Advance/Elevate/Rotate is **not** invalid — still `Error` from `movementWithinLimits` until CI8. Scan-only and both-empty NOOP are **not** invalid (CI4 stays skipped). `Finished` / `FinishedWithUnmappableVoxels` during a retry completes the step (no throw).

Retries happen **inside one** `step()` call. `step_index_` increments once on a successful Continue/Completed return, not per retry. After 3 invalid `nextStep` results, throw. Do not call a 4th time.

- [ ] **Step 1: Write failing tests**

```cpp
TEST(DroneControl, InvalidCommandRetriesThenExecutesValid) {
    Fixture fixture;
    const auto bad = common::types::MovementCommand{
        .type = static_cast<common::types::MovementCommandType>(99),
    };
    const auto ok = common::types::MovementCommand{
        .type = common::types::MovementCommandType::Advance,
        .distance = 10.0 * cm,
    };
    ScriptedAlgorithm algorithm{
        common::MappingAlgorithmDependencies{
            defaultMission(), defaultLidar(), defaultDrone(), fixture.stand_in_map},
        {
            {.movement = bad, .status = common::types::AlgorithmStatus::Working},
            {.movement = bad, .status = common::types::AlgorithmStatus::Working},
            {.movement = ok, .status = common::types::AlgorithmStatus::Working},
        },
    };
    mission_control_207190406_209543255::DroneControlImpl control{
        defaultDrone(), defaultMission(), defaultLidar(), fixture.lidar, fixture.gps,
        fixture.movement, fixture.output_map, algorithm,
    };
    EXPECT_EQ(control.step().status, common::types::DroneStepStatus::Continue);
    EXPECT_EQ(algorithm.call_index_, 3U);
    EXPECT_EQ(fixture.movement.advance_count_, 1);
    EXPECT_EQ(control.state().step_index, 1U);
}

TEST(DroneControl, InvalidCommandThrowsAfterMaxRetries) {
    Fixture fixture;
    const auto bad = common::types::MovementCommand{
        .type = static_cast<common::types::MovementCommandType>(99),
    };
    ScriptedAlgorithm algorithm{
        common::MappingAlgorithmDependencies{
            defaultMission(), defaultLidar(), defaultDrone(), fixture.stand_in_map},
        {
            {.movement = bad, .status = common::types::AlgorithmStatus::Working},
            {.movement = bad, .status = common::types::AlgorithmStatus::Working},
            {.movement = bad, .status = common::types::AlgorithmStatus::Working},
            {.movement = bad, .status = common::types::AlgorithmStatus::Working},
        },
    };
    mission_control_207190406_209543255::DroneControlImpl control{
        defaultDrone(), defaultMission(), defaultLidar(), fixture.lidar, fixture.gps,
        fixture.movement, fixture.output_map, algorithm,
    };
    EXPECT_THROW(
        { (void)control.step(); },
        std::runtime_error);
    EXPECT_EQ(algorithm.call_index_, 3U);
    EXPECT_EQ(fixture.movement.advance_count_, 0);
}

TEST(DroneControl, OversizeIsNotInvalidRetry) {
    // existing ReturnsErrorWhenMovementExceedsDroneLimits body:
    // still Error on first oversize; call_index_ == 1; no throw.
}
```

Add MC test: `ScriptedAlgorithm` with three invalid commands, `max_steps=5`, `EXPECT_THROW(control.runMission(), std::runtime_error)`.

- [ ] **Step 2: Run tests — expected FAIL** (today first invalid returns `Error`, `call_index_==1`, no throw).

- [ ] **Step 3: Implement retry in `step()`**

```cpp
constexpr int kMaxInvalidCommandRetries = 3;

[[nodiscard]] bool isInvalidMovementCommand(const common::types::MappingStepCommand& command) {
    return command.movement.has_value() &&
           !isSupportedMovementType(command.movement->type);
}
```

In `step()`, after building `current_state` / footprint, replace the single `nextStep` with:

```cpp
common::types::MappingStepCommand command{};
int invalid_tries = 0;
while (true) {
    command = mapping_algorithm_.nextStep(current_state, latest_scan_ptr);
    if (command.status == common::types::AlgorithmStatus::Finished ||
        command.status == common::types::AlgorithmStatus::FinishedWithUnmappableVoxels) {
        return {common::types::DroneStepStatus::Completed, {}};
    }
    if (!isInvalidMovementCommand(command)) {
        break;
    }
    ++invalid_tries;
    if (invalid_tries >= kMaxInvalidCommandRetries) {
        throw std::runtime_error("Invalid movement command after retries.");
    }
}
```

Remove or keep the unsupported-type `Error` in `applyMovement` as a defensive fallback (must not run for the throw path). Do **not** put this throw inside the Movement `catch (const std::exception&)`.

- [ ] **Step 4: Tests pass.** `ReturnsErrorWhenMovementExceedsDroneLimits` still `Error`. Collision Continue tests still pass.

---

### Task 10: CI3 docs

Delete the CI3 Known Issues row; compact `#`. Explained + matrix “Implemented optional rows”: retry `nextStep` up to `kMaxInvalidCommandRetries` then throw; targets DroneControl (propagates) and SimulationRun (`MISSION_EXCEPTION`). Append `bonus.txt` with `DroneControlImpl.cpp` / `kMaxInvalidCommandRetries`. HLD: mention invalid-command retry inside `step()` before movement.

---

### Task 11: CI3 verification gates

Same as Task 3. Scores must match baseline. VAR-03 `throw_algo` already throws — still no crash. Invalid-command throw is a new path; `test_mission_control` throw is the unit proof. If VAR-03 hangs or crashes, **stop**.

---

### Task 12: CI3 commit (human approval)

```text
feat: retry invalid algorithm commands then throw

Optional CI3: re-call nextStep up to kMaxInvalidCommandRetries, then
throw so SimulationRun can contain the failure.
```

---

### Task 13: CI8 — split oversize movement into legal steps

**Files:**
- Modify: `MissionControl/include/MissionControl/DroneControlImpl.h`
- Modify: `MissionControl/src/DroneControlImpl.cpp`
- Modify: `MissionControl/tests/test_drone_control.cpp`
- Modify: `MissionControl/tests/test_mission_control.cpp`

**Interfaces:**
- Consumes: `drone_.max_advance` / `max_elevate` / `max_rotate`, existing `movementWithinLimits`.
- Produces: `std::deque<common::types::MovementCommand> pending_movements_` on `DroneControlImpl`. Each fragment is a **separate** `step()`: first `step()` calls `nextStep` once, executes fragment 0, optional scan from the original command; later `step()` calls **skip** `nextStep`, execute the next fragment, no extra scan.

**Do not** clamp-in-place inside one `step()`. PDF: “each a separate step”.

Split rules:

- Advance `D` with `max_advance=M`: while remaining `> M`, queue `Advance(M)`; then leftover if `> 0`.
- Elevate: same on `abs(distance)`, preserve sign.
- Rotate: same on `angle` vs `max_rotate`; keep `rotation` direction.
- Hover: never oversize.

Apply CI10 clamp and CI2 ignore to **each executed fragment**, not only the original oversize command.

When `pending_movements_` is non-empty at the start of `step()`: do not call `nextStep`; pop one fragment; run `applyMovement` on a synthetic `MappingStepCommand` with only that movement (no scan); `++step_index_`; return Continue.

- [ ] **Step 1: Write failing tests**

```cpp
TEST(DroneControl, OversizeAdvanceSplitsAcrossStepsWithoutExtraNextStep) {
    Fixture fixture;
    ScriptedAlgorithm algorithm{
        common::MappingAlgorithmDependencies{
            defaultMission(), defaultLidar(), defaultDrone(), fixture.stand_in_map},
        {common::types::MappingStepCommand{
            .movement =
                common::types::MovementCommand{
                    .type = common::types::MovementCommandType::Advance,
                    .distance = 50.0 * cm,
                },
            .scan_orientation =
                Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},
            .status = common::types::AlgorithmStatus::Working,
        }},
    };
    mission_control_207190406_209543255::DroneControlImpl control{
        defaultDrone(), defaultMission(), defaultLidar(), fixture.lidar, fixture.gps,
        fixture.movement, fixture.output_map, algorithm,
    };
    // max_advance is 20 cm → fragments 20, 20, 10
    EXPECT_EQ(control.step().status, common::types::DroneStepStatus::Continue);
    EXPECT_EQ(algorithm.call_index_, 1U);
    EXPECT_EQ(fixture.movement.advance_count_, 1);
    EXPECT_DOUBLE_EQ(fixture.movement.last_advance_.numerical_value_in(cm), 20.0);
    EXPECT_EQ(fixture.lidar.scan_count_, 1);
    EXPECT_EQ(control.state().step_index, 1U);

    EXPECT_EQ(control.step().status, common::types::DroneStepStatus::Continue);
    EXPECT_EQ(algorithm.call_index_, 1U);
    EXPECT_EQ(fixture.movement.advance_count_, 2);
    EXPECT_EQ(fixture.lidar.scan_count_, 1);
    EXPECT_EQ(control.state().step_index, 2U);

    EXPECT_EQ(control.step().status, common::types::DroneStepStatus::Continue);
    EXPECT_EQ(algorithm.call_index_, 1U);
    EXPECT_EQ(fixture.movement.advance_count_, 3);
    EXPECT_DOUBLE_EQ(fixture.movement.last_advance_.numerical_value_in(cm), 10.0);
    EXPECT_EQ(control.state().step_index, 3U);
}

TEST(DroneControl, OversizeRotateSplits) {
    Fixture fixture;
    ScriptedAlgorithm algorithm{
        common::MappingAlgorithmDependencies{
            defaultMission(), defaultLidar(), defaultDrone(), fixture.stand_in_map},
        {common::types::MappingStepCommand{
            .movement =
                common::types::MovementCommand{
                    .type = common::types::MovementCommandType::Rotate,
                    .rotation = common::types::RotationDirection::Left,
                    .angle = 180.0 * horizontal_angle[deg],
                },
            .status = common::types::AlgorithmStatus::Working,
        }},
    };
    // FakeMovement::rotate must ++rotate_count_ and last_angle_
    mission_control_207190406_209543255::DroneControlImpl control{
        defaultDrone(), defaultMission(), defaultLidar(), fixture.lidar, fixture.gps,
        fixture.movement, fixture.output_map, algorithm,
    };
    EXPECT_EQ(control.step().status, common::types::DroneStepStatus::Continue);
    EXPECT_EQ(control.step().status, common::types::DroneStepStatus::Continue);
    EXPECT_EQ(fixture.movement.rotate_count_, 2);
    EXPECT_EQ(algorithm.call_index_, 1U);
}
```

Change `ReturnsErrorWhenMovementExceedsDroneLimits` to `OversizeAdvanceNoLongerErrorsOnFirstStep` — first `step()` is Continue, `advance_count_==1`, distance 20 cm.

`FailedOversizeStepLogsAndContinuesUntilFinished` in `test_mission_control.cpp`: oversize 500 cm with `max_steps=5` and a one-command script will now **split** (~25 fragments of 20 cm) and then `Finished`. It will **not** push `DRONE_STEP_FAILED`. Replace that integration test with: oversize then Finished-on-next-real-command is Completes **without** errors; CI9 coverage stays on FakeDroneControl tests from Task 1. Do not delete the FakeDroneControl tests.

- [ ] **Step 2: Run tests — expected FAIL** (`Error` on oversize, `advance_count_==0`).

- [ ] **Step 3: Header member**

```cpp
#include <deque>

std::deque<common::types::MovementCommand> pending_movements_{};
```

- [ ] **Step 4: Split helper + `step()` queue**

```cpp
[[nodiscard]] std::deque<common::types::MovementCommand> splitWithinLimits(
    const common::types::MovementCommand& command,
    const common::types::DroneConfigData& drone) {
    std::deque<common::types::MovementCommand> parts;
    if (movementWithinLimits(command, drone)) {
        parts.push_back(command);
        return parts;
    }
    switch (command.type) {
    case common::types::MovementCommandType::Advance: {
        auto remaining = command.distance;
        while (remaining > drone.max_advance) {
            auto frag = command;
            frag.distance = drone.max_advance;
            parts.push_back(frag);
            remaining = remaining - drone.max_advance;
        }
        if (remaining > 0.0 * common::cm) {
            auto frag = command;
            frag.distance = remaining;
            parts.push_back(frag);
        }
        break;
    }
    case common::types::MovementCommandType::Elevate: {
        const auto sign = command.distance < 0.0 * common::cm ? -1.0 : 1.0;
        auto mag = mp_units::abs(command.distance);
        while (mag > drone.max_elevate) {
            auto frag = command;
            frag.distance = sign * drone.max_elevate;
            parts.push_back(frag);
            mag = mag - drone.max_elevate;
        }
        if (mag > 0.0 * common::cm) {
            auto frag = command;
            frag.distance = sign * mag;
            parts.push_back(frag);
        }
        break;
    }
    case common::types::MovementCommandType::Rotate: {
        auto remaining = command.angle;
        while (remaining > drone.max_rotate) {
            auto frag = command;
            frag.angle = drone.max_rotate;
            parts.push_back(frag);
            remaining = remaining - drone.max_rotate;
        }
        if (remaining > 0.0 * common::horizontal_angle[common::deg]) {
            auto frag = command;
            frag.angle = remaining;
            parts.push_back(frag);
        }
        break;
    }
    case common::types::MovementCommandType::Hover:
        parts.push_back(command);
        break;
    }
    return parts;
}
```

At the top of `step()`, after footprint mark:

```cpp
if (!pending_movements_.empty()) {
    common::types::MappingStepCommand fragment{};
    fragment.movement = pending_movements_.front();
    pending_movements_.pop_front();
    fragment.status = common::types::AlgorithmStatus::Working;
    const auto move_result = applyMovement(fragment);
    if (move_result.status == common::types::DroneStepStatus::Error) {
        return move_result;
    }
    ++step_index_;
    return {common::types::DroneStepStatus::Continue, {}};
}
```

After a **new** `nextStep` command passes CI3 and is not Finished: if movement present and `!movementWithinLimits`, `auto parts = splitWithinLimits(*command.movement, drone_)`; if `parts.empty()`, Continue; else execute `parts.front()` this step (copy command, replace `.movement` with front), assign `pending_movements_` from the rest, then scan using the **original** `command.scan_orientation`. Delete the immediate-`Error` `movementWithinLimits` return.

Quantity subtraction must compile under `-Werror`. If `sign * drone.max_elevate` needs an explicit `PhysicalLength` construction, use `PhysicalLength{sign * drone.max_elevate}` or unwrap/rewrap `double` cm like MockMovement — do not leave a warning.

- [ ] **Step 5: Tests pass**, including FakeDroneControl CI9 tests and `OversizeIsNotInvalidRetry` (delete or rewrite that test: oversize is now split, not Error). Collision Continue tests must still call Movement once and Continue.

---

### Task 14: CI8 docs + HLD.pdf

- [ ] **Step 1:** Delete the CI8 Known Issues row; compact `#`. Remaining KI rows should be skipped optionals (CI4, CI6, CI7, CI11, CI12), lazy `.so`, `ISimulation`, Unmapped-passable, plan-batching bug.
- [ ] **Step 2:** Explained doc: CI8 implemented via `pending_movements_`; one `nextStep` per original oversize command; fragment steps skip `nextStep`. Note that this is an intentional exception to “one `nextStep` per `step()`” and only runs when the algorithm exceeded drone max.
- [ ] **Step 3:** Matrix implemented list + `bonus.txt` file:line for `splitWithinLimits` / `pending_movements_`.
- [ ] **Step 4:** HLD: replace “Each step invokes `nextStep` once” with: each step invokes `nextStep` once **unless** draining a split-oversize queue. Update mermaid with an `alt pending fragment` (no `nextStep`).
- [ ] **Step 5: Render `HLD.pdf` once**

```bash
docker run --rm -v "c:\Users\sagi1\Projects\DroneMapper\Drone-Mapper-ex3:/work" \
  -w /work drone-mapper-ex3-dev bash -lc 'bash scripts/render_hld_pdf.sh'
```

If the script needs extra tools that the image lacks, stop and report; do not commit a stale PDF that contradicts `docs/HLD.md`. Prefer updating markdown always; PDF is the zip artifact (e14/e15).

---

### Task 15: CI8 verification gates (include VAR-04)

- [ ] **Step 1: Unit tests** — shared ctest helper. PASS.
- [ ] **Step 2: verify-cell-runtime** — full 24. Scores must match 2026-09-03. If our algorithm ever emitted oversize, scores *could* change — treat that as unexpected; inspect `advance` sizes before accepting drift. No cell ≥ 60s; `large_out` short lidar WARN only.
- [ ] **Step 3: VAR-01..03** as before.
- [ ] **Step 4: VAR-04 once** (skill `--with-baseline`):

```bash
# same docker, extra target baseline_lawnmower_algorithm_plugin
bash Simulator/tests/manual/check_baseline_algorithm.sh "$BD"
```

Overall PASS only if VAR-01..04 are PASS. Baseline lawnmower is ~5 minutes.

---

### Task 16: CI8 commit (human approval)

```text
feat: split oversize movements into legal drone steps

Optional CI8: queue in-limit Advance/Elevate/Rotate fragments across
step() calls so each fragment is a separate step.
```

Include `HLD.pdf` only if Step 5 regenerated it.

---

## Self-review

**Spec coverage**

| PDF / KI row | Task |
|--------------|------|
| CI9 continue on Error | Tasks 1–4 |
| CI2 ignore world OOB | Tasks 5–8 |
| CI10 clamp mission bounds | Tasks 5–8 |
| CI3 invalid retry then throw | Tasks 9–12 |
| CI8 split oversize | Tasks 13–16 |
| Cell runtime after each | Tasks 3, 7, 11, 15 |
| VAR harness after each | Tasks 3, 7, 11, 15 (VAR-04 in 15) |
| bonus.txt + KI compact + HLD | Tasks 2, 6, 10, 14 |
| Frozen headers / no Algorithm change / no CI4–CI7/CI11/CI12 | Global Constraints |

**Placeholder scan:** none of TBD / “add tests later” / “similar to Task N” without code.

**Type consistency:** `runMissionSteps` signature is the same in Task 1 header, tests, and `runMission`. `kMaxInvalidCommandRetries` is 3 everywhere. `pending_movements_` is `std::deque<MovementCommand>`. FakeDroneControl CI9 tests are not deleted in CI8.

**Handoff risk:** after CI8, the old oversize-`Error` MC integration test must be rewritten; CI9 remains covered by FakeDroneControl.

---

Plan complete and saved to `docs/superpowers/plans/2026-09-04-optional-common-issues-recovery.md`. Two execution options:

**1. Subagent-Driven** — **not available for this plan** (you asked not to use expensive models with subagents).

**2. Inline Execution (use this)** — run the tasks in this session with executing-plans, same model, checkpoints after each issue’s verification gate and before each commit.

Which approach? (Reply “inline” / “2” to start.)
