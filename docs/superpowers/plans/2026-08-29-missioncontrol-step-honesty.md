# MissionControl Step Honesty Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `DroneControlImpl::step()` call `nextStep` once, execute movement then one scan (fuse), and honor co-emitted movement+scan — removing `kMaxScansPerStep` batching.

**Architecture:** Replace the scan-batching `while` in `DroneControlImpl.cpp` with a single sequential movement→scan path matching `frozen-interfaces.mdc`. Add gtests for co-emission and failure/scan interaction. Update docs that describe batching. Re-measure with the project A harness (`honest` column).

**Tech Stack:** C++20, gtest, Docker `drone-mapper-ex3-dev`, existing `scripts/benchmark/run_benchmark.py`.

## Global Constraints

- Spec: `docs/superpowers/specs/2026-08-29-missioncontrol-step-honesty-design.md`
- **Never `git commit` without explicit human approval** in this chat.
- **No Algorithm/ changes.** Co-emission is tested via `ScriptedAlgorithm` only.
- Keep Empty carving + non-null `latest_scan_` (decision 1).
- Recoverable movement failure (`blocked`/`boundary`) falls through to scan if present; hard failure / non-recoverable throw: no scan.
- Branch: continue `algorithm-benchmark-harness` (or rename later if splitting PRs).

---

### Task 1: Rewrite `DroneControlImpl::step()` + new gtests

**Files:**
- Modify: `MissionControl/src/DroneControlImpl.cpp:173-253`
- Modify: `MissionControl/tests/test_drone_control.cpp` (FakeMovement counters + new TESTs)

**Interfaces:**
- Consumes: existing fakes / `ScriptedAlgorithm`
- Produces: new step semantics per design pseudocode

- [ ] **Step 1: Re-grep** that nothing asserts `kMaxScansPerStep` or multi-scan per step:

```bash
rg "kMaxScansPerStep|scans_this_step" MissionControl/
```

Expected: only `DroneControlImpl.cpp` (about to delete).

- [ ] **Step 2: Add FakeMovement counters** in the test file:

```cpp
int advance_count_ = 0;
// in advance(): ++advance_count_; before other logic
```

- [ ] **Step 3: Write failing tests** (append to `test_drone_control.cpp`):

1. `ExecutesMovementAndScanInOneStep` — one command with Advance + scan_orientation; after one `step()`: `advance_count_==1`, `scan_count_==1`, `algorithm.call_index_==1`, status Continue, `state().step_index==1`.
2. `RecoverableBlockedStillScans` — throw blocked + scan in same command; Continue, `scan_count_==1`.
3. `HardMovementFailureSkipsScan` — `advance_ok_=false` without blocked/boundary message + scan; Error, `scan_count_==0`.
4. `AlwaysScanAlgorithmScansOncePerStep` — script of 5 Working+scan commands; call `step()` 5 times; `scan_count_==5` and `call_index_==5` (one nextStep per step).
5. Existing `ReturnsCompletedWhenAlgorithmFinishes` / `CollisionBlockedThrowContinues` remain green.

- [ ] **Step 4: Replace `step()` body** after the terminal-status check with:

```cpp
    if (command.movement.has_value()) {
        if (!movementWithinLimits(*command.movement, drone_)) {
            return common::types::DroneStepResult{
                common::types::DroneStepStatus::Error,
                "Movement command exceeds drone limits.",
            };
        }

        try {
            const common::types::MovementResult movement_result =
                executeMovement(movement_, *command.movement);
            if (!movement_result) {
                if (!isRecoverableMovementFailure(movement_result.message)) {
                    return common::types::DroneStepResult{
                        common::types::DroneStepStatus::Error,
                        movement_result.message.empty() ? "Movement failed."
                                                        : movement_result.message,
                    };
                }
                // recoverable: fall through to optional scan
            }
        } catch (const std::exception& ex) {
            if (!isRecoverableMovementFailure(ex.what())) {
                throw;
            }
            // recoverable throw: fall through to optional scan
        }
    }

    if (command.scan_orientation.has_value()) {
        latest_scan_ = lidar_sensor_.scan(*command.scan_orientation);
        has_latest_scan_ = true;
        ScanResultToVoxels::applyToMap(
            output_map_, gps_.position(), gps_.heading(), latest_scan_, lidar_);
        supplementGridAlignedFusion(
            output_map_, gps_.position(), gps_.heading(), latest_scan_, lidar_.z_max);
        markDroneFootprintEmpty(output_map_, gps_.position(), drone_.radius);
    }

    ++step_index_;
    return common::types::DroneStepResult{common::types::DroneStepStatus::Continue, {}};
```

Delete the entire `kMaxScansPerStep` / `while` block. Delete early `++step_index_; return Continue` inside recoverable branches (those now fall through).

- [ ] **Step 5: Build + test in Docker**

```bash
docker run --rm -e VCPKG_ROOT=/usr/local/vcpkg -v "$PWD:/work" -w /work drone-mapper-ex3-dev \
  bash -lc 'cmake --build --preset default -j$(nproc) --target test_drone_control && ./build/default/MissionControl/test_drone_control'
```

Expected: all DroneControl tests PASS.

- [ ] **Step 6: Propose commit**

```
feat: honor one scan and co-emitted movement per drone step
```

---

### Task 2: Documentation updates

**Files:**
- `docs/HLD.md` (~251–253)
- `docs/known-issues.md` rows 20–21
- `docs/mapping-algorithm-analysis.md` (~140–151)
- `docs/superpowers/specs/2026-08-28-independent-component-variants-design.md` (~109–113)
- Confirm `.cursor/rules/frozen-interfaces.mdc:66-67` needs **no** edit

- [ ] **Step 1:** Edit each doc per design "Documentation updates" table.
- [ ] **Step 2:** Propose commit `docs: align HLD and known issues with one-scan-per-step MC`

---

### Task 3: Honest-column baseline + ctest smoke

**Files:**
- Create: `docs/benchmarks/2026-08-29-post_b_honest.csv` (+ `.md`) — date may differ
- Modify: roadmap Project B status

- [ ] **Step 1:** Rebuild MissionControl `.so` in Docker; run harness:

```bash
# after apt/venv as in scripts/benchmark/README.md
scripts/benchmark/.venv/bin/python scripts/benchmark/run_benchmark.py \
  --columns honest --label post_b_honest --num-threads 8 \
  --baseline docs/benchmarks/2026-08-29-pre_b_baseline.csv
```

Note: baseline CSV has `ex2_comparable` column names — either run without `--baseline` and manually compare, or add a small note that baseline diff matches on cell only when column names align. Prefer writing a second CSV and comparing totals in the markdown by hand / summarize without column-key match.

Simpler: run `--columns honest --label post_b_honest` without baseline flag; document expected fall vs pre_b `ex2_comparable` totals in the md and roadmap.

- [ ] **Step 2:** `ctest --test-dir build/default --output-on-failure` in Docker (or at least MissionControl + Algorithm tests).
- [ ] **Step 3:** Optionally `check_adversarial_plugins.sh` if time.
- [ ] **Step 4:** Propose commit `docs: record post-B honest-column score regression`

---

## Spec coverage

| Requirement | Task |
|-------------|------|
| nextStep once / no batch loop | 1 |
| movement then scan | 1 |
| co-emission | 1 |
| recoverable still scans | 1 |
| hard failure skips scan | 1 |
| docs | 2 |
| honest harness measurement | 3 |
