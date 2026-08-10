# Code Review — Yoav's Track (U1–U4, Y1–Y9)

Findings ordered by severity. No code changes implied by this document.

---

## HIGH

### 1. Double output-map save (correctness bug)

**Files:** `MissionControlImpl.cpp` → `finalizeMission`, `SimulationRunImpl.cpp` → `run()`

`MissionControlImpl::finalizeMission` always calls `output_map.save(output_file)` before returning. Then `SimulationRunImpl::run()` also calls `output_map_->save(output_map_file_)`. Because `SimulationRunFactoryImpl` passes `output_path` as `output_map_file` in `MissionControlDependencies`, both paths are non-empty and identical.

**Consequences:**

- The map is written to disk twice per run.
- If `MissionControlImpl::save` throws, `finalizeMission` returns a `MissionRunStatus::Error` result with error `MAP_SAVE_FAILED`, causing `runMission()` to report failure — even though the run itself succeeded and `SimulationRunImpl` would have tried again. The mission's score ends up `-1.0` for an otherwise-correct run.

**Fix:** either pass an empty `output_map_file` in `MissionControlDependencies` (let `SimulationRunImpl` own the save), or remove the `output_map.save()` call from `finalizeMission`.

---

### 2. Empty `output_path` causes spurious `MAP_SAVE_FAILED`

**File:** `MissionControlImpl.cpp` → `finalizeMission`

```cpp
output_map.save(output_file);  // output_file may be ""
```

If the orchestrator passes an empty `output_path` (e.g., "no output desired"), `MissionControlImpl` calls `output_map.save("")`. `Map3DImpl::save("")` throws (can't save to an empty path), the exception is caught by `finalizeMission`, and the run is reported as `Error` with `MAP_SAVE_FAILED`. A logically successful mission gets a wrong status.

---

### 3. Composition parser silently uses default-constructed config on parse failure

**File:** `CompositionYamlParser.cpp`

```cpp
const auto sim_result = parseSimulationConfig(sim_path, log);
const simulator::types::SimulationConfigData sim_cfg = sim_result.value; // used even if !sim_result.ok
...
missions.push_back(parseMissionConfig(m_path, log).value); // same problem
```

When `parseSimulationConfig` or `parseMissionConfig` returns `ok = false`, the group is still added to the composition with a zero/default-constructed config. That run will use `map_filename = ""`, `map_resolution = 0`, `max_steps = 0`, etc. It should `continue` (skip the group) on failure instead of using the bad defaults.

---

## MEDIUM

### 4. `MockGPS` initial position is not snapped

**File:** `MockGPS.cpp`

```cpp
MockGPS::MockGPS(common::Position3D position, ...)
    : position_(position), ...  // stored raw, not snapped
```

`setPosition()` snaps to the GPS resolution grid, but the constructor directly assigns `position_` without calling `setPosition`. If `worldInitialDronePosition` returns a position that is not a multiple of `gps_resolution`, the first `gps_.position()` call in `DroneControlImpl::step()` returns an unsnapped coordinate. All subsequent positions (after any movement) are snapped. The algorithm sees a different precision on step 0 vs all subsequent steps, which can cause subtle navigation bias.

**Fix:** change the constructor to call `setPosition(position)` instead of member-initialising `position_`.

---

### 5. `SimulationRunImpl::run()` save exception propagates silently

**File:** `SimulationRunImpl.cpp`

```cpp
output_map_->save(output_map_file_);  // after runMission() — no exception handler
```

If directory creation fails or the save itself throws, the exception propagates out of `run()` uncaught. The comment says only `runMission()` exceptions are meant to propagate (for collision handling). A save failure would reach Sagi's worker thread with the same catch and be treated as a `-1` run — indistinguishable from a collision. It should be caught here and recorded as a `MAP_SAVE_FAILED` result entry.

---

### 6. All YAML parsers return `ok = true` on empty/partial config

**Files:** All five `*YamlParser.cpp` files

Every parser sets `result.ok = true` unconditionally before returning, even if none of the fields were successfully read (e.g., a YAML file that exists but has no recognizable keys). Default-constructed `DroneConfigData` with `radius = 0`, `max_advance = 0`, etc. silently enters the run matrix. A missing mandatory key (like `map_filename`) should set `ok = false`.

---

## LOW

### 7. Helper functions duplicated between `ScanResultToVoxels.cpp` and `DroneControlImpl.cpp`

`absoluteBeamOrientation`, `pointAlongBeam`, and `isMissDistance` are copy-pasted into both files. They are **currently identical**, but a future bug-fix in one will silently not apply to the other. These should be moved to a shared private utility header (e.g., `MissionControl/src/BeamMath.hpp`).

---

### 8. `RunErrorLog::log()` silently discards entries on stream failure

**File:** `RunErrorLog.cpp`

```cpp
void RunErrorLog::log(const common::types::ErrorRef& error) {
    if (!stream_.is_open()) {
        stream_.open(log_path_, std::ios::app);
    }
    stream_ << ...;  // no check if open succeeded
    stream_.flush();
}
```

Neither the initial `open` in the constructor nor the re-open attempt in `log()` checks whether the stream is actually open. A bad path silently swallows all log entries. Minimum fix: check `stream_.is_open()` before writing and either throw or fall back to `std::cerr`.

---

### 9. `MockLidar::traceBeam` recomputes invariant direction components inside the beam loop

**File:** `MockLidar.cpp`

```cpp
for (PhysicalLength distance = 0.0 * cm; distance <= config_.z_max; distance += step) {
    const double dir_x = dx.force_numerical_value_in(mp::one);  // same every iteration
    const double dir_y = dy.force_numerical_value_in(mp::one);  // same every iteration
    const double dir_z = dz.force_numerical_value_in(mp::one);  // same every iteration
    ...
}
```

`dx`, `dy`, `dz` don't change across iterations — move these three extractions out of the loop. This multiplies the overhead for every beam trace by the number of steps (~`z_max / (0.1 * resolution)`).

---

## NOTE

### 10. `SimulationRunImpl::run()` exception propagation requires Sagi's worker thread to catch it

**File:** `SimulationRunImpl.cpp`

The comment acknowledges this: MockMovement collision exceptions from `runMission()` propagate out of `run()`. If Sagi's S7 worker thread does not `catch (...)` every call to `ISimulationRun::run()`, the process will call `std::terminate`. This is documented intent, but it is a shared contract that must be explicitly verified during integration (vertical slice).

---

## Summary

| # | Severity | Location | Issue |
|---|---|---|---|
| 1 | HIGH | `MissionControlImpl`, `SimulationRunImpl` | Output map saved twice; `MAP_SAVE_FAILED` on correct runs |
| 2 | HIGH | `MissionControlImpl::finalizeMission` | Empty `output_path` → spurious `MAP_SAVE_FAILED` error |
| 3 | HIGH | `CompositionYamlParser.cpp` | Uses default config on parse failure instead of skipping group |
| 4 | MED | `MockGPS.cpp` | Initial position not snapped; inconsistent precision on step 0 |
| 5 | MED | `SimulationRunImpl.cpp` | Save exception after `runMission()` propagates as if it were a collision |
| 6 | MED | All YAML parsers | `ok = true` even when no meaningful fields were read |
| 7 | LOW | `ScanResultToVoxels.cpp` / `DroneControlImpl.cpp` | Duplicated beam-math helpers — divergence risk on future fixes |
| 8 | LOW | `RunErrorLog.cpp` | Stream failures silently discard log entries |
| 9 | LOW | `MockLidar.cpp` | Invariant `dir_x/y/z` recomputed inside tight beam-trace loop |
| 10 | NOTE | `SimulationRunImpl.cpp` | Collision exception propagation requires Sagi's catch — integration risk |
