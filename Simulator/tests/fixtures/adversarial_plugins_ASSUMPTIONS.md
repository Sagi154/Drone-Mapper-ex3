# var03_adversarial — TEST-ONLY plugin assumptions

These plugins are **grader / host-robustness fixtures**, not student algorithms.
Each `.cpp` is one SHARED library (`PREFIX ""`) that uses the frozen
`REGISTER_MAPPING_ALGORITHM` / `REGISTER_MISSION_CONTROL` macros. They are
legal under `common::` APIs and deliberately hostile.

Registration constructors (`MappingAlgorithmRegistration` /
`MissionControlRegistration`) are **undefined in these `.so` files**; the
host that `dlopen`s them must export those symbols (as `skeleton_host` and
the Simulator do).

No `new` / `delete`. Factory lambdas use `std::make_unique`. Constructors
take the frozen `*Dependencies` **by value** (as the macros do) and do not
own sensors, maps, or the peer plugin.

## Algorithms

| File | Class | `REGISTER_MAPPING_ALGORITHM` |
|---|---|---|
| `adversarial_throw_algorithm_plugin.cpp` | `AdversarialThrowAlgorithm` | yes |
| `adversarial_never_finish_algorithm_plugin.cpp` | `AdversarialNeverFinishAlgorithm` | yes |
| `adversarial_into_occupied_algorithm_plugin.cpp` | `AdversarialIntoOccupiedAlgorithm` | yes |
| `adversarial_bad_scan_orientation_algorithm_plugin.cpp` | `AdversarialBadScanOrientationAlgorithm` | yes |

All algorithm `nextStep` implementations ignore `latest_scan` (may be null).
None ever returns `Finished` / `FinishedWithUnmappableVoxels`.

### Throw algorithm

Every `nextStep` call throws `std::runtime_error` with message
`adversarial_throw_algorithm: nextStep`. The constructor does **not** throw
(so `dlopen` / factory construction can succeed).

### Never-finish algorithm

Every call returns `AlgorithmStatus::Working`, `movement = Hover`, and
`scan_orientation = nullopt`. Hosts that only stop on Finished must hit
`max_steps` (or hang if they have no cap).

### Into-occupied algorithm

Inspects **`output_map` only** (algorithms cannot see the hidden world map).

1. Sample voxel **centers** on a grid from `output_map.getMapConfig()`:
   `boundaries` × `resolution`. Frame is treated as the same as
   `DroneState.position`. **`MapConfig.offset` is not applied.**
2. A cell counts if `isInBounds(center)` and `atVoxel(center) == Occupied`.
   `PotentiallyOccupied` / `Unmapped` / `Empty` are ignored.
3. If `resolution <= 0` or any axis has zero cells (`max <= min` or span
   `< resolution`), treat as **no Occupied**.
4. If at least one Occupied cell exists, pick the **nearest** (Euclidean cm)
   to the current drone position; raster-order tie-break (x, then y, then z).
5. **Advance toward it:**
   - Heading convention matches `advance`: forward XY =
     `(cos(heading.horizontal), sin(heading.horizontal))`.
   - Desired yaw = `atan2(dy, dx)` in degrees.
   - If `|wrap180(desired - heading)| > 1°`, return `Rotate`
     (Right if delta > 0, Left if delta < 0) with that absolute angle.
     Angle is **not** clamped to `max_rotate`.
   - Otherwise return `Advance` with distance **`drone_config.max_advance`**
     (full step, including when already on the cell). No `Elevate`.
6. If **no** Occupied cell: `Advance` `max_advance` with current heading,
   every call.

`scan_orientation` is always `nullopt`. Status is always `Working`.

### Bad-scan-orientation algorithm

Every call returns `Working`, `movement = nullopt`, and

`scan_orientation = {horizontal: +1e12 deg, altitude: -1e12 deg}`.

Angles are **not** wrapped to `[0, 360)` / `[-90, 90]`. Hosts that pass this
straight into `ILidar::scan` or trig are under test.

## MissionControls

| File | Class | `REGISTER_MISSION_CONTROL` |
|---|---|---|
| `adversarial_throw_mission_control_plugin.cpp` | `AdversarialThrowMissionControl` | yes |
| `adversarial_empty_mission_control_plugin.cpp` | `AdversarialEmptyMissionControl` | yes |
| `adversarial_implausible_steps_mission_control_plugin.cpp` | `AdversarialImplausibleStepsMissionControl` | yes |

None of these MissionControls call `mapping_algorithm`, sensors, movement,
or `output_map`. `MissionRunResult.errors` is left empty.

### Throw MissionControl

`runMission` throws `std::runtime_error` with message
`adversarial_throw_mission_control: runMission`. Constructor does not throw.

### Empty MissionControl

Returns `{status: Completed, steps: 0}` immediately. Intended as a
zero-work plugin (comparative/competition still get a result object).

### Implausible-steps MissionControl

Returns `{status: Error, steps: 999999}` immediately, without running a
loop. `999999` is a fixed literal (not `max_steps`). Hosts that trust
`steps` for scoring / YAML `total_steps` are under test.

## Build

`CMakeLists.txt` in this directory builds **one SHARED target per `.cpp`**,
output names matching the stems (no `lib` prefix), C++20,
`-Wall -Wextra -Werror -pedantic`. Linked only to `common::common`
(headers + mp-units). Wired from the skeleton root via
`add_subdirectory(blind_deliverables/var03_adversarial)`.

These files are **not** part of a student zip (`Algorithm_<ids>.so` /
`MissionControl_<ids>.so`). Place the built `.so` files at CLI plugin
paths to exercise Simulator / foreign-host exception and result handling.
