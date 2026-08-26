---
name: port-ex2-component
description: Ports a component from the Drone-Mapper-ex2 implementation into the ex3 three-project layout, applying namespace, include, and API changes. Use when moving Map3DImpl, MockLidar, MockGPS, MockMovement, MapsComparison, MissionControlImpl, DroneControlImpl, MappingAlgorithmImpl, ScanResultToVoxels, YAML parsers, or any ex2 file into Drone-Mapper-ex3.
---

# Port Ex2 Component

Most of assignment 3 is assignment 2's code, relocated into three projects and re-fitted to changed
headers. Port **one component at a time** and keep its behavior identical.

Reference implementation: `../Drone-Mapper-ex2/{include/drone_mapper,src}/`.

## 1. Decide where it goes — before touching code

Look it up in `docs/component-placement.md`. If it isn't listed, apply the staff's realism test from
`context/Structuring the project.pdf`:

- Would this class exist inside a real drone or ground station? → `MissionControl/` or `Algorithm/`.
- Does it exist only because we are *simulating* the world (holds the hidden map, scores the result,
  parses the composition)? → `Simulator/`.
- Do **two** projects genuinely need it? → `UserCommon/`. Only then, never speculatively.

Never `common/` — that folder is course-published and read-only.

## 2. Move first, adapt second

Make the relocation its own commit so the diff stays reviewable:

```bash
git mv ...   # or copy from ../Drone-Mapper-ex2 and commit as-is
```

Then adapt in a second commit.

## 3. Apply the mechanical changes

Full table in `docs/api-delta-ex2-to-ex3.md`. The mechanical pass:

| Ex2 | Ex3 |
|-----|-----|
| `#include <drone_mapper/X.h>` | `#include <Common/X.h>` (capital `C`) |
| `#include <drone_mapper/ISimulation*.h>`, `types/SimulationTypes.h` | `#include <Simulator/...>` |
| `#include <drone_mapper/IDroneControl.h>` | `#include <MissionControl/IDroneControl.h>` |
| `namespace drone_mapper` | `common` / `simulator` / `mission_control` for published types |
| our own classes | wrap in `algorithm_207190406_209543255`, `mission_control_207190406_209543255`, or `user_common_207190406_209543255` |
| bare `types::` | `common::types::` or `simulator::types::` |

`Common/Types.h` no longer pulls in the simulation types — a file that used `types::SimulationConfigData`
via `Types.h` now needs an explicit `<Simulator/SimulationTypes.h>`, and if that file is in a plugin, it
shouldn't have needed those types at all.

## 4. Fix the API breakages

These will not be caught by a search-and-replace:

1. **`config_load_error` is gone** from `DroneConfigData`, `LidarConfigData`, `MissionConfigData`,
   `SimulationConfigData`. Our ex2 parsers and `SimulationRunFactoryImpl::appendConfigLoadErrors`
   depended on it. Replace with a `UserCommon` result type that pairs the parsed config with
   `std::vector<common::types::ErrorRef>`, and keep the "log immediately, score `-1`, continue" behavior.
2. **`MissionConfigData::boundaries` → `mission_bounds`.** The YAML key is still `boundaries`.
3. **`MappingStepCommand::fusion_max` is gone.** Delete the field and always fuse at full lidar `z_max`
   (our ex2 algorithm never set it, so no behavior change).
4. **`IMappingAlgorithm` constructor** takes `MappingAlgorithmDependencies` — a struct, by value.
5. **`MissionControlImpl` constructor** takes `MissionControlDependencies` — by value — and must now
   **construct its own `DroneControl`** from the lidar/gps/movement/output_map/algorithm references in
   that struct. In ex2 the factory wired `DroneControlImpl`; it no longer does.
6. **`SimulationCompositionData`** is now `simulation_mission_groups` (a vector of
   `tuple<SimulationConfigData, vector<MissionConfigData>>`) plus `drone_configs` / `lidar_configs`. The
   set of runs is unchanged: **(simulation, mission) pairs × drones × lidars**.
7. **`SimulationManagerReport`** gained `composition_file` — populate it.

## 5. Preserve the ex2 fixes we already paid for

Do not regress these while porting:

- Mission boundaries are **local** coordinates: add `simulation.map_axes_offset` to all six boundary values
  when building the output map config. Without it, the house scenario writes scans outside the output map.
- The scorer is seeded with the **world** spawn (local position + offset), which activates the
  reachability filter so sealed rooms don't penalise the score.
- Hidden maps are `.npy` of **mixed dtype** — `scenario_small`/`scenario_big` are `int8`,
  `scenario_house` is `uint8` with values up to `45`. Any value `>= 1` in a hidden map is `Occupied`,
  whatever the dtype. See `docs/map3d-contract.md`.
- Errors are logged the moment they occur — never a deferred flush at shutdown.
- Algorithm tuning that produced the ex2 scores (score-based frontier selection, 2-cell unmapped sphere,
  visit dampening, explore-distance cache, path compression) is documented in
  `../Drone-Mapper-ex2/docs/HLD.md` under "Algorithm State Machine". Port it whole; don't re-tune.

The current `MappingAlgorithmFrontier` port still has the Ex2 ALG28 hang: `findPath` (and the other
BFS helpers) only stop expanding when `isSpherePassable` is false, so `isSpherePassable` → `return true`
walks an unbounded grid. This is **not** a preserved fix — add a map-volume clip or expansion cap when
touching the planner. Details: `docs/ex2-grading-handoff.md` §3.

## 6. Register plugins

If the component is `MappingAlgorithmImpl` or `MissionControlImpl`, add the one registration line at global
scope in its `.cpp` and read `.cursor/skills/plugin-loading-and-registration/SKILL.md`:

```cpp
REGISTER_MAPPING_ALGORITHM(MappingAlgorithmImpl_207190406_209543255)
REGISTER_MISSION_CONTROL(MissionControlImpl_207190406_209543255)
```

## 7. Verify

1. Wire it into the right `CMakeLists.txt`, call `drone_warnings(<target>)`, and build:
   `cmake --build --preset default`. `-Werror` means warnings are failures.
2. Port the component's ex2 tests alongside it and run them. Same assertions passing after the migration is
   the proof the port is behavior-preserving.
3. Once the end-to-end path exists, run the instructor composition and compare scores against ex2's
   recorded results (`../Drone-Mapper-ex2/docs/HLD.md`: house_lower 100%, large_room ~92–96%,
   small_room ~87–90%, large_out ~80–88%, small_out ~75–89%, house_full ~56–62%). A large drop means the
   port broke something — most often the boundary offset or the map dtype handling.

## Do not port

- `src/maps_comparison_main.cpp` — the standalone `maps_comparison` binary is not required by assignment 3.
  Keep the `MapsComparison` class (the simulator needs it to score); drop the executable.
- Ex1 patterns that ex2 already rejected: `ScanProbing`, `TickSnapshot`, `SimulationState`, deferred
  error flushing, whole-program abort on a single scenario failure, `namespace su = mp_units::...` inside a
  function (e17s), getters exposing internal containers (e22).
