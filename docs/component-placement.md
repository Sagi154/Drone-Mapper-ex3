# Component placement: which ex2 file goes where in ex3

Ex3 has five folders and a hard rule that `common/` is course-published and untouchable. Every ex2
source file needs a home. The course staff explained their reasoning in
`context/Structuring the project.pdf`; this table applies it to our ex2 code.

## The staff's rule of thumb

> "The mocks we make are completely simulated, and would be replaced by real APIs of real components in
> real life." — so they live in `Simulator/src/`.

Read it as a **realism test**: if a class would exist inside a real drone or ground station, it belongs
to `MissionControl` or `Algorithm`. If it only exists because we are *simulating* the world, it belongs
to `Simulator`. Interfaces used across the `.so` boundary live in `common/`; interfaces used by exactly
one project live in that project's `common_*` subfolder.

## Folder map

| Folder | Role | Build artifact |
|--------|------|----------------|
| `common/` | course-published interfaces used by more than one project. **Read-only.** | none (INTERFACE lib `common::common`) |
| `Simulator/common_simulator/include/Simulator/` | published interfaces used only by the Simulator | none |
| `Simulator/{include,src}/` | our simulator implementation | `simulator_<ids>` executable |
| `MissionControl/common_mission_control/include/MissionControl/` | published `IDroneControl` | none |
| `MissionControl/{include,src}/` | our mission control implementation | `MissionControl_<ids>.so` |
| `Algorithm/{include,src}/` | our mapping algorithm | `Algorithm_<ids>.so` |
| `UserCommon/` | **our** code needed by more than one project. No build file — sources compile into each consumer. | none |

## Ex2 → ex3 placement

### Simulator project

| Ex2 file | Ex3 destination | Why |
|----------|-----------------|-----|
| `src/SimulationManager.cpp` | `Simulator/src/` | orchestration; grows the mode/threading logic |
| `src/SimulationRunImpl.cpp` | `Simulator/src/` | owns one run's object graph |
| `src/SimulationRunFactoryImpl.cpp` | `Simulator/src/` | now also resolves plugin factories from the registrar |
| `src/MockLidar.cpp` | `Simulator/src/` | pure simulation — holds the hidden map |
| `src/MockGPS.cpp` | `Simulator/src/` | pure simulation |
| `src/MockMovement.cpp` | `Simulator/src/` | pure simulation — detects real-map collisions |
| `src/Map3DImpl.cpp` | `Simulator/src/` | simulator owns both hidden and output maps; plugins see only `IMap3D`/`IMutableMap3D` |
| `src/MapsComparison.cpp` | `Simulator/src/` | scoring is the simulator's job |
| `src/io/*YamlParser.cpp`, `CompositionYamlParser.cpp` | `Simulator/src/io/` | only the simulator reads composition/config YAML |
| `src/io/SimulationOutputYamlWriter.cpp` | `Simulator/src/io/` | plus new comparative/competitive report writers |
| `src/io/SimulationCli.cpp` | `Simulator/src/io/` | rewritten for the two ex3 modes |
| `src/io/StderrErrorLog.cpp`, `RunPathHelpers.cpp`, `PathResolver.cpp` | `Simulator/src/io/` | simulator-side I/O |
| `src/drone_mapper_simulation_main.cpp` | `Simulator/src/` | becomes `simulator_<ids>` entry point |
| `src/maps_comparison_main.cpp` | **drop** | not required by assignment 3 |
| — | `Simulator/src/` (new) | registration `.cpp` files, registrar singleton, plugin loader (`dlopen`/`dlclose`), thread pool |

### MissionControl project

| Ex2 file | Ex3 destination | Why |
|----------|-----------------|-----|
| `src/MissionControlImpl.cpp` | `MissionControl/src/` | the plugin itself; takes `MissionControlDependencies` |
| `src/DroneControlImpl.cpp` | `MissionControl/src/` | mission control creates its own drone controller (see `MissionControlDependencies` comment) |
| `src/ScanResultToVoxels.cpp` | `MissionControl/src/` **or** `UserCommon/` | only `DroneControlImpl` calls it → keep in `MissionControl/` unless a second project needs it |
| — | `MissionControl/src/` (new) | verbose output writer, gated on `MissionControlDependencies::verbose` |

### Algorithm project

| Ex2 file | Ex3 destination | Why |
|----------|-----------------|-----|
| `src/MappingAlgorithmImpl.cpp` | `Algorithm/src/` | the plugin itself; takes `MappingAlgorithmDependencies` |
| `src/MappingAlgorithmFrontier.cpp/.h` | `Algorithm/src/` | private helper of the algorithm |

### UserCommon

Only put something here once a **second** project actually needs it. Genuine candidates:

| Ex2 file | Note |
|----------|------|
| `src/SimulationCoordUtil.cpp` | world↔voxel and offset math — the algorithm, drone control, and simulator all reason about grid coordinates |
| `src/io/TimeFormat.cpp` | ISO-8601 UTC timestamps for error logs (all three projects) and the `<time>` in output folder names |
| `src/io/RunErrorLog.cpp`, `include/.../io/IRunErrorLog.h` | immediate-flush error log; mission control and algorithm may both write error logs |
| a `ConfigParseResult<T>` type | replaces the removed `config_load_error` fields (see `docs/api-delta-ex2-to-ex3.md`) |

`UserCommon/` has **no build file**. Each project's CMakeLists adds the `UserCommon` include directory
and compiles the `.cpp` files it needs. That means a `UserCommon` translation unit can end up compiled
into the simulator **and** into a `.so`. Keep it small, dependency-light, and free of mutable global
state so duplicate copies cannot disagree.

All our `UserCommon` code goes in `namespace user_common_207190406_209543255`.

## Namespaces

| Code | Namespace |
|------|-----------|
| Course-published `common/` headers | `common`, `common::types` |
| Course-published `Simulator/common_simulator/` | `simulator`, `simulator::types` |
| Course-published `MissionControl/common_mission_control/` | `mission_control` |
| Our algorithm code | `algorithm_207190406_209543255` |
| Our mission control code | `mission_control_207190406_209543255` |
| Our shared code | `user_common_207190406_209543255` |
| Our simulator code | `simulator` (not ID-suffixed — the executable is not dynamically loaded) |

Assignment 3 (2026-08-26 forum refresh) requires these ID-suffixed namespaces in **snake_case**.
`.so` / executable basenames stay PascalCase-prefixed (`Algorithm_*.so`, etc.).

## Interfaces stay where the course put them

Do not move a published header between `common/` and a project's `common_*` folder even if it looks
misplaced. `IMissionControl` lives in `common/` (not `MissionControl/common_mission_control/`) precisely
because the Simulator needs it; `IDroneControl` lives in `MissionControl/common_mission_control/` because
only mission control needs it. Same logic for `IMappingAlgorithm` vs. the `Algorithm` project.
