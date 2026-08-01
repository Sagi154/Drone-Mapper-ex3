# API delta: ex2 skeleton → ex3 skeleton

The assignment doc says "interfaces are not changed from assignment 2". That is true of the **semantics**
but not of the **declarations** — every ex2 header was moved, re-namespaced, and several types changed
shape. Ported ex2 code will not compile until these are applied.

Verified by diffing `../Drone-Mapper-ex2/include/drone_mapper/**` against
`../ex_3_skeleton/{common,Simulator,MissionControl}/**`.

> **Baseline caveat:** "ex2" below means our own `Drone-Mapper-ex2` implementation, which itself
> deviated from the pristine `../ex_2_skeleton` in several frozen headers (see §3 and §4). Sections 1,
> 2, 5, and 6 describe genuine skeleton-to-skeleton evolution (confirmed against `ex_2_skeleton`
> directly). Sections 3 and 4 are **not** assignment-2→assignment-3 changes at all — they document
> where our own past edits to "do not change this interface" headers need to be reverted, because
> ex3's frozen versions match the pristine ex2 skeleton, not our modified one.

## 1. Header locations and namespaces

| Header | Ex2 | Ex3 | Namespace |
|--------|-----|-----|-----------|
| `Units.h`, `Types.h`, `types/*.h` | `include/drone_mapper/` | `common/include/Common/` | `common`, `common::types` |
| `IMap3D`, `IMutableMap3D`, `IGPS`, `ILidar`, `IDroneMovement`, `IMappingAlgorithm`, `IMissionControl` | `include/drone_mapper/` | `common/include/Common/` | `common` |
| `MappingAlgorithmFactory`, `MappingAlgorithmRegistration`, `MissionControlFactory`, `MissionControlRegistration` | — (new) | `common/include/Common/` | `common` |
| `ISimulation`, `ISimulationRun`, `ISimulationRunFactory`, `SimulationTypes.h` | `include/drone_mapper/` | `Simulator/common_simulator/include/Simulator/` | `simulator`, `simulator::types` |
| `IDroneControl` | `include/drone_mapper/` | `MissionControl/common_mission_control/include/MissionControl/` | `mission_control` |

Include-path change is **case-sensitive**: `#include <drone_mapper/IMap3D.h>` → `#include <Common/IMap3D.h>`
(capital `C`). Simulator headers use `<Simulator/...>`, mission-control headers `<MissionControl/...>`.

`SimulationTypes.h` is no longer pulled in by `Common/Types.h` — the simulator's composition and report
types are private to the Simulator project. Plugins cannot see them.

`MissionControl/.../IDroneControl.h` does `using namespace common;` inside `namespace mission_control`,
so `types::DroneStepResult` resolves there without qualification. Do not rely on that in files that
don't include it.

## 2. Constructor injection replaced by dependency structs

`IMappingAlgorithm`'s four-argument constructor is gone:

```cpp
// ex2
IMappingAlgorithm(const types::MissionConfigData&, const types::LidarConfigData&,
                  const types::DroneConfigData&, const IMap3D& output_map);

// ex3 — Common/IMappingAlgorithm.h
struct MappingAlgorithmDependencies {
    const types::MissionConfigData& mission_config;
    const types::LidarConfigData& lidar_config;
    const types::DroneConfigData& drone_config;
    const IMap3D& output_map;
};
explicit IMappingAlgorithm(MappingAlgorithmDependencies dependencies);
```

`MissionControlDependencies` is entirely new (`Common/MissionControlFactory.h`) and is what the
simulator hands to a mission-control plugin:

```cpp
struct MissionControlDependencies {
    const types::MissionConfigData& mission_config;
    const types::DroneConfigData& drone_config; // mission will create its own drone controller
    ILidar& lidar;
    IGPS& gps;
    IDroneMovement& movement;
    IMutableMap3D& output_map;
    IMappingAlgorithm& mapping_algorithm;
    std::filesystem::path output_map_file;
    bool verbose = false;
};
```

Two consequences:

- **`MissionControlImpl` now builds its own `DroneControl`** — the simulator no longer wires it. Our ex2
  `DroneControlImpl` therefore belongs to the `MissionControl` project, not the Simulator.
- **`verbose` and `output_map_file` arrive through the dependency struct**, not via constructor args or
  globals.

This is a genuine **architectural inversion**, not just a signature change: in ex2,
`SimulationRunFactoryImpl` built `DroneControlImpl` and handed the already-wired `IDroneControl&`
into `MissionControlImpl`. In ex3, `MissionControlImpl` receives raw `ILidar&`/`IGPS&`/
`IDroneMovement&` instead and must construct its own `DroneControlImpl` internally — consistent with
`IDroneControl.h` now living under `MissionControl/common_mission_control/`, private to that module.

The registration macros construct plugins as `std::make_unique<class_name>(std::move(dependencies))`,
so both concrete classes need a constructor taking the dependency struct **by value**.

## 3. Type changes

**None of the entries below are things ex3 "removed" or "changed" from assignment 2's true baseline.**
The pristine `../ex_2_skeleton` never had `config_load_error` or `fusion_max`, already named the field
`mission_bounds`, and already declared `ErrorRef` after `MissionConfigData`. Our own
`Drone-Mapper-ex2` edited these frozen ("do not change this interface") structs ourselves during ex2.
Ex3's `common/` headers simply match the original ex2 skeleton, not our modified one — so what looks
like an ex2→ex3 removal is actually **our own ex2-era addition that needs to be reverted/relocated**.

| Type | What our `Drone-Mapper-ex2` has | What ex3 (= pristine ex2 skeleton) has |
|------|---------------------------------|------------------------------------------|
| `DroneConfigData` | `std::optional<ErrorRef> config_load_error` (our addition) | no such field — never existed in the skeleton |
| `LidarConfigData` | `std::optional<ErrorRef> config_load_error` (our addition) | no such field — never existed in the skeleton |
| `MissionConfigData` | `config_load_error` (our addition); field named `boundaries` (our rename) | no `config_load_error`; field is `MappingBounds mission_bounds` (original skeleton name) |
| `SimulationConfigData` | `config_load_error` (our addition) | no such field — never existed in the skeleton |
| `MappingStepCommand` | `std::optional<PhysicalLength> fusion_max` (our addition) | no such field — never existed in the skeleton |
| `MissionTypes.h` | `ErrorRef` declared **before** `MissionConfigData` (moved by us, so `config_load_error` could use it) | `ErrorRef` declared **after** `MissionConfigData` — the original skeleton order |

Genuine skeleton-level changes (these *are* real ex2-skeleton → ex3-skeleton differences):

| Type | Change |
|------|--------|
| `VoxelOccupancy` | now `enum class VoxelOccupancy : signed char` |
| `Position3D` | gains `constexpr operator+` and `operator-` |
| `SimulationManagerReport` | gains `std::filesystem::path composition_file` |
| `MissionTypes.h` | includes `MapTypes.h` (not `Units.h`, which is what the pristine ex2 skeleton included) |

`config_load_error` was the biggest porting consequence in our own code: `SimulationRunFactoryImpl`
propagated per-config parse failures through those fields. Since the frozen ex3 structs never had
anywhere to put that (and never will), parse errors have to be carried by our own type in
`UserCommon` (e.g. a `ConfigParseResult<T>`) instead.

Losing `fusion_max` means the drone controller always fuses at full lidar `z_max` unless we compute
that cap some other way (e.g. internally from `lidar_config_` rather than round-tripping it through
`MappingStepCommand`). Our ex2 `MappingAlgorithmImpl` never actually set it
(`tests/components/test_mapping_algorithm.cpp` asserts `cmd.fusion_max.has_value()` is false), so
this should be no behavior change — just delete the field references from `DroneControlImpl`.

## 4. Composition data reverts to the skeleton's original nested shape

This is another case from §3's caveat: it is **not** ex3 restructuring assignment 2's format. The
pristine `../ex_2_skeleton` already has the nested, grouped shape — our own `Drone-Mapper-ex2` is what
flattened it into parallel, index-aligned vectors:

```cpp
// Drone-Mapper-ex2 (our own code) — two parallel, index-aligned vectors
std::vector<SimulationConfigData> simulations;
std::vector<MissionConfigData> missions;
std::vector<DroneConfigData> drones;
std::vector<LidarConfigData> lidars;

// ex_2_skeleton (pristine) AND ex_3_skeleton — grouping is explicit in the type,
// identical shape in both; ex3 only renames drones→drone_configs, lidars→lidar_configs
// and fully qualifies the element types as common::types::
std::vector<std::tuple<SimulationConfigData, std::vector<MissionConfigData>>> simulation_mission_groups;
std::vector<DroneConfigData> drone_configs; // drones (ex2)
std::vector<LidarConfigData> lidar_configs; // lidars (ex2)
```

The nested YAML shape is unchanged; the parser needs to materialize the nesting directly instead of
flattening to aligned pairs, undoing our ex2-era flattening. Run expansion stays the same set of
combinations: **(simulation, mission) pairs × drone_configs × lidar_configs** — 24 runs for
`inputs/sim_compose.yaml`.

`ISimulationRunFactory::create` parameter names changed (`simulation` → `simulation_config`, etc.) and
its mission/drone/lidar parameters are now explicitly `common::types::`-qualified. Signature is otherwise
identical.

## 5. Components the ex3 skeleton no longer provides

Ex2 shipped `MockLidar`, `MockGPS`, `MockMovement`, and `ScanResultToVoxels` as course-published
reference implementations we were told never to reimplement. **None of them are in the ex3 skeleton**,
along with everything else we wrote:

`MockLidar`, `MockGPS`, `MockMovement`, `Map3DImpl`, `ScanResultToVoxels`, `MapsComparison`,
`SimulationManager`, `SimulationRunImpl`, `SimulationRunFactoryImpl`, `MissionControlImpl`,
`DroneControlImpl`, `MappingAlgorithmImpl`, all YAML parsers, all I/O helpers.

All of it is ours to port and place. See `docs/component-placement.md`.

This is a status change, not just a location change: the "hardware simulation" mocks stop being a
frozen, off-limits shared contract and become **our own private `Simulator`-internal implementation**.
`MissionControlDependencies` (§2) hands plugins raw `ILidar&`/`IGPS&`/`IDroneMovement&`/
`IMutableMap3D&` references — whoever builds the Simulator supplies the concrete behavior behind them.
We can carry our ex2 mock implementations forward as a starting point, but we now own and may modify
them, and we're responsible for their correctness against the published interfaces: other teams'
`MissionControl`/`Algorithm` `.so` files will run against *our* Simulator's mocks in
comparative/competitive mode.

## 6. Build system

| | Ex2 | Ex3 |
|---|-----|-----|
| Project | one `drone_mapper` static lib + 3 executables | 4 CMakeLists: root + `Algorithm` + `MissionControl` + `Simulator` |
| `common/` | headers in the same target | `INTERFACE` library `common::common`, links `mp-units::mp-units` |
| Warnings | per-target, MSVC branch | root defines `drone_warnings(target)` → `-Wall -Wextra -Werror -pedantic` |
| Deps | mp-units, TinyNPY, yaml-cpp, gtest | same set (`vcpkg.json`: mp-units ≥2.3.0, yaml-cpp ≥0.9.0, tinynpy, gtest) |
| Presets | `default` → `build/default`, Ninja | same |

`Algorithm/CMakeLists.txt`, `MissionControl/CMakeLists.txt`, and `Simulator/CMakeLists.txt` are TODO
stubs whose comments state the intent: shared libraries with no `lib` prefix linking `common::common`,
and an executable linking `common::common` + `${CMAKE_DL_LIBS}` that **exports the registration
constructor symbols** for the plugins to find.

The ex3 skeleton has no `find_package(yaml-cpp)`, `find_package(TinyNPY)`, `find_package(GTest)` or
`enable_testing()` in the root `CMakeLists.txt` yet — add them when the corresponding code lands.

## 7. Inputs

`ex_3_skeleton/inputs/` is byte-identical to ex2's vendored `tests/data/instructor/` (modulo CRLF and
one blank line in `sim_compose.yaml`), plus three new files:

- `map/npy_to_cw.py`, `map/scenario_small.cw`, `map/scenario_big.cw`, `map/benchmark_map.cw` —
  ClassicWorld (gzipped NBT) exports of the same maps, for viewing in ClassiCube. The simulation YAMLs
  still reference `.npy`, so `.cw` is a visualization aid, not an input format. See
  `docs/open-questions.md`.

No spawn positions or boundaries changed since the ex2 Jul 2026 sync, so all ex2 tuning results carry over.
