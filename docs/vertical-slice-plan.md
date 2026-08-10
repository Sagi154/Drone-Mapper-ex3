# Vertical Slice — Implementation Plan

**Goal:** Build and run the real `simulator_207190406_209543255` binary end-to-end: load both
real plugin `.so`s via `dlopen`, run one complete mission through the full call chain, write the
output map, verify no crash. This proves `ENABLE_EXPORTS`, factory plumbing, and the full
dependency wiring are correct before any more features are added on top.

---

## Current state

Every building block is implemented and unit-tested:

| Component | Status | Where |
|---|---|---|
| `PluginLoader` (dlopen/dlclose) | ✅ done | `Simulator/src/PluginLoader.cpp` |
| `SimulationRunFactoryImpl` (wires mocks + plugins) | ✅ done | `Simulator/src/SimulationRunFactoryImpl.cpp` |
| `SimulationRunImpl` (drives one run) | ✅ done | `Simulator/src/SimulationRunImpl.cpp` |
| `RunMatrixOrchestrator` (matrix expand + dispatch) | ✅ done | `Simulator/src/RunMatrixOrchestrator.cpp` |
| `WorkDistributor` (thread pool) | ✅ done | `Simulator/src/WorkDistributor.cpp` |
| CLI parser (`parseSimulationCliArgs`) | ✅ done | `Simulator/src/io/SimulationCli.cpp` |
| YAML config parsers (all 4 configs + composition) | ✅ done | `Simulator/src/io/` |
| Algorithm plugin | ✅ done | `Algorithm/` → builds `Algorithm_207190406_209543255.so` |
| MissionControl plugin | ✅ done | `MissionControl/` → builds `MissionControl_207190406_209543255.so` |
| `MapsComparison` | ✅ stub (returns -1.0) | `Simulator/src/MapsComparison.cpp` |

**The only missing piece:** `Simulator/src/main.cpp` and its `add_executable` entry in
`Simulator/CMakeLists.txt`. The `TODO` comment at the bottom of that file marks the exact gap.

---

## What is NOT in scope here

- S10: comparative/competitive YAML report writers (separate task, after this)
- S11: output directory naming with timestamp + collision handling (separate task, after this)
- Y11: per-run output file naming coordination (separate task, after this)
- `MapsComparison` real 0–100 scoring (S8 full body — stub is fine for this milestone)

The binary at this stage: runs missions, writes output maps, prints per-run scores (which will be
-1.0 until MapsComparison is filled in), exits cleanly. No final YAML report file yet.

---

## Steps

### Step 1 — Write `Simulator/src/main.cpp`

The entry point wires together everything that already exists. No new logic; just the glue.

**Pseudocode (not final — adjust to match exact API signatures):**

```
parseSimulationCliArgs(argc, argv) → args | print usage + exit(1)

RunErrorLog log;
parseCompositionFile(args.simulation, log) → composition | print errors + exit(1)

PluginLoader loader;

if mode == Comparative:
    loader.loadAlgorithmSo(args.algorithm)
    loader.loadMissionControlsFromDirectory(args.mission_control_folder)
    for each mc in loader.missionControls():
        bindings.push_back({ mc.filename,
            new SimulationRunFactoryImpl(loader.algorithms()[0].factory, mc.factory) })

if mode == Competition:
    loader.loadMissionControlSo(args.mission_control)
    loader.loadAlgorithmsFromDirectory(args.algorithms_folder)
    for each algo in loader.algorithms():
        bindings.push_back({ algo.filename,
            new SimulationRunFactoryImpl(algo.factory, loader.missionControls()[0].factory) })

unsigned threads = args.num_threads.value_or(1);
output_root = resolve output directory (S11 handles final naming — use a temp path for now)

results = RunMatrixOrchestrator::run(bindings, composition, output_root, threads)

for each plugin_result in results:
    print plugin_result.plugin_filename, total runs, any -1.0 scores
```

Key contract points to get right in the actual implementation:
- `SimulationRunFactoryImpl` must be kept alive for the duration of `run()` — store them in a
  `std::vector<std::unique_ptr<SimulationRunFactoryImpl>>` parallel to the bindings vector; the
  `PluginMatrixBinding::factory` pointer is non-owning.
- `PluginLoader` must outlive the factories (it holds the `dlopen` handles). Keep it in `main()`
  scope above the bindings vector.
- `composition.base_path` must be set to the directory of the composition YAML file so relative
  map/mission/drone/lidar paths resolve correctly (check what the YAML parser populates).
- If either `loadAlgorithmSo` or `loadMissionControlSo` returns errors, print them and exit(1)
  before proceeding.

### Step 2 — Wire `Simulator/CMakeLists.txt`

Replace the `TODO` comment at the bottom with:

```cmake
set(SIMULATOR_MAIN_SOURCES
    src/main.cpp
    ${YAML_PARSER_SOURCES}
    ${CMAKE_SOURCE_DIR}/UserCommon/src/TimeFormat.cpp
    ${CMAKE_SOURCE_DIR}/UserCommon/src/RunErrorLog.cpp
    ${CMAKE_SOURCE_DIR}/UserCommon/src/SimulationCoordUtil.cpp
    src/Map3DImpl.cpp
    src/SimulationRunImpl.cpp
    src/SimulationRunFactoryImpl.cpp
    $<TARGET_OBJECTS:simulator_registration>
)

add_executable(simulator_207190406_209543255 ${SIMULATOR_MAIN_SOURCES})
set_target_properties(simulator_207190406_209543255 PROPERTIES ENABLE_EXPORTS ON)

target_include_directories(simulator_207190406_209543255 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/common_simulator/include
    ${CMAKE_SOURCE_DIR}/UserCommon/include
)

target_link_libraries(simulator_207190406_209543255 PRIVATE
    common::common
    simulator::loader        # PluginLoader + dlopen
    simulator::registrar     # PluginRegistrar singleton
    simulator::orchestrator  # RunMatrixOrchestrator + WorkDistributor
    simulator::scoring       # MapsComparison
    simulator::cli           # parseSimulationCliArgs
    ${CMAKE_DL_LIBS}
    TinyNPY::TinyNPY
    yaml-cpp::yaml-cpp
    Threads::Threads
)

drone_warnings(simulator_207190406_209543255)
```

Important: `ENABLE_EXPORTS ON` is required so that the `REGISTER_ALGORITHM` /
`REGISTER_MISSION_CONTROL` constructors (which are undefined symbols in the `.so` files) resolve
against the registrar singleton that lives inside the executable.

### Step 3 — Build

```bash
cmake --preset default
cmake --build --preset default --target simulator_207190406_209543255
```

Fix any link errors before proceeding (most likely causes: missing library in `target_link_libraries`,
or a source file listed twice causing duplicate symbol errors).

### Step 4 — Smoke-test end to end

Run with `inputs/sim_compose.yaml` using the built plugin `.so` paths. The `.so` files are placed
by CMake in the build tree — find them with:

```bash
find build -name "Algorithm_207190406_209543255.so" -o -name "MissionControl_207190406_209543255.so"
```

Typical invocations (adjust paths to your build directory):

**Comparative** (one algo, folder of MCs — for now point at the dir containing one MC .so):
```bash
./build/simulator_207190406_209543255 \
    -comparative \
    simulation=inputs/sim_compose.yaml \
    algorithm=build/Algorithm/Algorithm_207190406_209543255.so \
    mission_control_folder=build/MissionControl/
```

**Competition** (one MC, folder of algos):
```bash
./build/simulator_207190406_209543255 \
    -competition \
    simulation=inputs/sim_compose.yaml \
    mission_control=build/MissionControl/MissionControl_207190406_209543255.so \
    algorithms_folder=build/Algorithm/
```

**Pass criteria (vertical slice done):**
- Binary launches without segfault or abort
- Plugins load without errors (no `dlopen` failed / factory not registered messages)
- At least one `SimulationResult` is produced with `mission_score != NaN` (even -1.0 is fine)
- At least one output `.npy` map file is written to the output directory
- Binary exits with code 0

---

## After the vertical slice passes

These tasks become unblocked in order:

1. **S8 full body** — fill in `MapsComparison::compare()` with the real BFS scoring logic so
   `mission_score` is a meaningful 0–100 value.
2. **S10** — write the comparative/competitive YAML report files from the `PluginMatrixResult` table.
3. **S11 + Y11** — finalize output directory naming with timestamp collision handling and coordinate
   the per-run file naming pattern.
4. **Full integration tests** per `docs/workplan.md §Full integration & whole-system verification`.
