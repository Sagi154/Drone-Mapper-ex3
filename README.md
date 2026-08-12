# Assignment 3 - Drone Mapper

This is the core skeleton for assignment 3. You should update this README file.

Use the lowercase project namespaces `common`, `algorithm`, `mission_control`, and `simulator` in your implementation.

## Provided file tree

```text
.
|-- .devcontainer/...
|-- Algorithm/
|   |-- CMakeLists.txt
|   |-- include/Algorithm/
|   `-- src/
|-- MissionControl/
|   |-- CMakeLists.txt
|   |-- common_mission_control/include/MissionControl/IDroneControl.h
|   |-- include/MissionControl/
|   `-- src/
|-- Simulator/
|   |-- CMakeLists.txt
|   |-- common_simulator/include/Simulator/
|   |   |-- ISimulation.h
|   |   |-- ISimulationRun.h
|   |   |-- ISimulationRunFactory.h
|   |   `-- SimulationTypes.h
|   |-- include/Simulator/
|   `-- src/
|-- common/
|   |-- CMakeLists.txt
|   `-- include/Common/
|       |-- types/
|       |   |-- DroneTypes.h
|       |   |-- LidarTypes.h
|       |   |-- MapTypes.h
|       |   `-- MissionTypes.h
|       |-- IDroneMovement.h
|       |-- IGPS.h
|       |-- ILidar.h
|       |-- IMap3D.h
|       |-- IMappingAlgorithm.h
|       |-- IMissionControl.h
|       |-- IMutableMap3D.h
|       |-- MappingAlgorithmFactory.h
|       |-- MappingAlgorithmRegistration.h
|       |-- MissionControlFactory.h
|       |-- MissionControlRegistration.h
|       |-- Types.h
|       `-- Units.h
|-- .gitignore
|-- CMakeLists.txt
|-- CMakePresets.json
|-- README.md
|-- students.txt
|-- vcpkg-configuration.json
`-- vcpkg.json
```

## Output naming

Each run creates a fresh output directory (never reused):

- Comparative mode: `<mission_control_folder>/comparative_results_<UTC time>[_N]`
- Competition mode: `<algorithms_folder>/competition_<UTC time>[_N]`

`<UTC time>` is `currentUtcTimestamp()` (`UserCommon_207190406_209543255/TimeFormat.h`); `_N`
is appended starting at `_2` if a directory with that name already exists (same-second collision).

Inside that directory:

```text
<output_dir>/
  comparative_report.yaml                    # or competitive_report.yaml
  <plugin>_simulation_output.yaml            # per-plugin ex2-style score_report
  <plugin>_run_NNNN_output_map.npy           # per-run output map (NNNN = zero-padded cell index)
  <plugin>_run_NNNN_error.log                # per-run error log (planned; not yet emitted)
```

`NNNN` is the flat cell index from `RunMatrixOrchestrator::expand` (0-based, zero-padded to 4
digits), unique across the whole run matrix and stable for a given composition + plugin order.
`<plugin>` is always the loaded `.so` **filename**, never a path.
