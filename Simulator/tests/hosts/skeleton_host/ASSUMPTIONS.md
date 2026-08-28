# skeleton_host — semantic choices left unspecified by the frozen headers

This host is a **test-only** foreign simulator. It implements the published
`common::` sensor/map/movement interfaces so Assignment 3 plugins can be
`dlopen`ed without the submitting team's Simulator. Choices below are **this
host's** decisions, not claims that student simulators must match them.

## CLI

- Flags are `--key=value` (double dash, no spaces around `=`), any order.
- Required: `--algorithm`, `--mission-control`, `--simulation`, `--mission`.
- Optional: `--drone`, `--lidar`. If omitted, the host uses
  `<inputs_root>/drone/drone_small.yaml` and `<inputs_root>/lidar/lidar_short.yaml`,
  where `inputs_root` is the parent of the directory that contains the simulation
  YAML (staff layout `inputs/simulation/*.yaml`).
- Non-zero process status only for CLI misuse, missing/unreadable files, YAML
  parse failure, npy load failure, `dlopen` failure, or a missing registration
  factory. `runMission()` failures never abort: they become `HOST_STATUS=Error`
  and `main` still returns 0.
- Usage text is printed to stderr. Exact wording is host-local.

## YAML → C++ mapping (staff `inputs/` key shapes)

| YAML | C++ |
|---|---|
| `drone_config.dimensions_cm` | `DroneConfigData.radius = dimensions_cm / 2` (YAML comment: sphere **diameter**) |
| `simulation_config.map_resolution_cm` | `SimulationConfigData.map_resolution` |
| `initial_drone_position.{x_cm,y_cm,height_cm}` | `Position3D` in the **mission/API frame** |
| `initial_angle_deg` | `HorizontalAngle` (altitude starts at 0) |
| `map_axes_offset.{x_offset,y_offset,height_offset}` | `map_offset` |
| `mission_config.boundaries.*_boundary.{min_cm,max_cm}` | `MappingBounds` |
| `gps_resolution_cm` | `MissionConfigData.gps_resolution` (stored; **not** applied as snapping) |
| `output_mapping_resolution_factor` | loaded if present; **default 0.0** (header default). **Not used** to size the output map |
| lidar `z_min_cm`, `z_max_cm`, `d_cm`, `fov_circles` | `LidarConfigData` as-is |

`map_filename` is resolved in order: absolute path; beside the simulation YAML;
`<inputs_root>/<map_filename>` (staff `inputs/map/...`); then cwd.

## Coordinate frames

Heading: 0 = east (+X), 90 = south (+Y), 180 = west, 270 = north (YAML comment).
Altitude 0 = horizon, positive = up. Direction:

`forward = (cos(alt)*cos(horiz), cos(alt)*sin(horiz), sin(alt))`.

**Mission/API frame** (GPS, movement, `IMap3D` queries, mission bounds) uses the
numbers in the YAML (e.g. house spawn height `10`).

**Global/npy frame:** `global = mission + map_offset`. The npy voxel `(0,0,0)`
occupies global `[0, res)³`. That matches house maps whose first ~15 layers are
solid fill under `height_offset: 150`, and the YAML comment `height_cm: 10 # -> 160`.

Hidden-map mission-frame bounds are therefore
`[-offset, npy_extent - offset)` so that `atVoxel(mission_pos)` uses

`index = floor((mission - bounds.min) / resolution) = floor((mission + offset) / res)`.

## HostMap3D

- Npy axis order is **(X, Y, Z/height)**, C-order, last index fastest.
- Voxel dtype: TinyNPY 1-byte payload (**int8 or uint8**). **Non-zero = Occupied**,
  zero = Empty (ClassicWorld block IDs and 0/1 occupancy maps both work).
- `isInBounds` / indexing use a **half-open** cube `[min, max)` per axis.
- `atVoxel` outside that cube, or with a non-indexable position, returns
  `VoxelOccupancy::OutOfBounds`.
- `set` on out-of-bounds is a **no-op** (does not throw).
- `save` writes an int8 `.npy` of the stored `VoxelOccupancy` values.
- Fortran-order npy files are not rearranged; staff maps are C-order.

## Output map sizing

A **second**, empty `HostMap3D` is given to the plugins (`output_map`). It is
**not** a copy of the hidden map.

- `MapConfig.boundaries` = `mission_config.mission_bounds`
- `MapConfig.offset` = `simulation.map_offset` (metadata for consumers)
- `MapConfig.resolution` = `simulation.map_resolution` (not GPS, not
  `output_mapping_resolution_factor`)
- Voxel counts per axis: `n = round((max - min) / resolution)`, at least 1.
  Indexing is relative to mission `min`, so plugins querying GPS positions
  hit the intended cell.
- Every cell starts as `Unmapped`.

## HostGPS

Returns the true simulated center and heading. `gps_resolution_cm` is stored
and unused (Assignment 1: position sensor is exact). No quantization.

## HostLidar

- Beam origin = `gps.position()` (mission frame).
- `scan(scan_orientation)` treats the argument as the **world-frame** direction
  of Circle 0. The host does **not** add GPS heading. MissionControl is expected
  to pass an absolute FOV (e.g. drone heading plus a relative scan).
- Geometry from Assignment 1: Circle `k` has `4^k` beams; polar angle
  `atan(k * d / z_min)` at `z_min`. Circle 0 is the FOV center. `phi = 0` on
  each outer circle is along the FOV “up” axis (increasing altitude).
- Beams that miss within `z_max`, or leave the hidden map, are **omitted**.
- A hit closer than `z_min` is still reported with **distance 0**.
- Hit `LidarHit.angle` is the beam's world orientation (`atan2(y,x)` / altitude).
- **Hit order:** Circle 0, then Circle 1 beams `j = 0 .. 4^k-1`, etc.
- Ray vs voxels: uniform stepping at `clamp(0.25 * resolution, 0.25cm, 1cm)`.
  Distance is the first sample that lands in an Occupied cell (slightly after
  the true face). This is a test-host approximation, not Amanatides–Woo.

## HostMovement (failure style)

**Choice: failed `MovementResult` (`success = false`), not an exception.**

Wall collision and leaving mission bounds / the hidden map all:

1. increment `HOST_ILLEGAL_MOVE_ATTEMPTS`
2. leave pose unchanged
3. return `{false, "blocked: wall or boundary"}`

Rationale: this host must not `std::terminate`; counting illegal attempts
requires the call to return; plugins that ignore `MovementResult` keep running
so voxel counters are still meaningful. (Course FAULT-02 says a staff Mock
Movement **throws** on wall hits. This foreign host deliberately differs.)

`runMission()` is still wrapped in `catch (const std::exception&)` so a plugin
that throws does not kill the process.

Other movement rules:

- **Rotate Left** decreases heading (counter-clockwise when 90° is south);
  **Right** increases it. Heading is wrapped to `[0, 360)`. Rotate never
  collides. Requested angle is **not** clamped to `max_rotate`.
- **Advance** is in the XY plane along current heading (`cos`/`sin` of
  horizontal angle). Negative distance is backward. Altitude is ignored.
  Distance is **not** clamped to `max_advance`.
- **Elevate** changes Z only (positive up). Not clamped to `max_elevate`.
- Mission-bounds test for the **center** uses a **closed** interval `[min, max]`.
- The drone is a sphere of `radius = dimensions_cm / 2`. Strict interior
  overlap with an occupied voxel AABB (`dist² < r²`) is a wall. Grazing
  (`dist == r`) is allowed. A sphere whose center is outside the hidden map,
  or that overlaps no hidden voxel at all, is treated as a boundary failure.
- Paths are sampled from the start (exclusive) to the end (inclusive).

## Registration / `dlopen`

- `HostRegistrar` is a process-wide singleton.
- `MappingAlgorithmRegistration.cpp` / `MissionControlRegistration.cpp` only
  store the factory from the constructor the plugin's `REGISTER_*` macro runs
  during `dlopen`.
- Plugins are opened with `RTLD_NOW | RTLD_LOCAL`. The executable is built with
  `ENABLE_EXPORTS` so those constructors resolve against the host.
- Factories are taken immediately after each `dlopen` (last registration wins
  if a `.so` registers more than once).
- Plugin objects are destroyed, **factory `std::function` objects are cleared**, then `dlclose` (MC first, then algorithm). Destroying a factory after `dlclose` would call into unmapped plugin code. No `new`/`delete` in host sources (`unique_ptr` / containers only). TinyNPY may allocate internally.

## Summary lines

Printed to **stdout**, exactly these keys:

```
HOST_STATUS=Completed|MaxSteps|Error
HOST_STEPS=<MissionRunResult.steps>
HOST_VOXELS_EMPTY=<Empty cells in the output map>
HOST_VOXELS_OCCUPIED=<Occupied cells in the output map>
HOST_VOXELS_UNMAPPED=<Unmapped + PotentiallyOccupied cells>
HOST_ILLEGAL_MOVE_ATTEMPTS=<blocked advance/elevate count>
```

`HOST_STATUS` is `Error` if `runMission()` throws `std::exception`. Voxel
counts are from the **output** map, not the hidden npy. `OutOfBounds` is never
stored in the grid.

## Dependencies not invented as APIs

The host uses only frozen headers under `common/`,
`Simulator/common_simulator/`, plus yaml-cpp, TinyNPY, and POSIX `dlfcn`.
It does **not** implement `ISimulation` / `ISimulationRun` / `IDroneControl`
(MissionControl brings its own drone controller).
