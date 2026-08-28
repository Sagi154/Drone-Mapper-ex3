# var04_baseline_algorithm — baseline MappingAlgorithm assumptions

This plugin is a **legal, terminating** MappingAlgorithm for Assignment 3
independence / comparative tests. It explores with a coarse 3D lawnmower and
uses **only** `IMap3D` read access plus the three config blobs in
`MappingAlgorithmDependencies`. It never writes the map, never owns sensors,
and never requires a non-null `latest_scan`.

Registration constructor (`MappingAlgorithmRegistration`) is **undefined in
this `.so`**; the host that `dlopen`s it must export that symbol (as
`skeleton_host` and the Simulator do).

No `new` / `delete`. The factory lambda uses `std::make_unique`. The
constructor takes frozen `MappingAlgorithmDependencies` **by value** (as the
macro does).

## Registration

| File | Class | `REGISTER_MAPPING_ALGORITHM` |
|---|---|---|
| `baseline_lawnmower_algorithm_plugin.cpp` | `BaselineLawnmowerAlgorithm` | yes |

`nextStep` ignores `latest_scan` (may be `nullptr`). Planning uses
`DroneState` + `output_map` + configs only.

## Coordinate / heading conventions

Match `skeleton_host` / var03 into-occupied:

- Heading `0°` = east (+X), `90°` = south (+Y). Forward XY =
  `(cos(heading.horizontal), sin(heading.horizontal))`.
- Desired yaw to a target = `atan2(dy, dx)` in degrees.
- `Rotate` Right if `wrap180(desired - heading) > 0`, Left if `< 0`.
- `Elevate` distance is **signed** (positive = up). `Advance` distance is
  non-negative.
- `MapConfig.offset` is **not** applied. Voxel queries use the same frame as
  `DroneState.position`.

## Waypoint grid (lawnmower)

Built once on the first `nextStep` from `output_map.getMapConfig()`:

1. Cell counts per axis: `n = floor((max - min) / resolution)` on
   `boundaries` (same formula as var03). If `resolution <= 0` or any `n == 0`,
   the waypoint list is empty → finish immediately (see Termination).
2. Voxel **centers**: `min + (i + 0.5) * resolution`. A center is used only
   if `isInBounds(center)`.
3. **Stride** in cells:
   - XY: `max(1, round(max_advance / resolution))`, then capped so spacing
     is at most `lidar_config.z_max` when `z_max > 0`.
   - Z: `max(1, round(max_elevate / resolution))`, same `z_max` cap.
   The last cell index of each axis is always included (so the far face is
   not dropped when `n-1` is not on the stride).
4. **Radius inset:** drop centers closer than `drone_config.radius` to a
   boundary. If that would drop **all** centers on an axis, keep the
   un-inset list for that axis.
5. Fallback bounds: if map `boundaries` are degenerate (`max <= min` on an
   axis), use `mission_config.mission_bounds` instead. Resolution fallback:
   map `resolution`, else `mission_config.gps_resolution`, else `10 cm`.
6. Order: Z layers bottom → top; within a layer, Y increasing; within a row,
   X snakes (even `iz+iy` → +X, odd → −X).

## Per-step policy

`nextStep` picks **one** movement (never two). Order of priorities toward
the current waypoint:

1. Skip the waypoint if `atVoxel(center)` is `Occupied` or
   `PotentiallyOccupied` (do not fly into known obstacles). Skip `Empty`
   waypoints (already mapped). Visit `Unmapped` (and treat `OutOfBounds` as
   skip).
2. If `|dz|` > arrival epsilon: `Elevate` by `clamp(dz, ±max_elevate)`.
3. Else if XY distance > arrival epsilon:
   - If `|wrap180(desired - heading)| > 1°`: `Rotate` by
     `min(|delta|, max_rotate)` in the shorter direction.
   - Else: `Advance` by `min(xy_distance, max_advance)` (no overshoot).
4. Else: waypoint reached → advance the index and re-evaluate in the **same**
   `nextStep` call (no wasted Hover).

Arrival epsilon = `max(1 cm, 0.25 * resolution)`.

Every `Working` command also sets `scan_orientation` to a **world-frame**
look (host does not add GPS heading; this matches var01 lidar and var02).
Looks cycle with `step_index % 6`:

| i%6 | horizontal | altitude |
|---|---|---|
| 0 | 0° | 0° |
| 1 | 90° | 0° |
| 2 | 180° | 0° |
| 3 | 270° | 0° |
| 4 | 0° | +45° |
| 5 | 0° | −45° |

No extra Hover-only looks at a waypoint; coverage comes from the lawnmower
plus this cycling FOV.

Angles and distances **are** clamped to `max_rotate` / `max_advance` /
`max_elevate`. `max_rotate == 0` or `max_advance == 0` / `max_elevate == 0`
means that axis of motion is skipped (waypoint skipped if the error on a
disabled axis exceeds epsilon).

### Stuck detection

The plugin does not see `MovementResult`. If the previous command was
`Advance` or `Elevate` and GPS position changed by `< 0.5 cm` on all
relevant axes, the current waypoint is skipped (blocked / rejected move).
No local pathfinding around walls.

## Termination

Returns `Finished` or `FinishedWithUnmappableVoxels` (never runs forever).

Finish when **any** of:

1. Waypoint list is empty or fully consumed (skipped or visited).
2. `mission_config.max_steps == 0` (first call is terminal).
3. `state.step_index + 1 >= max_steps` (this call is the last legal step).
   The last step may still include a movement + scan; `status` is terminal
   so a host that stops on Finished records `Completed` rather than
   `MaxSteps`.

On a terminal command:

- `movement` = `Hover` if there is no remaining waypoint work; otherwise
  the planned move from Per-step policy (last-step case).
- After consuming waypoints: `Hover`, `scan_orientation = nullopt`.
- Status: sample every in-bounds voxel center (full resolution, not the
  coarse stride). If any center is `Unmapped` or `PotentiallyOccupied` →
  `FinishedWithUnmappableVoxels`. Else → `Finished`.
  `Empty` / `Occupied` count as mapped. `OutOfBounds` is ignored.

This is why a foreign MC that never carves `Empty` still gets a terminal
status (`FinishedWithUnmappableVoxels`) after the sweep, and why a small
bounded mission (`inputs/mission/small_mission_room.yaml`, `max_steps: 1000`)
completes well inside the cap (a few hundred steps at 30 cm / 20 cm stride).

## What this is not

- Not a student-grade mapper: no frontier BFS, no unknown-space frontier
  clustering, no obstacle circumnavigation.
- Not adversarial: does not throw, does not request implausible angles, does
  not ignore `max_*` clamps.
- Not a MissionControl.

## Build

`CMakeLists.txt` in this directory builds **one SHARED** target
`baseline_lawnmower_algorithm_plugin` (no `lib` prefix), C++20,
`-Wall -Wextra -Werror -pedantic`. Linked only to `common::common`.
Wired from the skeleton root via
`add_subdirectory(blind_deliverables/var04_baseline_algorithm)`.
