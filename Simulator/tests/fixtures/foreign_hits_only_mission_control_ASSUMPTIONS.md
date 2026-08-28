# foreign_hits_only_mission_control — TEST-ONLY MissionControl assumptions

This plugin is a **foreign MissionControl** for Assignment 3 independence /
compatibility testing. It is legal under the frozen `common::` APIs but
deliberately **unlike** a free-space-carving host MissionControl: it never marks
`Empty` along lidar beams, and it downsamples hits.

## Registration

- Class: `ForeignHitsOnlyMissionControl`
- Registered with `REGISTER_MISSION_CONTROL(ForeignHitsOnlyMissionControl)`
- Constructor takes `common::MissionControlDependencies` **by value** (as the
  factory lambda does) and stores that struct. The struct holds references to
  host-owned sensors/map/algorithm; this plugin does **not** own, move, or
  `unique_ptr` those objects. No `new` / `delete`.

## Mission loop

Per counted step, in order:

1. Build `DroneState` from `gps.position()`, `gps.heading()`, and
   `step_index = steps` (0-based count of completed steps so far).
2. Call `mapping_algorithm.nextStep(state, nullptr)` — **always** pass
   `nullptr` for `latest_scan` (never a non-null scan pointer).
3. If `cmd.movement` is set, apply it (see Movement).
4. If `cmd.scan_orientation` is set, perform **at most one**
   `lidar.scan(*cmd.scan_orientation)` and update the output map (see Hits).
5. Increment `steps`.
6. If `cmd.status` is `Finished` or `FinishedWithUnmappableVoxels`, return
   `MissionRunStatus::Completed` **after** applying that command’s movement
   and/or scan (terminal command is still executed).
7. If `steps >= mission_config.max_steps`, return
   `MissionRunStatus::MaxSteps`.
8. Otherwise continue.

If `max_steps == 0`, one step may still run; after it, unless the algorithm
finished, status is `MaxSteps` with `steps == 1`.

`MissionRunResult.errors` is left empty. This plugin does not write verbose
files (even if `dependencies.verbose` is true) and does not call `output_map`
save — the host owns persistence via `output_map_file`.

## Movement

Supported command types:

| `MovementCommandType` | Call |
|---|---|
| `Hover` | no-op |
| `Rotate` | `movement.rotate(rotation, angle)` |
| `Advance` | `movement.advance(distance)` |
| `Elevate` | `movement.elevate(distance)` |

**Failure policy (both documented choices):**

1. **Catch throw → continue:** each movement call is wrapped in
   `try/catch (const std::exception&)`. An exception does **not** abort the
   mission and does **not** set `MissionRunStatus::Error`; the loop continues
   (scan for this step still runs if requested).
2. **Ignore `success == false` → continue:** a `MovementResult` with
   `success == false` is ignored; the mission continues the same way.

Rationale: hosts differ (some return failed results, some throw on illegal
moves). This foreign MC must keep running so independence tests still produce
maps / step counts.

## Hits / Occupied writes (no free-space carving)

- At most **one** `lidar.scan(...)` per mission step; never batch multiple scans
  in one step.
- For each `LidarHit` in the scan result (host order preserved), write
  `Occupied` only when the hit index `i` satisfies `i % N == 0`.
- **N = 4** (every 4th hit, including index 0). Skipped hits are ignored.
- Do **not** mark `Empty` (or any other occupancy) along the beam.

### Hit → world position convention

`LidarHit` has no world-position field. This plugin reconstructs:

- **Origin** = `gps.position()` **after** that step’s optional movement
  (post-move GPS).
- **Direction** from `LidarHit.angle`, treated as **absolute world-frame**
  orientation (degrees):

  `forward = (cos(alt)*cos(horiz), cos(alt)*sin(horiz), sin(alt))`

  with `horiz` / `alt` from `hit.angle.horizontal` / `hit.angle.altitude`.
- **World hit** = `origin + forward * hit.distance` (cm).

Hosts may encode lidar angles differently; this plugin always uses the
convention above. Documented here so VAR-02 expectations are explicit.

## What this is testing

Relative to a typical free-space-carving MissionControl, this plugin:

- never passes scans into `nextStep`
- never carves `Empty`
- only sparsely marks `Occupied` (N=4)

Algorithms that hard-require dense free space or non-null `latest_scan` will
behave differently — that is intentional for independence testing.
