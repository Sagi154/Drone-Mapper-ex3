# Error handling matrix

Transcribed from `context/Common issues and handling.pdf` (course staff, 2026-07-25).

Two scenarios are **mandatory**. The rest are **optional** and may earn a bonus — but note that an
optional row still describes the behavior the graders consider correct, so implementing a few cheap ones
is worthwhile.

Staff definition to keep in mind:

> Gracefully handling issues is not that we don't throw an exception — it is that we adjust our behavior
> to correct the issue or continue despite it. For example, we might separate a command into 2, log the
> occurrence as a warning and try to keep running.

"Targets" is who must catch a thrown exception. `N` (number of retries) is our choice; pick a named
constant, not a magic number.

## Mandatory

| Scenario | Detector | Action | Targets |
|----------|----------|--------|---------|
| All scenarios below, with a **valid** algorithm | Algorithm | A valid algorithm avoids errors it can detect — it does not create the error in the first place | — |
| Algorithm returns a movement that collides the drone with a wall on the **actual** map | `MockMovement` (holds the real map, so it can detect the collision) | Throw an exception | `DroneControl`, `SimulationRun` |

The first row is the important one: our own algorithm must never emit an illegal move. The rest of the
table is about surviving **someone else's** faulty algorithm — which matters in ex3, because in
competitive mode our `MissionControl` runs other teams' algorithms, and in comparative mode our
`Algorithm` runs under other teams' mission controls.

## Optional (bonus)

| Scenario | Detector | Action | Targets |
|----------|----------|--------|---------|
| Faulty algorithm sends a movement that would go out of bounds | `DroneControl` | Gracefully ignore the illegal movement; do not pass it to the movement component | `DroneControl` |
| Faulty algorithm returns a command with invalid values | `DroneControl` | Gracefully try again; throw after N tries | `DroneControl`, `SimulationRun` |
| Faulty algorithm returns an empty movement **and** scan command (NOOP) | `DroneControl` | Gracefully handle the empty request, try again; throw after N failed attempts | `DroneControl`, `SimulationRun` |
| LiDAR returns an empty vector | `DroneControl` | Gracefully try again; throw after N tries | `DroneControl`, `SimulationRun` |
| Movement driver returns `false` for its own reasons | `DroneControl` | Gracefully handle; throw after N tries | `DroneControl`, `SimulationRun` |
| Algorithm returns a movement bigger than the max allowed | `DroneControl` | Split into 2+ commands, each a separate step | `DroneControl` |
| Drone returns `Error` status on a step | `MissionControl` | Log the error, continue | `MissionControl` |
| Algorithm returns a movement that leaves the mission bounds | `DroneControl` | Amend the movement so the drone stays inside mission bounds | `DroneControl` |
| GPS returns out-of-bound coordinates | `DroneControl` | Compare with internal coordinates; throw if those are also OOB, otherwise ignore | `DroneControl`, `SimulationRun` |
| Movement executed but GPS reports impossible coordinates | `DroneControl` | Try again to read GPS; return `Error` after N tries | `DroneControl` |

## How this interacts with the ex3 simulator

The matrix stops at `SimulationRun`. Above it, assignment 3 adds:

- The Simulator **need not** survive a plugin **crash** (segfault, `std::terminate` in a plugin thread),
  but must not crash in any other case — including a plugin that throws, hangs on a bad config, or fails
  to load.
- A plugin that could not be loaded or run goes into the report's `errors: [...]` list by `.so` filename,
  and the remaining plugins still run.
- Exceptions escaping a plugin must be caught at the run boundary inside the worker thread. An uncaught
  exception on a `std::thread` terminates the whole process, which would fail every other run in the
  matrix.

See `.cursor/rules/error-handling-logging.mdc` for the logging format and the score/report consequences.
