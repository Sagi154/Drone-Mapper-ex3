# Ex3 Work Plan — Sagi & Yoav

Concrete work items only — what to build, port, wire up, or verify. Grounded in `AGENTS.md`,
`docs/assignment3-checklist.md`, `docs/component-placement.md`, `docs/api-delta-ex2-to-ex3.md`,
`.cursor/rules/`, and the skeleton contents (`common/`, `Simulator/common_simulator/`,
`MissionControl/common_mission_control/` headers). Student IDs used below (`207190406`,
`209543255`) match every existing doc's naming examples — `students.txt` is filled in
(`Sagi Eisenberg, 207190406` / `Yoav Naaman, 209543255`).

No schedule, no estimates. Items are ordered by what must compile/exist before what, split between
two people along the boundary that blocks least: **the frozen headers under `common/`,
`Simulator/common_simulator/`, and `MissionControl/common_mission_control/` are a fixed contract —
both people code against those directly and independently, never against each other's concrete
classes, until the explicit meeting points called out below.**

## Joint setup completed (before track split)

Done on **2026-08-05** against the skeleton on `main` (pre-split baseline):

- **`students.txt`**: filled with real names + IDs (see above).
- **Baseline build verified**: `cmake --preset default` configures clean; `cmake --build --preset default`
  succeeds as a no-op (`ninja: no work to do`) with the three TODO-stub project `CMakeLists.txt`
  files still empty of targets. Known-good anchor for "it suddenly broke".
- **`Simulator/` subfolder layout settled and materialised** (see §Simulator layout below):
  `Simulator/src/io/`, `Simulator/tests/`, and `Simulator/tests/fixtures/` now exist with `.gitkeep`
  files — do not invent alternate subfolders.

### WSL local-build notes (resolved)

First configure attempt on this WSL host failed until the local toolchain was present. None of this
changed project sources; record it so the other person does not re-debug the same gaps:

| Gap | Resolution used here |
|---|---|
| No Linux `cmake` / `ninja` on PATH | `pip3 install --user cmake ninja` → `~/.local/bin` |
| No `pkg-config` (vcpkg gtest port needs it) | extract Ubuntu `pkg-config` / `libpkgconf3` debs into `~/.local` (no sudo) |
| `VCPKG_ROOT` unset | clone + bootstrap `~/vcpkg`; export `VCPKG_ROOT=$HOME/vcpkg` before configure |
| Preferred long-term | use the course `.devcontainer` (`VCPKG_ROOT=/usr/local/vcpkg`) when Docker/WSL integration is available |

### Simulator layout (authoritative)

Both people write into `Simulator/` independently. Use these directories and the expected filenames
unless there is a strong reason not to — if a name changes, update this section in the same commit.

```
Simulator/
  src/                          # core logic (not disk I/O)
    MockGPS.cpp, MockLidar.cpp, MockMovement.cpp     # Y2
    Map3DImpl.cpp                                    # Y3
    SimulationRunFactoryImpl.cpp                     # Y9
    SimulationRunImpl.cpp                            # Y10
    PluginRegistrar.cpp                              # S5
    MappingAlgorithmRegistration.cpp                 # S5
    MissionControlRegistration.cpp                   # S5
    PluginLoader.cpp                                 # S6
    WorkDistributor.cpp                              # S7
    MapsComparison.cpp                               # S8
    RunMatrixOrchestrator.cpp                        # S9
    io/                         # parsers, writers, CLI, output dirs
      SimulationCli.cpp                              # S4
      DroneConfigYamlParser.cpp                      # Y8
      LidarConfigYamlParser.cpp                      # Y8
      MissionConfigYamlParser.cpp                    # Y8
      SimulationConfigYamlParser.cpp                 # Y8
      CompositionYamlParser.cpp                      # Y8
      ComparativeReportWriter.cpp                    # S10
      CompetitiveReportWriter.cpp                    # S10
      SimulationOutputYamlWriter.cpp                 # S10
      OutputDirHelper.cpp                            # S11
  tests/                        # Simulator unit / integration tests
    fixtures/                   # test-only .so stubs for the loader (S6); never shipped
    manual/                     # Docker whole-system verification scripts (post feature-complete)
  include/Simulator/            # existing
  common_simulator/             # frozen — never touch
  CMakeLists.txt
```

**Rule:** `src/` = core logic; `src/io/` = anything that reads from or writes to disk (YAML, reports,
CLI, output-directory helpers). `SimulationCli` belongs in `io/` (same as ex2's `src/io/`).

## Ownership at a glance

| Area | Owner | Depends on frozen headers only? |
|---|---|---|
| `Algorithm/` (port `MappingAlgorithmImpl`, `MappingAlgorithmFrontier`) | **Sagi** | Yes — `common/` |
| `Simulator/` runtime shell: CLI parsing, plugin registrar + `dlopen`/`dlclose` loader, threading/work distribution, scoring (`MapsComparison`), run-matrix orchestration, comparative/competitive report writers, output-directory naming | **Sagi** | Yes, plus `simulator::` types from `Simulator/common_simulator/` |
| `MissionControl/` (port `MissionControlImpl`, `DroneControlImpl`, `ScanResultToVoxels`) | **Yoav** | Yes — `common/` + `MissionControl/common_mission_control/IDroneControl.h` |
| `Simulator/` world & data: mocks (`MockGPS`/`MockLidar`/`MockMovement`), `Map3DImpl`, YAML config/composition parsers, `SimulationRunFactoryImpl`, `SimulationRunImpl` | **Yoav** | Yes, plus `simulator::` types |
| `UserCommon/` — creation, convention, first shared types | **Yoav** creates the folder + convention; both add to it per the convention below | — |
| `common/` | Neither — read-only, verify only | — |

Both projects' plugin classes need the same naming discipline: per the assignment doc's literal
macro example (`REGISTER_MAPPING_ALGORITHM(MappingAlgorithmImpl_207190406_209543255)`), the
**concrete class itself carries the ID suffix**, declared inside `namespace
Algorithm_207190406_209543255 { ... }` / `namespace MissionControl_207190406_209543255 { ... }`,
and the macro is invoked at **global scope** with the fully-qualified name, e.g.:

```cpp
REGISTER_MAPPING_ALGORITHM(Algorithm_207190406_209543255::MappingAlgorithmImpl_207190406_209543255)
```

(This is exactly the ambiguity in `docs/open-questions.md` #1 — see the open-questions section at
the end for what changes if the answer differs.)

---

## `UserCommon/` — creation, convention, and the first shared types

`UserCommon/` does not exist yet and has no build file of its own; each consuming project's
`CMakeLists.txt` adds the `UserCommon/include` directory and compiles the `.cpp` files it needs.
Because the same translation unit can end up compiled into the executable **and** into one or two
`.so`s, this is the one place both of us write into — so the convention matters more than the code.

### Convention (apply to every future addition, not just the items below)

1. **Default to *not* using `UserCommon/`.** Write new shared-looking logic inside the one project
   that needs it first (`Algorithm/`, `MissionControl/`, or `Simulator/`).
2. **Promote to `UserCommon/` only when a second project genuinely needs the same logic** — not
   speculatively. The person who hits the second need does the move: `git mv` the file(s) into
   `UserCommon/`, update the *original* owner's `CMakeLists.txt` include path, and add their own
   project's `CMakeLists.txt` wiring in the same commit. This is a relocation, not a behavior
   change — don't refactor while moving.
3. **Once a file lives in `UserCommon/`, treat it like a mini frozen interface for whoever isn't
   touching it right now.** Before changing a shared `UserCommon/` header's shape (not just its
   body), check both consumers' source lists (`grep -rl` the two/three `CMakeLists.txt` files) and
   flag the change to the other person — a silent signature change recompiles into every consumer
   silently and can desync in ways `git blame` won't make obvious, since there's no single owning
   build target to catch it.
4. **Keep every `UserCommon/` file small, dependency-light, and free of mutable global/static
   state** (already required by `.cursor/rules/plugin-architecture.mdc`) — a duplicate copy
   compiled into two binaries must not be able to disagree with itself.
5. Everything here lives in `namespace UserCommon_207190406_209543255`.

### Known early items (two consumers are already certain — build these directly in `UserCommon/`, skip step 1 of the convention for just these)

**Item U1 ✅ DONE — `UserCommon/README.md` + the include-dir wiring pattern.**
Owner: **Yoav**. Depends on: nothing.
Create `UserCommon/` with a short `README.md` stating the convention above, and demonstrate the
CMake wiring once (e.g. in the first real consumer below) so the pattern is copy-pasteable:

```cmake
target_include_directories(<target> PRIVATE ${CMAKE_SOURCE_DIR}/UserCommon/include)
target_sources(<target> PRIVATE ${CMAKE_SOURCE_DIR}/UserCommon/src/TimeFormat.cpp ...)
```

Verify: the directory exists, has no `CMakeLists.txt` of its own, and the README is legible to
Sagi without needing to ask.

**Item U2 ✅ DONE — `TimeFormat` + `IRunErrorLog`/`RunErrorLog`.**
Owner: **Yoav**. Depends on: U1. Port from `../Drone-Mapper-ex2/include/drone_mapper/io/{TimeFormat,IRunErrorLog,RunErrorLog}.h`
and `src/io/{TimeFormat,RunErrorLog}.cpp`. ISO-8601 UTC formatting; the log line format is
`<ISO-8601 UTC> <ERROR_CODE> <message>`, flushed immediately, never at shutdown. This is needed by
**three** eventual consumers: `MissionControl` (mandatory/optional error-log rows), `Simulator`
(per-run error logs — `SimulationRunImpl`, item Y8 below), and optionally `Algorithm`. Build it
against that need now rather than waiting to discover it organically, since the requirement is
already explicit in `.cursor/rules/error-handling-logging.mdc`.
Verify: unit-test the timestamp format against a fixed `std::chrono::system_clock::time_point`;
unit-test that a line written via `RunErrorLog` is readable from disk immediately after the call
returns, before any explicit close/flush from the test.

**Item U3 ✅ DONE — `ConfigParseResult<T>`.**
Owner: **Yoav**. Depends on: nothing. Port the shape from
`../Drone-Mapper-ex2/include/drone_mapper/io/ConfigParseResult.h`
(`{ bool ok; T value; std::vector<common::types::ErrorRef> errors; }`), namespaced under
`UserCommon_207190406_209543255`. This is the direct replacement for the `config_load_error`
fields ex3's frozen `common::types` structs never had (see `docs/api-delta-ex2-to-ex3.md` §3) —
Simulator's YAML config parsers (item Y11) are the concrete first consumer and need this before
they can report "parsed, but with recoverable issues" without an exception or a bolted-on field.
Verify: compiles standalone; a throwaway parser stub returning `ConfigParseResult<int>` builds
clean under `-Wall -Wextra -Werror -pedantic`.

**Item U4 ✅ DONE — `SimulationCoordUtil`.**
Owner: **Yoav**. Depends on: nothing. Port `worldInitialDronePosition` and `isDroneSpawnPassable`
from `../Drone-Mapper-ex2/include/drone_mapper/SimulationCoordUtil.{h,cpp}`. First consumers:
`Simulator`'s run-factory (world spawn position, offset-shifted mission bounds — item Y12) and
`MissionControl`'s `DroneControlImpl` if it ends up doing its own bounds math (item Y8). Per the
convention above, do **not** wire this into `Algorithm/CMakeLists.txt` speculatively — the
algorithm reads the map only through `IMap3D::atVoxel(Position3D)` (world coordinates), and
`docs/mp-units-strong-types.mdc` explicitly wants voxel/offset math kept in one place, not
scattered into the algorithm. Only add Algorithm as a third consumer if a real need shows up while
porting `MappingAlgorithmFrontier`.
Verify: port ex2's coordinate-math unit tests; confirm the house scenario's known
`map_axes_offset.height_offset: 150` case still round-trips (world z 160 for local spawn z 10).

---

## Sagi's track: `Algorithm/` + Simulator runtime shell

Ordered by dependency. Every item here needs only `common/` (already frozen and complete) plus
whichever `UserCommon/` item is noted — nothing from Yoav's track.

**Progress (2026-08-13):** S1–S11 **done**. Real BFS scoring, report writers, and collision-checked
output dirs are wired into `main()`. Branch: `maps-scoring-reports-output-naming`.

**S1 — `Algorithm/CMakeLists.txt`.** ✅ **DONE**
Depends on: nothing. `SHARED` target, output name `Algorithm_207190406_209543255`, `PREFIX ""`,
links `common::common`, calls `drone_warnings()`. Start with an empty source list; add files as S2
lands.
Verify: `cmake --build --preset default` still succeeds tree-wide (empty target is a valid no-op).

**S2 — Port `MappingAlgorithmImpl` + `MappingAlgorithmFrontier`.** ✅ **DONE**
Depends on: S1. Source: `../Drone-Mapper-ex2/src/{MappingAlgorithmImpl.cpp,MappingAlgorithmFrontier.{h,cpp}}`
into `Algorithm/src/`. Apply the mechanical changes from `docs/api-delta-ex2-to-ex3.md` §1–§3:
`<Common/...>` includes, `explicit MappingAlgorithmImpl_207190406_209543255(common::MappingAlgorithmDependencies)`
constructor (by value), delete all `fusion_max` references (ex2's own algorithm never set it — no
behavior change), use `mission_bounds` (not `boundaries`). Wrap the class in
`namespace Algorithm_207190406_209543255`. Add
`REGISTER_MAPPING_ALGORITHM(Algorithm_207190406_209543255::MappingAlgorithmImpl_207190406_209543255)`
at **global scope** in `MappingAlgorithmImpl.cpp`.
Verify: `cmake --build --preset default --target Algorithm_207190406_209543255` clean under
`-Wall -Wextra -Werror -pedantic`.
`nm -DC --undefined-only build/default/Algorithm/Algorithm_207190406_209543255.so | grep Registration`
must show the registration constructor as **undefined** — if it's defined, the registration `.cpp`
leaked into the plugin target, which breaks the `dlopen` handshake later.

**S3 — Algorithm tests.** ✅ **DONE**
Depends on: S2. Port `test_mapping_algorithm.cpp` and `test_mapping_algorithm_frontier.cpp` into
`Algorithm/tests/`, with a small hand-written `IMap3D` fake local to these tests (no dependency on
Yoav's `Map3DImpl`). Add `find_package(GTest CONFIG REQUIRED)` + `enable_testing()` to the root
`CMakeLists.txt` — first test target in the tree, so this is the natural place to turn it on.
Verify: `ctest --preset default` (or the gtest binary directly) green. Confirm the port compiles
without any `fusion_max` reference and the ex2 assertion that used to check
`cmd.fusion_max.has_value() == false` is simply deleted, not "still passing" (the field is gone).

**S4 — Simulator CLI argument parser.** ✅ **DONE**
Depends on: nothing (pure `std::filesystem` + string parsing). Rewrite
`../Drone-Mapper-ex2/src/io/SimulationCli.cpp` for the two ex3 modes (`-comparative`,
`-competition` — note the second is *not* `-competitive`). Any argument order; `=` with no
surrounding spaces; collect **all** missing and **all** unsupported arguments before reporting
(don't stop at the first); validate a file argument is openable and a folder argument is
traversable and contains at least one file of the expected kind; print usage + a specific error;
return from `main` gracefully — never `exit()`. Parse and validate before touching the filesystem
for anything else (no output directory, no `dlopen`).
Verify: standalone unit tests (no dependency on the loader, threading, or plugins) covering:
scrambled argument order; two missing arguments reported together; two unsupported arguments
reported together; a nonexistent file path; a folder with zero `.so` files; an unknown mode flag.
All pass without the test process ever seeing `exit()` called (assert via a return-value-based
parse result type, not a process-exit side effect).

**S5 — Plugin registrar singleton + registration constructor bodies.** ✅ **DONE**
Depends on: nothing beyond `common/MappingAlgorithmRegistration.h` / `MissionControlRegistration.h`
(already exist, frozen). In `Simulator/src/`: a singleton holding the most recently registered
`MappingAlgorithmFactory`/`MissionControlFactory`, plus the two registration-constructor `.cpp`
bodies (`MappingAlgorithmRegistration::MappingAlgorithmRegistration(factory)` and the mission
control equivalent) that forward into it. These `.cpp` files must never be linked into a plugin
target (checked in S6/S9's build wiring).
Verify: a unit test constructs a `common::MappingAlgorithmRegistration` directly with a dummy
factory (no `.so`, no `dlopen` involved) and confirms the registrar's "take pending factory"
accessor returns it exactly once, and returns empty on a second call without an intervening
registration.

**S6 — `dlopen`/`dlclose` loader.** ✅ **DONE**
Depends on: S5. Load one `.so` by path, or every `.so` in a folder; `dlopen(path, RTLD_NOW | RTLD_LOCAL)`;
clear the registrar's pending slot before each `dlopen`, take it after; attribute the resulting
factory to the `.so` filename; on any failure (`dlopen` returns null, or nothing was registered),
record the filename for the caller (destined for the report's `errors: [...]`) and continue — never
throw out of the loader for a single bad plugin. Load everything up front on the main thread,
before any worker thread starts (removes `dlopen` entirely from the concurrent path).
Verify: build one or two throwaway test-only `.so` fixtures under `Simulator/tests/fixtures/` (a
trivial class with nothing but `REGISTER_MAPPING_ALGORITHM`) purely to exercise the loader without
waiting on the real `Algorithm`/`MissionControl` projects. Confirm: a valid fixture loads and its
factory is retrievable; a `.so` with no registration call lands in the "failed" list, not a crash;
`dlclose`ing and never `dlopen`-ing the same path again is enforced by the loader's own bookkeeping
(e.g. an assertion or early-return if asked to reload a cached path).

**S7 — Threading / work distribution.** ✅ **DONE**
Depends on: S4 (`num_threads` value). `std::vector<std::thread>` + `std::atomic<std::size_t>` index
into a result table pre-sized to `plugins_under_test × (simulation, mission) pairs × drone_configs
× lidar_configs`; cap worker count at the matrix size; each worker wraps its per-cell call in
`try { ... } catch (...) { record -1 + log; }` so one throwing cell never takes down the others or
calls `std::terminate`.
Verify: a synthetic test with a fake per-cell callable (increments an atomic counter, one variant
throws) run over a small synthetic matrix with `num_threads` absent / `=1` / `=2` / `=8`. Confirm:
worker count is exactly `min(requested, matrix_size - 1)` main-thread-inclusive semantics per
`.cursor/rules/threading-model.mdc` (never open a thread with nothing to do); every cell's slot is
written exactly once; the throwing cell's slot shows the failure without stopping sibling cells.

**S8 — `MapsComparison` (scoring).** ✅ **DONE**
Depends on: U4 (`SimulationCoordUtil`). Ported
`../Drone-Mapper-ex2/{include/drone_mapper/MapsComparison.h,src/MapsComparison.cpp}` into
`Simulator/src/` behind the frozen single-target signature
`simulator::MapsComparison::compare(origin, target, spawn) -> double`. 0–100 per-run score;
reachability filter seeded at world spawn (`worldInitialDronePosition`). Wired into
`SimulationRunImpl` for `Completed`/`MaxSteps`; `Error` stays `-1`. `simulator::scoring` linked
into the executable and `test_simulation_run_factory`.
Verify: `simulator_scoring_test` (fake `IMap3D`) green — identical/empty/mismatch/sealed-room +
spawn-aware cases.

**S9 — Run-matrix orchestrator.** ✅ **DONE** (wired into `main()` as of the end-to-end executable)
Depends on: S6 (loaded plugins), S7 (threading), the frozen `simulator::ISimulationRunFactory` /
`simulator::ISimulationRun` interfaces (already exist — **not** Yoav's concrete classes yet). Given
loaded plugins and a `simulator::types::SimulationCompositionData`, expand the run matrix and
dispatch each cell through `ISimulationRunFactory::create(...)` → `ISimulationRun::run()`,
collecting `simulator::types::SimulationResult`s into the pre-allocated table.
Per-cell output path (S11): `{output_root}/{plugin_filename}_run_<NNNN>_output_map.npy`.
Verify: unit test against a hand-written fake `ISimulationRunFactory`/`ISimulationRun` (returns
canned `SimulationResult`s) and a **hand-built** `SimulationCompositionData` literal — the frozen
type lets this be written and tested without waiting on Yoav's YAML parser at all. Confirm the
matrix size matches `groups × drone_configs × lidar_configs` for the hand-built literal.

**S10 — Comparative/competitive report writers + per-plugin ex2-style aggregate writer.** ✅ **DONE**
Depends on: S9's output shape. `ComparativeReportWriter` / `CompetitiveReportWriter` /
`SimulationOutputYamlWriter` under `Simulator/src/io/`, schemas per
`.cursor/rules/simulator-cli-and-outputs.mdc`. Grouping by `(total_score, total_steps)` sums;
competitive sort by score desc then steps asc. Called from `main.cpp` after the orchestrator;
failed load basenames collected into `errors: [...]`.
Verify: `simulator_report_writers_test` covers tie grouping, competitive sort, and per-plugin
`score_report` schema (incl. `composition_file`).

**S11 — Output directory + naming.** ✅ **DONE**
Depends on: S10, U2 (`TimeFormat`). `OutputDirHelper` creates
`<mission_control_folder>/comparative_results_<time>` /
`<algorithms_folder>/competition_<time>` with `_N` collision suffix. Per-run maps:
`<plugin>_run_NNNN_output_map.npy`. Documented in `README.md` (unblocks Y11).
Verify: `simulator_output_dir_test` creates two dirs in the same second without colliding;
end-to-end smoke writes under the named folders for both CLI modes.

---

## Yoav's track: `MissionControl/` + Simulator world & data

**Y1 ✅ DONE — `MissionControl/CMakeLists.txt`.**
Depends on: nothing. `SHARED` target, output name `MissionControl_207190406_209543255`, `PREFIX ""`,
links `common::common`, exposes `common_mission_control/include` **PRIVATE**.
Verify: `cmake --build --preset default` still succeeds tree-wide with an empty source list.

**Y2 ✅ DONE — Port `MockGPS`, `MockLidar`, `MockMovement`.**
Depends on: nothing (implement directly against frozen `IGPS`/`ILidar`/`IDroneMovement`; the hidden
map access can start against a minimal in-memory stand-in until Y3 lands, then be pointed at the
real `Map3DImpl`). Source: `../Drone-Mapper-ex2/{include/drone_mapper,src}/Mock{GPS,Lidar,Movement}.*`
into `Simulator/src/`. These are no longer course-frozen — carry the ex2 logic forward but we now
own and may fix bugs in it. `MockMovement` **throws** on a real-map wall collision — this is one of
the two mandatory scenarios in `docs/error-handling-matrix.md`.
Verify: port the collision-detection test (from `test_mock_lidar.cpp`'s ex2 equivalent / a new
focused test); confirm `MockMovement` throws exactly on the documented collision case and not on a
legal move adjacent to a wall.

**Y3 ✅ DONE — Port `Map3DImpl`.**
Depends on: nothing structurally; needs the TinyNPY vcpkg dependency wired. Source:
`../Drone-Mapper-ex2/{include/drone_mapper/Map3DImpl.h,src/Map3DImpl.cpp}` into `Simulator/src/`.
Implements both `IMap3D` and `IMutableMap3D`; dtype dispatch per `docs/map3d-contract.md` (hidden
maps are `int8` **or** `uint8`, any value `>= 1` is `Occupied`; output maps are always `int8`
storing the full `VoxelOccupancy` enum); role (hidden vs. output) decided at construction, not
inferred from dtype. Add `find_package(TinyNPY CONFIG REQUIRED)` to the root `CMakeLists.txt` (not
there yet).
Verify: port `test_map3d_impl.cpp`, keeping the three ex2 dtype-regression cases explicitly: `int8`
hidden map value `> 1` → `Occupied`; `uint8` hidden map values `2`/`3`/`4`/`18`/`45` → `Occupied`;
`int8` output map `-1` stays `Unmapped` (no unsigned-path clamping).

**Y4 ✅ DONE — Port `ScanResultToVoxels`.**
Depends on: nothing. Stays in `MissionControl/src/` per `docs/component-placement.md` — only
`DroneControlImpl` calls it, so it does **not** go into `UserCommon/` (no second project needs it
yet).
Verify: port whichever ex2 unit coverage exists for it (folded into Y5's `DroneControlImpl` test if
ex2 never isolated it).

**Y5 ✅ DONE — Port `DroneControlImpl`.**
Depends on: U4 (`SimulationCoordUtil`, if bounds math is needed), Y4. Implements the frozen
`mission_control::IDroneControl`. Step order (frozen contract, `frozen-interfaces.mdc`): read GPS
state → `algorithm.nextStep(state, latest_scan)` → validate movement → execute movement → scan →
`ScanResultToVoxels` → `output_map.set(...)` → return `DroneStepResult`. `IDroneControl.h` does
`using namespace common;` inside `namespace mission_control` — don't rely on that leaking into
files that don't include it directly. For the mandatory collision scenario: `DroneControl` must
**not** swallow the exception `MockMovement` throws — let it propagate out of `step()` (the actual
catch happens at the Simulator run boundary, item Y8/S7, not here).
Verify: port `test_drone_control.cpp` with hand-written fakes for `ILidar`/`IGPS`/`IDroneMovement`/
`IMutableMap3D`/`IMappingAlgorithm` — no dependency on Sagi's real `MappingAlgorithmImpl` or Yoav's
own `MockMovement`. Confirm a fake `IDroneMovement` that throws on `advance()` produces an
exception that is observable escaping `DroneControl::step()` in the test, not swallowed.

**Y6 ✅ DONE — Port `MissionControlImpl`.**
Depends on: Y5. Constructed from `common::MissionControlDependencies` (by value) — builds its own
`DroneControlImpl` internally from the raw `ILidar&`/`IGPS&`/`IDroneMovement&` references (the
ownership inversion from ex2 — see `docs/api-delta-ex2-to-ex3.md` §2). Drives the step loop until
`MissionRunStatus::{Completed, MaxSteps, Error}`. Writes a verbose output file **iff**
`dependencies.verbose` — format and content are our choice; write nothing when the flag is off.
Add `REGISTER_MISSION_CONTROL(MissionControl_207190406_209543255::MissionControlImpl_207190406_209543255)`
at global scope in its `.cpp`.
Verify: `cmake --build --target MissionControl_207190406_209543255` clean.
`nm -DC --undefined-only build/default/MissionControl/MissionControl_207190406_209543255.so | grep Registration`
shows the constructor undefined (same check as S2). Port `test_mission_control.cpp`; confirm a run
with `verbose = false` writes no extra file and one with `verbose = true` does.

**Y7 ✅ DONE — MissionControl tests wiring.**
Depends on: Y5, Y6. Add `MissionControl/CMakeLists.txt` test target reusing the root
`GTest`/`enable_testing()` that Sagi's S3 already turned on.
Verify: `ctest --preset default` picks up and passes the MissionControl suite alongside Algorithm's.

**Y8 ✅ DONE — YAML config + composition parsers.**
Depends on: U3 (`ConfigParseResult<T>`). Port
`../Drone-Mapper-ex2/src/io/{DroneConfigYamlParser,LidarConfigYamlParser,MissionConfigYamlParser,SimulationConfigYamlParser,CompositionYamlParser}.cpp`
into `Simulator/src/io/`, rebuilding the **nested** `simulation_mission_groups` shape (a
`std::tuple<SimulationConfigData, std::vector<MissionConfigData>>` per group) instead of ex2's
flattened parallel vectors — this is a revert to the pristine skeleton shape, not new logic (see
`docs/api-delta-ex2-to-ex3.md` §4). YAML key stays `boundaries`, populates `mission_bounds`. Each
parser returns `ConfigParseResult<T>` for recoverable per-field issues; a bad `.npy`/missing
resource still needs an immediate log entry, not a deferred one. Add
`find_package(yaml-cpp CONFIG REQUIRED)` to the root `CMakeLists.txt` (not there yet).
Verify: port `test_yaml_config_parsers.cpp`; parse `inputs/sim_compose.yaml` end to end and confirm
exactly 6 (simulation, mission) pairs × 2 `drone_configs` × 2 `lidar_configs` = **24** run cells
(per `docs/map3d-contract.md`).

**Y9 ✅ DONE — `SimulationRunFactoryImpl`.**
Depends on: Y2, Y3, U4. Implements `simulator::ISimulationRunFactory::create(simulation_config,
mission_config, drone_config, lidar_config, output_path)`. Builds the per-run mocks + hidden/output
`Map3DImpl` pair; applies `simulation.map_axes_offset` to the mission's boundaries when constructing
the output map's `MapConfig` (the ex2 fix that must not regress — without it the house scenario
writes scans outside the output map); invokes the two plugin factories with correctly-populated
`MappingAlgorithmDependencies` / `MissionControlDependencies`. **Commit the constructor signature
early** (see meeting points below) — it needs to accept a `common::MappingAlgorithmFactory` and a
`common::MissionControlFactory` (both frozen `std::function` types from `common/`), independent of
whether Sagi's loader exists yet.
Verify: unit test with hand-written lambda factories matching the exact frozen signatures — no
dependency on Sagi's real registrar/loader. Confirm the returned `ISimulationRun` is non-null and
the output map's `MapConfig` reflects the offset-shifted bounds for a house-scenario-shaped input.

**Y10 ✅ DONE — `SimulationRunImpl`.**
Depends on: Y9, S8 (`MapsComparison`). Implements
`simulator::ISimulationRun::run()`: call `missionControl->runMission()`, `save()` the output map
(skipped when `output_path` is empty — OQ-Y1), catch any exception escaping `runMission()` **here**
— this is the actual catch target for the mandatory `MockMovement` collision scenario (`DroneControl`
in Y5 lets it propagate; this is where it stops), converting it into a `-1`-scored `SimulationResult`
rather than propagating into the worker thread. Populate `resolution_request_status`
(`ACCEPTED`/`IGNORED`/`IGNORED TOO SMALL`) per run. Scoring via `MapsComparison::compare()` with
`worldInitialDronePosition` for `Completed`/`MaxSteps`.
Note: `SimulationResult::mission_results` is declared as a `std::vector` in the frozen
`SimulationTypes.h` even though `create()` takes a single `mission_config` — the straightforward
implementation populates it with the single `runMission()` outcome for that run.
Verify: unit test with a fake `IMissionControl` that throws mid-`runMission()`; confirm
`SimulationRunImpl::run()` does not propagate the exception further and instead returns a
`-1`-equivalent result with a logged error. End-to-end smoke against real plugins confirmed
exception containment + map save on 2026-08-12.

**Y11 ✅ DONE — Per-run output naming (coordinate with S11).**
Depends on: Y10, S11's chosen pattern (**documented in `README.md` as of 2026-08-13**). Maps use
`<plugin>_run_NNNN_output_map.npy`; error logs use `<plugin>_run_NNNN_error.log` derived from the
map path. `SimulationRunImpl` opens `RunErrorLog` whenever `output_path` is non-empty and mirrors
every `ErrorRef` immediately; the per-plugin YAML `error_log_file` field uses the same derivation.
Verify: for a full composition, every emitted filename is unique across the whole run matrix and
visibly traceable to (plugin, simulation, mission, drone, lidar) or run index by inspection.

---

## Where the two tracks must meet

These are the only points with a real wait — everywhere else, the frozen headers already let both
people compile and test independently.

| Meeting point | What Sagi needs | What Yoav needs | Smallest unblocking deliverable |
|---|---|---|---|
| `MapsComparison` (S8 ↔ Y10) | — | Callable signature | ✅ Real BFS body landed 2026-08-13; `SimulationRunImpl` scores `Completed`/`MaxSteps` via `worldInitialDronePosition`. |
| `MappingAlgorithmFactory`/`MissionControlFactory` handoff (S6 ↔ Y9) | — | The frozen `std::function` types already exist in `common/` | Nothing to build first — both sides just use the exact typedefs from `Common/MappingAlgorithmFactory.h` / `MissionControlFactory.h`. Yoav tests with hand-written lambdas; Sagi's loader supplies the real ones later; no header needs to change hands. |
| `SimulationRunFactoryImpl` concrete class (S9 ↔ Y9) | Yoav's constructor signature + a compiling `create()` | — | Yoav pushes a compiling `SimulationRunFactoryImpl.h`/`.cpp` with the agreed constructor and a stub `create()` (returns a trivially-succeeding fake run) on day one of Y9, before the real body is finished, so Sagi's orchestrator (S9) always has something concrete to `make_unique` against. |
| Per-run output naming (S11 ↔ Y11) | Yoav's per-run artifact names | Sagi's report's expectations of those names | ✅ S11 documented pattern in `README.md` (2026-08-13): `<plugin>_run_NNNN_{output_map.npy\|error.log}`. Y11 emits per-run error logs and fills YAML `error_log_file`. |
| `Simulator/CMakeLists.txt` final source list | Yoav's file list | Sagi's file list | Add sources incrementally as each item lands rather than merging once at the end; filenames don't collide — layout is **decided** (see §Simulator layout above): core under `Simulator/src/`, I/O under `Simulator/src/io/`, tests under `Simulator/tests/` (+ `fixtures/` for loader stubs). |
| End-to-end run (both CLI modes, real plugins) | Yoav's `Map3DImpl`/mocks/run-factory | Sagi's loader/CLI/threading/report writer | ✅ **DONE 2026-08-12** (vertical slice); reports + naming landed 2026-08-13. End-to-end scores still often `-1` when missions terminate as `Error` (spawn/collision) — scorer unit-tested separately. |

---

## Vertical slice — ✅ DONE (2026-08-12)

Waiting until both tracks are "done" to try running the real binary end to end was the single
biggest integration-risk item in this plan (dependency-struct shape, `dlopen`/`ENABLE_EXPORTS`
wiring, and factory plumbing are all new to ex3 and easy to get subtly wrong). Landed as a fuller
slice than the original "hardcoded one cell" sketch — real CLI + YAML composition + threading were
already available, so `main.cpp` wired those instead of stubs.

**What landed:**
- `Simulator/src/main.cpp` — `parseSimulationCliArgs` → `parseCompositionFile` → `PluginLoader`
  (comparative or competition) → `SimulationRunFactoryImpl` bindings → `RunMatrixOrchestrator::run`
  → summary print → destroy factories → `unloadAll`. Never calls `exit()`.
- `Simulator/CMakeLists.txt` — `add_executable(simulator_207190406_209543255 ...)` with
  `ENABLE_EXPORTS ON` and `$<TARGET_OBJECTS:simulator_registration>`.
- `SimulationRunImpl` — catch `runMission()` exceptions (mandatory `MockMovement` collision target),
  still attempt map save, return score `-1`.
- Orchestrator interim map path — `{output_root}/{plugin}/{cell}_output_map.npy`.

**Verified jointly against `inputs/sim_compose.yaml` (both CLI modes):** binary exits 0; both real
`.so`s `dlopen`; 24 `SimulationResult`s produced; output `.npy` maps written. This proves
`MissionControlDependencies`/`MappingAlgorithmDependencies`/the registration macros/`ENABLE_EXPORTS`
are wired correctly.

**Follow-up landed 2026-08-13 on `maps-scoring-reports-output-naming`:** real BFS scoring (S8),
comparative/competitive/per-plugin report YAML (S10), collision-checked
`comparative_results_<time>` / `competition_<time>` dirs + flat
`<plugin>_run_NNNN_output_map.npy` naming documented in `README.md` (S11). Y11 error-log naming
landed: `SimulationRunImpl` writes `<plugin>_run_NNNN_error.log` and the YAML writer fills
`error_log_file`.

---

## Full integration & whole-system verification (after both tracks are feature-complete)

**Done 2026-08-25** on branch `verify-full-integration-pass` (Docker Desktop course image
`drone-mapper-ex3-dev`, not host WSL). Full write-up: `docs/integration-verification-report.md`.
Harness: `Simulator/tests/manual/` (`run_all.sh`, `docker_verify_default.sh`, `docker_tsan.sh`;
`tsan` preset in `CMakePresets.json`). Composition used: `inputs/sim_compose.yaml` (24 cells).

| Check | Result |
|---|---|
| Both CLI modes (`-comparative` / `-competition`) | **PASS** — artifacts written; all 24 cells unscored (`score < 0`) on this composition (known product issue) |
| Re-run output-dir collision | **PASS** — distinct `comparative_results_<time>` dirs |
| `-verbose` on/off | **PARTIAL** — flag wired; no extra verbose files on this composition (Error cells → empty `output_map_file`) |
| Threading determinism (`num_threads` absent/1/2/8) | **PASS** — reports identical after stripping timestamps / scratch paths |
| CLI failure modes | **PASS** — usage + all problems named; no crash |
| Isolation / renamed-copy `.so` | **PASS** — registration undefined in plugins; two copies load cleanly |
| ThreadSanitizer (`-fsanitize=thread`) | **PASS** — 0 warnings (needs ~8 GB Docker VM RAM, build on container `/tmp`, often `--privileged` + `vm.mmap_rnd_bits=28`) |
| Frozen interfaces | **PASS** — no diffs under `common/`, `Simulator/common_simulator/`, `MissionControl/common_mission_control/` |

**Still open after this pass (not verification-harness bugs):**

- Fix or replace `inputs/sim_compose.yaml` so cells complete and score (currently all `-1` / Error).
- Re-check `-verbose` extra files on a composition that finishes with a real `output_map_file`.

Checklist (kept for re-run / grading prep):

- [x] **Both CLI modes against `inputs/`**: run `-comparative` and `-competition` per the exact
  invocations in `.cursor/skills/pre-submission-review/SKILL.md` §7. Confirm each produces its
  output folder, aggregate report, per-plugin YAML, output maps, and error logs.
- [x] **Re-run collision check**: run the same mode twice in immediate succession; confirm the second
  run's output directory does not collide with the first.
- [ ] **`-verbose` on/off**: confirm `MissionControl`'s verbose output only appears when the flag is
  passed. *(wired; needs a completing mission — see PARTIAL above)*
- [x] **Threading determinism**: run the same composition with `num_threads` absent, `=1`, `=2`, `=8`;
  diff the reports — scores and steps must be identical (ordering-independent fields aside), per
  `.cursor/rules/threading-model.mdc`.
- [x] **CLI failure modes**: a typo'd argument, a missing `=`, a nonexistent file, and an empty
  algorithm/mission-control folder — confirm a clean usage + error message naming every problem
  together, never a crash.
- [x] **Isolation / cross-`.so` check**: `nm -DC --undefined-only` on each `.so` should list the
  registration constructor as undefined and nothing else surprising; since another team's plugin
  isn't available, approximate it by loading a **renamed copy of our own** `Algorithm_*.so` under a
  second filename in the same process and confirming `RTLD_LOCAL` keeps the two instances from
  binding to each other's internals.
- [x] **Thread sanitizer**: one full comparative or competitive run built with `-fsanitize=thread`,
  clean.
- [x] **Frozen-interfaces check**: run `.cursor/skills/verify-frozen-interfaces/SKILL.md` — `git diff`
  and `git status --porcelain` against `common/`, `Simulator/common_simulator/`,
  `MissionControl/common_mission_control/` must both be empty.

---

## Path to a submittable artifact

- **`students.txt`**: **done** — `Sagi Eisenberg, 207190406` / `Yoav Naaman, 209543255`. Keep it
  free of `TODO:` before packaging.
- **`README.md`**: rewrite with the actual build presets, binary/`.so` names, both CLI invocations,
  and the output-map/error-log naming pattern agreed in the meeting-points table. Either person,
  once the naming stabilizes (after S11/Y11).
- **HLD as PDF**, in the submission root: class/sequence diagrams reflecting the real three-project
  plugin architecture (not ex2's monolith) — each person diagrams their own component, one person
  assembles and exports the final PDF. Keep it in sync with the code as it lands, not written once
  at the end from memory (graded e14/e15).
- **`bonus.txt`** — only if actually claiming a bonus (most likely candidate: Sagi's lazy
  load-once/unload-when-unused `.so` loading, built only after the mandatory eager-load path from
  S6 is solid). Points at real files/line numbers if written.
- **Known Issues excel** (optional, `docs/known-issues-guidelines.md`) — fill in incrementally as
  each person notices a corner they deliberately skipped (e.g. optional error-handling-matrix rows
  not implemented), not as a last-day exercise.
- **Re-diff the assignment docx**: `context/Advanced Topics TAU 2026B - Assignment 3.docx` left
  draft mode with a literal `[TBD]` bullet on Jul 26, 2026. Re-extract it and check the Moodle Ex 3
  forum before packaging — this is the one item that can silently invalidate earlier work if it
  changed (see open question #8 below).
- **Pre-submission structure pass**: run `.cursor/skills/pre-submission-review/SKILL.md` end to
  end — 5 folders present, `common/` untouched, `UserCommon/` has no build file, every artifact
  name carries both IDs, 4 build files, no binaries in the zip, `students.txt` has no `TODO:` left.
- **Zip**: `ex3_207190406_209543255.zip`, containing `Simulator/`, `Algorithm/`, `MissionControl/`,
  `common/`, `UserCommon/`, the root build file, `students.txt`, `README.md`, HLD PDF, and
  `bonus.txt`/Known Issues excel only if applicable.

---

## Open questions that would change this plan

From `docs/open-questions.md` — flagging only the ones that would change the split above or how an
item is verified if answered differently.

- **#1 (namespace nesting).** Planned assumption: flat `namespace Algorithm_207190406_209543255`
  (not nested under lowercase `algorithm::`), with the concrete plugin class carrying the ID suffix
  too (`MappingAlgorithmImpl_207190406_209543255`), matching the assignment doc's literal macro
  example. **If the answer is nested instead**, items S2 and Y6 change their namespace wrapper and
  the fully-qualified name in the `REGISTER_*` macro call — a rename across a handful of files, not
  a redesign, but it touches both people's plugin `.cpp` files, so whoever notices the forum answer
  first should update both rather than letting them drift apart.
- **#3 (scoring definition: `total_score`/`total_steps` as sums, via ported `MapsComparison`).**
  Planned assumption: port ex2's `MapsComparison` as-is, `total_score` = sum of per-run scores,
  `total_steps` = sum of `MissionRunResult::steps`. **If the metric is different**, item S8's
  scoring body and every downstream verification that checks report numbers (S10, the integration
  smoke test) needs new expected values — the function *signature* published in the S8↔Y10 meeting
  point stays valid either way, so this doesn't block Y10 from proceeding in the meantime.
- **#4 (comparative "same results" grouping).** Planned assumption: group by the exact
  `(total_score, total_steps)` pair. **If grouping is instead by byte-identical output maps**, item
  S10's grouping logic and its verification (which currently checks tie-breaking on the score/steps
  pair) both need rewriting to compare map files instead — worth confirming before S10 is finished
  and tested, since the two implementations are not a superset of each other.
- **#6 (whether tests are graded).** Planned assumption: port and keep ex2's test suites as
  self-verification for every port item above (S3, Y7, and the per-component tests named in each
  item), but do not add new bug-injection-style coverage beyond what's listed. **If a review
  guideline reappears requiring specific suites/filters**, every "Verify:" step in this plan that
  currently says "port ex2's test" would need to be re-checked against the new requirement — this
  doesn't change the work split, only whether the testing items above are sufficient.
- **#8 (assignment doc `[TBD]` / draft status).** No item in this plan depends on it directly, but
  it's why the "re-diff the assignment docx" submission item exists as a distinct late-stage check
  rather than being assumed subsumed by `docs/assignment3-checklist.md`.

Not flagged because they don't change the split or verification: #2 (already actioned as the whole
`UserCommon/` section above), #5 (`.cw` files — no parser reads them, nothing in this plan touches
them), #7 (already settled by the frozen `MappingAlgorithmDependencies` header having no `verbose`
field — not actually ambiguous for coding purposes, only for wording).

---

## Implementation open questions (raised during coding)

These arose during Yoav's track and are not in `docs/open-questions.md`. They affect Y10 and downstream
work and need a decision before those items are implemented.

- **OQ-Y1 — Who owns the output map save when `output_map_file` is empty?**
  `MissionControlImpl` calls `output_map.save(output_file)` inside `finalizeMission`. If the orchestrator
  (or a test) passes an **empty** `output_path`, `Map3DImpl::save("")` throws and `finalizeMission` returns
  a spurious `MAP_SAVE_FAILED` error, making an otherwise-successful mission report `Error`.
  `SimulationRunImpl::run()` also saves the map after `runMission()` returns — so the map is saved twice
  when `output_path` is non-empty.
  **Working resolution (applied in code-review fixes):** `MissionControlImpl` no longer saves the map;
  `SimulationRunImpl` is the sole save owner. The verbose log path for `MissionControlImpl` is derived from
  `output_map_file_` only when that path is non-empty.
  **Resolution confirmed:** empty `output_path` → `SimulationRunImpl` skips the save silently (no
  `MAP_SAVE_FAILED`). Keep this convention when S11's `OutputDirHelper` lands — pass a real path only
  when an on-disk map is wanted.
