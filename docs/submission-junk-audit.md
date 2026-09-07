# Submission junk audit (updated 2026-09-07, supersedes 2026-09-01)

Read-only scan of the working tree. **Do not submit this file** — it lives in `docs/`
with the other packaging notes. Zip a staging tree, not this repo.

**Headline:** production C++ is nearly clean — only one dead-code leftover remains
(`unused_scan` in `MappingAlgorithmImpl.cpp`; the other two from the 2026-09-01 pass are
gone). The real problem is still zip contents: a dump of the working tree pulls in agent
notes, ~360 MB of `build/`, ~34 MB of `tmp/`, and now also a handful of local-only dirs
(`.cache/`, root `.venv/`, `.vscode/`) that didn't exist on 2026-09-01. Inside the five
required folders, everything under `Simulator/tests/` (unit tests, plugin fixtures,
`skeleton_host`) is wired into the **unconditional** default CMake build — no
`BUILD_TESTING` guard exists anywhere in the tree — so almost all of it has to ship as
source or the documented build breaks. Only a small, enumerated set of non-CMake files
inside those folders (ASSUMPTIONS.md notes, three manual-only fixture YAMLs, `.gitkeep`
placeholders, the standalone `skeleton_host` `CMakeLists.txt`, and all of
`Simulator/tests/manual/`) can be dropped safely.

| Check | Result (2026-09-07) | 2026-09-01 |
|-------|----------------------|------------|
| `TODO` / `FIXME` / `HACK` in production `.cpp`/`.h` | 0 | 0 |
| Dead leftovers in `MappingAlgorithmImpl.cpp` | 1 (`unused_scan`, now line 410) | 3 |
| `build/` on disk (gitignored binaries) | ~360 MB (`default/` 318 MB + `opt/` 42 MB) | ~478 MB |
| `tmp/` on disk (gitignored) | ~34 MB, 2386 files | ~27 MB |
| `common/` extras vs skeleton | none (exactly 18 files) | none |
| `BUILD_TESTING` / `EXCLUDE_FROM_ALL` anywhere in CMake | 0 matches | not checked |

---

## What belongs in the zip

Assignment 3 wants five folders plus root docs. Everything else is development
scaffolding unless you deliberately include `inputs/` so graders can run the README
example.

| Path | Role | Zip? |
|------|------|------|
| `Simulator/` | One of the five required folders (see §"Inside the five folders" for what to trim) | Yes |
| `Algorithm/` | One of the five required folders | Yes |
| `MissionControl/` | One of the five required folders | Yes |
| `common/` | Frozen skeleton — do not add files (exactly 18 files, verified) | Yes |
| `UserCommon/` | Shared code, no `CMakeLists.txt` (correct) | Yes |
| `CMakeLists.txt` | 4th build file | Yes |
| `students.txt`, `README.md`, `HLD.pdf` | Required root docs | Yes |
| `CMakePresets.json`, `vcpkg.json`, `vcpkg-configuration.json` | Needed for the README `cmake --preset default` path | Yes, if graders use your build |
| `inputs/` | Instructor maps/YAML; README example uses them | Include if graders run the example — minus `inputs/profile_cell.yaml` (see below) |
| Known Issues `.xlsx` | Optional staff excel, exported at zip time from the **current** `docs/known-issues.md` table (13 rows: optional CI4/6/7/11/12, lazy `.so` load, AdvCpp leftovers #7–#13). Clone the staff Google Sheet, paste that table, export `.xlsx` to the zip **root**. Do **not** re-add resolved rows (CI9/CI2/CI10/CI3/CI8, mapping-track band gaps, Unmapped-as-passable, README/HLD, etc.) — those were pruned from the markdown. | Optional, include if submitting |
| `bonus.txt` | Include — claims implemented optional recoveries CI9, CI2, CI10, CI3, CI8 (file:line). Those are **not** Known Issues rows. | Include |
| `docs/known-issues.md` | Working list for the excel; staff want `.xlsx`, not markdown | No |
| `docs/known-issues-explained.md` | Internal walkthrough of the same rows | No |
| `docs/HLD.md` + `docs/hld/*.mmd` | Source for `HLD.pdf` | No — PDF only |
| `scripts/render_hld_pdf.sh` | PDF render helper | No |

### Known Issues excel and `bonus.txt` (zip time)

Source of truth: `docs/known-issues.md` (13 compacted rows). Walkthrough (not for the
zip): `docs/known-issues-explained.md`. Staff template: `docs/known-issues-guidelines.md`.

| Zip root file | Ship? | Contents |
|---------------|-------|----------|
| Known Issues `.xlsx` | Optional | Paste **only** the current markdown table into the staff Google Sheet clone, export `.xlsx`. Rows 1–6 = skipped optional CI4/6/7/11/12 + lazy `.so` load. Rows 7–13 = deferred AdvCpp leftovers (e03/e16/e13/e22/e21/e10/e23). |
| `bonus.txt` | **Yes** | Implemented optional recoveries: CI9, CI2, CI10, CI3, CI8. Do not list these in the excel. |
| `docs/known-issues.md` / `docs/known-issues-explained.md` | **No** | Markdown stays in `docs/` (already excluded as a tree). |

Do not put resolved/pruned items back into the excel (CI9/CI2/CI10/CI3/CI8, mapping-track
band gaps, Unmapped-as-passable, verbose flag, default-composition scoring, README, HLD,
MockMovement catch, UserCommon-only-in-Simulator, scan-batch hang, foreign-MC step
inflation, HLD e14/e15 leftover).

---

## Must stay out of the zip

These are in the working tree and would go in if you zip the repo instead of assembling
the five folders. The pre-submission example zip command only excludes `build/`, `.git/`,
and `tmp/` — that is **not** enough; four more local-only paths now exist that also
aren't covered by that exclude list.

| Path | What it is | Size (2026-09-07) | If included |
|------|------------|--------------------|--------------|
| `.cursor/` | Agent rules and skills, including `time_each_cell.py` | 0.16 MB | Graders see internal AI process notes |
| `AGENTS.md` | Cursor agent guide (status, skills, mapping-track scores) | 10 KB | Same — not a student deliverable |
| `docs/` | 87 files (grew from ~60): pickup notes, superpowers plans, benchmarks, workplan | 1.64 MB | Internal strategy and stale plans |
| `context/` | Assignment 1, 2, and 3 docx plus staff PDFs/xlsx | 0.74 MB | Confusing and unnecessary |
| `tmp/` | 2,386 files / ~34 MB of cell-timing, benchmarks, extracted txt | ~34 MB | Huge, gitignored, leftover runs |
| `build/` | `.so` / `.o` / `.exe` (gitignored); `default/` + `opt/` presets | ~360 MB | Automatic fail on the no-binaries rule |
| `scripts/` | Python benchmark harness + local `.venv` (~19 MB) + HLD render script | 19.21 MB | Dev tooling; `.venv` is large if a filesystem zip is used |
| `.devcontainer/` | Dockerfile / `devcontainer.json` | 2.7 KB | Not required; our toolchain |
| `.superpowers/` | Agent scratch under `sdd/`, 210 files | 5.09 MB | Would show how the algorithm was built |
| `.pytest_cache/` | pytest cache | 1.4 KB | Noise |
| **New since 2026-09-01 — not previously listed** | | | |
| `.cache/` | Local `clangd/` index, 159 files | 0.77 MB | IDE cache, not gitignored — must be excluded manually |
| `.venv/` (repo root) | Separate from `scripts/benchmark/.venv` | 27.51 MB | Root-level Python env; gitignored but large |
| `.vscode/` | `settings.json` (local editor prefs) | 205 B | Not gitignored (only `.vscode/*` minus `extensions.json`/`settings.example.json`) — check before a blind copy |
| `compile_commands.json` | Empty reparse-point/symlink stub from `CMAKE_EXPORT_COMPILE_COMMANDS ON` (root `CMakeLists.txt` line 6) | 0 B | Harmless if included (empty), but not a deliverable — exclude on principle |
| `_pdf_extract/` | **No longer exists** — removed since 2026-09-01 | — | n/a |

There is no dedicated zip script, CPack, or `.gitattributes export-ignore`. `git archive`
still ships `.cursor/`, `AGENTS.md`, `docs/`, `context/`, and `scripts/` unless filtered.

---

## Inside the five folders

This is what actually lands if you zip `Simulator/`, `Algorithm/`, `MissionControl/`,
`common/`, and `UserCommon/` as they sit today. **Root `CMakeLists.txt` has no
`BUILD_TESTING` guard and no `EXCLUDE_FROM_ALL` anywhere in the tree** — every test
executable and every fixture/`skeleton_host` shared library is part of the default `ALL`
build (`find_package(GTest CONFIG REQUIRED)` + `enable_testing()` unconditionally at
root). That means almost everything under `*/tests/` must ship as source, or the
documented `cmake --preset default` build breaks. Only the items explicitly listed as
"safe to omit" below are not referenced by any CMake target.

### Dead code and stale comments (safe to delete later)

| Where | What | Severity |
|-------|------|----------|
| `Algorithm/src/MappingAlgorithmImpl.cpp:410` | `[[maybe_unused]] const types::LidarScanResult* unused_scan = latest_scan;` — dummy so `-Werror` is quiet, never read | Omit the parameter name or `(void)latest_scan` |
| `Simulator/src/MapsComparison.cpp:2` | Header cites `../Drone-Mapper-ex2/src/MapsComparison.cpp` | Sibling-repo path in shipped source |
| `Simulator/src/SimulationRunFactoryImpl.cpp:4-9` | "Key ex2 fix that must NOT regress" plus house-offset story | Useful internally; odd for graders |
| `Algorithm/src/MappingAlgorithmFrontier.h:3,79` | ex1 port note; comment mentions `ALG28` (ex2 ticket id) | Internal ticket, not meaningful to graders |
| `Algorithm/src/PathShaping.h:3` and `UserCommon/.../LidarCone.h:3` | Still say NBV policy / future NBV scoring | Stale names; live algorithm is wavefront |
| `Algorithm/tests/test_mapping_algorithm.cpp:363` | "scanning phase is driven internally" | Leftover from the old phase machine |

**Already gone since 2026-09-01:** `axisSign()` (was lines 36-44) and `ensurePlanningReady()`
(was lines 81-86) have both been removed from `MappingAlgorithmImpl.cpp` — no matches remain
in the tree. `SimulationOutputYamlWriter.h` was renamed to `Simulator/include/Simulator/io/
SimulatorReports.h` and no longer carries the ex2 sibling-path comment. The
`DroneControlImpl.cpp:188-189` "hang class" comment has also been removed.

Plan-batching (`pending_plans`, queued runner-ups) is live production logic, not junk.

### Tests and fixtures that ship with the folders — required vs. safe to omit

Assignment 3 does not require tests, but every project `CMakeLists.txt` always builds
them and `enable_testing()` runs unconditionally at root. Confirmed **no** `BUILD_TESTING`
guard and **no** `EXCLUDE_FROM_ALL` anywhere in any `CMakeLists.txt`.

**Must stay (all referenced by unconditional CMake targets):**

| Path | CMake target(s) | Evidence |
|------|------------------|----------|
| `Algorithm/tests/*.cpp`, `FakeMap3D.h`, `StubPluginRegistration.cpp` | `algorithm_test` | `add_executable(algorithm_test tests/test_mapping_algorithm.cpp ...)` + `gtest_discover_tests(algorithm_test)`, `Algorithm/CMakeLists.txt:18-51` |
| `MissionControl/tests/test_drone_control.cpp`, `test_mission_control.cpp` | `test_drone_control`, `test_mission_control` | `MissionControl/CMakeLists.txt:23-58` |
| `Simulator/tests/*.cpp` (18 unit-test executables, e.g. `test_simulator_mocks`, `test_map3d_impl`, `test_yaml_config_parsers`, `simulator_cli_test`, `simulator_loader_test`) | one `add_executable` each | `Simulator/CMakeLists.txt` lines 3-451, all unconditional |
| `Simulator/tests/fixtures/*.cpp` (14 plugin sources: `faulty_wall_algorithm_plugin`, `foreign_hits_only_mission_control_plugin`, 7× `adversarial_*`, `baseline_lawnmower_algorithm_plugin`, `valid_algorithm_plugin`, `unregistered_plugin`, `valid_mission_control_plugin`) | 14 `add_library` SHARED targets | Same file, lines ~409-508; several are in `add_dependencies(simulator_207190406_209543255 ...)`, others feed `simulator_loader_test` |
| `Simulator/tests/hosts/skeleton_host/src/*.cpp` + `include/*.h` (11 source + 8 header files) | `skeleton_host` executable | `add_executable(skeleton_host ${SKELETON_HOST_DIR}/src/main.cpp ...)`, `Simulator/CMakeLists.txt:497-525` |

**`skeleton_host` clarification:** it *is* built unconditionally by the default target
(no `EXCLUDE_FROM_ALL`), but it is **not** a dependency of the production
`simulator_207190406_209543255` executable — it's a second, independent host used only to
test our plugins against a foreign simulator. Its *sources* still have to ship (removing
them breaks `cmake --preset default`), but conceptually it plays no role in the graded
run — see `docs/component-placement.md` if reusing this reasoning elsewhere.

**Safe to omit — confirmed zero CMake references (~10 files, ~50 KB total):**

| Path | Why safe |
|------|----------|
| `Simulator/tests/manual/` (entire directory — 22 files: shell scripts, docker helpers, its own `README.md`) | Zero CMake references; only used by the dev verification harness (`run_all.sh`, VAR-01…04 checks) |
| `Simulator/tests/fixtures/adversarial_plugins_ASSUMPTIONS.md` | Not referenced by CMake |
| `Simulator/tests/fixtures/foreign_hits_only_mission_control_ASSUMPTIONS.md` | Not referenced by CMake |
| `Simulator/tests/fixtures/baseline_lawnmower_algorithm_ASSUMPTIONS.md` | Not referenced by CMake |
| `Simulator/tests/fixtures/tiny_compose.yaml` | Only used by `tests/manual/*.sh`; the C++ tests use `inputs/sim_compose.yaml` and temp files |
| `Simulator/tests/fixtures/tiny_compose_adversarial.yaml` | Same |
| `Simulator/tests/fixtures/tiny_mission_adversarial.yaml` | Same |
| `Simulator/tests/hosts/skeleton_host/ASSUMPTIONS.md` | Not referenced by CMake; explicitly notes the built `.so`/host isn't part of a student zip |
| `Simulator/tests/hosts/skeleton_host/CMakeLists.txt` | Standalone-build-only file; `skeleton_host` is built via the parent `Simulator/CMakeLists.txt`, not `add_subdirectory`'d from this one |
| Six `.gitkeep` files (`Algorithm/{src,include/Algorithm}`, `MissionControl/{src,include/MissionControl}`, `Simulator/{src,include/Simulator}`) | Directories now have real files; placeholders are no-ops |

To drop the fixture/`skeleton_host`/unit-test *sources themselves* from the zip (beyond
this omit-list) would require a CMake change first: gate GTest, every test executable,
every fixture `add_library`, and the `simulator_207190406_209543255` `add_dependencies`
block behind `BUILD_TESTING` (or similar) — and even then, `Algorithm/tests`,
`MissionControl/tests`, and the Simulator unit tests would still need to build under a
default `cmake --preset default` invocation with no extra flags, since that's what the
README documents. Until such a change lands, the safe-to-omit list above is the full set.

### `inputs/` extras (only if you include `inputs/`)

25 git-tracked files total. Runtime-needed: `sim_compose.yaml`, all `drone/`, `lidar/`,
`simulation/`, `mission/` YAMLs, and the three `.npy` maps (`scenario_small.npy`,
`scenario_big.npy`, `scenario_house.npy`).

| File | Notes |
|------|-------|
| `inputs/profile_cell.yaml` | Single-cell composition (large_out + large_mission_out) added for `ALGO_PROFILE` dev runs. Not the instructor 24-cell matrix. Drop from a zip that includes `inputs/`. |
| `inputs/map/*.cw` (`scenario_small.cw`, `scenario_big.cw`, `benchmark_map.cw`) and `npy_to_cw.py` | ClassicWorld visualization, not runtime. Simulation YAMLs point at `.npy`. Harmless skeleton extras; optional to keep. |

---

## Do not treat as junk

| Thing | Why keep it |
|-------|-------------|
| `simulator::ISimulation` | Frozen published header, implemented by `SimulationImpl`; do not delete the header. |
| `common/` | Exactly 18 skeleton files, verified 2026-09-07. No extras. |
| `UserCommon/` (no `CMakeLists.txt`) | Correct. `LidarCone` / `ConeTemplate` / `BeamMath` are still used. |
| `HLD.pdf` at repo root | Required. Editable source stays in `docs/` and should not be zipped. |
| `Algorithm/src/ExplorationPlan.h` | Internal header, used by algorithm — not dead code despite the name resembling a doc file. |

---

## Small README nit — fixed

The 2026-09-01 audit flagged `README.md` line 92 citing
`UserCommon_207190406_209543255/TimeFormat.h` instead of the real include dir
`user_common_207190406_209543255/TimeFormat.h`. **Verified fixed** as of 2026-09-07.

---

## Suggested zip recipe (when packaging)

Assemble a staging tree with only:

- the five folders, each minus the "safe to omit" items listed above under
  `Simulator/tests/` (manual/, three ASSUMPTIONS.md, three tiny_*.yaml fixtures,
  skeleton_host's ASSUMPTIONS.md + standalone CMakeLists.txt, six `.gitkeep` files)
- root `CMakeLists.txt` / presets / vcpkg files
- `README.md`, `students.txt`, `HLD.pdf`, `bonus.txt`
- optionally Known Issues `.xlsx` at the zip root (export from current `docs/known-issues.md`; never ship the markdown)
- optionally `inputs/` minus `profile_cell.yaml`

Then zip that staging tree. Do **not** zip the git working copy, and do **not** rely on
`git archive` — it still ships `.cursor/`, `AGENTS.md`, `docs/`, `context/`, `scripts/`,
and would also need explicit excludes for the local-only, non-gitignored paths noted
above (`.cache/`, `.vscode/`, `compile_commands.json`).

If you later want tests/fixtures/`skeleton_host` fully out of the zip (beyond the
omit-list), that is a CMake change first: gate GTest, every test executable, every
fixture `add_library`, `skeleton_host`, and the `simulator_207190406_209543255`
`add_dependencies` block behind `BUILD_TESTING` (or similar), then confirm
`cmake --preset default` (no extra flags, matching the README) still succeeds. Until then,
the C++ test/fixture/host sources have to stay inside the five folders or the documented
build fails.

Cleanup order if doing a pass before zip: staging list first (this doc), then the
`unused_scan` leftover, then optional comment / `.gitkeep` / test-doc trim.
