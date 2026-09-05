# Submission junk audit (2026-09-01)

Read-only scan of the working tree. **Do not submit this file** — it lives in `docs/`
with the other packaging notes. Zip a staging tree, not this repo.

**Headline:** production C++ is already clean of `TODO` / `FIXME` / TEMP profiling. The
real problem is zip contents: a dump of the working tree would pull in agent notes,
~478 MB of `build/`, and ~27 MB of `tmp/`. Inside the five folders the test source is
small (~0.25 MB) and CMake currently needs it; only a few dead leftovers and porting
comments are worth cleaning.

| Check | Result |
|-------|--------|
| `TODO` / `FIXME` / `HACK` in production `.cpp`/`.h` | 0 |
| Dead leftovers in `MappingAlgorithmImpl.cpp` | 3 (`axisSign`, `unused_scan`, `ensurePlanningReady`) |
| `build/` on disk (gitignored binaries) | ~478 MB |
| `common/` extras vs skeleton | none |

---

## What belongs in the zip

Assignment 3 wants five folders plus root docs. Everything else is development
scaffolding unless you deliberately include `inputs/` so graders can run the README
example.

| Path | Role | Zip? |
|------|------|------|
| `Simulator/` | One of the five required folders | Yes |
| `Algorithm/` | One of the five required folders | Yes |
| `MissionControl/` | One of the five required folders | Yes |
| `common/` | Frozen skeleton — do not add files | Yes |
| `UserCommon/` | Shared code, no `CMakeLists.txt` (correct) | Yes |
| `CMakeLists.txt` | 4th build file | Yes |
| `students.txt`, `README.md`, `HLD.pdf` | Required root docs | Yes |
| `CMakePresets.json`, `vcpkg.json`, `vcpkg-configuration.json` | Needed for the README `cmake --preset default` path | Yes, if graders use your build |
| `inputs/` | Instructor maps/YAML; README example uses them | Include if graders run the example |
| Known Issues `.xlsx` | Optional staff excel, not the markdown | Optional, at zip time |
| `bonus.txt` | Only if claiming a bonus (we are not) | Do not add |
| `docs/known-issues.md` | Working list; do not submit markdown | No |
| `docs/HLD.md` + `docs/hld/*.mmd` | Source for `HLD.pdf` | No — PDF only |
| `scripts/render_hld_pdf.sh` | PDF render helper | No |

---

## Must stay out of the zip

These are in the working tree and would go in if you zip the repo instead of assembling
the five folders. The pre-submission example zip command only excludes `build/`, `.git/`,
and `tmp/` — that is not enough.

| Path | What it is | If included |
|------|------------|-------------|
| `.cursor/` | Agent rules and skills, including `time_each_cell.py` | Graders see internal AI process notes |
| `AGENTS.md` | Cursor agent guide (status, skills, mapping-track scores) | Same — not a student deliverable |
| `docs/` | ~60 files: pickup notes, superpowers plans, benchmarks, workplan | Internal strategy and stale plans |
| `context/` | Assignment 1, 2, and 3 docx plus staff PDFs/xlsx | Confusing and unnecessary |
| `tmp/` | 2,256 files / ~27 MB of cell-timing, benchmarks, extracted txt | Huge, gitignored, leftover runs |
| `build/` | ~478 MB of `.so` / `.o` / `.exe` (gitignored) | Automatic fail on the no-binaries rule |
| `scripts/` | Python benchmark harness + local `.venv` (~19 MB) + HLD render script | Dev tooling; `.venv` is large if a filesystem zip is used |
| `.devcontainer/` | Dockerfile / `devcontainer.json` | Not required; our toolchain |
| `.superpowers/`, `.pytest_cache/`, `_pdf_extract/` | Agent scratch (~4 MB, includes leftover `.so`), pytest cache, empty extract dir | Binaries plus noise |
| `docs/superpowers/` | Implementation plans and design specs | Would show how the algorithm was built |

There is no dedicated zip script, CPack, or `.gitattributes export-ignore`. `git archive`
still ships `.cursor/`, `AGENTS.md`, `docs/`, `context/`, and `scripts/` unless filtered.

---

## Inside the five folders

This is what actually lands if you zip `Simulator/`, `Algorithm/`, `MissionControl/`,
`common/`, and `UserCommon/` as they sit today.

### Dead code and stale comments (safe to delete later)

| Where | What | Severity |
|-------|------|----------|
| `Algorithm/src/MappingAlgorithmImpl.cpp:36–44` | `axisSign()` is defined with `[[maybe_unused]]` and never called | Remove function |
| `Algorithm/src/MappingAlgorithmImpl.cpp:284` | `unused_scan = latest_scan;` dummy so `-Werror` is quiet. The pointer is never read | Omit the parameter name or `(void)latest_scan` |
| `Algorithm/src/MappingAlgorithmImpl.cpp:81–86` | `ensurePlanningReady()` only sets `planning_initialized = true`; no other work | Remove method if nothing else needs the flag |
| `Simulator/src/MapsComparison.cpp:1–5` | Header cites `../Drone-Mapper-ex2/src/MapsComparison.cpp` | Sibling-repo path in shipped source |
| `Simulator/include/Simulator/io/SimulationOutputYamlWriter.h:1–2` | Same ex2 sibling-path comment | Porting provenance |
| `Simulator/src/SimulationRunFactoryImpl.cpp:4–9` | "Key ex2 fix that must NOT regress" plus house-offset story | Useful internally; odd for graders |
| `Algorithm/src/MappingAlgorithmFrontier.h:3` and `:79` | ex1 port note; comment mentions `ALG28` (ex2 ticket id) | Internal ticket, not meaningful to graders |
| `Algorithm/src/PathShaping.h:3` and `UserCommon/.../LidarCone.h:3` | Still say NBV policy / future NBV scoring | Stale names; live algorithm is wavefront |
| `MissionControl/src/DroneControlImpl.cpp:188–189` | "hang class from always-Working+scan" | Optional trim; the contract note is useful |
| `Algorithm/tests/test_mapping_algorithm.cpp:363` | "scanning phase is driven internally" | Leftover from the old phase machine |
| Six empty `.gitkeep` files | `Algorithm/` `MissionControl/` `Simulator/` `src` and `include` dirs now have real files | Optional drop |

**Already gone:** the wavefront plan still tells you to strip `TEMP PROFILING` /
`ALGO_PROFILE` from `MappingAlgorithmImpl.cpp`. That block is not in the tree. No
`#if 0`, no leftover `g_profile`, no `TEMP` comments in production sources.

Plan-batching (`pending_plans`, queued runner-ups) is live production logic, not junk.

### Tests that ship with the folders

Assignment 3 does not require tests, but every project `CMakeLists.txt` always builds
them. Root `CMakeLists.txt` does `find_package(GTest REQUIRED)`.
`Simulator/CMakeLists.txt` also always builds `skeleton_host` and the adversarial /
foreign / lawnmower fixture `.so` targets, and `add_dependencies` ties those fixtures
to the production simulator executable.

**Must stay unless CMake is gated:** `Algorithm/tests`, `MissionControl/tests`, Simulator
unit tests, `tests/fixtures/*.cpp`, and `tests/hosts/skeleton_host/`. Removing them
without wrapping CMake in `if(BUILD_TESTING)` breaks `cmake --preset default`.

Tracked test source is small (~82 files, ~0.25 MB). Size is not the issue.
`skeleton_host` is a second full host; fixture plugins are labeled "never shipped" in
comments, but the sources live in `Simulator/`.

**Can omit without breaking the build:** `Simulator/tests/manual/` (22 shell scripts,
docker helpers) and the `ASSUMPTIONS.md` files under `fixtures/` and `skeleton_host/`.
CMake does not compile those. `adversarial_plugins_ASSUMPTIONS.md` even says those
files are not part of a student zip — that note is about the built `.so`, but the
markdown itself is also not a deliverable.

### `inputs/` extras (only if you include `inputs/`)

| File | Notes |
|------|-------|
| `inputs/profile_cell.yaml` | Single-cell composition added for `ALGO_PROFILE`. Not the instructor 24-cell matrix. Drop from a zip that includes `inputs/`. |
| `inputs/map/*.cw` and `npy_to_cw.py` | ClassicWorld visualization, not runtime. Simulation YAMLs point at `.npy`. Harmless skeleton extras; optional to keep. |

---

## Do not treat as junk

| Thing | Why keep it |
|-------|-------------|
| `simulator::ISimulation` | Frozen published header, implemented by `SimulationImpl`; do not delete the header. |
| `common/` | 18 skeleton files only. No extras. |
| `UserCommon/` (no `CMakeLists.txt`) | Correct. `LidarCone` / `ConeTemplate` / `BeamMath` are still used. |
| `HLD.pdf` at repo root | Required. Editable source stays in `docs/` and should not be zipped. |

---

## Small README nit

`README.md` line 92 cites `UserCommon_207190406_209543255/TimeFormat.h`. The real
include dir is `user_common_207190406_209543255/TimeFormat.h`. Not junk, but wrong if
graders follow the path.

---

## Suggested zip recipe (when packaging)

Assemble a staging tree with only:

- the five folders
- root `CMakeLists.txt` / presets / vcpkg files
- `README.md`, `students.txt`, `HLD.pdf`
- optionally `inputs/` minus `profile_cell.yaml`

Then zip that staging tree. Do **not** zip the git working copy.

If you later want tests out of the zip, that is a CMake change first: gate GTest,
`algorithm_test`, `skeleton_host`, and the fixture `add_dependencies` behind
`BUILD_TESTING` (or similar). Until then, the C++ test sources have to stay inside
the five folders or the documented build fails.

Cleanup order if doing a pass before zip: staging list first, then the three
`MappingAlgorithmImpl` leftovers, then optional comment / `.gitkeep` / test-doc trim.
