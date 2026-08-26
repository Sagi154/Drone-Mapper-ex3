# Pickup — Assignment 3 compliance audit (2026-08-26)

**Read this first** the next time we open this project. It is the standing answer from the
instructor-doc audit: what already matches Assignment 3, what still deviates, and the order to
fix it. Do not treat `docs/workplan.md` item codes as the current queue — the three-project
implementation exists; the remaining work is compliance and scoring.

**Authoritative instructor sources** (docx/pdf win over condensed docs):

- `context/Advanced Topics TAU 2026B - Assignment 3.docx`
- `context/Common issues and handling.pdf`
- `context/Structuring the project.pdf`
- `context/Submission Guidelines - Advanced Topics Assignments - 2026B.docx`
- `context/AdvCpp Review Guideline.docx` + `context/Error Code Key.xlsx`

Visual tables live in the Cursor canvas `ex3-assignment-compliance.canvas.tsx` (open beside
chat). Working Known Issues rows: `docs/known-issues.md`. Integration smoke:
`docs/integration-verification-report.md`.

---

## Verdict

The implementation **does not currently follow the assignment with zero deviation**. Plugin
layout, CLI shape, threading, registration, frozen headers, `-verbose` wiring, all-folder
`errors:` reporting, and snake_case plugin/UserCommon namespaces match the instructor docs. One
default-scenario failure and two submission-doc gaps do not.

Highest-leverage next work is below.

---

## Next session — do these in order

1. **Get at least one provided composition cell to `Completed` / `MaxSteps` with a real score.**
   `inputs/sim_compose.yaml` is 24/24 Error (`SPAWN_NOT_PASSABLE` / `MISSION_EXCEPTION`). AdvCpp
   happy-flow on default scenarios is a large `b04` risk. Unit tests already score Completed
   runs — the scorer is not the failure. After this lands, re-run
   `Simulator/tests/manual/check_verbose.sh` on a completing cell for the file-list check
   (`*.verbose.txt` only appears when `-verbose` is set).
2. **Rewrite `README.md`** (remove skeleton placeholder; names + IDs, cmake presets, both CLI
   lines, `.so`/exe names, keep the output-naming section). **Add HLD PDF** at zip-root
   (e14/e15).
3. **Export Known Issues excel** only at zip time: copy `docs/known-issues.md` into the staff
   Google Sheet clone. Agent skill: `.cursor/skills/populate-known-issues/SKILL.md`.

---

## Fixed 2026-08-26

- **`-verbose` reaches MissionControl.** `SimulationRunFactoryImpl` takes a required `bool verbose`
  ctor arg; `main.cpp` passes `args.verbose`; `create` sets `MissionControlDependencies.verbose`
  from that member (frozen `ISimulationRunFactory::create` unchanged). Evidence:
  `Simulator/include/Simulator/SimulationRunFactoryImpl.h`,
  `Simulator/src/SimulationRunFactoryImpl.cpp`, `Simulator/src/main.cpp`. Unit proof:
  `SimulationRunFactory.PassesVerboseTrueToMissionControl` /
  `PassesVerboseFalseToMissionControl`. File-list `check_verbose.sh` still waits on a completing
  composition (item 1 above) — `SPAWN_NOT_PASSABLE` skips `runMission()` before verbose write.
- **All-folder-plugin load failure writes `errors:`.** Empty-`bindings` path still calls
  `writeComparativeReport` / `writeCompetitiveReport` with empty `results` and filled
  `failed_plugins`. Evidence: `Simulator/src/main.cpp` empty-`bindings` branch. Integration:
  `Simulator/tests/manual/check_all_folder_plugins_fail.sh`.
- **Snake_case plugin / UserCommon namespaces.** Code and include path use
  `algorithm_207190406_209543255`, `mission_control_207190406_209543255`,
  `user_common_207190406_209543255` (include dir `UserCommon/include/user_common_…/`). `.so` /
  exe / CMake target names unchanged. Evidence: Algorithm / MissionControl / UserCommon sources;
  Docker: full `ctest` green, `nm` Registration symbols present, comparative CLI loads plugins.
  Docs/rules swept: `docs/assignment3-checklist.md`, `docs/component-placement.md`,
  `.cursor/rules/project-context.mdc`, `.cursor/rules/plugin-architecture.mdc`,
  `.cursor/skills/pre-submission-review/SKILL.md`, `.cursor/skills/port-ex2-component/SKILL.md`.

---

## Must fix (mandatory assignment / AdvCpp)

### 1. Default composition never completes

AdvCpp: happy flow must finish on provided default scenarios. CLI modes run and write reports;
every cell stays `mission_score < 0`. See `docs/integration-verification-report.md`.

---

## Passes (no deviation from the assignment text)

- Five folders; ID-suffixed `simulator_207190406_209543255`, `Algorithm_*.so`,
  `MissionControl_*.so`; namespaces `algorithm_*` / `mission_control_*` / `user_common_*`;
  `REGISTER_*` at global scope; `PREFIX ""`;
  registration `.cpp` only in Simulator; `ENABLE_EXPORTS`; `dlclose` after destroying plugin
  objects; no reload; `RTLD_LOCAL`.
- Frozen `common/`, `Simulator/common_simulator/`, `MissionControl/common_mission_control/` —
  no diff vs `main`.
- CLI: `-comparative` / `-competition`, any order, usage lists **all** missing/unsupported
  args, missing file/folder/empty-`.so`-folder, no `exit()`.
- `-verbose` reaches `MissionControlDependencies.verbose` via the factory constructor (unit).
- Output dirs `comparative_results_<time>` / `competition_<time>`, report field names, sort
  rules, per-plugin YAML + maps + logs; unloadable folder plugins (including all of them)
  appear in aggregate `errors: [...]`.
- Threading: absent/`1` = main only; `N>=2` = N workers plus main; cap unused threads;
  recreate plugin instances per cell.
- Mandatory Common-issues pair: algorithm will not plan through known `Occupied`;
  `MockMovement` throws on a real-map wall; `SimulationRunImpl` catches. (`DroneControlImpl`
  deliberately does **not** catch — listed in Known Issues as “developed in a different way”.)
- No `new`/`delete` in production sources; C++20 + `-Wall -Wextra -Werror -pedantic`;
  `students.txt` filled; `UserCommon/` has no build file.
- `MapsComparison` is a real 0–100 BFS scorer (the CMake “stub returns -1” comment is stale).

---

## Not ready for the zip (required docs)

- `README.md` still has the skeleton placeholder (no names/IDs, no build/CLI). Output naming
  is already documented.
- No HLD PDF in the repo root.

---

## Not deviations (staff contradictions — keep current choices)

- Skeleton `common/CMakeLists.txt` vs assignment “no makefile in `common/`” — keep the skeleton.
- Scoring and comparative grouping are unspecified — working assumptions in
  `docs/open-questions.md` (#3 sums, #4 group by `(total_score, total_steps)`).
- Assignment still ends in `[TBD]` (left draft 2026-07-26). Re-diff before Sep 6.
  *(2026-08-26 forum refresh already applied to `context/`; only namespace snake_case changed.)*
- **`context/Submission Guidelines - Advanced Topics Assignments - 2026B.docx` still describes
  Ex1** (`drone_mapper`, `map_output.txt`, three `original_output` folders, zip `ID1_ID2.zip`).
  Assignment 3’s own docx wins for zip name `ex3_<id1>_<id2>.zip`, CLI, `students.txt`, and the
  5-folder layout. Compiler flags, HLD PDF, readme names/IDs, no binaries, and forum-approved
  libs still apply. Do **not** add Ex1 leftover folders as Known Issues.

Optional Common-issues rows and lazy `.so` load/unload are **bonus**, not required. Eager load
is explicitly allowed.

---

## Known Issues (working file, not the excel)

`docs/known-issues.md` — staff example-table columns. Row 2 (default composition) is
**FIX BEFORE SUBMIT**. Rows 5–14 are skipped optional Common-issues PDF scenarios (one row
each). English only. At zip time, copy into the Google Sheet and export `.xlsx` to the zip
root — do not submit the markdown.

---

## Stale notes elsewhere (do not trust)

| Doc | What is stale |
|-----|----------------|
| `docs/work-split-status.md` | Written 2026-08-13; pickup is this file. |
| `Simulator/CMakeLists.txt` comment “MapsComparison stub returns -1” | Body is the full BFS port. |
