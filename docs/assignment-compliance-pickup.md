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
`docs/integration-verification-report.md`. Instructor-catalog follow-up (Points 1–4):
`docs/instructor-test-catalog-followup-roadmap.md` — **done 2026-08-28**. Re-verify with
`.cursor/skills/verify-instructor-test-catalog/SKILL.md` (or `Simulator/tests/manual/run_all.sh`).

---

## Verdict

The implementation **does not currently follow the assignment with zero deviation**. Plugin
layout, CLI shape, threading, registration, frozen headers, `-verbose` wiring, all-folder
`errors:` reporting, and snake_case plugin/UserCommon namespaces match the instructor docs.
Default composition `inputs/sim_compose.yaml` is **24/24 `COMPLETED` with `mission_score >= 0`**
(wall-collision recovery + planner improvements). Submission README and root `HLD.pdf` are present.
Instructor-catalog harness gaps and related skills (Points 1–4) are closed. Remaining gap before
Sep 6: zip-time Known Issues excel export and pre-submission packaging (re-diff assignment docx if
forum changed).

Highest-leverage next work is below.

---

## Next session — do these in order

1. **Export Known Issues excel** at zip time: copy `docs/known-issues.md` into the staff Google
   Sheet clone and export `.xlsx` to the zip root. Agent skill:
   `.cursor/skills/populate-known-issues/SKILL.md`.
2. **Pre-submission packaging:** run `.cursor/skills/pre-submission-review/SKILL.md` end to end,
   then zip `ex3_207190406_209543255.zip` (no binaries; include `README.md`, `HLD.pdf`,
   `students.txt`, five folders, root build file).
3. **Optional before zip:** run `.cursor/skills/verify-instructor-test-catalog/SKILL.md` for a
   catalog-ID PASS/FAIL/AMBIGUOUS report (Docker `ctest` + `run_all.sh` + pre-submission + AdvCpp
   rubric). Also run `.cursor/skills/verify-independent-component-variants/SKILL.md` (default
   VAR-01…03; add `--with-baseline` for VAR-04) after merging
   `independent-component-variants`.

---

## Fixed 2026-08-28 (independence variants)

- **Independent component variants (VAR-01…04)** on branch `independent-component-variants`
  (VAR-04 check green; commit may still be pending). Blind fixtures + `skeleton_host`,
  `check_foreign_host.sh`, `check_foreign_mission_control.sh`, `check_adversarial_plugins.sh`,
  `check_baseline_algorithm.sh`. Findings then: hits-only step inflation and a scan-batch
  hang (former known-issues #20/#21; both removed 2026-09-01 after project B). Re-verify:
  `.cursor/skills/verify-independent-component-variants/SKILL.md`. Spec/plan:
  `docs/superpowers/specs/2026-08-28-independent-component-variants-design.md`,
  `docs/superpowers/plans/2026-08-28-independent-component-variants.md`.

## Fixed 2026-08-28

- **Instructor test-catalog follow-up (Points 1–4).** UNSPEC-07 in `docs/open-questions.md` §9;
  manual harness gaps closed (`tiny_compose.yaml`, distinct fixture `.so`s, renamed
  `check_output_dir_collision.sh`, FAULT-02 / CLI-03/04/05/08 / OUT-02 / multi-plugin scripts;
  full `run_all.sh` green in Docker); `pre-submission-review` extended (ZIP-15/13/17/01/04/05);
  new `advcpp-rubric-review` and `verify-instructor-test-catalog` skills. Evidence: roadmap
  `docs/instructor-test-catalog-followup-roadmap.md`; plan
  `docs/superpowers/plans/2026-08-28-close-instructor-test-catalog-gaps.md`;
  `Simulator/tests/manual/`.

## Fixed 2026-08-27

- **Submission README + root HLD PDF.** `README.md` rewritten with author names/IDs, `cmake --preset
  default` build/run, both CLI invocations, `.so`/exe names, and the existing output-naming section.
  Root `HLD.pdf` (AdvCpp e14/e15) with class and sequence diagrams for the three-project plugin
  layout; editable source `docs/HLD.md`, render script `scripts/render_hld_pdf.sh`. Evidence:
  `README.md`, `HLD.pdf` (former known-issues rows 3–4; removed 2026-09-01).
- **Default composition scoring (full matrix).** `inputs/sim_compose.yaml` comparative:
  **24/24 `COMPLETED`**, `mission_score >= 0`, `MISSION_EXCEPTION` 0, wall-clock ~276s (8 threads).
  Design: `docs/superpowers/specs/2026-08-27-wall-collision-recovery-and-planner-design.md`.
  Changes: house spawn `height_cm` 150; `DroneControlImpl` catches recoverable MockMovement
  throws → Continue; `kMaxMovingStallTicks = 2`; scan-during-travel always on; Dijkstra soft
  Unmapped cost (Empty=1, Unmapped=4) with early stop once an Empty-reachable frontier exists.
  Evidence: MissionControl / Algorithm sources + `CollisionBlockedThrowContinues` /
  `FrontierPrefersEmptyOverUnmappedPath`.

## Fixed 2026-08-26

- **Default composition scoring (partial).** Before: `inputs/sim_compose.yaml` was 24/24 Error — 8
  `SPAWN_NOT_PASSABLE` (house, `height_cm: 10`), 16 `MISSION_EXCEPTION`. Fix:
  `inputs/simulation/house_simulation.yaml` `height_cm` 10→150 (world z 300). After: `SPAWN_NOT_PASSABLE`
  = 0; **4 house cells `Completed` with `mission_score` 100**; 20 cells still Error /
  `MISSION_EXCEPTION` (superseded 2026-08-27 — now 24/24).
- **`-verbose` reaches MissionControl.** `SimulationRunFactoryImpl` takes a required `bool verbose`
  ctor arg; `main.cpp` passes `args.verbose`; `create` sets `MissionControlDependencies.verbose`
  from that member (frozen `ISimulationRunFactory::create` unchanged). Evidence:
  `Simulator/include/Simulator/SimulationRunFactoryImpl.h`,
  `Simulator/src/SimulationRunFactoryImpl.cpp`, `Simulator/src/main.cpp`. Unit proof:
  `SimulationRunFactory.PassesVerboseTrueToMissionControl` /
  `PassesVerboseFalseToMissionControl`. File-list `check_verbose.sh`: `*.verbose.txt` appears with
  `-verbose`, absent without (verified on a completing house cell).
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

*(Default composition full matrix scored and submission README/HLD delivered — see Fixed 2026-08-27.
No open mandatory code or doc gaps before zip packaging.)*

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
  `MockMovement` throws on a real-map wall; `DroneControlImpl` catches recoverable
  `blocked`/`boundary` throws → Continue; `SimulationRunImpl` remains the backstop.
- No `new`/`delete` in production sources; C++20 + `-Wall -Wextra -Werror -pedantic`;
  `students.txt` filled; `UserCommon/` has no build file.
- `MapsComparison` is a real 0–100 BFS scorer.

---

## Not ready for the zip (required docs)

*(README and HLD PDF done 2026-08-27 — see Fixed. Remaining zip-root items: Known Issues `.xlsx`
if submitting the optional excel; `bonus.txt` only if claiming bonus.)*

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

`docs/known-issues.md` — staff example-table columns. Resolved/stale rows removed and
renumbered `1..n` (2026-09-01). Remaining: optional Common-issues PDF skips, lazy `.so`
load, unused `ISimulation`, Unmapped-as-passable, plan-batching short-lidar score drop.
English only. At zip time, copy into the Google Sheet and export `.xlsx` to the zip root —
do not submit the markdown.

---

## Stale notes elsewhere (do not trust)

| Doc | What is stale |
|-----|----------------|
| `docs/work-split-status.md` | Written 2026-08-13; header points here. |
| `docs/workplan.md` | Historical S/Y queue; header points here. Do not treat item codes as open work. |
| `docs/vertical-slice-plan.md` | Historical milestone plan; header points here. |
| `docs/integration-verification-report.md` Aug 25 body | Baseline only; see 2026-08-28 addendum + `run_all.sh`. |
