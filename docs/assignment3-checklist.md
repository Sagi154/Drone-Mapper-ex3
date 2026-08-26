# Assignment 3 — Mandatory Checklist

Condensed from `context/Advanced Topics TAU 2026B - Assignment 3.docx`. **On conflict, the docx wins.**

**Submission deadline:** Sep 6, 2026, 23:30.

The docx was in DRAFT until Jul 26, 2026 23:00 and still ends with a `[TBD]` bullet — re-read it before submission and diff against this file.

## What ex3 adds on top of ex2

Assignment 2 was one monolithic simulator binary. Assignment 3 splits it into three separately built
projects, turns two of them into dynamically loaded shared libraries, and adds a multithreaded
simulator that runs many missions in parallel in one of two modes.

| Aspect | Ex2 | Ex3 |
|--------|-----|-----|
| Build artifacts | 1 executable + 1 test binary + `maps_comparison` | `simulator_<ids>` executable + `Algorithm_<ids>.so` + `MissionControl_<ids>.so` |
| Wiring | direct linking | `dlopen` + self-registering factories |
| Runs | sequential, one composition | comparative or competitive matrix, multithreaded |
| Report | `simulation_output.yaml` | `comparative_report` / `competitive_report` YAML **plus** per-plugin ex2-style output YAML |

## Submission structure (5 folders, 4 build files)

Zip named `ex3_<student1_id>_<student2_id>.zip` containing:

| Path | Contents | Build file? |
|------|----------|-------------|
| `Simulator/` | simulator class + helpers; executable `simulator_<ids>` | yes |
| `Algorithm/` | mapping algorithm → `Algorithm_<ids>.so` | yes |
| `MissionControl/` | mission control → `MissionControl_<ids>.so` | yes |
| `common/` | course-published files, used **as-is**; **do not add your own files here** | no |
| `UserCommon/` | *our* files needed by more than one project | no |
| (root) | 4th build file that builds all three projects | yes |

Plus, directly in the zip root: `students.txt` (one line per submitter: name + id) and `README.md`.

**Not in the zip:** binary files, external libraries. Only standard C++ libraries or libraries
explicitly approved in the course forum.

> `UserCommon/` **does not exist in the skeleton** — we must create it. See `docs/open-questions.md`.

## Naming — every artifact carries both student IDs

With IDs `207190406` and `209543255`:

| Thing | Value |
|-------|-------|
| Simulator executable | `simulator_207190406_209543255` |
| Algorithm shared object | `Algorithm_207190406_209543255.so` |
| Mission control shared object | `MissionControl_207190406_209543255.so` |
| Algorithm namespace | `algorithm_207190406_209543255` |
| Mission control namespace | `mission_control_207190406_209543255` |
| UserCommon namespace | `user_common_207190406_209543255` |
| Submission zip | `ex3_207190406_209543255.zip` |

`students.txt` in the skeleton still has `TODO:` placeholders — fill it in.

Each part must be able to run against **another team's** implementation of the other parts, so
nothing may depend on our own symbols leaking across the `.so` boundary.

## Simulator CLI — two modes

```text
./simulator_<ids> -comparative simulation=<composition_yaml> mission_control_folder=<folder> \
    algorithm=<algorithm_so> [num_threads=<num>] [-verbose]

./simulator_<ids> -competition simulation=<composition_yaml> mission_control=<mission_control_so> \
    algorithms_folder=<folder> [num_threads=<num>] [-verbose]
```

Argument rules (all of these are graded behaviors):

- Arguments may appear in **any order**; all are mandatory except `num_threads` (and `-verbose`).
- `=` has no spaces around it.
- **Unsupported** arguments → print usage + an error listing **all** unsupported arguments, then finish.
- **Missing** arguments → print usage + an error detailing **all** missing arguments, then finish.
- A file argument pointing at a missing/unopenable file → usage + proper error, then finish.
- A folder argument pointing at a missing/untraversable folder, **or a folder with zero files of the
  desired kind** → usage + proper error, then finish.
- Exact usage text and error wording are our choice.

## Mode semantics

**Comparative** — one algorithm, every `MissionControl` `.so` in `mission_control_folder`, all
configurations in the composition YAML.

- Output dir: `<mission_control_folder>/comparative_results_<time>`
- Report: `comparative_report` YAML, `results_summary` **sorted by size of the agreeing group, descending**;
  mission controls producing identical results are grouped into one `same_results` list.

**Competitive** — one `MissionControl`, every algorithm `.so` in `algorithms_folder`.

- Output dir: `<algorithms_folder>/competition_<time>`
- Report: `competitive_report` YAML, `results_summary` **sorted by `total_score` descending, then
  `total_steps` ascending**.

Both modes:

- `<time>` must be generated fresh per run to avoid collisions with existing folders.
- If the folder cannot be created → write a proper error to screen.
- Results folder holds: all output map files (unique, traceable names), error log(s), **one**
  aggregate report YAML, **and** one ex2-style `Simulation Result Output File` per mission control /
  per algorithm with that plugin's name in the filename.
- `errors: [...]` in the report lists plugins that could not be loaded or run.

## Threading

- Missing `num_threads`, or `num_threads == 1` → single thread (the main thread only).
- `num_threads >= 2` → that many worker threads **in addition** to the main thread. The total is
  therefore never exactly 2.
- Open fewer threads than requested when there is not enough work to utilize them.
- Main thread blocking in `join()` is fine.
- Prefer not locking; lock when you must.
- Pre-allocate the result table when its shape is known in advance — no sparse matrix needed.
- Creating algorithm / mission-control **instances** from factories is cheap → recreate per use, do
  **not** cache instances. This is different from loading/unloading `.so` files, which must not repeat.
- **Bonus available** for loading each `.so` once, lazily, and unloading when no longer needed —
  without ever reloading it.

## Registration and factories

`common/MappingAlgorithmRegistration.h` and `common/MissionControlRegistration.h` are published
**as-is**; the headers must not change. The `.cpp` implementing the registration constructors belongs
to the **Simulator project only** — a singleton registrar is the suggested approach.

Each plugin puts one line at global scope in its `.cpp`:

```cpp
REGISTER_MAPPING_ALGORITHM(MappingAlgorithmImpl_207190406_209543255)
REGISTER_MISSION_CONTROL(MissionControlImpl_207190406_209543255)
```

See `.cursor/rules/plugin-architecture.mdc` for the full loading contract.

## Additional requirements

- `MissionControl` writes verbose output files **iff** `-verbose` is on the command line.
- `MissionControl` and `Algorithm` may write error logs.
- The Simulator need not survive a **crash** of a plugin, but must not crash in any other case.
- No `new` / `delete`.
- Prefer `unique_ptr`; `shared_ptr` only for genuine shared ownership with unknown lifetime.
- Always `dlclose` before exit — and never while objects from that `.so` are still alive.
- Interfaces are described as "not changed from assignment 2", but the skeleton headers **did** change
  shape. See `docs/api-delta-ex2-to-ex3.md`.

## Algorithm minimum bar

- Do not fly the drone into walls.
- Map all relevant surroundings within the configured boundaries.
- Be efficient and exact.

There is a class competition with a bonus for the best algorithms.

## Carried-over grading context (no ex3 review guideline published yet)

- Build must succeed on Linux with gcc 11.4+ and `-std=c++20 -Wall -Wextra -Werror -pedantic`
  (`context/Submission Guidelines - Advanced Topics Assignments - 2026B.docx`).
- Manual code review uses the AdvCpp rubric — see `docs/review-error-codes.md`.
- A `Known Issues` excel is optional but improves the grade — see `docs/known-issues-guidelines.md`.
- `bonus.txt` only if claiming bonus features. HLD as PDF in the submission root.
