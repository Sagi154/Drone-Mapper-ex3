---
name: pre-submission-review
description: Verifies the ex3 submission zip matches the assignment's 5-folder/4-build-file structure, ID-suffixed naming, and no-binaries rule before packaging. Use when preparing the submission zip, renaming artifacts with student IDs, or double-checking compliance before the Sep 6 deadline.
---

# Pre-Submission Review

Assignment 3's packaging rules are stricter and more novel than ex2's — a wrong folder or a missing
ID in a filename is a compliance failure (`e18`, 0/2/4) independent of whether the code works. This is
the ex3-specific risk worth a dedicated pass; it has no ex2 equivalent because ex2 shipped one project.

Full source: `docs/assignment3-checklist.md`. Run this checklist against the actual working tree, not
from memory.

## 1. Five top-level folders in the zip

```text
Simulator/       (with a build file inside)
Algorithm/       (with a build file inside)
MissionControl/  (with a build file inside)
common/          (unmodified from the skeleton — see step 2)
UserCommon/      (ours; no build file)
```

Plus, directly in the zip root (not nested in any folder): `students.txt`, `README.md`, and the 4th
build file that builds all three projects.

- [ ] `find . -maxdepth 1 -type d` shows exactly these 5 project folders (plus dotfiles/`context`/`inputs`,
      which are ours to keep for dev but should be excluded from the submission zip — check
      `.gitignore` / the zip step against the assignment's file list, since only the 5 folders + the 3
      root files + build files are actually required).
- [ ] `UserCommon/` exists and has **no** `CMakeLists.txt` of its own.

## 2. `common/` is untouched

```bash
git diff --stat main -- common/   # should be empty once main tracks the pristine skeleton copy
```

- [ ] No file added to `common/`.
- [ ] No header inside `common/` edited (`docs/api-delta-ex2-to-ex3.md` lists every one).

## 3. Naming carries both student IDs everywhere

Confirm against `students.txt` (fill in the real IDs — the skeleton ships `TODO:` placeholders):

- [ ] Simulator executable: `simulator_<id1>_<id2>`
- [ ] Algorithm shared object: `Algorithm_<id1>_<id2>.so` (no `lib` prefix)
- [ ] MissionControl shared object: `MissionControl_<id1>_<id2>.so` (no `lib` prefix)
- [ ] Algorithm code lives in `namespace algorithm_<id1>_<id2>`
- [ ] MissionControl code lives in `namespace mission_control_<id1>_<id2>`
- [ ] UserCommon code lives in `namespace user_common_<id1>_<id2>`
- [ ] `REGISTER_MAPPING_ALGORITHM(MappingAlgorithmImpl_<id1>_<id2>)` and
      `REGISTER_MISSION_CONTROL(MissionControlImpl_<id1>_<id2>)` appear at global scope, once each
- [ ] Zip file itself: `ex3_<id1>_<id2>.zip`

```bash
grep -rn "TODO: Student" students.txt && echo "FIX students.txt before zipping"
```

## 4. Build files

- [ ] 4 total: `Simulator/CMakeLists.txt`, `Algorithm/CMakeLists.txt`, `MissionControl/CMakeLists.txt`,
      and one at the zip root building all three.
- [ ] Clean build from scratch succeeds with gcc 11.4+ and
      `-std=c++20 -Wall -Wextra -Werror -pedantic` (`.cursor/rules/build-and-deps.mdc`).
- [ ] No manual install steps beyond `vcpkg install` — every dependency is in `vcpkg.json` (e11).

## 5. No binaries in the zip

- [ ] `find . -name '*.so' -o -name '*.o' -o -name '*.exe'` under the would-be zip contents returns
      nothing (build artifacts, not source, must be excluded).
- [ ] Only approved external libraries are used; anything else is either removed or has forum approval
      documented in `README.md` (main code) or `bonus.txt` (bonus-only usage).

## 5a. No `new`/`delete` in source (`ZIP-15` / `e13`)

Grep shippable sources — `Simulator/src/`, `Algorithm/src/`, `MissionControl/src/`, `UserCommon/`
— for bare `new` / `delete` expressions. Exclude test files and fixtures (`*/tests/*`,
`*/test_*`, `*/fixture*`, `*/mock*` that live under those paths if any are not already under
`Simulator/`).

```bash
grep -rn --include='*.cpp' --include='*.h' \
     -E '\bnew\b|\bdelete\b' \
     Simulator/src/ Algorithm/src/ MissionControl/src/ UserCommon/
```

- [ ] Zero results (or only operator-overload / placement-new in approved third-party headers).
      If hits found: replace with `std::make_unique`/`std::make_shared` or stack allocation.
      `malloc`/`free` are also forbidden — add `-E 'malloc\(|free\('` to the grep if in doubt.

## 5b. Mocks are under `Simulator/` only (`ZIP-13`)

`MockLidar`, `MockGPS`, `MockMovement`, `Map3DImpl`, and `MapsComparison` must live exclusively
under `Simulator/` — they must **not** appear as source files or `#include` targets inside
`Algorithm/` or `MissionControl/`.

```bash
# Should print nothing:
grep -rn --include='*.cpp' --include='*.h' \
     -E 'MockLidar|MockGPS|MockMovement|Map3DImpl|MapsComparison' \
     Algorithm/ MissionControl/
```

- [ ] Zero results. Any hit means a mock leaked into a plugin namespace — move/refactor so the
      mock lives only in `Simulator/src/` (per `context/Structuring the project.pdf`).

## 5c. `inputs/` tree sanity (`ZIP-17`)

Staff expect certain `inputs/` files to exist at submission / grading time. Check we haven't
accidentally gitignored or deleted instructor-provided maps or YAML.

```bash
ls inputs/
```

- [ ] `inputs/` directory exists in the working tree.
- [ ] At least one `.yaml` / `.yml` composition file present (the one used in §7 smoke pass).
- [ ] At least one `.npy` map file present (reachable via that YAML).
- [ ] No instructor-provided file has been deleted or moved to a non-submission path.

Note: `inputs/` is **ours to keep for dev** but should **not** itself be excluded from the zip
if the graders' test scripts reference a relative `inputs/` path — confirm with the assignment
docx or forum before stripping it.

## 5d. Produced-zip archive check (`ZIP-01` / `ZIP-04` / `ZIP-05`)

Steps §1–5c all inspect the working tree. This step builds and inspects the **actual archive** so
nothing extra gets swept in, and the zip name / root placement are verified exactly.

```bash
# Build the zip (adjust path/command to match your zip step):
cd <parent-of-submission-root>
zip -r ex3_207190406_209543255.zip ex3_207190406_209543255/ \
    --exclude '*/build/*' --exclude '*/.git/*' --exclude '*/tmp/*'

# Inspect the zip:
unzip -l ex3_207190406_209543255.zip | head -40
```

- [ ] **ZIP-01** — archive name is exactly `ex3_<id1>_<id2>.zip` (no extra prefix, no `.tar`).
- [ ] **ZIP-04** — all five top-level folders (`Simulator/`, `Algorithm/`, `MissionControl/`,
      `common/`, `UserCommon/`) appear at the zip root with **no extra nesting**
      (i.e., not `ex3_207190406_209543255/Simulator/…`).
- [ ] **ZIP-05** — `students.txt`, `README.md`, and the root `CMakeLists.txt` appear at the zip
      root alongside the folders, not nested inside any subfolder.
- [ ] Re-run `find` for binaries inside the extracted zip to confirm §5's check holds for the
      actual archive contents (build artifacts sometimes land here if the zip step is too broad).

## 6. Documentation present and accurate

- [ ] `README.md` — build/run instructions match the actual build files and binary names; describes the
      output-map naming pattern chosen for comparative/competitive result folders.
- [ ] `students.txt` — one line per submitter, name + id, no `TODO:` left.
- [ ] HLD as a PDF in the submission root, matching the current class/sequence diagrams (e14/e15).
- [ ] `bonus.txt` only if a bonus is actually being claimed, pointing at real files/line numbers.
- [ ] Known Issues excel (optional, `docs/known-issues-guidelines.md`) if we have anything worth listing.

## 7. Functional smoke pass (both modes)

Run each mode once against `inputs/` before packaging — a working build that fails at runtime is still a
`b01`/`b04`:

```bash
./build/default/Simulator/simulator_<id1>_<id2> -comparative \
    simulation=inputs/sim_compose.yaml \
    mission_control_folder=<folder with at least one MissionControl_*.so> \
    algorithm=<path to Algorithm_*.so>

./build/default/Simulator/simulator_<id1>_<id2> -competition \
    simulation=inputs/sim_compose.yaml \
    mission_control=<path to a MissionControl_*.so> \
    algorithms_folder=<folder with at least one Algorithm_*.so>
```

- [ ] Both produce a `comparative_results_<time>` / `competition_<time>` folder with a report YAML,
      per-plugin output YAML, output maps, and error logs.
- [ ] Re-running immediately after does **not** collide with the previous output folder.
- [ ] `-verbose` produces extra `MissionControl` output; omitting it does not.
- [ ] Try `num_threads=1`, an unset `num_threads`, and `num_threads=<N>` for `N` ≥ 2 — scores should match.
- [ ] Deliberately pass a bad argument (typo, missing `=`) and a nonexistent file/folder — confirm a
      clean usage + error message, not a crash, and that **all** problems are reported together.

## Reference

- `docs/assignment3-checklist.md` — full mandatory requirements
- `docs/api-delta-ex2-to-ex3.md` — what changed vs. the ex2 skeleton
- `docs/open-questions.md` — assumptions to double check against the forum before the deadline
