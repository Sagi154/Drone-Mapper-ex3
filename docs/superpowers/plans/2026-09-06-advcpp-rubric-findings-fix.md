# AdvCpp Rubric Findings Fix — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development, but with the human-approval override in "Execution Mode" below (per-task, not continuous). Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fix the low-risk, high-confidence findings from the `advcpp-rubric-review` pass run on
`known-issues-fixes` @ `292277a` (2026-09-06), one finding at a time, in an isolated worktree, each
gated by a full `verify-cell-runtime` 24-cell run and explicit human approval before moving on.

**Architecture:** Eighteen independent tasks (Task 0 = setup, Tasks 1–17 = one rubric finding —
or one tightly-related cluster of the same finding across files — each). Every task is a
self-contained mechanical refactor: no behavior change to `nextStep`/`step` outputs is intended.
Tasks that touch `Algorithm/` or `MissionControl/` hot paths get the full 24-cell
`verify-cell-runtime` gate; tasks that touch only `Simulator/` glue, docs, or dead code still get
the gate (per the user's explicit instruction) but a `wall_sum`/`score_sum` match to baseline is
the expected (trivial) outcome, not a risk.

**Tech Stack:** C++20, CMake presets (`default` = Debug, `opt` = Release), Docker image
`drone-mapper-ex3-dev`, GoogleTest/CTest, `verify-cell-runtime`'s `time_each_cell.py`.

## Execution Mode (overrides subagent-driven-development's default)

`subagent-driven-development`'s default is continuous execution (no stopping between tasks). The
user explicitly asked for the opposite here:

1. Fresh implementer subagent per task, model `cursor-grok-4.6-high` (never a more expensive model)
   for both implementer and task-reviewer roles — this project's convention throughout this whole
   AdvCpp-rubric-review thread.
2. After the implementer's diff review passes, run the full `verify-cell-runtime` 24-cell suite in
   the worktree using the **Clean verify-cell-runtime procedure** below (wipe `build/opt` and
   `tmp/bench-out`, then a fresh CMake configure + full Release rebuild — never reuse a previous
   `build/opt` or skip configure).
3. Report the per-task diff summary + the 24-cell table (scores vs. baseline, wall_sum, any
   WARN/FAIL) to the human.
4. **Stop. Wait for explicit human approval before starting the next task.** Do not dispatch the
   next task's implementer until the human says to continue. This applies after every task,
   including Task 0.
5. If `verify-cell-runtime` shows a FAIL, a new WARN that wasn't there before, or any score
   regression beyond noise, do not proceed — report it and let the human decide whether to fix,
   revert the task's commit, or accept the drift.

## Global Constraints

- Base branch: **`known-issues-fixes`** (not `main`). This whole rubric review was run against
  `known-issues-fixes` @ `292277a`, which is where VAR-01 Approach A already landed; `main` in this
  repo is the untouched course skeleton mirror (see `AGENTS.md`) and is not the integration branch
  for this project's own work. This is a deliberate, documented exception to
  `.cursor/rules/git-workflow.mdc`'s "branch from `main`" default — call it out in the PR
  description when this branch is eventually merged.
- Never touch `common/`, `Simulator/common_simulator/`, `MissionControl/common_mission_control/`.
- No wall-clock abort in `Algorithm/` or `MissionControl/`. `max_steps` stays the only mission-length
  limit (`.cursor/skills/verify-cell-runtime/SKILL.md`).
- No behavior change to scoring or step counts is *intended* by any task in this plan — these are
  rubric-quality refactors (headers, const-correctness, dedup, magic numbers, docs), not algorithm
  changes. Any score/step delta on a 24-cell run is a bug in that task, not an accepted tradeoff,
  **except** where a task's own text says otherwise.
- Compare every 24-cell run against the documented baseline in
  `.cursor/skills/verify-cell-runtime/SKILL.md` → "Documented baseline (2026-09-06)":
  `score_sum = 1832.747`, `wall_sum = 30.4s`, `wall_max = 3.0s`, 0 WARN/FAIL.
- **Every** `verify-cell-runtime` gate in this plan (Task 0 confirmation and every later task)
  MUST use the **Clean verify-cell-runtime procedure** below. Never reuse an existing `build/opt`
  (even if `CMakeCache.txt` is present). Never "just rebuild Algorithm `.o`/`.so`". Never run
  Debug `build/default` as a substitute. A stale object file from a previous task would make the
  gate meaningless.
- `Algorithm/` code style: `.cursor/rules/adv-cpp-standards.mdc` — no magic numbers, `-Wall -Wextra
  -Werror -pedantic` clean.
- Git workflow (`.cursor/rules/git-workflow.mdc`): kebab-case branch name, no item codes/owner
  names, Conventional Commits, **human approval required before every `git commit`** — the
  implementer subagent stages (`git add`) and shows the diff + proposed message; it does not run
  `git commit` itself unless the human (you, relaying task-level approval) explicitly approves that
  exact message in the same turn.
- One rubric finding (or tightly-coupled cluster across files) per commit. Do not combine two
  tasks' changes into one commit.
- Every task must leave `ctest --test-dir build/default -R Algorithm` (or the relevant subset:
  `Algorithm`, `MissionControl`, `Simulator`) green before the implementer reports DONE.

## Clean verify-cell-runtime procedure (mandatory for every gate)

This **overrides** `.cursor/skills/verify-cell-runtime/SKILL.md` §"Procedure" item 1, which
reuses `build/opt` when `CMakeCache.txt` exists and only deletes Algorithm `.o`/`.so`. For this
plan, that is not clean enough: a leftover `CMakeCache`, object file, or previous CSV would mix
the previous task's binaries into the next measurement.

**Every** step in this plan that says `verify-cell-runtime` (including Task 0 Step 2 and Task 17's
final run) means **exactly** this sequence, in the worktree, via Docker image `drone-mapper-ex3-dev`.
Host is Windows PowerShell: set `$env:MSYS_NO_PATHCONV = "1"` before `docker run`, bind-mount as
`-v "<worktree>:/work"`, and use `-w //work` (double slash).

```bash
docker run --rm -e PYTHONUNBUFFERED=1 -e VCPKG_ROOT=/usr/local/vcpkg \
  -v "<worktree>:/work" -w //work drone-mapper-ex3-dev bash -lc '
  set -euo pipefail
  rm -rf /work/build/opt /work/tmp/bench-out
  cmake -S . -B build/opt -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_TOOLCHAIN_FILE="$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake"
  cmake --build build/opt -j"$(nproc)" --target \
    Algorithm_207190406_209543255 \
    simulator_207190406_209543255 \
    MissionControl_207190406_209543255
  python3 .cursor/skills/verify-cell-runtime/scripts/time_each_cell.py \
    --build-dir /work/build/opt
'
```

Required wipe (do not skip any line):

| Path | Why |
|------|-----|
| `build/opt` (entire tree, including `CMakeCache.txt`) | Fresh configure + full Release rebuild; no leftover `.o` / `.so` / link artifacts from a previous task |
| `tmp/bench-out` (entire tree) | Fresh CSV / verdict files; do not append to or reread a previous run's `per-cell-wall.csv` |

Forbidden substitutes:

- Reusing `build/opt` because `CMakeCache.txt` already exists
- Only deleting `Algorithm/.../*.o` and `Algorithm_*.so` (the skill's default incremental rebuild)
- `cmake --build` without a preceding `rm -rf build/opt` + `cmake -S . -B build/opt`
- Running `time_each_cell.py` against `build/default` (Debug)
- Using `num_threads=8` / `run_benchmark.py` as a stand-in for the 24-cell per-cell wall

Then judge the printed table and `tmp/bench-out/per-cell-wall.csv` against the 2026-09-06 baseline
exactly as the skill's §2–§3 require.

## Deferred findings (not in this plan — rationale)

The rubric review surfaced more findings than are safe to batch into low-risk, single-verification
tasks. These are **not** included below; flag them as Known Issues (`populate-known-issues`) or
revisit in a dedicated, higher-scrutiny plan if the human wants them fixed later:

| Finding(s) | Why deferred |
|---|---|
| e03 hot-path libm-parity unwraps (`movementToward`/`predictPose`, `PathShaping::stepCostForPath`, `ScanPlanning` travel heading, `MockMovement`/`MockGPS` trig, `BeamMath::pointAlongBeam`, `MapsComparison` quantize, LOS `sqrt`) | The rubric review itself already labels these "Accepted — keep unless the 24-cell matrix is re-run." Touching them risks the exact libm-rounding score drift this project has spent two prior plans (Approach A/B) stabilizing. |
| e16 (`double radius_cm`/`step_cm`/radian APIs across `ConeTemplate`, `LidarCone`, `MappingAlgorithmFrontier`, `WavefrontPlanner`) | Touches every call site on the scan/frontier hot path for a cosmetic type-safety win (1–2 rubric points); regression blast radius is the whole 24-cell score table. |
| e13 (raw `const T*` in `MatrixCell`, `WavefrontPlanner::plan`'s out-param, `rankClusters`, `ScanPlanning::ScoredDirection`) | Structural API changes across planner/report code for a `m`/`n` rubric line; not mechanical enough for a single cheap-model task with a tight diff. |
| e22 (`ConeTemplateCache::get` returning `const vector&`, `ScanPlanning.h`/`ExplorationPlan.h` exposing `detail` types) | Same class of invasive, hot-path-adjacent API change; low value for the risk. |
| Remaining e21 sites beyond Task 15 below (`PathShaping` string-pull re-walk, `VoxelStamp::mark` re-quantize, `findPathTo` missing the `runBoundedSearch` memo) | Each needs its own correctness argument on the hot path; bundling them risks a silent score regression that's hard to attribute to one of several simultaneous changes. |
| Remaining e10 duplication (`LidarCone`/`ConeTemplate` walk overlap, `MockLidar::scan` ring rebuild, `PluginLoader::loadPlugins`/`applyFilesystemChecks` comparative-vs-competition loops) | Lower rubric weight than Tasks 7–9; higher effort to de-duplicate safely than the single-file swaps already in scope. |

---

### Task 0: Worktree, branch, and baseline confirmation

**Files:** none (git/build only)

- [x] **Step 1: Reset the stale branch and create the worktree**

`fix-advcpp-rubric-findings` already exists locally and on `origin`, but it has no unique commits
(`git merge-base fix-advcpp-rubric-findings known-issues-fixes` == the branch's own tip — it is a
pure ancestor of `known-issues-fixes`, i.e. stale). Reset it to the current `known-issues-fixes`
tip instead of stacking on top of the old stale ref:

```bash
cd C:/Users/sagi1/Projects/DroneMapper/Drone-Mapper-ex3
git fetch origin
git branch -f fix-advcpp-rubric-findings known-issues-fixes
git worktree add ../ex3-advcpp-rubric-fixes fix-advcpp-rubric-findings
cd ../ex3-advcpp-rubric-fixes
```

Expected: new directory `C:\Users\sagi1\Projects\DroneMapper\ex3-advcpp-rubric-fixes` checked out
on `fix-advcpp-rubric-findings`, `git log -1 --oneline` shows `292277a` (or later, if
`known-issues-fixes` has moved).

- [x] **Step 2: Confirm current 24-cell baseline in the fresh worktree**

Run the **Clean verify-cell-runtime procedure** above (wipe `build/opt` + `tmp/bench-out`, fresh
CMake configure, full Release rebuild, then `time_each_cell.py`). This is a **confirmation** run,
not a new baseline — it must reproduce `score_sum = 1832.747`, `wall_sum ≈ 30s`, 0 WARN/FAIL
(small wall-time noise is fine; score must match to the CSV's precision). Do not treat the
skill's incremental "delete Algorithm `.o` then rebuild" as sufficient here.

Expected: matches the documented 2026-09-06 baseline table. If it does not match (score drift, new
WARN/FAIL), stop here and report — do not start Task 1 against an already-drifted tree.

- [x] **Step 3: Report and wait**

Report the worktree path, branch, HEAD sha, and the confirmation table. **Stop. Wait for approval**
before Task 1.

---

### Task 1 (e01): Split UTC timestamp helpers out of `IRunErrorLog.h`

**Rubric finding:** e01 (m: 0, n: 1, s: 2) — `IRunErrorLog.h:14` groups an unrelated concern
(free-function UTC timestamp helpers) with the `IRunErrorLog` interface.

**Files:**
- Modify: `UserCommon/include/user_common_207190406_209543255/IRunErrorLog.h`
- Create: `UserCommon/include/user_common_207190406_209543255/TimeFormat.h`
- Modify: `UserCommon/src/TimeFormat.cpp` (rename from wherever `currentUtcTimestamp` /
  `formatUtcTimestamp` are currently defined — locate with
  `rg -l "currentUtcTimestamp" UserCommon/src`)
- Modify: every `.cpp`/`.h` that includes `IRunErrorLog.h` **only** for the timestamp helpers
  (search `rg -l "currentUtcTimestamp|formatUtcTimestamp" --type cpp --type h`) to include
  `TimeFormat.h` instead (or in addition, if it also uses `IRunErrorLog` itself).
- Test: any existing test file covering these helpers (search
  `rg -l "currentUtcTimestamp|formatUtcTimestamp" Simulator/tests Algorithm/tests
  MissionControl/tests UserCommon` if a `UserCommon/tests` tree exists) — update its `#include` the
  same way, do not change its assertions.

**Interfaces:**
- Consumes: nothing from earlier tasks.
- Produces: `user_common_207190406_209543255::currentUtcTimestamp()` and
  `formatUtcTimestamp(std::chrono::system_clock::time_point)` now live in `TimeFormat.h`, same
  namespace, same signatures — every other task in this plan can assume that location if it needs
  them.

- [x] **Step 1: Create the new header**

```cpp
// TimeFormat.h — UTC timestamp helpers shared by run logging and report writers.
#pragma once

#include <chrono>
#include <string>

namespace user_common_207190406_209543255 {

/// Returns the current UTC time as an ISO-8601 string, e.g. "2026-08-05T17:30:00Z".
[[nodiscard]] std::string currentUtcTimestamp();

/// Formats a given time_point as an ISO-8601 UTC string.
/// Useful in tests where a fixed reference point is needed.
[[nodiscard]] std::string formatUtcTimestamp(std::chrono::system_clock::time_point tp);

} // namespace user_common_207190406_209543255
```

- [x] **Step 2: Trim `IRunErrorLog.h` to just the interface**

```cpp
#pragma once

// Forward-declare only — consumers that create ErrorRef objects must include
// <Common/types/MissionTypes.h> themselves.
namespace common::types {
struct ErrorRef;
} // namespace common::types

namespace user_common_207190406_209543255 {

class IRunErrorLog {
public:
    virtual ~IRunErrorLog() = default;

    virtual void log(const common::types::ErrorRef& error) = 0;
};

} // namespace user_common_207190406_209543255
```

- [x] **Step 3: Point the `.cpp` that defines the helpers at the new header**

Find it: `rg -l "currentUtcTimestamp" UserCommon/src`. Rename that translation unit's file to
`TimeFormat.cpp` if it was previously named after `IRunErrorLog` (e.g. `RunErrorLog.cpp` may
already be split correctly — check before renaming; if the helpers already live in their own
`.cpp`, just retarget its `#include "IRunErrorLog.h"` to `#include
<user_common_207190406_209543255/TimeFormat.h>`). Add `TimeFormat.h`/`TimeFormat.cpp` to
`UserCommon`'s source list wherever `UserCommon/src/*.cpp` files are registered (check how
`RunErrorLog.cpp` is wired into `Algorithm/CMakeLists.txt` / `MissionControl/CMakeLists.txt` /
`Simulator/CMakeLists.txt` — `UserCommon/` has no build file of its own; each consumer's
`CMakeLists.txt` globs or lists `UserCommon/src/*.cpp`, so a new file needs the same treatment
there, in **all three** consumer `CMakeLists.txt`).

- [x] **Step 4: Fix every include site**

```bash
rg -l "currentUtcTimestamp|formatUtcTimestamp" --type-add 'cpp:*.{cpp,h,hpp}' -t cpp
```

For each hit that includes `IRunErrorLog.h` only for these functions, swap the include. For a hit
that uses both `IRunErrorLog` and the timestamp functions, add both includes.

- [x] **Step 5: Build and test**

```bash
docker run --rm -e VCPKG_ROOT=/usr/local/vcpkg -v "<worktree>:/work" -w //work \
  drone-mapper-ex3-dev bash -lc '
  set -euo pipefail
  cmake --preset default
  cmake --build --preset default
  ctest --test-dir build/default --output-on-failure
'
```

Expected: build succeeds, all existing tests pass unchanged (this is a pure header split, no
logic change).

- [x] **Step 6: `verify-cell-runtime` gate + stop for approval**

Run the **Clean verify-cell-runtime procedure** (wipe `build/opt` + `tmp/bench-out`, fresh CMake
configure, full Release rebuild). Never reuse a previous `build/opt`.

Run the full 24-cell suite per the skill. Expected: identical `score_sum`/per-cell scores to the
Task 0 confirmation (this task touches no algorithm logic — `wall_s` noise only). Report the table.
**Stop. Wait for approval.** (gate done 2026-09-06: PASS, scores match Task 0)

- [ ] **Step 7: Commit (after approval)**

```bash
git add -A
git commit -m "refactor: split UTC timestamp helpers out of IRunErrorLog.h"
```

Committed as `730b4b5` after human approval.

---

### Task 2 (e06): `DroneControlImpl` — const-ref lidar/GPS

**Rubric finding:** e06 (m: 0, n: 1, s: 2) — `DroneControlImpl.h:19` ctor takes `ILidar&` /
`IGPS&`; `step()` only calls their `const`-qualified methods (`scan`, `position`, `heading`).

**Files:**
- Modify: `MissionControl/include/MissionControl/DroneControlImpl.h`
- Modify: `MissionControl/src/DroneControlImpl.cpp`
- Test: `MissionControl/tests/test_drone_control_impl.cpp` (or the equivalent file — locate with
  `rg -l "DroneControlImpl" MissionControl/tests`) — only if it constructs mocks that are
  non-const `ILidar&`/`IGPS&` in a way that breaks binding to a `const&` parameter (it should not;
  const-ref binds to a non-const lvalue fine).

**Interfaces:**
- Consumes: nothing new.
- Produces: `DroneControlImpl`'s ctor signature is now `(..., const common::ILidar& lidar_sensor,
  const common::IGPS& gps, common::IDroneMovement& movement, ...)` — `movement_` stays mutable
  (rotate/advance/elevate are non-const on `IDroneMovement`). No other task in this plan depends
  on this signature.

- [ ] **Step 1: Confirm `step()` never calls a mutating method on `lidar_sensor_`/`gps_`**

Read `MissionControl/src/DroneControlImpl.cpp`'s `step()` and `state()`. Confirm every use of
`lidar_sensor_` is `lidar_sensor_.scan(...)` (check `ILidar::scan`'s declared const-ness in
`common/include/Common/ILidar.h` — do not modify that frozen header, only confirm) and every use of
`gps_` is `gps_.position()` / `gps_.heading()` (check `common/include/Common/IGPS.h`, same rule:
read-only). If either interface method is *not* const, stop this task and report — do not force a
const binding that would fail to compile; leave the finding as a Known Issue instead.

- [ ] **Step 2: Change the header**

In `DroneControlImpl.h`, change:

```cpp
    DroneControlImpl(const common::types::DroneConfigData& drone,
                     const common::types::MissionConfigData& mission,
                     const common::types::LidarConfigData& lidar,
                     common::ILidar& lidar_sensor,
                     common::IGPS& gps,
                     common::IDroneMovement& movement,
                     common::IMutableMap3D& output_map,
                     common::IMappingAlgorithm& mapping_algorithm);
```

to:

```cpp
    DroneControlImpl(const common::types::DroneConfigData& drone,
                     const common::types::MissionConfigData& mission,
                     const common::types::LidarConfigData& lidar,
                     const common::ILidar& lidar_sensor,
                     const common::IGPS& gps,
                     common::IDroneMovement& movement,
                     common::IMutableMap3D& output_map,
                     common::IMappingAlgorithm& mapping_algorithm);
```

and the two member declarations:

```cpp
    const common::ILidar& lidar_sensor_;
    const common::IGPS& gps_;
```

- [ ] **Step 3: Update the `.cpp` ctor definition** to match (`const common::ILidar&
  lidar_sensor`, `const common::IGPS& gps` in the parameter list and member-init list — the
  member-init list itself does not change syntactically, only the types flow through from the
  header).

- [ ] **Step 4: Build and test**

```bash
docker run --rm -e VCPKG_ROOT=/usr/local/vcpkg -v "<worktree>:/work" -w //work \
  drone-mapper-ex3-dev bash -lc '
  set -euo pipefail
  cmake --build build/default
  ctest --test-dir build/default --output-on-failure -R MissionControl
'
```

Expected: compiles clean (no `-Wall -Wextra -Werror -pedantic` warnings — a non-const→const-ref
narrowing here should not trigger any), `MissionControl` test suite green.

- [ ] **Step 5: `verify-cell-runtime` gate + stop for approval**

Expected: identical scores to Task 0/1's confirmation. **Stop. Wait for approval.**

- [ ] **Step 6: Commit (after approval)**

```bash
git commit -am "refactor: take DroneControlImpl's lidar and GPS by const-ref"
```

---

### Task 3 (e08): `DroneControlImpl` — drop the unused `mission_` member

**Rubric finding:** e08 (m: 0, n: 1, s: 2) — ctor stores `MissionConfigData mission_` but
`DroneControlImpl.cpp` never reads it.

**Files:**
- Modify: `MissionControl/include/MissionControl/DroneControlImpl.h`
- Modify: `MissionControl/src/DroneControlImpl.cpp`
- Test: whichever test constructs `DroneControlImpl` (`rg -l "DroneControlImpl(" MissionControl/tests`)

**Interfaces:**
- Consumes: Task 2's ctor signature (this task edits the same ctor again — do this task after
  Task 2 is committed, not in parallel).
- Produces: `DroneControlImpl`'s ctor drops the `mission` parameter entirely. **Confirm first**
  that nothing outside `DroneControlImpl` needs this call site's `mission` argument preserved —
  check every construction site (`rg -n "DroneControlImpl(" --type cpp`, likely
  `SimulationRunFactoryImpl.cpp`) and remove the argument there too.

- [ ] **Step 1: Grep to confirm `mission_` is truly dead**

```bash
rg -n "mission_\b" MissionControl/src/DroneControlImpl.cpp
```

Expected: only the ctor's member-init (`mission_(mission)`) and the header's declaration — no read
anywhere in `step()`/`state()`/`applyMovement()`/`applyScanIfRequested()`. If you find a real read,
stop this task and report instead of deleting a used member.

- [ ] **Step 2: Remove from the header**

Delete the `const common::types::MissionConfigData& mission` parameter from the ctor declaration
and delete the `common::types::MissionConfigData mission_;` member.

- [ ] **Step 3: Remove from the `.cpp`** — drop the parameter from the ctor definition and its
  member-init entry.

- [ ] **Step 4: Fix every call site**

```bash
rg -n "DroneControlImpl(" --type-add 'cpp:*.{cpp,h}' -t cpp
```

Remove the now-extra `mission` argument at each construction site (do not remove any *other*
argument — only the one this task deleted).

- [ ] **Step 5: Build and test** — same command as Task 2 Step 4. Expected: green, no leftover
  "unused parameter" warnings anywhere else that called this ctor.

- [ ] **Step 6: `verify-cell-runtime` gate + stop for approval**

Run the **Clean verify-cell-runtime procedure** (wipe `build/opt` + `tmp/bench-out`, fresh CMake
configure, full Release rebuild). Never reuse a previous `build/opt`.

Expected: identical scores. **Stop. Wait for approval.**

- [ ] **Step 7: Commit (after approval)**

```bash
git commit -am "refactor: drop DroneControlImpl's unused mission config member"
```

---

### Task 4 (e07): Make `ConeTemplateCache::get` and `PluginLoader::tryOpen` const

**Rubric finding:** e07 (m: 0, n: 1, s: 2) — two lookup methods that only mutate an internal cache
(logically const) are declared non-const.

**Files:**
- Modify: `UserCommon/include/user_common_207190406_209543255/ConeTemplate.h`
- Modify: `UserCommon/src/ConeTemplate.cpp`
- Modify: `Simulator/include/Simulator/PluginLoader.h`
- Modify: `Simulator/src/PluginLoader.cpp`
- Test: `UserCommon` cone-template test (`rg -l "ConeTemplateCache" Algorithm/tests
  UserCommon` — likely `test_cone_template.cpp`), `Simulator/tests/test_plugin_loader.cpp` (or
  equivalent — `rg -l "PluginLoader" Simulator/tests`).

**Interfaces:**
- Consumes: nothing new.
- Produces: `ConeTemplateCache::get(...)` is now `const`; its five identity fields
  (`built_`, `res_cm_`, `z_min_`, `z_max_`, `d_`, `fov_circles_`, `templates_`) become `mutable`.
  `PluginLoader::tryOpen(...)` is now `const`.

- [ ] **Step 1: `ConeTemplateCache` — mark the cache fields `mutable`, mark `get` const**

In `ConeTemplate.h`:

```cpp
class ConeTemplateCache {
public:
    [[nodiscard]] const std::vector<detail::ConeTemplate>& get(
        const common::types::LidarConfigData& lidar,
        PhysicalLength resolution) const;

private:
    [[nodiscard]] static std::vector<detail::ConeTemplate> build(
        const common::types::LidarConfigData& lidar, PhysicalLength resolution);

    mutable std::vector<detail::ConeTemplate> templates_{};
    mutable bool built_ = false;
    mutable double res_cm_ = 0.0;
    mutable double z_min_ = 0.0;
    mutable double z_max_ = 0.0;
    mutable double d_ = 0.0;
    mutable std::size_t fov_circles_ = 0;
};
```

Update `ConeTemplate.cpp`'s `ConeTemplateCache::get` definition signature to add trailing `const`
(no body changes — every assignment to `templates_`/`built_`/etc. is now legal because those
fields are `mutable`).

- [ ] **Step 2: Grep for any caller that holds a non-const `ConeTemplateCache&` specifically
  *because* `get` was non-const** (unlikely, but confirm):

```bash
rg -n "ConeTemplateCache" --type-add 'cpp:*.{cpp,h}' -t cpp
```

If a caller stores `ConeTemplateCache&` as a member purely to call `get`, it may now be safely
changed to `const ConeTemplateCache&` too — but that is optional polish, not required by this task;
skip it if it would touch a file outside this task's list.

- [ ] **Step 3: `PluginLoader::tryOpen` — add `const`**

In `PluginLoader.h`:

```cpp
    [[nodiscard]] DlHandle tryOpen(const std::filesystem::path& so_path, std::string& canonical_out,
                                   std::string& error_detail) const;
```

Read `PluginLoader.cpp`'s `tryOpen` body first: confirm it only *reads* `loaded_canonical_paths_`
(per the rubric finding) and does not call `dlopen` bookkeeping that mutates `algorithms_`,
`mission_controls_`, or `handles_`. If it does mutate any of those, do not add `const` — report
instead of forcing a cast. If confirmed read-only, add `const` to the `.cpp` definition too.

- [ ] **Step 4: Build and test**

```bash
docker run --rm -e VCPKG_ROOT=/usr/local/vcpkg -v "<worktree>:/work" -w //work \
  drone-mapper-ex3-dev bash -lc '
  set -euo pipefail
  cmake --build build/default
  ctest --test-dir build/default --output-on-failure -R "Algorithm|Simulator"
'
```

- [ ] **Step 5: `verify-cell-runtime` gate + stop for approval**

`ConeTemplateCache::get` is on the scan hot path — this is the one sub-part of this task worth
double-checking for wall-time noise, but no *score* change is expected (identical logic, only
`const`-ness changed). **Stop. Wait for approval.**

- [ ] **Step 6: Commit (after approval)**

```bash
git commit -am "refactor: mark ConeTemplateCache::get and PluginLoader::tryOpen const"
```

---

### Task 5 (e08): `MappingAlgorithmImpl` — move stateless private helpers to the `.cpp`

**Rubric finding:** e08 / GUIDELINE — `MappingAlgorithmImpl.h:46` declares ~7 private helpers
(`movementToward`, `predictPose`, `buildArrivalSweep`, `targetClusterAlive`, `reachedWaypoint`,
`samePosition`, `finishIfUnmapped`, `handleReplan`, `updateProgressWindow`, `emitMovementOrScan`,
`remainingSteps`, `pruneExpiredBlockedCells`, `replan`, `adoptPlan`, `popPendingPlan`) even though
the class already uses the pimpl pattern (`std::unique_ptr<Impl> impl_`) — none of these need to be
in the public class API surface. **This is the highest-risk task in this plan** because it touches
the core `nextStep` call graph; keep the diff mechanical (pure relocation, zero logic change).

**Files:**
- Modify: `Algorithm/include/Algorithm/MappingAlgorithmImpl.h`
- Modify: `Algorithm/src/MappingAlgorithmImpl.cpp`
- Test: `Algorithm/tests/test_mapping_algorithm.cpp` — must not need any of these methods (they are
  already `private`, so if the test suite compiles today, it never called them directly). Do not
  need to touch the test file unless it uses a friend/whitebox trick — check first with
  `rg -n "friend|impl_\." Algorithm/include/Algorithm/MappingAlgorithmImpl.h
  Algorithm/tests/test_mapping_algorithm.cpp`.

**Interfaces:**
- Consumes: nothing new.
- Produces: none of these become free functions callable from outside `MappingAlgorithmImpl.cpp`
  — they move into `Impl` as private methods (since the class already forwards through `Impl`) or
  into an anonymous namespace in the `.cpp` if they take no `Impl` state. **Do not change any
  function's parameters, return type, or logic — only its enclosing scope.**

- [ ] **Step 1: Read `MappingAlgorithmImpl.cpp`'s `Impl` struct definition** to see whether `Impl`
  already declares its own methods (likely yes, since `impl_->something()` is the pimpl pattern) or
  whether the outer class's private methods are free functions taking `Impl&`/`const Impl&`
  explicitly. Match whichever pattern the `.cpp` already uses — do not introduce a second pattern.

- [ ] **Step 2: For each header-declared private method, classify it:**
  - **Stateless** (only reads its parameters, e.g. `samePosition(a, b)`, `predictPose(state,
    movement)` if it does not touch `impl_`) → move to an anonymous namespace in
    `MappingAlgorithmImpl.cpp`, drop the declaration from the header, drop the
    `MappingAlgorithmImpl_207190406_209543255::` qualifier from the definition, change call sites
    inside the `.cpp` from `this->foo(...)` / `foo(...)` (already unqualified if called from within
    the class) to the plain free-function call.
  - **Stateful** (reads/writes `impl_`'s fields) → move the declaration into the `Impl` struct
    definition inside the `.cpp` (private method of `Impl`, not of the outer class), update the
    definition's qualifier from `MappingAlgorithmImpl_207190406_209543255::foo` to `Impl::foo`,
    and update the outer class's `nextStep`/dtor to call `impl_->foo(...)` instead of `foo(...)`.

- [ ] **Step 3: Trim the header** — delete every one of these fifteen private method declarations
  from `MappingAlgorithmImpl.h`; leave `impl_`, the six `static constexpr` tuning constants, the
  public ctor/`nextStep`/dtor, and the deleted copy/move members untouched.

- [ ] **Step 4: Build after every 2–3 methods moved**, not all fifteen at once — this keeps compile
  errors attributable to the last small edit:

```bash
docker run --rm -e VCPKG_ROOT=/usr/local/vcpkg -v "<worktree>:/work" -w //work \
  drone-mapper-ex3-dev bash -lc '
  set -euo pipefail
  cmake --build build/default --target Algorithm_207190406_209543255
'
```

- [ ] **Step 5: Full test pass**

```bash
docker run --rm -e VCPKG_ROOT=/usr/local/vcpkg -v "<worktree>:/work" -w //work \
  drone-mapper-ex3-dev bash -lc '
  set -euo pipefail
  cmake --build build/default
  ctest --test-dir build/default --output-on-failure -R Algorithm
'
```

Expected: 100% of the existing `Algorithm` test suite passes with zero assertion changes — if any
test needed to change, this task stopped being a pure relocation; revert and re-scope.

- [ ] **Step 6: `verify-cell-runtime` gate + stop for approval**

Run the **Clean verify-cell-runtime procedure** (wipe `build/opt` + `tmp/bench-out`, fresh CMake
configure, full Release rebuild). Never reuse a previous `build/opt`.

Expected: **byte-identical** `score_sum` and per-cell scores/steps to baseline (pure code motion,
same compiled logic). Any score or step-count delta here is a bug — do not proceed to commit if
one appears; debug it as this task's own problem before reporting. **Stop. Wait for approval.**

- [ ] **Step 7: Commit (after approval)**

```bash
git commit -am "refactor: move MappingAlgorithmImpl's private helpers out of the class API"
```

---

### Task 6 (e10): De-duplicate `PluginLoader`'s algorithm/mission-control load flow

**Rubric finding:** e10 (m: 0, n: 1, s: 3) — `loadOneAlgorithm` and `loadOneMissionControl` are the
same dlopen → pending-factory-take → error-path flow, just against a different registrar slot.

**Files:**
- Modify: `Simulator/src/PluginLoader.cpp`
- Modify: `Simulator/include/Simulator/PluginLoader.h` only if the shared helper needs a new private
  declaration (prefer keeping it a `.cpp`-local template/lambda so the header does not change).
- Test: `Simulator/tests/test_plugin_loader.cpp` (or equivalent).

**Interfaces:**
- Consumes: Task 4's `tryOpen(...) const` (do this task after Task 4).
- Produces: no public API change — `loadOneAlgorithm`/`loadOneMissionControl` keep their existing
  signatures and behavior; only their bodies share a common private template/lambda.

- [ ] **Step 1: Read both methods' current bodies side-by-side** and identify the exact shape of
  the shared flow: `tryOpen` → on failure, build a `PluginLoadOutcome` failure with
  `error_detail`; on success, `PluginRegistrar::takePending*Factory()` → on empty optional, failure
  outcome ("plugin did not register a factory"); on success, push into `algorithms_`/
  `mission_controls_`, push the handle into `handles_`, insert into `loaded_canonical_paths_`,
  return success outcome.

- [ ] **Step 2: Write one private template method** in `PluginLoader.cpp`'s anonymous namespace or
  as a private `PluginLoader` method parameterized on the factory-take function and the
  storage vector, e.g.:

```cpp
namespace {

template <typename Loaded, typename TakeFactoryFn, typename BuildLoadedFn>
[[nodiscard]] PluginLoadOutcome loadOnePlugin(simulator::PluginLoader& self,
                                              const std::filesystem::path& so_path,
                                              std::vector<simulator::DlHandle>& handles,
                                              std::unordered_set<std::string>& loaded_paths,
                                              std::vector<Loaded>& storage,
                                              TakeFactoryFn&& take_factory,
                                              BuildLoadedFn&& build_loaded) {
    std::string canonical;
    std::string error_detail;
    simulator::DlHandle handle = self.tryOpen(so_path, canonical, error_detail);
    if (!handle) {
        return simulator::PluginLoadOutcome::failure(error_detail);
    }
    auto factory = take_factory();
    if (!factory) {
        return simulator::PluginLoadOutcome::failure(
            canonical + ": did not register a factory");
    }
    storage.push_back(build_loaded(canonical, std::move(*factory)));
    handles.push_back(std::move(handle));
    loaded_paths.insert(canonical);
    return simulator::PluginLoadOutcome::success();
}

} // namespace
```

Adjust the exact parameter/return types to match what `PluginLoadOutcome`,
`LoadedAlgorithmPlugin`/`LoadedMissionControlPlugin`, and `PluginRegistrar::takePendingAlgorithmFactory`/
`takePendingMissionControlFactory` actually declare (read
`Simulator/include/Simulator/PluginLoadTypes.h` and the registrar header before writing this — the
sketch above is the shape, not verbatim code to paste unchecked).

- [ ] **Step 3: Rewrite `loadOneAlgorithm`/`loadOneMissionControl`** to each be a 2–4 line call into
  `loadOnePlugin<...>(...)` with their own `take_factory`/`build_loaded` lambdas.

- [ ] **Step 4: Build and test** — same as Task 4 Step 4, filtered to `Simulator`.

- [ ] **Step 5: `verify-cell-runtime` gate + stop for approval**

Plugin loading happens once per run, before any mission step — expected: zero score/step change,
possibly a few ms off total wall time (noise). **Stop. Wait for approval.**

- [ ] **Step 6: Commit (after approval)**

```bash
git commit -am "refactor: share PluginLoader's algorithm/mission-control load flow"
```

---

### Task 7 (e10): One shared angle-wrap / orientation-normalize helper

**Rubric finding:** e10 (m: 0, n: 1, s: 3) — `MockLidar.cpp` has a local `wrap_deg` /
`normalize_orientation` that duplicates `beam_math::normalizeOrientation`
(`UserCommon/src/BeamMath.cpp`); `PathShaping.cpp` has a third local `wrapDeg`.

**Files:**
- Modify: `Simulator/src/MockLidar.cpp` (delete local `wrap_deg`/`normalize_orientation`, call
  `beam_math::normalizeOrientation` instead)
- Modify: `Algorithm/src/PathShaping.cpp` (delete local `wrapDeg`, call
  `beam_math::normalizeOrientation` or a shared `beam_math::wrapDeg` — see Step 1)
- Modify (if needed): `UserCommon/include/user_common_207190406_209543255/BeamMath.h` — only if
  `wrapDeg` itself (not just `normalizeOrientation`) needs to become a public function; check
  whether `PathShaping.cpp`'s usage is "wrap one angle" vs. "normalize an `Orientation`" — they may
  not be the same operation.
- Test: `UserCommon` beam-math test, `Simulator` mock-lidar test, `Algorithm` path-shaping test
  (`rg -l "wrapDeg\|normalizeOrientation" --type cpp -g "test_*"`).

**Interfaces:**
- Consumes: nothing new.
- Produces: if `wrapDeg` becomes a new public `beam_math::wrapDeg(double deg) -> double` (or
  quantity-typed) function, later tasks may reuse it — none currently plan to, but note it in the
  task's report so a future cleanup pass knows it exists.

- [ ] **Step 1: Read all three implementations** (`BeamMath.cpp::normalizeOrientation`'s internal
  wrap logic, `MockLidar.cpp`'s local `wrap_deg`/`normalize_orientation`, `PathShaping.cpp`'s local
  `wrapDeg`) and confirm they compute the same result for the same input domain (mod-360 wrap into
  `[0, 360)` or `(-180, 180]` — confirm which convention `BeamMath.cpp` uses and that the other two
  match it exactly, including tie-breaking at the boundary). **If the conventions differ even
  slightly, do not force a merge that changes behavior — report the discrepancy instead.**

- [ ] **Step 2: If `MockLidar.cpp`'s use is exactly `normalizeOrientation`'s job**, delete its local
  helpers, add `#include <user_common_207190406_209543255/BeamMath.h>`, replace call sites.

- [ ] **Step 3: If `PathShaping.cpp`'s `wrapDeg` is a lower-level primitive that
  `normalizeOrientation` calls internally but doesn't expose**, extract that primitive into
  `BeamMath.h` as `[[nodiscard]] double wrapDeg(double degrees);` (or the quantity-typed
  equivalent already used elsewhere in `BeamMath.h`), have `normalizeOrientation` call it too (no
  behavior change to `normalizeOrientation`), then have `PathShaping.cpp` call the shared one and
  delete its local copy.

- [ ] **Step 4: Build and test** — `ctest --test-dir build/default --output-on-failure -R
  "Algorithm|Simulator|UserCommon"` (adjust the `-R` filter to whatever suite names actually exist —
  check with `ctest --test-dir build/default -N` first).

- [ ] **Step 5: `verify-cell-runtime` gate + stop for approval**

This is the one task in this cluster with real hot-path exposure (`PathShaping` runs every replan;
`MockLidar` runs every scan). Expected: **no** score change if Step 1's equivalence check was
correct — a score change here means the "same convention" assumption was wrong; do not paper over
it, report and let the human decide whether to accept the small drift or revert. **Stop. Wait for
approval.**

- [ ] **Step 6: Commit (after approval)**

```bash
git commit -am "refactor: share one angle-wrap helper across MockLidar and PathShaping"
```

---

### Task 8 (e10): De-duplicate `keyToPoint`

**Rubric finding:** e10 (m: 0, n: 1, s: 3) — `ScanPlanning.cpp`'s `keyToPoint` is a copy of
`MappingAlgorithmFrontier.cpp`'s.

**Files:**
- Modify: `Algorithm/src/ScanPlanning.cpp` (delete local `keyToPoint`, call the shared one)
- Modify (if needed): `Algorithm/src/MappingAlgorithmFrontier.h` — export `keyToPoint` alongside
  the already-exported `quantizePosition` (it is currently `.cpp`-local or header-local; check
  which and export it the same way `quantizePosition` is exported: free function in
  `algorithm_207190406_209543255::detail`).
- Test: `Algorithm/tests/test_mapping_algorithm_frontier.cpp`,
  `Algorithm/tests/test_scan_planning.cpp` (or equivalent — `rg -l "keyToPoint"
  Algorithm/tests`).

**Interfaces:**
- Consumes: nothing new.
- Produces: `algorithm_207190406_209543255::detail::keyToPoint(const GridKey&, const
  common::types::MapConfig&) -> common::Position3D` becomes a header-declared free function next
  to `quantizePosition` in `MappingAlgorithmFrontier.h`.

- [ ] **Step 1: Read both `keyToPoint` implementations byte-for-byte.** They must be identical
  (same corner-anchored math as `quantizePosition`'s inverse, per the Approach A geometry fix) — if
  they differ in even one term, stop and report; do not silently pick one and call it a dedup, since
  that would be a behavior change requiring its own `verify-cell-runtime` justification, not this
  task's.

- [ ] **Step 2: Add the declaration to `MappingAlgorithmFrontier.h`**, right after
  `quantizePosition`'s declaration:

```cpp
[[nodiscard]] common::Position3D keyToPoint(const GridKey& key, const common::types::MapConfig& config);
```

Move the definition (currently duplicated) into `MappingAlgorithmFrontier.cpp` if it is not already
there, in the `algorithm_207190406_209543255::detail` namespace, unqualified free function (matching
`quantizePosition`'s existing definition style in that file).

- [ ] **Step 3: Delete `ScanPlanning.cpp`'s local `keyToPoint`**, add `#include
  "MappingAlgorithmFrontier.h"` if not already present (it likely already is, since
  `ScanPlanning.cpp` almost certainly uses `GridKey`), and update its call sites to the
  now-shared, unqualified `keyToPoint(...)` (same namespace, no qualification needed if both files
  are in `algorithm_207190406_209543255::detail`).

- [ ] **Step 4: Build and test**

```bash
docker run --rm -e VCPKG_ROOT=/usr/local/vcpkg -v "<worktree>:/work" -w //work \
  drone-mapper-ex3-dev bash -lc '
  set -euo pipefail
  cmake --build build/default
  ctest --test-dir build/default --output-on-failure -R Algorithm
'
```

- [ ] **Step 5: `verify-cell-runtime` gate + stop for approval**

Expected: byte-identical scores (Step 1's equivalence check makes this a pure dedup). **Stop. Wait
for approval.**

- [ ] **Step 6: Commit (after approval)**

```bash
git commit -am "refactor: share one keyToPoint between ScanPlanning and MappingAlgorithmFrontier"
```

---

### Task 9 (e03): `MockMovement` — use `mp_units::abs` instead of unwrapping to `double`

**Rubric finding:** e03 (s: 2 — no m/n column) — `rotate`/`advance`/`elevate`'s limit checks
unwrap to `std::abs` on a raw `double` just to compare magnitude against a quantity limit.

**Files:**
- Modify: `Simulator/src/MockMovement.cpp`
- Test: `Simulator/tests/test_mock_movement.cpp` (or equivalent).

**Interfaces:** none — this is a pure internal rewrite of three limit checks; `MockMovement`'s
public methods (`rotate`, `advance`, `elevate`) keep their exact signatures and return values.

- [ ] **Step 1: Check what `DroneControlImpl` (or wherever the rubric review's "DroneControl
  already does this" comment points) actually does**, to match the established quantity-abs idiom
  in this codebase exactly:

```bash
rg -n "mp_units::abs|abs(" MissionControl/src/DroneControlImpl.cpp
```

- [ ] **Step 2: Rewrite `rotate`'s check.** Current:

```cpp
    const double deg_val = angle.numerical_value_in(deg);
    const double max_deg = drone_.max_rotate.numerical_value_in(deg);
    if (std::abs(deg_val) > max_deg) {
        return {false, "rotate: angle exceeds max_rotate"};
    }
```

New (keep the exact same message strings and `MovementResult` shape — only the comparison changes):

```cpp
    if (mp_units::abs(angle) > drone_.max_rotate) {
        return {false, "rotate: angle exceeds max_rotate"};
    }
```

Confirm `mp_units::abs` accepts `common::HorizontalAngle` and that `angle`/`drone_.max_rotate` are
directly comparable (same quantity type) before deleting the unwrap — if a `quantity_cast` is
needed, add it, don't force a comparison that won't compile.

- [ ] **Step 3: Rewrite `advance`'s check** the same way with `common::PhysicalLength`:

```cpp
    if (mp_units::abs(distance) > drone_.max_advance) {
        return {false, "advance: distance exceeds max_advance"};
    }
```

Keep `dist_cm` further down in `advance` **only if it is still used** for the trig-based new-position
math below the limit check (it is, per the current file — `MockMovement.cpp`'s trig block is e03
but explicitly deferred in this plan's "Deferred findings" table; do not touch it in this task).
Recompute `dist_cm` from `distance.numerical_value_in(cm)` right where the trig block needs it, or
leave the existing `dist_cm` line in place below the now-simplified limit check — whichever keeps
the diff smallest.

- [ ] **Step 4: Rewrite `elevate`'s check** the same way with `common::PhysicalLength` /
  `drone_.max_elevate`, same caveat about keeping `dist_cm` for the deferred trig block below it.

- [ ] **Step 5: Build and test**

```bash
docker run --rm -e VCPKG_ROOT=/usr/local/vcpkg -v "<worktree>:/work" -w //work \
  drone-mapper-ex3-dev bash -lc '
  set -euo pipefail
  cmake --build build/default
  ctest --test-dir build/default --output-on-failure -R Simulator
'
```

Expected: identical pass/fail behavior on the existing rotate/advance/elevate-limit tests
(boundary-exact-equal cases matter here — confirm the test suite already covers `== limit` exactly
at the boundary, since `mp_units::abs(x) > limit` must reject/accept the same boundary cases as
`std::abs(x_cm) > limit_cm` did).

- [ ] **Step 6: `verify-cell-runtime` gate + stop for approval**

Run the **Clean verify-cell-runtime procedure** (wipe `build/opt` + `tmp/bench-out`, fresh CMake
configure, full Release rebuild). Never reuse a previous `build/opt`.

Expected: identical scores (same comparison, just not unwrapped). **Stop. Wait for approval.**

- [ ] **Step 7: Commit (after approval)**

```bash
git commit -am "refactor: compare MockMovement's limit checks as quantities, not raw doubles"
```

---

### Task 10 (e03): `SimulationRunFactoryImpl::outputMapConfig` — quantity division instead of unwrap

**Rubric finding:** e03 (s: 2) — `(sim.map_resolution.force_numerical_value_in(cm) / factor) *
cm` unwraps just to divide by a plain `double` factor.

**Files:**
- Modify: `Simulator/src/SimulationRunFactoryImpl.cpp`
- Test: `Simulator/tests/test_simulation_run_factory_impl.cpp` (or equivalent — `rg -l
  "outputMapConfig" Simulator/tests`).

**Interfaces:** none — internal helper, same return value for the same input.

- [ ] **Step 1: Read the surrounding lines exactly** (shown in the exploration above):

```cpp
    const double factor = mission.output_mapping_resolution_factor >= 1.0
                              ? mission.output_mapping_resolution_factor
                              : 1.0;
    config.resolution =
        (sim.map_resolution.force_numerical_value_in(cm) / factor) * cm;
```

- [ ] **Step 2: Rewrite as a direct quantity division:**

```cpp
    const double factor = mission.output_mapping_resolution_factor >= 1.0
                              ? mission.output_mapping_resolution_factor
                              : 1.0;
    config.resolution = sim.map_resolution / factor;
```

Confirm `common::PhysicalLength / double` returns `common::PhysicalLength` directly (it should,
per mp-units' scalar-division operator) — if the compiler disagrees (e.g. it returns a different
but equivalent quantity type that needs `quantity_cast`), add the minimal cast, don't force
`force_numerical_value_in` back in.

- [ ] **Step 3: Build and test**

```bash
docker run --rm -e VCPKG_ROOT=/usr/local/vcpkg -v "<worktree>:/work" -w //work \
  drone-mapper-ex3-dev bash -lc '
  set -euo pipefail
  cmake --build build/default
  ctest --test-dir build/default --output-on-failure -R Simulator
'
```

Expected: identical `config.resolution` values (floating-point division associativity is the same
whether or not you round-trip through `cm` — verify with the existing test's exact assertions, not
just "it compiles").

- [ ] **Step 4: `verify-cell-runtime` gate + stop for approval**

This value feeds every output map's resolution — a real risk spot for silent map-shape drift.
Expected: identical scores; if not, do not proceed, this is the one place in this task cluster
where "just compiles" isn't enough evidence. **Stop. Wait for approval.**

- [ ] **Step 5: Commit (after approval)**

```bash
git commit -am "refactor: compute output map resolution as a quantity division"
```

---

### Task 11 (e04): Replace C-style neighbor/axis arrays with `std::array`

**Rubric finding:** e04 (m: 0, n: 1, s: 3) — five files use C arrays (`kFaceOffsets[6]`,
`const double dx[4]`/`dy[4]`, `const Orientation axes[]`, `static const int kDx[6]`/`kDy`/`kDz`)
where `std::array` is the idiomatic choice.

**Files:**
- Modify: `Algorithm/src/ScanPlanning.cpp` (`kFaceOffsets[6]`)
- Modify: `Algorithm/src/MappingAlgorithmFrontier.cpp` (its 6-neighbor table)
- Modify: `Algorithm/src/WavefrontPlanner.cpp` (`dx[4]`/`dy[4]`)
- Modify: `UserCommon/src/LidarCone.cpp` (`axes[]`)
- Modify: `Simulator/src/MapsComparison.cpp` (`kDx`/`kDy`/`kDz`)
- Test: whichever tests already cover neighbor iteration for these files — no test *content*
  change expected, only confirm they still pass (`std::array`'s iteration and indexing are
  drop-in compatible with a C array's).

**Interfaces:** none — this is purely a container-type swap; every one of these tables is consumed
by a range-`for` or indexed loop that works identically over `std::array<T, N>`.

- [ ] **Step 1: `ScanPlanning.cpp`** — find `kFaceOffsets`'s current declaration
  (`rg -n "kFaceOffsets" Algorithm/src/ScanPlanning.cpp`). If it is e.g.:

```cpp
constexpr Offset kFaceOffsets[6] = { /* ... */ };
```

change to:

```cpp
constexpr std::array<Offset, 6> kFaceOffsets = { /* ... same initializer ... */ };
```

Add `#include <array>` if not already present. Leave every element value untouched — copy the
existing initializer list verbatim.

- [ ] **Step 2: `MappingAlgorithmFrontier.cpp`** — same transformation for its 6-neighbor offset
  table (`rg -n "static const|constexpr.*\[6\]|\[4\]" Algorithm/src/MappingAlgorithmFrontier.cpp`
  to find it precisely; it uses the same `{dx, dy, dz}` face-offset shape as `ScanPlanning.cpp`'s —
  confirm whether it's worth sharing one table between the two files as a bonus, but do **not**
  expand this task's scope to a cross-file share unless it is a trivial one-line change; if it
  needs new plumbing, leave the two tables separate and just convert each to `std::array` in place).

- [ ] **Step 3: `WavefrontPlanner.cpp`** — find:

```cpp
const double dx[4] = { /* ... */ };
const double dy[4] = { /* ... */ };
```

change to:

```cpp
constexpr std::array<double, 4> dx = { /* ... same values ... */ };
constexpr std::array<double, 4> dy = { /* ... same values ... */ };
```

(Add `constexpr` here only if the original values are literal constants with no runtime
dependency — check first; if they depend on a runtime value, keep them non-`constexpr` but still
`std::array`.)

- [ ] **Step 4: `LidarCone.cpp`** — find:

```cpp
const Orientation axes[] = { /* ... */ };
```

Count the initializer's elements and change to:

```cpp
constexpr std::array<Orientation, N> axes = { /* ... same values ... */ };
```

with `N` replaced by the actual literal count (do not use `axes[]`'s auto-sizing — `std::array`
needs an explicit or deduced size; use `std::to_array` instead if you'd rather not count by hand:
`constexpr auto axes = std::to_array<Orientation>({ /* ... */ });`, requires `#include <array>` and
C++20, which this project already targets).

- [ ] **Step 5: `MapsComparison.cpp`** — find:

```cpp
static const int kDx[6] = { /* ... */ };
static const int kDy[6] = { /* ... */ };
static const int kDz[6] = { /* ... */ };
```

change to three `constexpr std::array<int, 6>` (drop `static` — file-scope `constexpr` already has
internal linkage in a `.cpp`, so `static` is redundant once you switch; if the codebase convention
elsewhere keeps `static constexpr` together, match that convention instead of fighting it — check
`rg -n "static constexpr" Simulator/src/MapsComparison.cpp` for the file's own precedent first).

- [ ] **Step 6: Build after each file**, not all five at once:

```bash
docker run --rm -e VCPKG_ROOT=/usr/local/vcpkg -v "<worktree>:/work" -w //work \
  drone-mapper-ex3-dev bash -lc '
  set -euo pipefail
  cmake --build build/default --target Algorithm_207190406_209543255 simulator_207190406_209543255
'
```

- [ ] **Step 7: Full test pass**

```bash
ctest --test-dir build/default --output-on-failure -R "Algorithm|Simulator|UserCommon"
```

- [ ] **Step 8: `verify-cell-runtime` gate + stop for approval**

Expected: identical scores (every table's *values* are unchanged; only the container type moved
from a raw array to `std::array`, which has identical layout/iteration semantics). **Stop. Wait
for approval.**

- [ ] **Step 9: Commit (after approval)**

```bash
git commit -am "refactor: replace C-style neighbor/axis tables with std::array"
```

---

### Task 12 (e17): Forward-declare heavy interface headers

**Rubric finding:** e17 (m: 0, n: 1, s: 3) — `SimulationRunImpl.h` includes six complete `I*`
headers only to destroy `unique_ptr` members; `DroneControlImpl.h` includes five for reference
members; `PluginLoader.h` includes `<dlfcn.h>` in a public header only for `DlCloser::dlclose`.

**Files:**
- Modify: `Simulator/include/Simulator/SimulationRunImpl.h`
- Modify: `Simulator/src/SimulationRunImpl.cpp`
- Modify: `MissionControl/include/MissionControl/DroneControlImpl.h`
- Modify: `Simulator/include/Simulator/PluginLoader.h`
- Modify: `Simulator/src/PluginLoader.cpp`
- Test: existing suites for all three files — no test changes expected (this only moves *where*
  a type becomes complete, not any runtime behavior).

**Interfaces:** none — same public API, same behavior. `SimulationRunImpl`'s destructor moves
from implicitly-defined-in-header to explicitly-declared-in-header/defined-in-`.cpp` (required for
the forward-declare + `unique_ptr` pattern to compile — `unique_ptr<T>`'s destructor needs `T`
complete at the point the *owning* type's destructor is instantiated, which must now be in the
`.cpp` where the real headers are included).

- [ ] **Step 1: `SimulationRunImpl.h`** — for each `unique_ptr<I...>` member that is only
  destroyed (never otherwise dereferenced in the header, e.g. inline getters that return the raw
  pointer are fine, but anything calling a method on `*member_` in the header is not), replace
  `#include <Common/I....h>` with a forward declaration:

```cpp
namespace common {
class IGPS;
class ILidar;
class IDroneMovement;
class IMutableMap3D;
class IMappingAlgorithm;
} // namespace common
```

(exact list depends on what's actually there — read the file first; do not blanket-forward-declare
a type that the header also uses by value or needs a complete definition for, e.g. any type used in
an inline method body in the header itself).

Add a declared-but-not-defined destructor to the class:

```cpp
    ~SimulationRunImpl() override;
```

- [ ] **Step 2: `SimulationRunImpl.cpp`** — add back the full `#include`s for whichever headers
  the `.cpp` now needs (it already almost certainly includes most of them, since it calls real
  methods on these interfaces), and define the destructor:

```cpp
SimulationRunImpl::~SimulationRunImpl() = default;
```

placed after every member's real type is complete (i.e. after the `#include`s, anywhere in the
`.cpp`, conventionally right after the ctor).

- [ ] **Step 3: `DroneControlImpl.h`** — same pattern for its five reference members. **Caveat:**
  Task 2 already changed `lidar_sensor_`/`gps_` to `const common::ILidar&`/`const common::IGPS&`
  — do this task *after* Task 2 is committed so the forward-declare list matches the then-current
  member types. A reference member does not need a declared destructor (no ownership, nothing to
  destroy) — only `unique_ptr` members force that. Forward-declare all five interface types used
  only as reference members; add the destructor-declaration dance only if a `unique_ptr` member
  also exists in this class (check first — from the earlier read, `DroneControlImpl` has no
  `unique_ptr` members, only references, so **no destructor change is needed here**, only the
  forward declarations plus keeping `#include <MissionControl/IDroneControl.h>` since that's the
  base class, which must stay complete).

- [ ] **Step 4: `PluginLoader.h`** — move `#include <dlfcn.h>` out of the header. `DlCloser`'s
  `operator()` calls `::dlclose`, so either:
  - (a) forward-declare nothing and instead give `DlCloser` a non-inline `operator()` declared in
    the header, defined in `PluginLoader.cpp` (which already includes `<dlfcn.h>` or now needs to):

```cpp
// PluginLoader.h
struct DlCloser {
    void operator()(void* handle) const noexcept;
};
```

```cpp
// PluginLoader.cpp
#include <dlfcn.h>
// ...
void DlCloser::operator()(void* handle) const noexcept {
    if (handle != nullptr) {
        ::dlclose(handle);
    }
}
```

- [ ] **Step 5: Build after each of the three files**, one at a time:

```bash
docker run --rm -e VCPKG_ROOT=/usr/local/vcpkg -v "<worktree>:/work" -w //work \
  drone-mapper-ex3-dev bash -lc '
  set -euo pipefail
  cmake --build build/default
'
```

Forward-declaration mistakes show up as compile errors immediately (incomplete-type use) — fix by
either adding back one specific include in the `.h` (if the header truly needs the complete type,
e.g. an inline method calls something on it) or moving the offending inline method's body into the
`.cpp` instead of over-including.

- [ ] **Step 6: Full test pass**

```bash
ctest --test-dir build/default --output-on-failure -R "Simulator|MissionControl"
```

- [ ] **Step 7: `verify-cell-runtime` gate + stop for approval**

Expected: identical scores (compile-time-only change). **Stop. Wait for approval.**

- [ ] **Step 8: Commit (after approval)**

```bash
git commit -am "refactor: forward-declare interface headers instead of including them"
```

---

### Task 13 (e21): `ScanResultToVoxels` — one walk instead of two

**Rubric finding:** e21 (m: 0, n: 1, s: 3) — `applyScanToMap` traces every beam, then
`supplementGridAlignedFusion` traces again over the same scan.

**Files:**
- Modify: `MissionControl/src/ScanResultToVoxels.cpp`
- Test: `MissionControl/tests/test_scan_result_to_voxels.cpp` (or equivalent).

**Interfaces:** none — `applyScanToMap`'s public signature and the resulting map contents must be
**byte-identical** to before; this task only changes *how many times* the scan is walked, not
*what* gets written to the map.

- [ ] **Step 1: Read both functions completely** and write down, in the task report (not in code),
  the exact order of operations each currently performs per beam: what `applyScanToMap`'s first
  pass writes (likely per-beam-hit voxel marking) and what `supplementGridAlignedFusion`'s second
  pass adds (likely grid-aligned samples along the beam that the raw hit walk skips). Confirm
  whether the second pass's writes ever depend on the first pass having already completed for a
  *different* beam (e.g. "don't grid-align-fuse a voxel some other beam already marked Occupied
  this scan") — if such a cross-beam ordering dependency exists, merging the two walks into one
  per-beam pass could change results at voxel boundaries where two beams disagree. **If you find
  this dependency, stop and report — do not merge.** If both passes are purely per-beam-independent
  (each beam's grid-aligned fusion only reads/writes that beam's own path), proceed.

- [ ] **Step 2: Merge into one per-beam walk.** Restructure `applyScanToMap` so that, for each
  beam, it performs the original hit-walk step immediately followed by that beam's
  grid-aligned-fusion step, before moving to the next beam — instead of looping over all beams
  twice. Delete `supplementGridAlignedFusion` as a separate second pass and inline its per-beam
  logic into the same loop (keep it as a private helper function taking one beam's data, just
  called once per beam inside the merged loop instead of once per beam in its own separate loop).

- [ ] **Step 3: Build and test**

```bash
docker run --rm -e VCPKG_ROOT=/usr/local/vcpkg -v "<worktree>:/work" -w //work \
  drone-mapper-ex3-dev bash -lc '
  set -euo pipefail
  cmake --build build/default
  ctest --test-dir build/default --output-on-failure -R MissionControl
'
```

Expected: **every** existing test on `applyScanToMap`/map contents passes with identical
assertions — this is the strongest signal that Step 1's independence check was correct.

- [ ] **Step 4: `verify-cell-runtime` gate + stop for approval**

This is a real hot-path change (every scan, every step that scans). Expected: identical scores; a
small wall-time *improvement* is plausible (one fewer full walk per scan) and welcome, but is not
the goal — do not chase it if it doesn't materialize. Any score change means Step 1's independence
assumption was wrong somewhere the tests didn't catch — do not proceed, report instead. **Stop.
Wait for approval.**

- [ ] **Step 5: Commit (after approval)**

```bash
git commit -am "perf: fuse grid-aligned scan supplementing into the primary beam walk"
```

---

### Task 14 (e23): Name the bare magic numbers the rubric flagged

**Rubric finding:** e23 (m: 0, n: 1, s: 1) — several bare numeric literals encode domain rules
without a named constant.

**Files:**
- Modify: `Simulator/src/io/DroneConfigYamlParser.cpp` (`* v / 2.0` diameter→radius)
- Modify: `Algorithm/src/ScanPlanning.cpp` (bare `90.0 * deg` / `-90.0 * deg` zenith/nadir probes;
  also its bare `1e-6` ceiling-test epsilon)
- Modify: `MissionControl/src/ScanResultToVoxels.cpp` (bare `3`/`2`/`1`/`0` occupancy ranks)
- Modify: `Simulator/src/main.cpp` and `Simulator/src/SimulationRunImpl.cpp` (and
  `Simulator/src/RunMatrixOrchestrator.cpp`) — share one named `kErrorScore = -1.0` instead of
  three independent bare `-1`/`-1.0` sentinels
- Modify: `Algorithm/src/MappingAlgorithmImpl.cpp` — its `movementToward` already has
  `kPositionEpsilon` declared at a nearby line but uses a second bare `1e-6` instead of reusing it
- Modify: `Algorithm/src/WavefrontPlanner.cpp` (bare `1e-6` ceiling epsilon, and bare `1.0`
  `expected_rate` for forced/escape plans, twice)
- Test: no test file should need changes (named constants have the same value as the literals they
  replace) — but re-run every affected suite to confirm.

**Interfaces:** none — pure literal-to-named-constant substitution, identical numeric values.

**Do this task as several small, self-contained edits in one commit** (they are all the same kind
of change, low individual risk, and splitting each numeric literal into its own
task/commit/verify-cell-runtime cycle would be nine near-identical gates for one rubric line) —
but still run the tests and the one `verify-cell-runtime` gate for the whole batch before
committing.

- [ ] **Step 1: `DroneConfigYamlParser.cpp`** — find the diameter→radius line
  (`rg -n "/ 2" Simulator/src/io/DroneConfigYamlParser.cpp`). If it reads roughly:

```cpp
result.radius = diameter_value / 2.0;
```

introduce a file-local named constant in an anonymous namespace at the top of the file:

```cpp
namespace {
constexpr double kDiameterToRadius = 2.0;
} // namespace
```

and use `diameter_value / kDiameterToRadius`. (If the surrounding code already works in typed
`PhysicalLength` quantities rather than raw `double`, keep the constant typed the same way — do
not introduce an unwrap that wasn't there before.)

- [ ] **Step 2: `ScanPlanning.cpp` zenith/nadir** — find the travel-probe lines using bare `90.0 *
  deg` / `-90.0 * deg` (`rg -n "90.0 \* deg" Algorithm/src/ScanPlanning.cpp`). Add near the top of
  the file (anonymous namespace, alongside any other file-local constants already there):

```cpp
namespace {
constexpr common::VerticalAngle kZenith = 90.0 * common::deg;
constexpr common::VerticalAngle kNadir = -90.0 * common::deg;
} // namespace
```

(Use whichever angle-quantity alias the surrounding code already uses for these probes — check the
variable's declared type at the call site rather than assuming `VerticalAngle`.) Replace both bare
literals with `kZenith`/`kNadir`.

- [ ] **Step 3: `ScanPlanning.cpp`'s bare `1e-6`** — if `WavefrontPlanner.cpp` (Step 6 below)
  ends up needing the *same* epsilon value for the *same* purpose (ceiling-height comparison), and
  both files already share other constants via a common header, consider one shared
  `kHeightEpsilonCm` — but only if there is already a shared constants header both files include;
  do **not** create a new shared header just for this. Otherwise, add a file-local
  `kCeilingHeightEpsilonCm = 1e-6` (or typed equivalent) in each file independently.

- [ ] **Step 4: `ScanResultToVoxels.cpp` occupancy ranks** — find the bare `3`/`2`/`1`/`0` priority
  literals (`rg -n "\b[0-3]\b" MissionControl/src/ScanResultToVoxels.cpp` — noisy, narrow to the
  actual rank-assignment lines by reading the file). Name them for what they mean, e.g.:

```cpp
namespace {
constexpr int kOccupiedRank = 3;
constexpr int kEmptyRank = 2;
constexpr int kUnmappedRank = 1;
constexpr int kOutOfBoundsRank = 0;
} // namespace
```

(Match the actual semantics — confirm which literal maps to which `VoxelOccupancy` value by reading
the surrounding code; do not guess the mapping shown here without checking.)

- [ ] **Step 5: Shared `kErrorScore`** — `main.cpp` and `SimulationRunImpl.cpp` (and
  `RunMatrixOrchestrator.cpp`, per the finding) each declare their own `-1`/`-1.0` sentinel for a
  failed run's score. Pick one location to own the canonical constant — the most natural home is
  wherever `SimulationResult`/`MissionRunResult`'s score field is first produced on the
  error path, e.g. a new constant in `Simulator/include/Simulator/RunMatrixTypes.h` (already
  included by all three files, per the earlier exploration) or a small existing shared header:

```cpp
/// Score assigned to a run that failed before or during mission execution
/// (startup error, uncaught exception, or an ErrorRef-producing scenario).
inline constexpr double kErrorScore = -1.0;
```

Replace every bare `-1`/`-1.0` score sentinel in `main.cpp`, `SimulationRunImpl.cpp`, and
`RunMatrixOrchestrator.cpp` with `kErrorScore` (add the include where needed). **Do not** touch any
`-1` that means something else in these files (e.g. an unrelated index sentinel) — only the score
sentinel.

- [ ] **Step 6: `MappingAlgorithmImpl.cpp`'s duplicate epsilon** — find where `kPositionEpsilon` is
  already declared (per the finding, nearby) and where `movementToward` uses a second bare `1e-6`
  instead. Replace the bare literal with `kPositionEpsilon` — **only if** both epsilons are used for
  the same kind of comparison (position-closeness); if `movementToward`'s `1e-6` is actually a
  different unit/purpose (e.g. an angle epsilon, not a position epsilon), do not force the reuse —
  add its own named constant instead and report the distinction.

- [ ] **Step 7: `WavefrontPlanner.cpp`** — name its bare `1e-6` ceiling epsilon (same pattern as
  Step 3/6) and its bare `1.0` `expected_rate` for forced/escape plans (appears twice):

```cpp
namespace {
constexpr double kForcedEscapeExpectedRate = 1.0;
} // namespace
```

Replace both occurrences.

- [ ] **Step 8: Build and full test pass**

```bash
docker run --rm -e VCPKG_ROOT=/usr/local/vcpkg -v "<worktree>:/work" -w //work \
  drone-mapper-ex3-dev bash -lc '
  set -euo pipefail
  cmake --build build/default
  ctest --test-dir build/default --output-on-failure
'
```

- [ ] **Step 9: `verify-cell-runtime` gate + stop for approval**

Expected: byte-identical scores — every constant carries the exact same numeric value as the
literal it replaced. **Stop. Wait for approval.**

- [ ] **Step 10: Commit (after approval)**

```bash
git commit -am "refactor: name magic numbers flagged by the AdvCpp rubric review (e23)"
```

---

### Task 15 (GUIDELINE): Guard `CompositionYamlParser` against non-scalar / partially-failed YAML

**Finding:** GUIDELINE (from the AdvCpp Review Guideline's "invalid config → graceful exit, not a
crash") — `parseOneSimulationGroup`'s `entry["simulation_config"].as<std::string>()` has no
`IsScalar()` guard and no `try`/`catch`, so a non-scalar `simulation_config` value throws
`YAML::Exception` out of `main` uncaught; separately, failed drone/lidar-config parses inside
`parseCompositionFile` still get pushed into `composition.drone_configs`/`lidar_configs` and
`result.ok` is left `true` even when a nested parse failed.

**Files:**
- Modify: `Simulator/src/io/CompositionYamlParser.cpp`
- Test: `Simulator/tests/test_composition_yaml_parser.cpp` (or equivalent — `rg -l
  "parseCompositionFile\|parseOneSimulationGroup" Simulator/tests`) — **add** two new test cases
  (this task needs new test coverage since it's fixing a real gap, not moving existing logic).

**Interfaces:** `parseCompositionFile`'s signature is unchanged. Its behavior changes: a
composition file with a non-scalar `simulation_config` value now produces a `COMPOSITION_INVALID`
`ConfigParseResult` (via `detail::logRecoverable` + `result.errors.push_back(...)`, matching this
file's existing error-reporting idiom) instead of throwing; a composition file where any listed
drone/lidar config fails to parse now reports `ok = false` with an error entry, instead of silently
including a default-constructed config.

- [ ] **Step 1: Write the failing tests first (TDD).** Add to the composition-parser test file:

```cpp
TEST(CompositionYamlParserTest, NonScalarSimulationConfigIsRejectedGracefully) {
    // Write a temp composition YAML where simulation_config is a mapping/sequence,
    // not a scalar path string, e.g.:
    //   simulation_compositions:
    //     simulations:
    //       - simulation_config: {nested: true}
    //         mission_configs: [mission.yaml]
    //     drone_configs: [drone.yaml]
    //     lidar_configs: [lidar.yaml]
    // (Use this test file's existing temp-file helper if one exists — check for a
    // fixture like `writeTempYaml` before adding a new one.)
    FakeRunErrorLog log;
    const auto result = parseCompositionFile(temp_path, log);
    EXPECT_FALSE(result.ok);
    // At minimum, must not throw — the EXPECT_FALSE above already proves that,
    // since a throw would abort the test with an unhandled exception instead of
    // reaching this assertion.
}

TEST(CompositionYamlParserTest, FailedDroneConfigParseMakesCompositionNotOk) {
    // Write a temp composition YAML whose drone_configs entry points at a file
    // that fails to parse as a drone config (e.g. missing required fields, or
    // reuse this test file's existing "invalid drone config" fixture if one
    // exists).
    FakeRunErrorLog log;
    const auto result = parseCompositionFile(temp_path, log);
    EXPECT_FALSE(result.ok);
}
```

Adapt the exact fixture-writing mechanics (temp file helpers, `FakeRunErrorLog` or whatever mock
implements `IRunErrorLog` in this test file already) to match this file's existing conventions —
read the file first; do not invent a second temp-file helper if one exists.

- [ ] **Step 2: Run the new tests and confirm they fail** (first one either throws uncaught or
  currently returns `ok = true`; second currently returns `ok = true` because the finding says so).

```bash
ctest --test-dir build/default --output-on-failure -R CompositionYamlParser
```

Expected: FAIL (or crash) on both new tests.

- [ ] **Step 3: Fix `parseOneSimulationGroup`'s non-scalar guard.** Current:

```cpp
    if (!entry["simulation_config"]) {
        detail::logRecoverable(log, "COMPOSITION_INVALID",
                               "[simulation_compositions] entry missing simulation_config — skipped");
        return false;
    }

    const auto sim_path =
        resolveConfigPath(base_dir, entry["simulation_config"].as<std::string>());
```

New:

```cpp
    if (!entry["simulation_config"] || !entry["simulation_config"].IsScalar()) {
        detail::logRecoverable(log, "COMPOSITION_INVALID",
                               "[simulation_compositions] entry missing or non-scalar "
                               "simulation_config — skipped");
        return false;
    }

    const auto sim_path =
        resolveConfigPath(base_dir, entry["simulation_config"].as<std::string>());
```

- [ ] **Step 4: Fix the drone/lidar-config parse-failure gap in `parseCompositionFile`.** Current:

```cpp
    for (const auto& p : drone_paths) {
        composition.drone_configs.push_back(
            parseDroneConfig(resolveConfigPath(base_dir, p), log).value);
    }
    for (const auto& p : lidar_paths) {
        composition.lidar_configs.push_back(
            parseLidarConfig(resolveConfigPath(base_dir, p), log).value);
    }

    result.ok    = true;
    result.value = std::move(composition);
    return result;
```

New:

```cpp
    for (const auto& p : drone_paths) {
        const auto drone_result = parseDroneConfig(resolveConfigPath(base_dir, p), log);
        if (!drone_result.ok) {
            result.errors.push_back({"COMPOSITION_INVALID",
                                      "Failed to parse drone_config \"" +
                                          resolveConfigPath(base_dir, p).string() + "\""});
            return result;
        }
        composition.drone_configs.push_back(drone_result.value);
    }
    for (const auto& p : lidar_paths) {
        const auto lidar_result = parseLidarConfig(resolveConfigPath(base_dir, p), log);
        if (!lidar_result.ok) {
            result.errors.push_back({"COMPOSITION_INVALID",
                                      "Failed to parse lidar_config \"" +
                                          resolveConfigPath(base_dir, p).string() + "\""});
            return result;
        }
        composition.lidar_configs.push_back(lidar_result.value);
    }

    result.ok    = true;
    result.value = std::move(composition);
    return result;
```

Match `ConfigParseResult`'s actual error-entry type (confirm the `{code, message}` shape against
`UserCommon/include/user_common_207190406_209543255/ConfigParseResult.h` before writing — the
snippet above assumes a two-field aggregate matching the existing `result.errors.push_back({...})`
calls already in this same file).

- [ ] **Step 5: Run the new tests again — confirm both pass.** Then run the full existing suite to
  confirm no regression on any composition file that currently parses successfully:

```bash
ctest --test-dir build/default --output-on-failure -R "CompositionYamlParser|Simulator"
```

- [ ] **Step 6: `verify-cell-runtime` gate + stop for approval**

Run the **Clean verify-cell-runtime procedure** (wipe `build/opt` + `tmp/bench-out`, fresh CMake
configure, full Release rebuild). Never reuse a previous `build/opt`.

All 24 of this project's real `sim_compose.yaml` cells presumably already have well-formed,
all-scalar `simulation_config` values and successfully-parsing drone/lidar configs — expected: zero
change to the 24-cell table (this task only changes behavior on inputs the current suite doesn't
exercise). **Stop. Wait for approval.**

- [ ] **Step 7: Commit (after approval)**

```bash
git commit -am "fix: reject non-scalar simulation_config and propagate nested config parse failures"
```

---

### Task 16 (e14/e15): Fix HLD class and sequence diagram drift

**Rubric finding:** e14 (m: 0, n: 1, s: 3) / e15 (m: 0, n: 1, s: 3) — the class diagram
(`docs/hld/class-overview.mmd`) omits `IRunErrorLog`, `PluginMatrixBinding`/`PluginMatrixResult`,
`PathShaping`/`ScanPlanning`/`ExplorationPlan`, `WavefrontPlanner`'s composition of
`MappingAlgorithmFrontier`, and the factory-held `MappingAlgorithmFactory`/`MissionControlFactory`
on `SimulationRunFactoryImpl`; the sequence diagrams
(`docs/hld/seq-comparative-cell.mmd`, `docs/hld/seq-drone-step.mmd`) draw `distributeWork` inside a
per-cell loop (live code calls it once over the whole matrix) and show the recoverable-throw path
as "no scan write" (live code still scans after a recoverable `Continue`).

**Files:**
- Modify: `docs/hld/class-overview.mmd`
- Modify: `docs/hld/seq-comparative-cell.mmd`
- Modify: `docs/hld/seq-drone-step.mmd`
- Modify: `docs/HLD.md` (any prose that describes the now-fixed drift, e.g. the MockMovement
  "returns a message containing blocked/boundary" line, which should say "throws
  `std::runtime_error`" to match `MockMovement.cpp`'s actual behavior)
- Regenerate: `docs/hld/class-overview.png`, `docs/hld/seq-comparative-cell.png`,
  `docs/hld/seq-drone-step.png`, and `HLD.pdf` at the repo root — locate the render script
  (`rg -l "class-overview" scripts/ 2>/dev/null` or check `docs/HLD.md`'s own instructions for how
  these were last generated) and re-run it. **This task has no code files and needs no
  `verify-cell-runtime` gate in the "does the algorithm still work" sense — run it anyway per this
  plan's Execution Mode, expect a trivial pass-through confirmation.**

**Interfaces:** none — documentation only.

- [ ] **Step 1: Class diagram** — open `docs/hld/class-overview.mmd` and add:
  - `IRunErrorLog` as an interface, with `RunErrorLog ..|> IRunErrorLog` (find `RunErrorLog`'s
    existing node and add the realization arrow plus the new interface node next to it, following
    this file's existing style for other `I*` interfaces).
  - `PluginMatrixBinding` and `PluginMatrixResult` as classes near `MatrixCell`, with
    `PluginMatrixBinding --> ISimulationRunFactory` (read
    `Simulator/include/Simulator/RunMatrixTypes.h` first to get their actual field names right if
    the diagram shows fields).
  - `PathShaping`, `ScanPlanning`, `ExplorationPlan` as classes/modules near `WavefrontPlanner`,
    with `WavefrontPlanner --> MappingAlgorithmFrontier` (composition, not sibling — read
    `Algorithm/src/WavefrontPlanner.h`'s `frontier_` member, already shown above in this plan, to
    confirm the relationship type).
  - `SimulationRunFactoryImpl --> MappingAlgorithmFactory` and `--> MissionControlFactory` edges
    (read `Simulator/src/SimulationRunFactoryImpl.h`/`.cpp` for the actual member names).
  - Move `PluginLoader::unloadAll()` off the `main` box onto `PluginLoader`'s own box if it is
    currently misattributed (per the rubric review's finding), keeping `main --> PluginLoader` as
    the calling edge.

- [ ] **Step 2: Comparative-cell sequence** — open `docs/hld/seq-comparative-cell.mmd` and:
  - Move the `distributeWork` call outside the per-cell loop: draw one `expandRunMatrix` call that
    produces the full cell list, then one `distributeWork` call over that whole list, rather than a
    loop containing `distributeWork` per cell (read `Simulator/src/RunMatrixOrchestrator.cpp`'s
    `runPluginMatrix` to confirm the exact call order before redrawing).
  - Show nested YAML parsing under `parseCompositionFile` (a note or a few nested calls to
    `parseSimulationConfig`/`parseMissionConfig`/`parseDroneConfig`/`parseLidarConfig`) instead of
    only showing the top-level call.
  - Add the `Loader->>Reg: takePending*Factory` step and the `Main->>Factory: ctor(algo, mc)` /
    `PluginMatrixBinding` construction step between "load plugins" and "run matrix".
  - Insert the per-run `RunErrorLog` setup (`errorLogPathFromOutputMap`) before `runMission`, with
    an alt/opt block for the startup-error → score `-1` (now `kErrorScore`, if Task 14 landed first
    — order this task after Task 14 for that cross-reference to be accurate) skip path.
  - Split the report step into two calls: `writeSimulationOutputYaml` (per-plugin) then
    `writeModeReport`/`writeComparativeReport` (aggregate), instead of one combined step.

- [ ] **Step 3: Drone-step sequence** — open `docs/hld/seq-drone-step.mmd` and:
  - Add the footprint-carve step (`markDroneFootprintEmpty`) before `nextStep`, matching
    `DroneControlImpl::step()`'s actual order (read the `.cpp` to confirm exact placement).
  - Remove the "Continue → no scan write" claim from the recoverable-throw alt branch; show that
    `applyScanIfRequested` still runs after a recoverable `Continue`, gated only on whether the
    command carried a `scan_orientation` (not on whether the move succeeded).
  - Add an `AlgorithmStatus::Finished` alt branch (no further move/scan) and an over-limit/`Error`
    alt branch, alongside the existing normal-step branch.

- [ ] **Step 4: `docs/HLD.md` prose fix** — find the `MockMovement` description (per the rubric
  review, around the line describing collision handling) and correct "throws or returns a message
  containing `blocked`/`boundary`" to accurately state that `MockMovement::advance`/`elevate` only
  **throw** `std::runtime_error` on collision (never return a failed `MovementResult` for that
  case — only the pre-throw limit checks in Task 9 return `{false, message}`), and that
  `DroneControlImpl` catches the exception (and any failed `MovementResult`) and continues.

- [ ] **Step 5: Regenerate the rendered artifacts.** Locate the render script/command this project
  already used to produce `docs/hld/*.png` and `HLD.pdf` (check `docs/HLD.md`'s own notes, or
  `rg -rn "mmdc\|mermaid-cli\|render_hld" .` for a prior invocation) and re-run it so the PNGs and
  the PDF reflect the corrected diagrams. If no such script exists in the repo, ask the human how
  the original diagrams were rendered before inventing a new toolchain step.

- [ ] **Step 6: Confirm `verify-cell-runtime` still matches baseline** (trivial — no code changed;
  this step exists only to honor the Execution Mode's uniform per-task gate). Report the table
  (expected identical to Task 15's). **Stop. Wait for approval.**

- [ ] **Step 7: Commit (after approval)**

```bash
git commit -am "docs: fix HLD class and sequence diagram drift found by rubric review"
```

---

### Task 17: Final whole-branch review and Known Issues filing

**Files:** none new — read-only review of Tasks 1–16's combined diff, plus
`docs/known-issues.md` if any deferred finding should be recorded there.

- [ ] **Step 1: Generate the whole-branch diff** against the base recorded in Task 0:

```bash
git diff known-issues-fixes...fix-advcpp-rubric-findings --stat
git diff known-issues-fixes...fix-advcpp-rubric-findings > /tmp/advcpp-rubric-fixes-full.diff
```

- [ ] **Step 2: Dispatch a final code-review subagent** (model: still `cursor-grok-4.6-high`, per
  this whole plan's cost constraint — even the "most capable" recommendation in
  `subagent-driven-development`'s Model Selection section is overridden by the user's explicit
  "don't use expensive models" instruction for this plan) over the full diff, checking: (a) every
  task's stated "no behavior change" claim actually holds across the combined diff, not just
  task-by-task; (b) no task's change was left half-applied (e.g. a call site missed in Task 6's
  grep sweep); (c) the branch's `ctest --test-dir build/default --output-on-failure` and one final
  `verify-cell-runtime` 24-cell run are both clean.

- [ ] **Step 3: File Known Issues rows** for the "Deferred findings" table at the top of this plan,
  using `.cursor/skills/populate-known-issues/SKILL.md`'s column schema, so the AdvCpp rubric's e03
  (hot-path libm parity — already documented as accepted), e13, e16, e21 (remaining sites), e22, and
  e10 (remaining duplication) findings are recorded as consciously-not-fixed rather than silently
  dropped.

- [ ] **Step 4: Report final status** — full task list with commit shas, final 24-cell table vs.
  baseline, and the Known Issues rows filed. **Stop. Wait for the human's decision on how to land
  this branch** (merge to `known-issues-fixes` via PR, per `finishing-a-development-branch`, is the
  expected next step, but that decision belongs to the human, not this plan).

---

## Self-Review Notes

- **Spec coverage:** every non-deferred row from the `advcpp-rubric-review` table in this thread
  maps to exactly one task above (Task 1 → e01; Task 2/3 → e06/e08 DroneControlImpl; Task 4 → e07;
  Task 5 → e08 MappingAlgorithmImpl; Tasks 6–8 → e10; Tasks 9–10 → e03 (the two non-deferred sites);
  Task 11 → e04; Task 12 → e17; Task 13 → e21 (the one non-deferred site); Task 14 → e23; Task 15 →
  the GUIDELINE finding; Task 16 → e14/e15). Deferred findings are listed explicitly with
  rationale, not silently dropped.
- **Placeholder scan:** every code step above shows real code or an explicit, checkable
  read-first-then-transform instruction (e.g. "read the file, confirm X, then apply this exact
  transform") — no "add appropriate handling"/"TBD" steps remain.
- **Type consistency:** `keyToPoint` (Task 8), `kErrorScore` (Task 14), and `TimeFormat.h`'s two
  functions (Task 1) are each defined once and referenced by name consistently in every later task
  that touches nearby code (Task 16 references `kErrorScore` from Task 14 for the sequence-diagram
  fix; task ordering in the list above already accounts for this).

## Execution Handoff

Plan complete and saved to
`docs/superpowers/plans/2026-09-06-advcpp-rubric-findings-fix.md`. Per the Execution Mode section,
this plan does **not** use continuous subagent-driven-development or a separate executing-plans
session — it uses subagent-driven-development's per-task dispatch/review mechanics, but with the
human-approval gate re-inserted after every task's `verify-cell-runtime` run, as explicitly
requested. Say "start Task 0" (or "start Task N" to resume) when ready.
