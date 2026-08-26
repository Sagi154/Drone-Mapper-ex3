# Integration & Whole-System Verification Report

Run **2026-08-25**, Docker Desktop Linux engine (`drone-mapper-ex3-dev`, Ubuntu 24.04, gcc 13.3),
not WSL Ubuntu. Composition: `inputs/sim_compose.yaml` (24 run cells). Scripts:
`Simulator/tests/manual/`.

Default artifacts: `build/default/` (bind mount). TSan artifacts were built on the container's
own disk (`/tmp/ex3`) after bind-mount TSan compiles stalled and the engine OOM'd at 2 GiB RAM.
After raising the WSL2 VM to 8 GB (`.wslconfig`), TSan completed.

## 1. Functional smoke pass (both CLI modes)

- Command: `./Simulator/tests/manual/run_smoke_pass.sh /work/build/default` (inside Docker)
- Result: both modes exit 0. Comparative wrote
  `/tmp/ex3_verify/mission_controls/comparative_results_2026-08-25T18:21:39Z`. Competition wrote
  `/tmp/ex3_verify/algorithms/competition_2026-08-25T18:22:20Z`. Each plugin: 24 runs. Reports,
  per-plugin YAML, error logs, and output maps present (maps missing for some Error cells).
- Note: every cell was **unscored** (`24 unscored (score < 0)`). That matches missions ending as
  `Error` (spawn/collision) already called out in `docs/workplan.md`.
- Verdict: **PASS** (CLI + artifacts). Scoring of this composition is a known product issue, not a
  script failure.

## 2. Collision re-run check

- Command: `./Simulator/tests/manual/check_collision.sh`
- Result: `PASS: 2 distinct directories, no collision`
  (`…18:23:03Z` and `…18:23:26Z`).
- Verdict: **PASS**

## 3. `-verbose` on/off

- Command: `./Simulator/tests/manual/check_verbose.sh`
- Result: file lists for off vs on were the same shape (reports, `*_error.log`, some
  `*_output_map.npy`, `startup_error.log`). No extra `*.verbose.txt` files.
- Cause: two stacked gaps. (1) `main.cpp` never reads `args.verbose`;
  `SimulationRunFactoryImpl` hardcodes `MissionControlDependencies.verbose = false`, so
  MissionControl never sees the flag. (2) Even after that is wired, `MissionControlImpl`
  only writes `*.verbose.txt` when `verbose` is set **and** `output_map_file` is non-empty;
  Error/unscored cells on this composition often have no usable map path.
- Verdict: **FAIL**. The CLI parses `-verbose` but the flag is **not** wired through to
  MissionControl. Re-check after passing `args.verbose` into the factory constructor, on a
  composition that completes with a real `output_map_file`.

## 4. Threading determinism (absent / 1 / 2 / 8)

- Command: `./Simulator/tests/manual/check_threading.sh`
- Result: after stripping `generated_at_utc` and `mission_control_folder` (per-run scratch paths),
  `PASS: absent == t1`, `t2`, `t8`.
- Verdict: **PASS**

## 5. CLI failure modes

- Command: `./Simulator/tests/manual/check_cli_failures.sh`
- Result: no crash (exit 0 from `main` on parse failure). Named `bogus_arg`; both missing
  `mission_control_folder` and `algorithm`; missing simulation path; empty folder (no `.so`);
  malformed `simulation...` reported as unsupported + missing `simulation`.
- Verdict: **PASS**

## 6. Isolation / cross-`.so` check

- Command: `./Simulator/tests/manual/check_isolation.sh`
- Result: registration constructors undefined in both `.so`s (resolved from the executable).
  Competition with `Algorithm_*.so` and `Algorithm_*_copy2.so` listed both in `results_summary`,
  `errors: []`.
- Verdict: **PASS**

## 7. ThreadSanitizer

- Command: `Simulator/tests/manual/docker_tsan.sh` with `--privileged` (ASLR) and
  `cmake --build --parallel 2`, after Docker Total Memory **7.756GiB**.
- Result: three comparative runs (`num_threads=8`, `=2`, absent).
  `grep -c "WARNING: ThreadSanitizer"` → **0**.
- Earlier attempts: bind-mount compile hung; 2 GiB VM killed the engine; TSan aborted with
  `unexpected memory mapping` until privileged + `vm.mmap_rnd_bits=28`.
- Verdict: **PASS** (this host, after RAM + ASLR workaround)

## 8. Frozen-interfaces check

- Command: `git diff --name-status main -- common/ Simulator/common_simulator/ MissionControl/common_mission_control/`
  and `git status --porcelain` on the same paths.
- Result: empty.
- Verdict: **PASS**

## Update 2026-08-26 (default composition scoring)

After `house_simulation.yaml` spawn fix (`height_cm` 10→150) on branch
`fix-default-composition-scoring`: `SPAWN_NOT_PASSABLE` cleared; **4 house cells `Completed` with
`mission_score` 100**; 20 cells still Error / `MISSION_EXCEPTION`. AdvCpp happy-flow bar met.
`check_verbose.sh` file-list passes on a completing cell. Full `ctest` 85/85 green. The 2026-08-25
smoke results below remain the baseline for that run; see `docs/assignment-compliance-pickup.md`
for current status.

## Open follow-ups

- ~~20/24 `sim_compose.yaml` cells Error / `MISSION_EXCEPTION`.~~ **Resolved 2026-08-27:**
  comparative 24/24 `COMPLETED`, all scores ≥ 0, `MISSION_EXCEPTION` 0 (~276s). See
  `docs/known-issues.md` row 2 and
  `docs/superpowers/specs/2026-08-27-wall-collision-recovery-and-planner-design.md`.
- TSan on Docker Desktop needs ~8 GB VM RAM, compile on Linux disk (not `C:\` bind mount), and
  often `--privileged` for ASLR. Documented in `Simulator/tests/manual/docker_tsan.sh`.
