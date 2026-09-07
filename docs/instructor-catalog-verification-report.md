# Instructor catalog verification

Date (UTC): 2026-09-06
Branch: `known-issues-fixes`
SHA: `8fda059525d9092ab1f4f49aee32d849a136f182`
Switches: `--skip-rubric --skip-zip`
Docker image: `drone-mapper-ex3-dev:latest` (`2a3184f3d93b`)

## Stage results

| Stage | Status | Evidence |
|-------|--------|----------|
| Build | PASS | `cmake --preset default && cmake --build --preset default` exit 0 |
| ctest | PASS | 175/175 passed, 11.64 s |
| `run_all.sh` | PASS | `run_all.sh: all default-preset checks finished`; exit 0; ~61 min |
| pre-submission-review | PASS | Working-tree §§1–5c + §6; §5d produced-zip SKIP (`--skip-zip`) |
| advcpp-rubric-review | SKIP | `--skip-rubric` |

## Catalog ID table

| ID | Classification | Status | Evidence |
|----|----------------|--------|----------|
| CLI-01 | MANDATORY | PASS | `run_smoke_pass.sh` comparative exit 0; 24/24, 0 unscored |
| CLI-02 | MANDATORY | PASS | `run_smoke_pass.sh` competition exit 0; 24/24, 0 unscored |
| CLI-03 | MANDATORY | PASS | `check_cli_argument_order.sh` scrambled comparative; competition order still thin |
| CLI-04 | MANDATORY | PASS | `check_cli_failures.sh` two-missing-args names `mission_control_folder algorithm` |
| CLI-05 | MANDATORY | PASS | `check_cli_failures.sh` dual unsupported args (`foo bar`) |
| CLI-06 | MANDATORY | PASS | `check_cli_failures.sh` nonexistent simulation file |
| CLI-07 | MANDATORY | PASS | `check_cli_failures.sh` empty `mission_control_folder` |
| CLI-08 | MANDATORY | PASS | `check_output_dir_unwritable.sh` Permission denied, no crash |
| PLUGIN-01 | MANDATORY | PASS | `check_multi_plugin_outputs.sh` comparative two MCs |
| PLUGIN-02 | MANDATORY | PASS | `check_multi_plugin_outputs.sh` competition two algorithms |
| PLUGIN-03 | MANDATORY | PASS | `check_isolation.sh` + fixture `.so`s (nm + competition with `valid_algorithm_plugin.so`) |
| PLUGIN-04 | MANDATORY | PASS | `check_all_folder_plugins_fail.sh` `errors` in both reports |
| OUT-01 | MANDATORY | PASS | `check_output_dir_collision.sh` two distinct `comparative_results_*` |
| OUT-02 | MANDATORY | PASS | `check_competition_output_dir.sh` two distinct `competition_*` |
| OUT-03 | MANDATORY | PASS | smoke + multi-plugin: maps, error logs, summary YAML, per-plugin YAML |
| YAML-OUT-01 | MANDATORY | PASS | unit `test_comparative_report_writer` + smoke `comparative_report.yaml` |
| YAML-OUT-02 | MANDATORY | PASS | unit `test_competitive_report_writer` + smoke `competitive_report.yaml` |
| YAML-OUT-03 | MANDATORY | PASS | `check_multi_plugin_outputs.sh` per-plugin YAMLs |
| YAML-IN-01 | MANDATORY | PASS | unit `test_yaml_config_parsers` + smoke load of `inputs/sim_compose.yaml` |
| LOG-01 | MANDATORY | PASS | `check_verbose.sh`: `.verbose.txt` only with `-verbose` |
| ERR-01 | MANDATORY | PASS | CLI / unwritable / wall-fault / unloadable plugins: process stayed alive |
| FAULT-01 | MANDATORY | AMBIGUOUS | No dedicated oracle; smoke 0 unscored is weak evidence only |
| FAULT-02 | MANDATORY | PASS | `check_wall_collision_fault.sh` + unit `CollisionBlockedThrowContinues` |
| THREAD-01 | MANDATORY | PASS | `check_threading.sh` absent == t1 |
| THREAD-02 | MANDATORY | PASS | `check_threading.sh` absent == t2 == t8 |
| MAP-ALGO | MANDATORY (metrics unspecified) | AMBIGUOUS | Intent only; no numeric threshold in catalog |
| CLI-OPT-01 | OPTIONAL | PASS | smoke without `num_threads` |
| CLI-OPT-02 | OPTIONAL | PASS | `check_verbose.sh` |
| FAULT-OPT-01 | OPTIONAL | AMBIGUOUS | not exercised as a catalog script |
| FAULT-OPT-02 | OPTIONAL | AMBIGUOUS | not exercised as a catalog script |
| FAULT-OPT-03 | OPTIONAL | AMBIGUOUS | not exercised as a catalog script |
| FAULT-OPT-04 | OPTIONAL | AMBIGUOUS | not exercised as a catalog script |
| FAULT-OPT-05 | OPTIONAL | AMBIGUOUS | not exercised as a catalog script |
| FAULT-OPT-06 | OPTIONAL | AMBIGUOUS | not exercised as a catalog script |
| FAULT-OPT-07 | OPTIONAL | AMBIGUOUS | not exercised as a catalog script |
| FAULT-OPT-08 | OPTIONAL | AMBIGUOUS | not exercised as a catalog script |
| FAULT-OPT-09 | OPTIONAL | AMBIGUOUS | not exercised as a catalog script |
| FAULT-OPT-10 | OPTIONAL | AMBIGUOUS | not exercised as a catalog script |
| UNSPEC-01 | GENUINELY-UNSPECIFIED | AMBIGUOUS | open-questions working assumption only |
| UNSPEC-02 | GENUINELY-UNSPECIFIED | AMBIGUOUS | open-questions working assumption only |
| UNSPEC-03 | GENUINELY-UNSPECIFIED | AMBIGUOUS | open-questions working assumption only |
| UNSPEC-04 | GENUINELY-UNSPECIFIED | AMBIGUOUS | open-questions working assumption only |
| UNSPEC-05 | GENUINELY-UNSPECIFIED | AMBIGUOUS | open-questions working assumption only |
| UNSPEC-06 | GENUINELY-UNSPECIFIED | AMBIGUOUS | open-questions working assumption only |
| UNSPEC-07 | GENUINELY-UNSPECIFIED | AMBIGUOUS | open-questions §9 working assumption only |
| UNSPEC-08 | GENUINELY-UNSPECIFIED | AMBIGUOUS | open-questions working assumption only |
| UNSPEC-09 | GENUINELY-UNSPECIFIED | AMBIGUOUS | open-questions working assumption only |
| UNSPEC-10 | GENUINELY-UNSPECIFIED | AMBIGUOUS | open-questions working assumption only |
| UNSPEC-11 | GENUINELY-UNSPECIFIED | AMBIGUOUS | open-questions working assumption only |
| UNSPEC-12 | GENUINELY-UNSPECIFIED | AMBIGUOUS | open-questions working assumption only |
| UNSPEC-13 | GENUINELY-UNSPECIFIED | AMBIGUOUS | open-questions working assumption only |
| UNSPEC-14 | GENUINELY-UNSPECIFIED | AMBIGUOUS | open-questions working assumption only |
| UNSPEC-15 | GENUINELY-UNSPECIFIED | AMBIGUOUS | open-questions working assumption only |
| UNSPEC-16 | GENUINELY-UNSPECIFIED | AMBIGUOUS | open-questions working assumption only |
| UNSPEC-17 | GENUINELY-UNSPECIFIED | AMBIGUOUS | open-questions working assumption only |
| ZIP-01 | MANDATORY | SKIP | `--skip-zip` (produced archive not built) |
| ZIP-02 | MANDATORY | PASS | five project folders present; `UserCommon/` has no `CMakeLists.txt` |
| ZIP-03 | MANDATORY | PASS | root + Algorithm + MissionControl + Simulator `CMakeLists.txt`; no student makefile in `UserCommon/` |
| ZIP-04 | MANDATORY | SKIP | `--skip-zip`; working tree `students.txt` is filled (Sagi / Yoav, no `TODO:`) |
| ZIP-05 | MANDATORY | SKIP | `--skip-zip`; working tree `README.md` and `HLD.pdf` exist at repo root |
| ZIP-06 | MANDATORY | PASS | no `.so`/`.o`/`.exe` under shippable source trees; deps in `vcpkg.json` |
| ZIP-07 | MANDATORY | PASS | `verify-frozen-interfaces` empty vs `main` + porcelain for `common/` |
| ZIP-08 | MANDATORY | PASS | `verify-frozen-interfaces` empty vs `main` for `common_simulator/` / `common_mission_control/` |
| ZIP-09 | MANDATORY | PASS | `simulator_207190406_209543255`, `Algorithm_207190406_209543255.so`, `MissionControl_207190406_209543255.so` |
| ZIP-10 | MANDATORY | PASS | Algorithm/MC `SHARED` + `PREFIX ""`; simulator executable `ENABLE_EXPORTS ON` |
| ZIP-11 | MANDATORY | PASS | `REGISTER_MAPPING_ALGORITHM` / `REGISTER_MISSION_CONTROL` in plugin `.cpp`; registration `.cpp` in Simulator |
| ZIP-12 | MANDATORY | PASS | `algorithm_207190406_209543255`, `mission_control_207190406_209543255`, `user_common_207190406_209543255` |
| ZIP-13 | MANDATORY | PASS | no `#include` of mocks/`Map3DImpl`/`MapsComparison` in Algorithm or MissionControl sources |
| ZIP-14 | MANDATORY | PASS | `UserCommon/` + `user_common_207190406_209543255` |
| ZIP-15 | MANDATORY | PASS | shippable `src` grep: comments + `= delete` only, no allocating `new`/`delete` |
| ZIP-16 | MANDATORY | PASS | `CMAKE_CXX_STANDARD 20`; `drone_warnings` → `-Wall -Wextra -Werror -pedantic`; build succeeded |
| ZIP-17 | MANDATORY | PASS | `inputs/` has `sim_compose.yaml` and `inputs/map/*.npy` |
| e01–e23 | (rubric) | SKIP | `--skip-rubric` |

## Gaps

- FAULT-01 and MAP-ALGO remain AMBIGUOUS (no numeric / oracle evidence in this orchestration).
- All `UNSPEC-*` and unrun `FAULT-OPT-*` rows are AMBIGUOUS by design.
- ZIP-01 / ZIP-04 / ZIP-05 not scored from a produced archive (`--skip-zip`). Packaging still needs `pre-submission-review` zip steps before upload.

## Rubric appendix

Skipped (`--skip-rubric`).

## Overall

**PASS** — no selected-stage FAIL; no covered MANDATORY FAIL. AMBIGUOUS and SKIP rows listed above do not fail this report.
