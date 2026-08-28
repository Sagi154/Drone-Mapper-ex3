# Manual / end-to-end verification scripts

These scripts drive the real `simulator_207190406_209543255` binary against the real
`Algorithm_207190406_209543255.so` / `MissionControl_207190406_209543255.so` plugins (and test
fixture `.so`s) to verify whole-system behavior the unit tests can't exercise alone (dlopen
isolation, CLI-level failure modes, threading determinism, ThreadSanitizer, catalog black-box
rows).

Run **in Docker** (course image from `.devcontainer/Dockerfile`), not a Windows-native compiler:

```text
docker build -t drone-mapper-ex3-dev -f .devcontainer/Dockerfile .devcontainer
docker run --rm -e VCPKG_ROOT=/usr/local/vcpkg \
  -v <repo>:/work -w /work drone-mapper-ex3-dev \
  bash /work/Simulator/tests/manual/run_in_docker.sh
```

Default-preset checks: `run_all.sh` (after a `cmake --preset default` build). Full catalog-keyed
report: `.cursor/skills/verify-instructor-test-catalog/SKILL.md`. TSan: `docker_tsan.sh` with
`--privileged` so TSan can run on current kernels; compile happens on the container's `/tmp`, not
the Windows bind mount.

Scratch output is under `/tmp/ex3_verify/` inside the container (never under `inputs/`). Many
scripts use `inputs/tiny_compose.yaml` where a full 24-cell matrix is unnecessary.

## Scripts invoked by `run_all.sh`

| Script | What it checks |
|--------|----------------|
| `make_fixture_dirs.sh` | Prepares scratch dirs + copies distinct fixture `.so`s |
| `run_smoke_pass.sh` | Both CLI modes smoke (artifacts written) |
| `check_output_dir_collision.sh` | Comparative OUT-01: two runs → distinct `comparative_results_*` dirs *(renamed from `check_collision.sh` — not FAULT-02)* |
| `check_competition_output_dir.sh` | Competition OUT-02: distinct `competition_*` dirs |
| `check_output_dir_unwritable.sh` | CLI-08: non-writable results parent → usage/error, no crash |
| `check_wall_collision_fault.sh` | FAULT-02: MockMovement wall throw path via faulty fixture `.so`, no crash |
| `check_verbose.sh` | `-verbose` on/off → `*.verbose.txt` present/absent |
| `check_threading.sh` | `num_threads` absent/1/2/8 report equivalence |
| `check_cli_failures.sh` | CLI-04/05: missing/unsupported args named in message; no crash |
| `check_cli_argument_order.sh` | CLI-03: permuted argv order still accepted |
| `check_all_folder_plugins_fail.sh` | All folder plugins unloadable → aggregate `errors:` |
| `check_isolation.sh` | Distinct fixture `.so`s load under `RTLD_LOCAL` |
| `check_multi_plugin_outputs.sh` | Two distinct `.so`s per mode; each gets `<name>_simulation_output.yaml` |
| `check_foreign_host.sh` | VAR-01: our Algorithm + MissionControl under blind `skeleton_host` on staff maps |

Helpers (not always in `run_all.sh`): `run_in_docker.sh`, `docker_verify_default.sh`,
`docker_tsan.sh`.
