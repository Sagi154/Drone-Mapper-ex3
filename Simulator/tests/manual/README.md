# Manual / end-to-end verification scripts

These scripts drive the real `simulator_207190406_209543255` binary against the real
`Algorithm_207190406_209543255.so` / `MissionControl_207190406_209543255.so` plugins to verify
whole-system behavior the unit tests can't exercise alone (dlopen isolation, CLI-level failure
modes, threading determinism, ThreadSanitizer).

Run **in Docker** (course image from `.devcontainer/Dockerfile`), not a Windows-native compiler:

```text
docker build -t drone-mapper-ex3-dev -f .devcontainer/Dockerfile .devcontainer
docker run --rm -e VCPKG_ROOT=/usr/local/vcpkg \
  -v <repo>:/work -w /work drone-mapper-ex3-dev \
  bash /work/Simulator/tests/manual/run_in_docker.sh
```

Default-preset checks: `run_all.sh` (after a `cmake --preset default` build). TSan:
`docker_tsan.sh` with `--privileged` so TSan can run on current kernels; compile happens on the
container's `/tmp`, not the Windows bind mount.

Scratch output is under `/tmp/ex3_verify/` inside the container (never under `inputs/`).

- `check_multi_plugin_outputs.sh` — comparative + competition with two distinct `.so`s per mode; asserts each plugin gets `<name>_simulation_output.yaml`.
