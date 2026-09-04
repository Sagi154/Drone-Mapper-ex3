# Drone Mapper — Assignment 3

**Authors:** Sagi Eisenberg (207190406), Yoav Naaman (209543255)

TAU Advanced Topics in Programming (2026B). Three separately built projects: a
`simulator_<ids>` executable that `dlopen`s `Algorithm_<ids>.so` and
`MissionControl_<ids>.so`.

| Artifact | Name |
|----------|------|
| Executable | `simulator_207190406_209543255` |
| Algorithm plugin | `Algorithm_207190406_209543255.so` |
| MissionControl plugin | `MissionControl_207190406_209543255.so` |

Plugin / UserCommon **namespaces** (code): `algorithm_207190406_209543255`,
`mission_control_207190406_209543255`, `user_common_207190406_209543255`.

## Build

Requires Docker image `drone-mapper-ex3-dev` (or an equivalent Linux + vcpkg + Ninja
environment with `VCPKG_ROOT` set). Dependencies come from `vcpkg.json` via the
CMake toolchain — no manual `apt`/`pip` library installs.

```bash
cmake --preset default
cmake --build --preset default
```

Outputs land under `build/default/`:

- `build/default/Simulator/simulator_207190406_209543255`
- `build/default/Algorithm/Algorithm_207190406_209543255.so`
- `build/default/MissionControl/MissionControl_207190406_209543255.so`

Optional ThreadSanitizer preset: `cmake --preset tsan` / `cmake --build --preset tsan`.

## Run

Arguments may appear in any order. The `=` sign has no spaces around it.

### Comparative mode

```bash
./simulator_207190406_209543255 -comparative \
  simulation=<composition.yaml> \
  mission_control_folder=<folder_with_MissionControl_*.so> \
  algorithm=<Algorithm_*.so> \
  [num_threads=<N>] \
  [-verbose]
```

### Competition mode

```bash
./simulator_207190406_209543255 -competition \
  simulation=<composition.yaml> \
  mission_control=<MissionControl_*.so> \
  algorithms_folder=<folder_with_Algorithm_*.so> \
  [num_threads=<N>] \
  [-verbose]
```

`num_threads` absent or `1`: work runs on the main thread only. `N >= 2`: `N` worker
threads plus the main thread. `-verbose` enables MissionControl verbose files.

Example (from the repo root, after build), using the provided composition:

```bash
BUILD=build/default
SCRATCH=/tmp/ex3_mc
mkdir -p "$SCRATCH"
cp "$BUILD/MissionControl/MissionControl_207190406_209543255.so" "$SCRATCH/"
"$BUILD/Simulator/simulator_207190406_209543255" -comparative \
  simulation=inputs/sim_compose.yaml \
  mission_control_folder="$SCRATCH" \
  algorithm="$BUILD/Algorithm/Algorithm_207190406_209543255.so"
```

## Tests

```bash
ctest --test-dir build/default --output-on-failure
```

## Output naming

Each run creates a fresh output directory (never reused):

- Comparative mode: `<mission_control_folder>/comparative_results_<UTC time>[_N]`
- Competition mode: `<algorithms_folder>/competition_<UTC time>[_N]`

`<UTC time>` is `currentUtcTimestamp()` (`user_common_207190406_209543255/IRunErrorLog.h`); `_N`
is appended starting at `_2` if a directory with that name already exists (same-second collision).

Inside that directory:

```text
<output_dir>/
  comparative_report.yaml                    # or competitive_report.yaml
  <plugin>_simulation_output.yaml            # per-plugin ex2-style score_report
  <plugin>_run_NNNN_output_map.npy           # per-run output map (NNNN = zero-padded cell index)
  <plugin>_run_NNNN_error.log                # per-run error log
```

`NNNN` is the flat cell index from `expandRunMatrix` (0-based, zero-padded to 4
digits), unique across the whole run matrix and stable for a given composition + plugin order.
`<plugin>` is always the loaded `.so` **filename**, never a path.
