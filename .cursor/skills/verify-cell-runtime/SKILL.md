---
name: verify-cell-runtime
description: >-
  Measures wall-clock time and score for each of the 24 inputs/sim_compose.yaml
  cells serially on Release, then compares them to the per-cell runtime bar
  (~10s small maps, ~60s grader-risk, no cell routinely near 60s). Use when
  changing the mapping algorithm, after a 24-cell score column, when checking
  b05 timeout risk, or when asked whether cells are too slow / how long each
  run takes. Do not treat num_threads=8 matrix wall-clock as per-cell time.
---

# Verify Cell Runtime

Times **each** of the 24 composition cells as its own process (Release,
`num_threads=1`) and judges wall-clock against the policy below. This is clock
**2** (per-cell). It is **not** `run_benchmark.py --num-threads 8` (clock **3**).

Announce: “Using verify-cell-runtime.”

## Time caps for algorithm work — follow these, do not invent others.

There is no wall-clock timeout in our simulator and Assignment 3 does not require one. Do not add an elapsed-time abort, alarm, thread kill, or “if runtime > N then Finished” in Algorithm or MissionControl. A run ends when the algorithm returns Finished / FinishedWithUnmappableVoxels, or when MissionControl hits mission_config.max_steps. That step cap is the real per-run limit. Use the YAML value; do not invent a smaller one in code.

Instructor runtime bar (from Ex2; Assignment 3 never restated it, but rubric b05 still exists as “timeout on scenario (1 minute)”):

- Each instructor scenario / integration cell should finish in about 1 minute at most.
- Treat ~60 seconds per cell as the grader-risk budget, not a feature to implement.
- Small maps: they expect mapping in about 10 seconds. More than 2 minutes on a small map can lose points. “Only 3× slower than another team” was still acceptable in Ex2.

Do not confuse these three clocks:

1. max_steps — hard mission stop. Obey the YAML.
2. Per-cell wall time — one (simulation, mission, drone, lidar) cell. Prefer well under 60s. Small-room cells should feel like ~10s, not minutes.
3. Full inputs/sim_compose.yaml — 24 cells, currently run with num_threads=8. This is allowed to take minutes, not tens of minutes. Current baseline: ~276s wall-clock, 24/24 COMPLETED, mission_score >= 0.

The 276s figure is the whole matrix in parallel, not 276s per cell. A cell that takes around a minute is already in b05 danger even if the 24-cell job still finishes in a few minutes.

Per-mission step budgets (6 aligned pairs × 2 drones × 2 lidars = 24 cells):

- large_mission_room.yaml: 500 — short; must not be slow per step
- small_mission_room.yaml: 1000 — instructor small-map case; aim ~10s, stay well under 60s
- small_mission_out.yaml: 2000 — medium
- house_mission_lower.yaml: 2000 — medium; currently easy to score well
- house_mission_full.yaml: 10000 — long; cost per nextStep matters more than total steps
- large_mission_out.yaml: 10000 — long; highest hang / timeout risk

Hard fail vs soft fail:

- Hang / unbounded search is the real disaster (Ex2 ALG28: BFS with no occupancy bound, killed by timeout). Every path search must be bounded by the map. No infinite loops in nextStep.
- Slow but finishing is a score/rubric risk (b05, “too slow on small maps”), not a crash.
- Finishing via MaxSteps is allowed. Burning all 10000 steps on a small map is not a win if that map should have been done in seconds.

When iterating:

1. Unit / frontier tests first (seconds).
2. Then one focused cell (small_mission_room / tiny_compose.yaml) and time it.
3. Then the two long missions (house_mission_full, large_mission_out) — those are where a heavier planner blows the 1-minute cell budget.
4. Full 24-cell compose last. Do not use it as the inner loop.
5. Keep 24/24 COMPLETED and mission_score >= 0. Do not trade completeness for speed by flying into known Occupied, disabling scan-during-travel, or hard-blocking Unmapped (those were already rejected).
6. If compose wall-clock jumps from ~5 minutes toward tens of minutes, stop and profile nextStep / findPath. Do not “fix” it with a wall-clock cap.

Success bar for a timing-related change: small-room cell well under 60s (ideally ~10s); no cell should routinely sit near 60s; full compose stays minutes-scale; no hangs under `timeout 60` around a single cell.

## What this skill measures

| Clock | How | Do not |
|-------|-----|--------|
| **2** per-cell wall | This skill: 24 serial Release processes | Use Debug `build/default`; use `num_threads=8` as a substitute |
| **3** full compose | Optional extra: `run_benchmark.py --build-dir build/opt --columns honest --num-threads 8` | Treat ~200–276 s matrix time as “each cell was under 60 s” |
| **1** max_steps | YAML only | Invent a smaller cap in Algorithm/MC |

`num_threads=8` overlaps **different** cells. It does not make one cell faster.

## Procedure

### 1. Release binaries

Docker image `drone-mapper-ex3-dev`. Bind-mount timestamps: delete Algorithm `.o`/`.so` under `build/opt` before rebuilding.

```bash
docker run --rm -e PYTHONUNBUFFERED=1 -e VCPKG_ROOT=/usr/local/vcpkg \
  -v "<repo>:/work" -w /work drone-mapper-ex3-dev bash -lc '
  set -euo pipefail
  if [ ! -f build/opt/CMakeCache.txt ]; then
    cmake -S . -B build/opt -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_TOOLCHAIN_FILE="$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake"
  fi
  rm -f build/opt/Algorithm/CMakeFiles/Algorithm_207190406_209543255.dir/src/*.o
  rm -f build/opt/Algorithm/Algorithm_207190406_209543255.so
  cmake --build build/opt -j$(nproc) --target \
    Algorithm_207190406_209543255 \
    simulator_207190406_209543255 \
    MissionControl_207190406_209543255
  python3 .cursor/skills/verify-cell-runtime/scripts/time_each_cell.py \
    --build-dir /work/build/opt
'
```

Host is Windows: no `&&` / bash heredocs on PowerShell; keep the `bash -lc` string as above.

### 2. Judge the CSV

The script writes `tmp/bench-out/per-cell-wall.csv` and prints a verdict table.

| Verdict | Meaning |
|---------|---------|
| **PASS** | Cell finished; `wall_s` well under 60 s (small_room also ≤ 15 s) |
| **WARN** | Finished; `wall_s` ≥ 45 s, or small_room > 15 s (still < 60 s) |
| **FAIL** | `wall_s` ≥ 60 s, hang (`timeout` / no finish), crash, or score < 0 |
| **HANG** | Process hit `--hang-timeout` (default 180 s) — unbounded-search class; FAIL |

Do **not** implement the 60 s bar as an abort inside Algorithm/MissionControl. `--hang-timeout` is only a measurement wrapper to detect ALG28-style hangs.

### 3. Report (required)

```markdown
# Cell runtime verification

Date (UTC): …
SHA: …
Build: Release `build/opt`

| Cell | Group | Score | Steps | Status | wall_s | Verdict |
|------|-------|-------|-------|--------|--------|---------|
| … | … | … | … | … | … | PASS/WARN/FAIL |

Overall: FAIL if any FAIL/HANG. Otherwise PASS with WARNs listed.
```

Overall **FAIL** does not mean “add a timer.” It means profile `nextStep` / `exploreReachable` on the slow cells (usually `house_full` / `large_out`).

## Focused first (iteration order)

If the user did not ask for all 24:

1. `algorithm_test` (seconds)
2. `--only-group small_room` (instructor small-map)
3. `--only-group house_full --only-group large_out` (1-minute budget risk)
4. Full 24 last

```bash
python3 .cursor/skills/verify-cell-runtime/scripts/time_each_cell.py \
  --build-dir /work/build/opt --only-group small_room
```

## Anti-patterns

- Debug `run_all.sh` / `run_smoke_pass.sh` as a runtime check (no `num_threads`, Debug, silent 24-cell serial)
- Reading 8-thread compose wall-clock as per-cell time
- Killing a cell at 60 s inside the algorithm
- Lowering YAML `max_steps` to “make it faster”
- Skipping long missions because they are “allowed 10000 steps”
