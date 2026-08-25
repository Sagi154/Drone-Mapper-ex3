# Work Split — Current Status & Next Steps

Generated against the real source tree on 2026-08-13.

---

## Actual completion state (source tree vs. workplan markers)

| Item | Workplan says | Reality |
|---|---|---|
| Y10 `SimulationRunImpl` | ✅ DONE | ✅ Implemented; contains `runMission()` exceptions; scores via `MapsComparison` |
| End-to-end `main.cpp` wiring | ✅ DONE | ✅ CLI → plugins → orchestrator → report writers → named output dir |
| S8 real BFS scoring | ✅ DONE | ✅ `MapsComparison::compare` ported; wired in `SimulationRunImpl`; `simulator::scoring` linked |
| S10 report writers | ✅ DONE | ✅ Comparative / competitive / per-plugin YAML writers called from `main.cpp` |
| S11 output dir naming | ✅ DONE | ✅ `OutputDirHelper` + flat `<plugin>_run_NNNN_output_map.npy`; pattern in `README.md` |
| Y11 per-run naming | ✅ DONE | ✅ `<plugin>_run_NNNN_error.log` from map stem; YAML `error_log_file` filled |

---

## What landed this session (Sagi)

### S8 — Real BFS scoring
Ported ex2 reachability/BFS scoring into `Simulator/src/MapsComparison.cpp` behind the frozen
single-target signature. Unit tests use a hand-written fake `IMap3D`. `SimulationRunImpl` scores
`Completed`/`MaxSteps` runs with `worldInitialDronePosition`; `Error` stays at `-1`.

### S10 — Report writers
- `ComparativeReportWriter` / `CompetitiveReportWriter` / `SimulationOutputYamlWriter`
- Wired from `main.cpp` after `RunMatrixOrchestrator::run`
- Load-failure `.so` filenames collected into report `errors: [...]`

### S11 — Output dirs + per-run naming
- `OutputDirHelper`: `comparative_results_<time>` / `competition_<time>` with collision suffix
- Orchestrator writes flat `{output_root}/{plugin}_run_NNNN_output_map.npy`
- Pattern documented in `README.md` (unblocks Y11)

---

## What Yoav can do now

### Y11 — Per-run output file naming ✅ DONE
`<plugin>_run_NNNN_error.log` is derived from the map path, written by `SimulationRunImpl`, and
reported in per-plugin YAML `error_log_file`. `startup_error.log` remains for pre-matrix parse
failures.

Best remaining uses of time:
- HLD diagrams for `MissionControl/` components
- Help smoke-test full composition once algorithm/mission runs reach `Completed`/`MaxSteps`
  (current `inputs/sim_compose.yaml` smoke still ends many cells in `SPAWN_NOT_PASSABLE` /
  `MISSION_EXCEPTION`, so end-to-end scores stay `-1` until those missions terminate cleanly —
  unit tests already cover the scorer)

---

## Joint task — ✅ DONE

### Vertical slice — `main.cpp` + `CMakeLists.txt` wiring
See prior PR #3 / branch `end-to-end-comparative-run`. Superseded on
`maps-scoring-reports-output-naming` by S8/S10/S11 wiring above.

---

## Recommended split for the next session

| Who | What | Why now |
|---|---|---|
| **Either** | HLD diagrams / fuller README rewrite | Submission deliverables |
| **Either** | Investigate why full-composition cells often `Error` before `Completed` | Needed for meaningful end-to-end scores |

---

## Remaining submission deliverables (either person)

- **`README.md`** rewrite — build presets, binary/`.so` names, both CLI invocations (output
  naming pattern section already landed with S11).
- **HLD as PDF** — each person diagrams their own components; one person assembles + exports.
- **`bonus.txt`** — only if claiming a bonus (most likely: Sagi's lazy-load path in S6).
- **Known Issues excel** — fill incrementally as corner cases are deliberately skipped.
- **Pre-submission structure pass** — run `.cursor/skills/pre-submission-review/SKILL.md`
  end to end before zipping.
