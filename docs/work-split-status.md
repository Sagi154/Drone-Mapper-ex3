# Work Split — Current Status & Next Steps

Generated against the real source tree on 2026-08-12.

---

## Actual completion state (source tree vs. workplan markers)

| Item | Workplan says | Reality |
|---|---|---|
| Y10 `SimulationRunImpl` | ✅ DONE | ✅ Implemented; also contains `runMission()` exceptions so maps still save |
| End-to-end `main.cpp` wiring | ✅ DONE | ✅ `Simulator/src/main.cpp` + `add_executable` with `ENABLE_EXPORTS ON` |
| S10 report writers | not started | ❌ `ComparativeReportWriter.cpp`, `CompetitiveReportWriter.cpp`, `SimulationOutputYamlWriter.cpp` missing from `src/io/` |
| S11 output dir naming | not started | ❌ `OutputDirHelper.cpp` missing from `src/io/` — `main.cpp` still uses a temp `simulator_vertical_slice_<epoch>` root |
| S8 real BFS scoring | ⏳ stub | ⏳ `MapsComparison.cpp` exists, `compare()` returns `-1.0` |
| Y11 per-run naming | ⏳ waiting on S11 | ⏳ Orchestrator currently writes `<cell>_output_map.npy`; final pattern still TBD in `README.md` |

---

## What Sagi can do — zero coordination with Yoav needed

### S8 — Fill in `MapsComparison::compare()` (no blocker)
`U4 (SimulationCoordUtil)` is done. Port the real BFS/flood-fill 0–100 scoring from
`../Drone-Mapper-ex2/src/MapsComparison.cpp`. Use a hand-written fake `IMap3D` in tests — no
dependency on Yoav's `Map3DImpl`. After the body lands, wire `compare()` into
`SimulationRunImpl` and link `simulator::scoring` into the executable.

### S10 — Comparative/competitive/per-plugin YAML report writers (no blocker)
S9 (orchestrator + output shape) is done and already called from `main()`. Write:
- `Simulator/src/io/ComparativeReportWriter.cpp`
- `Simulator/src/io/CompetitiveReportWriter.cpp`
- `Simulator/src/io/SimulationOutputYamlWriter.cpp`

against the exact schemas in `.cursor/rules/simulator-cli-and-outputs.mdc`. Hand-build
`PluginMatrixResult` literals for unit tests — no dependency on Yoav's track. Then call the
writers from `main.cpp` after `RunMatrixOrchestrator::run`.

### S11 — Output directory naming (blocked only on S10, which Sagi owns)
Once S10 is done, write `Simulator/src/io/OutputDirHelper.cpp`:
- `comparative_results_<time>` / `competition_<time>` patterns
- collision counter appended when directory already exists

Replace `makeTempOutputRoot()` in `main.cpp`. Document the chosen **per-run artifact filename
pattern** in `README.md` — Yoav's Y11 reads that and follows it; no code handoff is needed.
(Current interim pattern from the orchestrator: `{output_root}/{plugin}/{cell}_output_map.npy`.)

**Sagi's order: S8 → S10 → S11** (each feeds the next; all owned by Sagi)

---

## What Yoav can do — zero coordination with Sagi needed

### Y11 — Per-run output file naming
Y10 (`SimulationRunImpl`) is done. The only remaining dependency is **S11's naming pattern** —
once Sagi documents it in `README.md`, Yoav aligns any remaining per-run artifact names (error
logs, etc.) with that pattern.

**OQ-Y1 ✅ done in tree:** `SimulationRunImpl` skips save when `output_path` is empty (no
`MAP_SAVE_FAILED`). Exception containment for `runMission()` is also in place (mandatory
`MockMovement` collision catch target).

**Yoav's honest state:** little purely-his code remains. Best uses of time while Sagi works on
S8/S10/S11:
- HLD diagrams for `MissionControl/` components (submission deliverable)
- Help smoke-test S10/S11 once they land

---

## Joint task — ✅ DONE

### Vertical slice — `main.cpp` + `CMakeLists.txt` wiring
**Plan:** `docs/vertical-slice-plan.md` (note: that doc's `composition.base_path` mention is
stale — `parseCompositionFile` already resolves relative paths via the composition file's parent)

Landed on branch `end-to-end-comparative-run` / PR #3:
- `Simulator/src/main.cpp` — CLI → composition parse → `PluginLoader` → bindings →
  `RunMatrixOrchestrator::run` → summary print → unload
- `Simulator/CMakeLists.txt` — `simulator_207190406_209543255` with `ENABLE_EXPORTS ON` +
  `$<TARGET_OBJECTS:simulator_registration>`
- `SimulationRunImpl` — catch `runMission()` exceptions, still save output map, score `-1`
- Orchestrator per-cell path — `{plugin}/{cell}_output_map.npy`

**Pass criteria (verified 2026-08-12 against `inputs/sim_compose.yaml`):**
- Binary launches without segfault
- Both `.so` plugins load without errors
- 24 `SimulationResult`s produced (scores `-1.0` until S8)
- ≥1 output `.npy` written (16 of 24 cells in full composition; some cells skip save on empty/failed maps)
- Binary exits with code 0 (both `-comparative` and `-competition`)

---

## Recommended split for the next session

| Who | What | Why now |
|---|---|---|
| **Sagi** alone | S8 real BFS scoring + wire into `SimulationRunImpl` | Meaningful scores in output |
| **Sagi** alone | S10 report writers + call from `main.cpp` | Final feature |
| **Sagi** alone | S11 output dir naming + document pattern in `README.md` | Final feature + unblocks Y11 |
| **Yoav** (after S11) | Y11 per-run file naming (follow pattern in `README.md`) | Pre-submission wiring |
| **Either** | HLD diagrams / README rewrite once naming stabilises | Submission deliverables |

---

## Remaining submission deliverables (either person)

- **`README.md`** rewrite — build presets, binary/`.so` names, both CLI invocations, output
  naming pattern. Either person; once S11/Y11 naming stabilises.
- **HLD as PDF** — each person diagrams their own components; one person assembles + exports.
  Keep in sync as code lands (graded e14/e15).
- **`bonus.txt`** — only if claiming a bonus (most likely: Sagi's lazy-load path in S6, if
  built on top of the mandatory eager-load). Must point at real file/line numbers.
- **Known Issues excel** — fill incrementally as corner cases are deliberately skipped.
- **Pre-submission structure pass** — run `.cursor/skills/pre-submission-review/SKILL.md`
  end to end before zipping.
