# Work Split — Current Status & Next Steps

Generated against the real source tree on 2026-08-10.

---

## Actual completion state (source tree vs. workplan markers)

| Item | Workplan says | Reality |
|---|---|---|
| Y10 `SimulationRunImpl` | ⏳ DEFERRED | ✅ **Implemented** (`Simulator/src/SimulationRunImpl.cpp` exists) |
| S10 report writers | not started | ❌ `ComparativeReportWriter.cpp`, `CompetitiveReportWriter.cpp`, `SimulationOutputYamlWriter.cpp` missing from `src/io/` |
| S11 output dir naming | not started | ❌ `OutputDirHelper.cpp` missing from `src/io/` |
| S8 real BFS scoring | ⏳ stub | ⏳ `MapsComparison.cpp` exists, `compare()` returns `-1.0` |
| `main.cpp` | not started | ❌ missing from `src/` |

---

## What Sagi can do — zero coordination with Yoav needed

### S8 — Fill in `MapsComparison::compare()` (no blocker)
`U4 (SimulationCoordUtil)` is done. Port the real BFS/flood-fill 0–100 scoring from
`../Drone-Mapper-ex2/src/MapsComparison.cpp`. Use a hand-written fake `IMap3D` in tests — no
dependency on Yoav's `Map3DImpl`.

### S10 — Comparative/competitive/per-plugin YAML report writers (no blocker)
S9 (orchestrator + output shape) is done. Write:
- `Simulator/src/io/ComparativeReportWriter.cpp`
- `Simulator/src/io/CompetitiveReportWriter.cpp`
- `Simulator/src/io/SimulationOutputYamlWriter.cpp`

against the exact schemas in `.cursor/rules/simulator-cli-and-outputs.mdc`. Hand-build
`PluginMatrixResult` literals for unit tests — no dependency on Yoav's track.

### S11 — Output directory naming (blocked only on S10, which Sagi owns)
Once S10 is done, write `Simulator/src/io/OutputDirHelper.cpp`:
- `comparative_results_<time>` / `competition_<time>` patterns
- collision counter appended when directory already exists

Document the chosen **per-run artifact filename pattern** in `README.md` — Yoav's Y11 reads
that and follows it; no code handoff is needed.

**Sagi's order: S8 → S10 → S11** (each feeds the next; all owned by Sagi)

---

## What Yoav can do — zero coordination with Sagi needed

### Y11 — Per-run output file naming
Y10 (`SimulationRunImpl`) is already implemented. The only dependency is **S11's naming
pattern** — once Sagi documents it in `README.md`, Yoav wires it into `SimulationRunImpl`.

While waiting for S11: review `SimulationRunImpl.cpp` against the **OQ-Y1 open question**
(empty `output_path` → skip map save silently, do not trigger `MAP_SAVE_FAILED`). This is
documented in `docs/workplan.md §Implementation open questions`.

**Yoav's honest state:** very little purely-his work remains. Best uses of time while Sagi
works on S8/S10/S11:
- Help with the vertical slice `main.cpp` (joint task below — most impactful)
- HLD diagrams for `MissionControl/` components (submission deliverable)
- OQ-Y1 fix in `SimulationRunImpl` if not already handled

---

## Joint task — do this together on the same PC

### Vertical slice — `main.cpp` + `CMakeLists.txt` wiring
**Plan:** `docs/vertical-slice-plan.md`

All components exist. This is glue code only — no new logic. The plan has the full pseudocode
for `main.cpp` and the exact `add_executable` block for `CMakeLists.txt`.

**Why this is the most valuable thing to do next:** it immediately proves whether
`dlopen`/`ENABLE_EXPORTS`/factory plumbing actually works end-to-end. Everything after this
point (S10 report output, S11 naming, full integration) is filling in features around a
mechanism already known to work. Discovering a wiring bug now is orders of magnitude cheaper
than discovering it after S10/S11 are written.

**Pass criteria:**
- Binary launches without segfault
- Both `.so` plugins load without errors
- At least one `SimulationResult` is produced (score of `-1.0` is fine at this stage)
- At least one output `.npy` map is written
- Binary exits with code 0

---

## Recommended split for the next session

| Who | What | Why now |
|---|---|---|
| **Yoav** alone | OQ-Y1 fix in `SimulationRunImpl` | Unblocks vertical slice correctness |
| **Together** | Vertical slice `main.cpp` + `CMakeLists.txt` | Unblocks everything — most impactful |
| **Sagi** alone (after vertical slice) | S8 real BFS scoring | Meaningful scores in output |
| **Sagi** alone | S10 report writers | Final feature |
| **Sagi** alone | S11 output dir naming + document pattern in `README.md` | Final feature + unblocks Y11 |
| **Yoav** (after S11) | Y11 per-run file naming (follow pattern in `README.md`) | Pre-submission wiring |

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
