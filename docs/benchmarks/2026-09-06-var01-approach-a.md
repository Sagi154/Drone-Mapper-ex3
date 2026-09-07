# Cell runtime baseline after VAR-01 Approach A (2026-09-06)

Measured 2026-09-06 against Release `build/opt`, Docker `drone-mapper-ex3-dev`,
`python3 .cursor/skills/verify-cell-runtime/scripts/time_each_cell.py`,
composition `inputs/sim_compose.yaml` (24 cells), `num_threads=1` (one process per cell).

**CSV refresh (same day):** `known-issues-fixes` @ `c70762c` after AdvCpp rubric
refactors. Scores/steps unchanged vs `a254810`. Walls 29.9s / 3.0s (was 30.4s /
3.0s). Rebuild kept vcpkg in `build/opt`; only `tmp/bench-out` was wiped.

Original VAR-01 landing tree: `known-issues-fixes` @ `a254810` (algorithm tip
`2732aca`: corner-anchored `sphereIntersectsCellBox` +
`kLocalSearchExpansionCap = 3000` with one full-map fallback).

This **replaces** `.cursor/skills/verify-cell-runtime`’s 2026-09-03 table as the
canonical per-cell reference. The 2026-09-03 CSV remains historical
(`docs/benchmarks/2026-09-03-main-score-parity.md`).

## Totals

| Field | Value |
|-------|--------|
| overall | **PASS** — FAIL=0, WARN=0, wall_sum=29.9s, wall_max=3.0s, cells_ge_60s=0 |
| score_sum | **1832.747** |
| CSV (skill) | `.cursor/skills/verify-cell-runtime/baseline-2026-09-06-per-cell-wall.csv` |
| CSV (docs) | `docs/benchmarks/2026-09-06-var01-approach-a-per-cell-wall.csv` |

Vs 2026-09-03: score_sum 1769.8 → 1832.7; wall_sum ~176s → 30.4s; the two
`large_out` short-lidar WARNs (~45–46s) are gone.

## What changed in the algorithm

1. **Corner-anchored sphere-vs-voxel clearance** (`sphereIntersectsCellBox`) so the
   planner matches `Map3DImpl` / `skeleton_host` voxel boxes. Independence **VAR-01**
   (`HOST_ILLEGAL_MOVE_ATTEMPTS=0`) PASSes.
2. **Local replan search cap** (`detail::kLocalSearchExpansionCap = 3000`) with a
   single full-map fallback when the local BFS finds no frontier cluster.

Do not treat the 2026-09-03 WARN pair as expected anymore.
