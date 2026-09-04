# Main score / wall parity after libm numeric restore

Measured 2026-09-03 against Release `build/opt`, Docker `drone-mapper-ex3-dev`,
`python3 .cursor/skills/verify-cell-runtime/scripts/time_each_cell.py --hang-timeout 200`,
composition `inputs/sim_compose.yaml` (24 cells), `num_threads=1` (one process per cell).

Branch: `fix-advcpp-rubric-findings` after restoring unwrap→libm→rewrap in movement, GPS,
beam, sphere, scoring, and planner hot paths (typed APIs kept).  
Baseline: `main` @ `9374aea`, clean rebuild of the same targets immediately after the branch run.

## Why

Quantity-path `si::cos` / `si::sin` and some `quantity_cast` multiplies changed float order vs
main. Outdoor cells drifted in score and sometimes path length. Public APIs remain
`PhysicalLength` / `Position3D` / angles; see `.cursor/rules/mp-units-strong-types.mdc`
(“Score / libm parity”) and `docs/advcpp-rubric-review.md`.

## Totals

| Tree | overall | FAIL | WARN | wall_sum | wall_max | scores vs main |
|------|---------|------|------|----------|----------|----------------|
| Branch (score-fix) | PASS | 0 | 2 | ~176s | 46.2s | **identical** |
| Main `9374aea` | PASS | 0 | 2 | 170.6s | 45.4s | — |

WARN cells: `large_out` short lidars (~45–46s). No cells ≥ 60s. Wall gap is sequential noise.

## Per-cell (score identical; walls branch vs main)

| Cell | Score | Branch wall (s) | Main wall (s) | Verdict |
|------|------:|----------------:|--------------:|---------|
| house_lower · small · long | 100.00 | 0.10 | 0.08 | PASS |
| house_lower · small · short | 100.00 | 0.08 | 0.07 | PASS |
| house_lower · large · long | 100.00 | 0.07 | 0.06 | PASS |
| house_lower · large · short | 100.00 | 0.11 | 0.07 | PASS |
| house_full · small · long | 37.19 | 10.69 | 10.92 | PASS |
| house_full · small · short | 48.33 | 13.78 | 12.78 | PASS |
| house_full · large · long | 20.99 | 8.05 | 7.60 | PASS |
| house_full · large · short | 15.34 | 9.14 | 8.55 | PASS |
| large_out · small · long | 82.27 | 7.26 | 7.12 | PASS |
| large_out · small · short | 44.56 | 46.11 | 45.41 | WARN |
| large_out · large · long | 74.01 | 14.52 | 14.07 | PASS |
| large_out · large · short | 32.00 | 46.24 | 45.17 | WARN |
| large_room · small · long | 95.04 | 0.22 | 0.20 | PASS |
| large_room · small · short | 95.10 | 0.28 | 0.24 | PASS |
| large_room · large · long | 94.74 | 0.14 | 0.13 | PASS |
| large_room · large · short | 96.47 | 0.23 | 0.21 | PASS |
| small_out · small · long | 77.05 | 3.51 | 3.35 | PASS |
| small_out · small · short | 78.69 | 4.66 | 4.42 | PASS |
| small_out · large · long | 50.55 | 1.73 | 1.56 | PASS |
| small_out · large · short | 93.43 | 5.61 | 5.31 | PASS |
| small_room · small · long | 86.47 | 0.89 | 0.81 | PASS |
| small_room · small · short | 72.76 | 1.32 | 1.18 | PASS |
| small_room · large · long | 87.04 | 0.54 | 0.50 | PASS |
| small_room · large · short | 87.83 | 0.82 | 0.79 | PASS |

## Raw CSVs

- `docs/benchmarks/2026-09-03-branch-after-score-fix-per-cell-wall.csv`
- `docs/benchmarks/2026-09-03-main-9374aea-clean-per-cell-wall.csv`
