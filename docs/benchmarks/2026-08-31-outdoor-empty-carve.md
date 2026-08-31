# Outdoor Empty-carve (post score-aware nav)

Measured 2026-08-31 against Release `build/opt` (`-O3 -DNDEBUG`), Docker
`drone-mapper-ex3-dev`, `python scripts/benchmark/run_benchmark.py --build-dir build/opt
--columns honest`. Composition: `inputs/sim_compose.yaml` (24 cells), `num_threads=8`.

Not a new project letter. This is the score-aware-nav follow-up that restores D-class
outdoor Empty carving **without** globally unmasking cone gain (that path collapsed rooms).

## What changed in the policy

- **Open-volume missions** (XY and height spans ≥ 200 cm: `small_out` / `large_out`):
  count Unmapped volume in a cone, not just the frontier shell, so MissionControl can
  carve spawn-reachable Empty air (pass-1) and Empty outside that set (pass-2 credit).
- **Downward cones stay masked** (and are skipped at `max_height` / in the house). Full
  unmask of look-down painted ground Occupied/PO and dropped `large_out` large-drone cells.
- **Long lidar on `large_out` stays masked horizontally** (`z_max` > 90 cm). Horizon
  volume with 150 cm range punched into buildings (73 → 39). Short lidar (`z_max` ≤ 90)
  keeps horizon volume — `large_out` large+short **85.6** (inside 80–88).
- **`small_out` (200 cm cube)** allows horizon volume for both lidars, and caps the
  arrival sweep at 4 directions so the 2000-step budget visits more poses.
- **Outdoor long-lidar ranking** uses cluster `volume_count` (Unmapped+Empty pocket) with
  Empty-surface approach; short lidar keeps Empty-surface ranking.
- **House**: skip downward cones at every layer; after a stay-and-sweep, prefer one-step
  descend if Unmapped remains in the column. On this column that did **not** move
  `house_full` (still 30.46) — the local XY surface never clears.

Do **not** globally unmask cone gain. Rooms stay masked.

## Totals vs earlier columns

| Column | Score sum | Steps | MAX_STEPS | ERROR |
|--------|-----------|-------|-----------|-------|
| Post-C honest | 1331.0 | 9101 | 2 | 0 |
| Post-F honest | 1402.6 | 50632 | 2 | 0 |
| Score-aware nav | 1589.4 | 92452 | 0 | 0 |
| **Outdoor Empty-carve** | **1793.4** | **92464** | **3** | **0** |

## Ex2 band comparison

- `house_lower`: mean=100.00 band=[100.0, 100.0] **inside**
- `house_full`: mean=30.46 band=[56.0, 62.0] **below** (unchanged vs score-aware nav; gap 25.5)
- `large_out`: mean=68.50 band=[80.0, 88.0] **below** (was 59.84; gap 11.5)
- `large_room`: mean=94.47 band=[92.0, 96.0] **inside**
- `small_out`: mean=72.81 band=[75.0, 89.0] **below** (was 30.46; gap 2.2 — three cells in/above band)
- `small_room`: mean=82.11 band=[87.0, 90.0] **below** (unchanged; large-drone 85–86)

## Per-cell highlights vs score-aware nav

| Cell | Score-aware nav | Outdoor Empty-carve |
|------|-----------------|---------------------|
| large_out large+short | 66.93 | **85.58** (inside band) |
| large_out small+long | 35.98 | **51.28** |
| small_out small+long | 33.74 | **81.83** (inside band) |
| small_out small+short | 29.05 | **82.35** (inside band) |
| small_out large+short | 26.78 | **90.89** |
| small_out large+long | 32.26 | 36.16 (still the group drag) |
| house_full * | 15–48 | **identical** |
| rooms / house_lower | — | **no regress** (tiny 84.33→84.28 noise on small_room small+long) |

## Remaining gap to ex2

`house_full` (30 → 56–62), `large_out` mean (68.5 → 80–88; small+long still short of F's 70),
and `small_out` large+long (36). Pickup: `docs/mapping-algorithm-rewrite-pickup.md`.
