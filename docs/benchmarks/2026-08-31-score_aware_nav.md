# Score-aware nav patch (post-F follow-up)

Measured 2026-08-31 against Release `build/opt` (`-O3 -DNDEBUG`), Docker
`drone-mapper-ex3-dev`, `python scripts/benchmark/run_benchmark.py --build-dir build/opt
--columns honest`. Composition: `inputs/sim_compose.yaml` (24 cells), `num_threads=8`.

This is **not** a new project letter. It is the post-F investigation: why three
`house_full` cells scored 0.06, why we burned 10k steps without raising the score, and
the navigation/scan-gate fixes that followed. Spec still
`docs/superpowers/specs/2026-08-31-wavefront-frontier-exploration-design.md`.

## What changed in the policy

- In-bounds start whose sphere clips the mission AABB is passable (OOB is not a wall).
  Occupied face-neighbours still block. Large-drone `house_full` 3-step quit is gone.
- `WavefrontPlanner` emits `findUnstickPath` when the start sphere is Occupied-blocked
  (painted floor under a 7.5 cm radius).
- Cluster `cell_count` / approach key are the Empty surface, not the Unmapped volume
  (so `kMinInformationRate` can in principle bind).
- Downward cones are skipped at `max_height` (avoids `z_min` PO into the slab). If
  Unmapped is still below, the planner forces a one-step descend.
- Stay plan with an empty arrival sweep is treated as rate 0. On this column it did
  **not** bind — big cells still find a scan or a move for the whole budget.

Do **not** globally unmask cone gain. That path was measured (wip1/wip2) and collapses
rooms via pass-2 Occupied/PO.

## Totals vs earlier columns

| Column | Score sum | Steps | MAX_STEPS | ERROR |
|--------|-----------|-------|-----------|-------|
| Post-C honest | 1331.0 | 9101 | 2 | 0 |
| Post-F honest | 1402.6 | 50632 | 2 | 0 |
| **Score-aware nav** | **1589.4** | **92452** | **0** | **0** |

## Ex2 band comparison

- `house_lower`: mean=100.00 band=[100.0, 100.0] **inside** (spawn-above-box scoring artifact; do not scan into that volume)
- `house_full`: mean=30.46 band=[56.0, 62.0] **below** (was 6.50; 0.06×3 gone; small+short **48.33**)
- `large_out`: mean=59.84 band=[80.0, 88.0] **below** (was 43.47)
- `large_room`: mean=94.47 band=[92.0, 96.0] **inside**
- `small_out`: mean=30.46 band=[75.0, 89.0] **below** (was 38.36 — still the D-83 gap)
- `small_room`: mean=82.12 band=[87.0, 90.0] **below** (large-drone cells ~85–86, up from 49–59; small+short 72.5 vs post-F 85.6)

## Per-cell vs post-F (honest)

| Cell | Post-F | Score-aware nav |
|------|--------|-----------------|
| house_full small+long | 0.06 / 10000 MAX | **37.19** / 9995 |
| house_full small+short | 25.81 / 9994 | **48.33** / 9995 |
| house_full large+long | 0.06 / 3 | **20.99** / 9995 |
| house_full large+short | 0.06 / 3 | **15.34** / 9995 |
| large_out large+long | 53.72 | **72.86** |
| large_out large+short | 12.91 | **66.93** |
| large_out small+long | 70.33 | 35.98 (regressed) |
| small_room large+* | 59.4 / 49.8 | **85.4 / 86.3** |

## Early cut

`kMinInformationRate` still does not stop the 10k-step house/outdoor cells: they keep a
non-empty arrival sweep or a move. Empty-stay→rate 0 was added and produced a
byte-identical column to the pre-cut run. Cutting further needs a pass-1-done signal,
not a bigger constant.

## Remaining gap to ex2

Outdoor Empty carving (D's 83 on `small_out`) without re-enabling unmasked room gain, and
the rest of `house_full` (30 → 56–62), especially large-drone cells. Pickup:
`docs/mapping-algorithm-rewrite-pickup.md`.
