# Post-F honest column (wavefront frontier)

Measured 2026-08-31 against `3ca01e7` plugins, Docker `drone-mapper-ex3-dev`,
`build/opt` **Release** (`CMAKE_BUILD_TYPE=Release`, CMake default `-O3 -DNDEBUG`).
The F plan's “`-O2`” meant this Release tree, not a separate `-O2` build.
Harness: `python scripts/benchmark/run_benchmark.py --build-dir build/opt --columns honest`
then `--columns adversarial`. Composition: `inputs/sim_compose.yaml` (24 cells),
`num_threads=8`. `kMinInformationRate` kept at **0.25**.

The 60 s spec gate **did not pass**. Cells finished (no timeout, no hang) but
`house_full` (drone_small / lidar_short) took **65.4 s** wall and `large_out`
(same drone/lidar) took **154.2 s**. A 10000-step mission at ~180 steps/s needs
~56 s of compute plus overhead; 60 s is ~3–5 s too tight for `house_full` and
far too tight for `large_out`. Matrix wall (8 threads) **203.1 s**.

## Totals vs earlier columns

| Column | Score sum | Steps | MAX_STEPS | ERROR |
|--------|-----------|-------|-----------|-------|
| Post-C honest | 1331.0 | 9101 | 2 | 0 |
| Lawnmower honest | 646.4 | 57108 | 0 | 0 |
| **Post-F honest** | **1402.6** | **50632** | **2** | **0** |
| Post-F adversarial | 468.2 | 1465 | 2 | 0 |

Honest sum **1402.6 > 1331.0** (post-C) and **> 646.4** (lawnmower). Zero `ERROR`
in either column.

## Column `honest`

- cells: 24
- total_score (non-negative only): 1402.5982
- total_steps: 50632
- cells_scored: 24
- cells_max_steps: 2
- cells_errored: 0

### Ex2 band comparison

- `house_lower`: mean=100.00 band=[100.0, 100.0] **inside**
- `house_full`: mean=6.50 band=[56.0, 62.0] **below**
- `large_out`: mean=43.47 band=[80.0, 88.0] **below**
- `large_room`: mean=92.30 band=[92.0, 96.0] **inside**
- `small_out`: mean=38.36 band=[75.0, 89.0] **below**
- `small_room`: mean=70.03 band=[87.0, 90.0] **below**

### Profiling cell (`drone_small` + `lidar_short`) vs D / post-C

| Cell | Post-C | D | Post-F | Wall (F) |
|------|--------|---|--------|----------|
| house_lower | 100.00 | 100.00 | 100.00 COMPLETED | 0.6 s |
| small_room | 82.57 | 76.37 MAX_STEPS | **85.60 COMPLETED** | 1.5 s |
| large_room | 92.65 | 93.04 MAX_STEPS | 93.01 MAX_STEPS | 0.8 s |
| small_out | 25.17 | **83.49** MAX_STEPS | 34.54 COMPLETED | 9.2 s |
| house_full | 6.31 | timed out @ 120 s | **25.81 COMPLETED** | **65.4 s** |
| large_out | 16.81 | timed out @ 120 s | 36.92 COMPLETED | **154.2 s** |

`small_room` recovers past post-C 82.57. `small_out` is **not** in the 80s — D's
83-class outdoor gain is given back (34.54 is still above post-C 25). `house_full`
moves up from 6.30 toward 56–62 on this one cell only (25.81); the other three
`house_full` variants collapse to ~0.06 (two finish in 3 steps; small+long hits
`MAX_STEPS` at 10000).

## `kMinInformationRate` sweep

Six profiling compose files under `tmp/profiling/compose_*.yaml`. Values
{0.10, 0.25, 0.50}. No 60 s kill; cells ran to completion.

| Cell | 0.10 score / wall | 0.25 score / wall | 0.50 score / wall |
|------|-------------------|-------------------|-------------------|
| house_lower | 100.00 / 0.5 s | 100.00 / 0.6 s | 100.00 / 0.5 s |
| small_room | 85.60 / 1.5 s | 85.60 / 1.5 s | 85.60 / 1.6 s |
| large_room | 93.01 / 0.9 s | 93.01 / 0.8 s | 93.01 / 0.9 s |
| small_out | 34.54 / 9.6 s | 34.54 / 9.2 s | 34.54 / 9.5 s |
| house_full | 25.81 / 64.7 s | 25.81 / 65.4 s | 25.81 / 64.2 s |
| large_out | 36.92 / 151.7 s | 36.92 / 154.2 s | 36.92 / 152.6 s |

All three rates produced **byte-identical scores and step counts**. The constant
is not binding on these cells (they hit `max_steps` or finish for other reasons).
Kept **0.25**. No other rate is strictly better on `small_room`, and none restores
`small_out`'s 83-class score.

## Column `adversarial`

CSV: `docs/benchmarks/2026-08-31-post_f_adversarial.csv`. Hits-only foreign MC.
Zero `ERROR`. Score sum 468.2, 1465 steps, 2× `MAX_STEPS` (`large_room` long-lidar
cells). Most other cells finish in ~28 steps with near-zero outdoor/room scores;
`house_lower` stays 100. Robustness check only — not the optimization target.

## Spec criteria (honest)

1. Every cell within 60 s at Release: **failed** (`house_full` 65.4 s, `large_out` 154.2 s). Cells completed; gate is too tight.
2. Sum > 1331.0: **yes** (1402.6). Profiling `small_room` ≥ 82.57: **yes** (85.60). Every D-finished cell ≥ D: **no** (`small_out` 34.54 < 83.49).
3. `small_out` stays in the 80s: **no** (mean 38.36; profiling 34.54).
4. `house_full` moves up from 6.30 toward 56–62: **partial** (profiling 25.81; group mean 6.50 because three variants ~0.06).
5. Zero `ERROR` in either column: **yes**.
