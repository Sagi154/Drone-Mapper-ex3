# Benchmark summary

Comparison vs post-B honest MC (`docs/benchmarks/2026-08-29-post_b_honest.md`):
score sum 1335.4 → **1331.0** (~flat); steps 17693 → **9101** (~0.51×); MAX_STEPS
cells 8 → **2**. Project C gain-gating + lidar-derived scan directions cut wasted scans;
clearance fix did not introduce errors.

## Column `honest`

- cells: 24
- total_score (non-negative only): 1331.0355
- total_steps: 9101
- cells_scored: 24
- cells_max_steps: 2
- cells_errored: 0

### Ex2 band comparison

- `house_lower`: mean=100.00 band=[100.0, 100.0] **inside**
- `house_full`: mean=8.08 band=[56.0, 62.0] **below**
- `large_out`: mean=31.06 band=[80.0, 88.0] **below**
- `large_room`: mean=93.15 band=[92.0, 96.0] **inside**
- `small_out`: mean=33.26 band=[75.0, 89.0] **below**
- `small_room`: mean=67.21 band=[87.0, 90.0] **below**
