# Benchmark summary

Comparison vs pre-B batching MC (`docs/benchmarks/2026-08-29-pre_b_baseline.md`,
column `ex2_comparable`): score sum 1371.3 → **1335.4**; steps 4946 → **17693** (~3.6×);
MAX_STEPS cells 0 → **8**. Expected project-B fall under one-scan-per-step accounting.

## Column `honest`

- cells: 24
- total_score (non-negative only): 1335.3717
- total_steps: 17693
- cells_scored: 24
- cells_max_steps: 8
- cells_errored: 0

### Ex2 band comparison

- `house_lower`: mean=100.00 band=[100.0, 100.0] **inside**
- `house_full`: mean=10.49 band=[56.0, 62.0] **below**
- `large_out`: mean=38.82 band=[80.0, 88.0] **below**
- `large_room`: mean=94.47 band=[92.0, 96.0] **inside**
- `small_out`: mean=24.74 band=[75.0, 89.0] **below**
- `small_room`: mean=65.32 band=[87.0, 90.0] **below**

