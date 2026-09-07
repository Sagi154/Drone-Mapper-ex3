# Cell runtime verification

Date (UTC): 2026-09-06
SHA: `8fda059525d9092ab1f4f49aee32d849a136f182`
Build: Release `build/opt`
CSV: `tmp/bench-out/per-cell-wall.csv`

| Cell | Group | Score | Steps | Status | wall_s | Verdict |
|------|-------|------:|------:|--------|-------:|---------|
| house_simulation+house_mission_lower\|drone_small\|lidar_long | house_lower | 100.00 | 3 | COMPLETED | 0.2 | PASS |
| house_simulation+house_mission_lower\|drone_small\|lidar_short | house_lower | 100.00 | 3 | COMPLETED | 0.1 | PASS |
| house_simulation+house_mission_lower\|drone_large\|lidar_long | house_lower | 100.00 | 3 | COMPLETED | 0.1 | PASS |
| house_simulation+house_mission_lower\|drone_large\|lidar_short | house_lower | 100.00 | 3 | COMPLETED | 0.1 | PASS |
| house_simulation+house_mission_full\|drone_small\|lidar_long | house_full | 47.46 | 2700 | COMPLETED | 3.0 | PASS |
| house_simulation+house_mission_full\|drone_small\|lidar_short | house_full | 21.00 | 1100 | COMPLETED | 2.3 | PASS |
| house_simulation+house_mission_full\|drone_large\|lidar_long | house_full | 20.08 | 800 | COMPLETED | 1.9 | PASS |
| house_simulation+house_mission_full\|drone_large\|lidar_short | house_full | 14.89 | 600 | COMPLETED | 1.5 | PASS |
| large_simulation_out+large_mission_out\|drone_small\|lidar_long | large_out | 73.50 | 1700 | COMPLETED | 0.9 | PASS |
| large_simulation_out+large_mission_out\|drone_small\|lidar_short | large_out | 74.95 | 4100 | COMPLETED | 2.7 | PASS |
| large_simulation_out+large_mission_out\|drone_large\|lidar_long | large_out | 75.59 | 3400 | COMPLETED | 1.5 | PASS |
| large_simulation_out+large_mission_out\|drone_large\|lidar_short | large_out | 72.02 | 3500 | COMPLETED | 2.9 | PASS |
| large_simulation_room+large_mission_room\|drone_small\|lidar_long | large_room | 97.68 | 187 | COMPLETED | 0.2 | PASS |
| large_simulation_room+large_mission_room\|drone_small\|lidar_short | large_room | 97.26 | 193 | COMPLETED | 0.2 | PASS |
| large_simulation_room+large_mission_room\|drone_large\|lidar_long | large_room | 97.31 | 198 | COMPLETED | 0.2 | PASS |
| large_simulation_room+large_mission_room\|drone_large\|lidar_short | large_room | 97.52 | 198 | COMPLETED | 0.3 | PASS |
| small_simulation_out+small_mission_out\|drone_small\|lidar_long | small_out | 93.87 | 1993 | COMPLETED | 1.1 | PASS |
| small_simulation_out+small_mission_out\|drone_small\|lidar_short | small_out | 94.14 | 1993 | COMPLETED | 2.4 | PASS |
| small_simulation_out+small_mission_out\|drone_large\|lidar_long | small_out | 45.38 | 1600 | COMPLETED | 3.0 | PASS |
| small_simulation_out+small_mission_out\|drone_large\|lidar_short | small_out | 83.86 | 1994 | COMPLETED | 2.4 | PASS |
| small_simulation_room+small_mission_room\|drone_small\|lidar_long | small_room | 85.96 | 990 | COMPLETED | 0.9 | PASS |
| small_simulation_room+small_mission_room\|drone_small\|lidar_short | small_room | 87.31 | 993 | COMPLETED | 1.1 | PASS |
| small_simulation_room+small_mission_room\|drone_large\|lidar_long | small_room | 85.30 | 996 | COMPLETED | 0.9 | PASS |
| small_simulation_room+small_mission_room\|drone_large\|lidar_short | small_room | 67.65 | 993 | COMPLETED | 1.4 | PASS |

Overall: **PASS** — FAIL=0, WARN=0, wall_sum=31.2s, wall_max=3.0s, cells_ge_60s=0. Scores match the 2026-09-06 baseline (score_sum 1832.747).
