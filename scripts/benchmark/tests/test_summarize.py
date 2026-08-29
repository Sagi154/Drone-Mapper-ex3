from pathlib import Path

from report_parser import Row
from summarize import baseline_diff, ex2_band_report, read_csv, write_csv

REPO = Path(__file__).resolve().parents[3]
REF = REPO / "docs" / "benchmarks" / "ex2-reference.csv"


def test_ex2_band_inside():
    rows = [
        Row(
            "ex2_comparable",
            "small_simulation_room+small_mission_room|drone_small|lidar_short",
            88.0,
            10,
            "COMPLETED",
        ),
        Row(
            "ex2_comparable",
            "small_simulation_room+small_mission_room|drone_small|lidar_long",
            89.0,
            11,
            "COMPLETED",
        ),
        Row(
            "ex2_comparable",
            "small_simulation_room+small_mission_room|drone_large|lidar_short",
            87.5,
            12,
            "COMPLETED",
        ),
        Row(
            "ex2_comparable",
            "small_simulation_room+small_mission_room|drone_large|lidar_long",
            90.0,
            13,
            "COMPLETED",
        ),
    ]
    report = ex2_band_report(rows, REF)
    small = next(r for r in report if r["group"] == "small_room")
    assert small["verdict"] == "inside"


def test_baseline_diff_flags_score_drop():
    base = [Row("c", "cell_a", 90.0, 10, "COMPLETED")]
    cur = [Row("c", "cell_a", 80.0, 12, "COMPLETED")]
    _deltas, regressions = baseline_diff(cur, base)
    assert any("cell_a" in r for r in regressions)


def test_csv_roundtrip(tmp_path):
    rows = [Row("ex2_comparable", "a+b|c|d", 1.5, 3, "COMPLETED")]
    path = tmp_path / "out.csv"
    write_csv(rows, path)
    back = read_csv(path)
    assert back[0].score == 1.5 and back[0].steps == 3
