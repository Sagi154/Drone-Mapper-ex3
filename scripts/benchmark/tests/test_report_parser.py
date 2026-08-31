from pathlib import Path

import pytest
import yaml

from report_parser import parse_simulation_output

FIX = Path(__file__).parent / "fixtures" / "sample_simulation_output.yaml"

COMPOSE_3 = {
    "simulation_compositions": {
        "simulations": [
            {
                "simulation_config": "simulation/small_simulation_room.yaml",
                "mission_configs": ["mission/small_mission_room.yaml"],
            }
        ],
        "drone_configs": [
            "drone/drone_small.yaml",
            "drone/drone_large.yaml",
            "drone/drone_extra.yaml",
        ],
        "lidar_configs": ["lidar/lidar_long.yaml"],
    }
}


def test_parse_three_rows():
    rows = parse_simulation_output(FIX, COMPOSE_3, column="ex2_comparable")
    assert len(rows) == 3
    assert rows[0].score == 88.75 and rows[0].status == "COMPLETED" and rows[0].steps == 42
    assert rows[1].status == "MAX_STEPS"
    assert rows[2].score == -1.0 and rows[2].status == "ERROR"
    assert "drone_small" in rows[0].cell


def test_rejects_wrong_count():
    bad = {
        "simulation_compositions": {
            "simulations": COMPOSE_3["simulation_compositions"]["simulations"],
            "drone_configs": ["drone/drone_small.yaml"],
            "lidar_configs": ["lidar/lidar_long.yaml"],
        }
    }
    with pytest.raises(ValueError, match="expected 1"):
        parse_simulation_output(FIX, bad, column="ex2_comparable")


def test_rejects_duplicate_indices(tmp_path):
    data = yaml.safe_load(FIX.read_text(encoding="utf-8"))
    data["score_report"]["runs"][1]["config_indices"] = dict(
        data["score_report"]["runs"][0]["config_indices"]
    )
    p = tmp_path / "dup.yaml"
    p.write_text(yaml.dump(data), encoding="utf-8")
    with pytest.raises(ValueError, match="duplicate"):
        parse_simulation_output(p, COMPOSE_3, column="x")
