from pathlib import Path

import yaml

from cell_labels import (
    expected_cell_count,
    label_for_indices,
    scenario_group_for_label,
    stem,
)

REPO = Path(__file__).resolve().parents[3]
COMPOSE = REPO / "inputs" / "sim_compose.yaml"


def _load():
    return yaml.safe_load(COMPOSE.read_text(encoding="utf-8"))


def test_stem_strips_path_and_extension():
    assert stem("mission/house_mission_full.yaml") == "house_mission_full"


def test_expected_cell_count_sim_compose_is_24():
    assert expected_cell_count(_load()) == 24


def test_label_house_full_drone_large_lidar_short():
    label = label_for_indices(_load(), 0, 1, 1, 1)
    assert label == "house_simulation+house_mission_full|drone_large|lidar_short"


def test_scenario_group_mapping():
    assert (
        scenario_group_for_label(
            "house_simulation+house_mission_lower|drone_small|lidar_long"
        )
        == "house_lower"
    )
    assert (
        scenario_group_for_label(
            "large_simulation_room+large_mission_room|drone_small|lidar_short"
        )
        == "large_room"
    )
