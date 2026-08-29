"""Resolve composition config_indices to stable cell labels."""

from __future__ import annotations

from pathlib import PurePosixPath
from typing import Any


def stem(path: str) -> str:
    return PurePosixPath(path.replace("\\", "/")).stem


def expected_cell_count(composition: dict[str, Any]) -> int:
    root = composition["simulation_compositions"]
    n = 0
    for group in root["simulations"]:
        n += (
            len(group["mission_configs"])
            * len(root["drone_configs"])
            * len(root["lidar_configs"])
        )
    return n


def label_for_indices(
    composition: dict[str, Any],
    simulation: int,
    mission: int,
    drone: int,
    lidar: int,
) -> str:
    root = composition["simulation_compositions"]
    group = root["simulations"][simulation]
    sim = stem(group["simulation_config"])
    mis = stem(group["mission_configs"][mission])
    dro = stem(root["drone_configs"][drone])
    lid = stem(root["lidar_configs"][lidar])
    return f"{sim}+{mis}|{dro}|{lid}"


def scenario_group_for_label(cell: str) -> str:
    """Map a cell label to an ex2 scenario-group key."""
    left = cell.split("|", 1)[0]
    _sim, mission = left.split("+", 1)
    if "house_mission_lower" in mission:
        return "house_lower"
    if "house_mission_full" in mission:
        return "house_full"
    if "large_mission_out" in mission:
        return "large_out"
    if "large_mission_room" in mission:
        return "large_room"
    if "small_mission_out" in mission:
        return "small_out"
    if "small_mission_room" in mission:
        return "small_room"
    raise ValueError(f"cannot map cell to scenario group: {cell}")
