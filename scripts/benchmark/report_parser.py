from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

from cell_labels import expected_cell_count, label_for_indices


@dataclass
class Row:
    column: str
    cell: str
    score: float
    steps: int
    status: str
    errors: list[dict[str, str]] = field(default_factory=list)


def parse_simulation_output(
    path: Path | str,
    composition: dict[str, Any],
    column: str,
) -> list[Row]:
    path = Path(path)
    root = yaml.safe_load(path.read_text(encoding="utf-8"))
    runs = root["score_report"]["runs"]
    expected = expected_cell_count(composition)
    if len(runs) != expected:
        raise ValueError(f"{path}: expected {expected} runs, got {len(runs)}")

    seen: set[tuple[int, int, int, int]] = set()
    rows: list[Row] = []
    for run in runs:
        idx = run["config_indices"]
        key = (
            int(idx["simulation"]),
            int(idx["mission"]),
            int(idx["drone"]),
            int(idx["lidar"]),
        )
        if key in seen:
            raise ValueError(f"{path}: duplicate config_indices {key}")
        seen.add(key)

        results = run.get("mission_results") or []
        status = results[0]["status"] if results else "ERROR"
        steps = int(results[0]["steps"]) if results else 0
        errors = list(results[0].get("errors") or []) if results else []

        rows.append(
            Row(
                column=column,
                cell=label_for_indices(composition, *key),
                score=float(run["mission_score"]),
                steps=steps,
                status=str(status),
                errors=errors,
            )
        )
    return rows
