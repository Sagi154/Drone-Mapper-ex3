"""Summarize benchmark rows: CSV I/O, totals, ex2 bands, baseline diffs."""

from __future__ import annotations

import csv
from collections import defaultdict
from pathlib import Path
from typing import Any

from cell_labels import scenario_group_for_label
from report_parser import Row


def write_csv(rows: list[Row], path: Path | str) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(
            f, fieldnames=["column", "cell", "score", "steps", "status"]
        )
        writer.writeheader()
        for row in rows:
            writer.writerow(
                {
                    "column": row.column,
                    "cell": row.cell,
                    "score": row.score,
                    "steps": row.steps,
                    "status": row.status,
                }
            )


def read_csv(path: Path | str) -> list[Row]:
    path = Path(path)
    rows: list[Row] = []
    with path.open(encoding="utf-8", newline="") as f:
        for raw in csv.DictReader(f):
            rows.append(
                Row(
                    column=raw["column"],
                    cell=raw["cell"],
                    score=float(raw["score"]),
                    steps=int(raw["steps"]),
                    status=raw["status"],
                )
            )
    return rows


def column_totals(rows: list[Row]) -> dict[str, Any]:
    scored = [r for r in rows if r.score >= 0]
    return {
        "n": len(rows),
        "total_score": sum(r.score for r in scored),
        "total_steps": sum(r.steps for r in rows),
        "cells_scored": len(scored),
        "cells_max_steps": sum(1 for r in rows if r.status == "MAX_STEPS"),
        "cells_errored": sum(1 for r in rows if r.status == "ERROR" or r.score < 0),
    }


def _load_ex2_reference(path: Path | str) -> dict[str, tuple[float, float]]:
    path = Path(path)
    bands: dict[str, tuple[float, float]] = {}
    with path.open(encoding="utf-8", newline="") as f:
        for raw in csv.DictReader(f):
            bands[raw["group"]] = (float(raw["low"]), float(raw["high"]))
    return bands


def ex2_band_report(rows: list[Row], reference_csv: Path | str) -> list[dict[str, Any]]:
    bands = _load_ex2_reference(reference_csv)
    by_group: dict[str, list[float]] = defaultdict(list)
    for row in rows:
        if row.score < 0:
            continue
        by_group[scenario_group_for_label(row.cell)].append(row.score)

    report: list[dict[str, Any]] = []
    for group, (low, high) in bands.items():
        scores = by_group.get(group, [])
        if not scores:
            report.append(
                {
                    "group": group,
                    "mean_score": None,
                    "band_low": low,
                    "band_high": high,
                    "verdict": "missing",
                }
            )
            continue
        mean = sum(scores) / len(scores)
        if mean < low:
            verdict = "below"
        elif mean > high:
            verdict = "above"
        else:
            verdict = "inside"
        report.append(
            {
                "group": group,
                "mean_score": mean,
                "band_low": low,
                "band_high": high,
                "verdict": verdict,
            }
        )
    return report


def baseline_diff(
    current: list[Row], baseline: list[Row]
) -> tuple[list[str], list[str]]:
    base_map = {(r.column, r.cell): r for r in baseline}
    deltas: list[str] = []
    regressions: list[str] = []
    for row in current:
        key = (row.column, row.cell)
        prev = base_map.get(key)
        if prev is None:
            deltas.append(f"{key}: new cell score={row.score}")
            continue
        d_score = row.score - prev.score
        d_steps = row.steps - prev.steps
        deltas.append(
            f"{row.column}/{row.cell}: score {prev.score}->{row.score} ({d_score:+.3f}), "
            f"steps {prev.steps}->{row.steps} ({d_steps:+d}), "
            f"status {prev.status}->{row.status}"
        )
        worse_status = (prev.status == "COMPLETED" and row.status != "COMPLETED") or (
            prev.status != "ERROR" and row.status == "ERROR"
        )
        if row.score < prev.score or worse_status:
            regressions.append(
                f"{row.column}/{row.cell}: score {prev.score}->{row.score}, "
                f"status {prev.status}->{row.status}"
            )
    return deltas, regressions


def render_markdown_summary(
    rows_by_column: dict[str, list[Row]],
    *,
    ex2_reference: Path | str | None = None,
    baseline: list[Row] | None = None,
) -> str:
    lines: list[str] = ["# Benchmark summary", ""]
    for column, rows in rows_by_column.items():
        totals = column_totals(rows)
        lines.append(f"## Column `{column}`")
        lines.append("")
        lines.append(f"- cells: {totals['n']}")
        lines.append(f"- total_score (non-negative only): {totals['total_score']:.4f}")
        lines.append(f"- total_steps: {totals['total_steps']}")
        lines.append(f"- cells_scored: {totals['cells_scored']}")
        lines.append(f"- cells_max_steps: {totals['cells_max_steps']}")
        lines.append(f"- cells_errored: {totals['cells_errored']}")
        lines.append("")
        if ex2_reference is not None:
            lines.append("### Ex2 band comparison")
            lines.append("")
            for entry in ex2_band_report(rows, ex2_reference):
                mean = entry["mean_score"]
                mean_s = "n/a" if mean is None else f"{mean:.2f}"
                lines.append(
                    f"- `{entry['group']}`: mean={mean_s} "
                    f"band=[{entry['band_low']}, {entry['band_high']}] "
                    f"**{entry['verdict']}**"
                )
            lines.append("")
    if baseline is not None:
        flat = [r for rows in rows_by_column.values() for r in rows]
        _deltas, regressions = baseline_diff(flat, baseline)
        lines.append("## Baseline regressions")
        lines.append("")
        if not regressions:
            lines.append("None.")
        else:
            for item in regressions:
                lines.append(f"- {item}")
        lines.append("")
    return "\n".join(lines)
