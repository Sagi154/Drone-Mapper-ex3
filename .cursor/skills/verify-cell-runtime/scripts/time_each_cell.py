#!/usr/bin/env python3
"""Time each sim_compose cell serially (Release). Judge wall_s vs the 60s bar."""

from __future__ import annotations

import argparse
import csv
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

import yaml

SCRIPT_DIR = Path(__file__).resolve().parent


def find_repo(start: Path) -> Path:
    for p in [start, *start.parents]:
        if (p / "inputs" / "sim_compose.yaml").is_file():
            return p
    raise SystemExit("could not find repo root (inputs/sim_compose.yaml)")


REPO = find_repo(SCRIPT_DIR)
sys.path.insert(0, str(REPO / "scripts" / "benchmark"))
from cell_labels import label_for_indices, scenario_group_for_label  # noqa: E402
from report_parser import parse_simulation_output  # noqa: E402

# wall_s >= fail_s → FAIL. wall_s >= warn_s → WARN. small_room warn_s is tighter.
BUDGETS: dict[str, tuple[float, float]] = {
    # group: (warn_s, fail_s)
    "house_lower": (10.0, 60.0),
    "house_full": (45.0, 60.0),
    "large_out": (45.0, 60.0),
    "large_room": (15.0, 60.0),
    "small_out": (30.0, 60.0),
    "small_room": (15.0, 60.0),
}


def rel(from_dir: Path, target: Path) -> str:
    return os.path.relpath(target.resolve(), from_dir.resolve()).replace("\\", "/")


def verdict(group: str, wall_s: float, status: str, score: object) -> str:
    if status.startswith("HANG") or status.startswith("SIM_EXIT"):
        return "FAIL"
    try:
        if float(score) < 0:
            return "FAIL"
    except (TypeError, ValueError):
        return "FAIL"
    warn_s, fail_s = BUDGETS.get(group, (45.0, 60.0))
    if wall_s >= fail_s:
        return "FAIL"
    if wall_s >= warn_s:
        return "WARN"
    return "PASS"


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--build-dir", type=Path, default=REPO / "build" / "opt")
    p.add_argument("--out-dir", type=Path, default=REPO / "tmp" / "bench-out")
    p.add_argument("--hang-timeout", type=float, default=180.0)
    p.add_argument(
        "--only-group",
        action="append",
        default=[],
        help="Repeatable. house_lower, house_full, large_out, large_room, small_out, small_room",
    )
    p.add_argument(
        "--only-cell",
        action="append",
        default=[],
        help="Repeatable exact cell label (sim+mission|drone|lidar)",
    )
    p.add_argument(
        "--only-cell-file",
        type=Path,
        help="Text file of exact cell labels, one per line",
    )
    return p.parse_args()


def main() -> int:
    args = parse_args()
    build = args.build_dir.resolve()
    sim = build / "Simulator" / "simulator_207190406_209543255"
    algo = build / "Algorithm" / "Algorithm_207190406_209543255.so"
    mc = build / "MissionControl" / "MissionControl_207190406_209543255.so"
    for required in (sim, algo, mc):
        if not required.is_file():
            print(f"missing binary: {required}", file=sys.stderr)
            return 2

    compose_path = REPO / "inputs" / "sim_compose.yaml"
    composition = yaml.safe_load(compose_path.read_text(encoding="utf-8"))
    root = composition["simulation_compositions"]
    args.out_dir.mkdir(parents=True, exist_ok=True)
    scratch = REPO / "tmp" / "cell-timing"
    if scratch.exists():
        shutil.rmtree(scratch)
    yaml_dir = scratch / "yamls"
    yaml_dir.mkdir(parents=True)

    only = set(args.only_group)
    only_cells = set(args.only_cell)
    if args.only_cell_file is not None:
        text = args.only_cell_file.read_text(encoding="utf-8")
        only_cells.update(
            line.strip() for line in text.splitlines() if line.strip() and not line.startswith("#")
        )
    rows_out: list[dict[str, object]] = []
    n = 0
    planned = 0
    for sim_i, group in enumerate(root["simulations"]):
        for mis_i, mission in enumerate(group["mission_configs"]):
            for drone_i, drone in enumerate(root["drone_configs"]):
                for lidar_i, lidar in enumerate(root["lidar_configs"]):
                    planned += 1
                    label = label_for_indices(
                        composition, sim_i, mis_i, drone_i, lidar_i
                    )
                    scen = scenario_group_for_label(label)
                    if only and scen not in only:
                        continue
                    if only_cells and label not in only_cells:
                        continue
                    n += 1
                    one = {
                        "simulation_compositions": {
                            "simulations": [
                                {
                                    "simulation_config": rel(
                                        yaml_dir,
                                        REPO / "inputs" / group["simulation_config"],
                                    ),
                                    "mission_configs": [
                                        rel(yaml_dir, REPO / "inputs" / mission)
                                    ],
                                }
                            ],
                            "drone_configs": [
                                rel(yaml_dir, REPO / "inputs" / drone)
                            ],
                            "lidar_configs": [
                                rel(yaml_dir, REPO / "inputs" / lidar)
                            ],
                        }
                    }
                    ypath = yaml_dir / f"cell_{planned:02d}.yaml"
                    ypath.write_text(yaml.dump(one), encoding="utf-8")
                    mc_dir = scratch / f"mc_{planned:02d}"
                    mc_dir.mkdir()
                    shutil.copy2(mc, mc_dir / mc.name)
                    cmd = [
                        str(sim),
                        "-comparative",
                        f"simulation={ypath}",
                        f"mission_control_folder={mc_dir}",
                        f"algorithm={algo}",
                        "num_threads=1",
                    ]
                    print(f"[{n}] START {label}", flush=True)
                    t0 = time.perf_counter()
                    try:
                        proc = subprocess.run(
                            cmd,
                            cwd=str(REPO),
                            capture_output=True,
                            text=True,
                            timeout=args.hang_timeout,
                        )
                        wall_s = time.perf_counter() - t0
                        timed_out = False
                    except subprocess.TimeoutExpired:
                        wall_s = time.perf_counter() - t0
                        timed_out = True
                        proc = None

                    if timed_out:
                        print(
                            f"[{n}] HANG  {label}  wall={wall_s:.1f}s "
                            f"(hang-timeout={args.hang_timeout:.0f}s)",
                            flush=True,
                        )
                        rows_out.append(
                            {
                                "cell": label,
                                "group": scen,
                                "score": "",
                                "steps": "",
                                "status": "HANG",
                                "wall_s": f"{wall_s:.3f}",
                                "verdict": "FAIL",
                            }
                        )
                        continue

                    assert proc is not None
                    if proc.returncode != 0:
                        print(
                            f"[{n}] FAIL exit={proc.returncode} "
                            f"wall={wall_s:.1f}s\n{(proc.stderr or '')[-2000:]}",
                            file=sys.stderr,
                            flush=True,
                        )
                        rows_out.append(
                            {
                                "cell": label,
                                "group": scen,
                                "score": "",
                                "steps": "",
                                "status": f"SIM_EXIT_{proc.returncode}",
                                "wall_s": f"{wall_s:.3f}",
                                "verdict": "FAIL",
                            }
                        )
                        continue

                    results = sorted(
                        mc_dir.glob("comparative_results_*"),
                        key=lambda p: p.stat().st_mtime,
                    )
                    yamls = list(results[-1].glob("*_simulation_output.yaml"))
                    parsed = parse_simulation_output(yamls[0], one, "honest")
                    r = parsed[0]
                    v = verdict(scen, wall_s, r.status, r.score)
                    print(
                        f"[{n}] {v:4} {label}  score={r.score:.4f} "
                        f"steps={r.steps} status={r.status} wall={wall_s:.1f}s",
                        flush=True,
                    )
                    rows_out.append(
                        {
                            "cell": label,
                            "group": scen,
                            "score": r.score,
                            "steps": r.steps,
                            "status": r.status,
                            "wall_s": f"{wall_s:.3f}",
                            "verdict": v,
                        }
                    )

    csv_path = args.out_dir / "per-cell-wall.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(
            f,
            fieldnames=[
                "cell",
                "group",
                "score",
                "steps",
                "status",
                "wall_s",
                "verdict",
            ],
        )
        w.writeheader()
        w.writerows(rows_out)

    print(
        "\n# Per-cell wall (Release, num_threads=1, one process per cell)",
        flush=True,
    )
    print(
        f"{'verdict':<7} {'group':<14} {'wall_s':>8} {'score':>8} "
        f"{'steps':>7} status  cell",
        flush=True,
    )
    fails = 0
    warns = 0
    for row in rows_out:
        if row["verdict"] == "FAIL":
            fails += 1
        elif row["verdict"] == "WARN":
            warns += 1
        print(
            f"{row['verdict']:<7} {row['group']:<14} "
            f"{float(row['wall_s']):8.1f} {str(row['score']):>8} "
            f"{str(row['steps']):>7} {row['status']:<10} {row['cell']}",
            flush=True,
        )
    walls = [float(r["wall_s"]) for r in rows_out] or [0.0]
    overall = "FAIL" if fails else "PASS"
    print(
        f"\nwrote {csv_path}\n"
        f"overall={overall}  cells={len(rows_out)}  FAIL={fails}  WARN={warns}  "
        f"wall_sum={sum(walls):.1f}s  wall_max={max(walls):.1f}s  "
        f"cells_ge_60s={sum(1 for x in walls if x >= 60.0)}",
        flush=True,
    )
    return 1 if fails else 0


if __name__ == "__main__":
    raise SystemExit(main())
