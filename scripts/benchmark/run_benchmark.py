#!/usr/bin/env python3
"""Run comparative/competition sweeps and emit labelled benchmark CSVs."""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

import yaml

from cell_labels import expected_cell_count
from report_parser import Row, parse_simulation_output
from summarize import read_csv, render_markdown_summary, write_csv

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_COMPOSE = REPO_ROOT / "inputs" / "sim_compose.yaml"
TINY_COMPOSE = (
    REPO_ROOT / "Simulator" / "tests" / "fixtures" / "tiny_compose.yaml"
)
EX2_REF = REPO_ROOT / "docs" / "benchmarks" / "ex2-reference.csv"

OUR_ALGO = "Algorithm_207190406_209543255.so"
OUR_MC = "MissionControl_207190406_209543255.so"
FOREIGN_MC = "foreign_hits_only_mission_control_plugin.so"


def _find_simulator(build_dir: Path) -> Path:
    candidates = [
        build_dir / "Simulator" / "simulator_207190406_209543255",
        build_dir / "Simulator" / "simulator_207190406_209543255.exe",
    ]
    for path in candidates:
        if path.is_file():
            return path
    raise FileNotFoundError(f"simulator binary not found under {build_dir}")


def _latest_results_dir(parent: Path, prefix: str) -> Path:
    matches = sorted(
        [p for p in parent.glob(f"{prefix}_*") if p.is_dir()],
        key=lambda p: p.stat().st_mtime,
        reverse=True,
    )
    if not matches:
        raise FileNotFoundError(f"no {prefix}_* directory under {parent}")
    return matches[0]


def _stage_mc(src: Path, scratch: Path, column: str) -> Path:
    dest_dir = scratch / f"mc_{column}"
    dest_dir.mkdir(parents=True, exist_ok=True)
    dest = dest_dir / src.name
    shutil.copy2(src, dest)
    return dest_dir


def _parse_plugin_yaml(
    results_dir: Path, composition: dict, column: str
) -> list[Row]:
    yamls = list(results_dir.glob("*_simulation_output.yaml"))
    if not yamls:
        raise FileNotFoundError(
            f"no *_simulation_output.yaml in {results_dir}"
        )
    if len(yamls) != 1:
        raise ValueError(
            f"expected exactly one simulation_output.yaml in {results_dir}, "
            f"got {[p.name for p in yamls]}"
        )
    return parse_simulation_output(yamls[0], composition, column)


def run_hosts_column(
    *,
    simulator: Path,
    composition_path: Path,
    composition: dict,
    algorithm: Path,
    mc_plugin: Path,
    column: str,
    scratch: Path,
    num_threads: int,
) -> list[Row]:
    staged = _stage_mc(mc_plugin, scratch, column)
    cmd = [
        str(simulator),
        "-comparative",
        f"simulation={composition_path}",
        f"mission_control_folder={staged}",
        f"algorithm={algorithm}",
        f"num_threads={num_threads}",
    ]
    print(f"[benchmark] running column={column}: {' '.join(cmd)}", flush=True)
    proc = subprocess.run(cmd, cwd=str(REPO_ROOT), capture_output=True, text=True)
    if proc.returncode != 0:
        raise RuntimeError(
            f"simulator failed for column={column} (exit {proc.returncode})\n"
            f"stdout:\n{proc.stdout}\nstderr:\n{proc.stderr}"
        )
    results_dir = _latest_results_dir(staged, "comparative_results")
    rows = _parse_plugin_yaml(results_dir, composition, column)
    expected = expected_cell_count(composition)
    if len(rows) != expected:
        raise ValueError(
            f"column={column}: expected {expected} rows, got {len(rows)}"
        )
    return rows


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=["hosts", "algorithms"], default="hosts")
    parser.add_argument("--build-dir", type=Path, default=REPO_ROOT / "build" / "default")
    parser.add_argument("--composition", type=Path, default=DEFAULT_COMPOSE)
    parser.add_argument("--quick", action="store_true")
    parser.add_argument(
        "--columns",
        default="ex2_comparable,adversarial",
        help="comma-separated: ex2_comparable,adversarial,honest",
    )
    parser.add_argument("--algorithm", type=Path, default=None)
    parser.add_argument("--our-mc", type=Path, default=None)
    parser.add_argument("--foreign-mc", type=Path, default=None)
    parser.add_argument("--num-threads", type=int, default=8)
    parser.add_argument("--scratch", type=Path, default=None)
    parser.add_argument("--label", default="run")
    parser.add_argument("--baseline", type=Path, default=None)
    parser.add_argument(
        "--out-dir", type=Path, default=REPO_ROOT / "docs" / "benchmarks"
    )
    parser.add_argument(
        "--dry-parse-file",
        type=Path,
        default=None,
        help="Skip simulator; parse this simulation_output.yaml",
    )
    parser.add_argument(
        "--dry-parse-column",
        default="ex2_comparable",
        help="Column name when using --dry-parse-file",
    )
    args = parser.parse_args(argv)

    composition_path = TINY_COMPOSE if args.quick else args.composition
    composition_path = composition_path.resolve()
    composition = yaml.safe_load(composition_path.read_text(encoding="utf-8"))

    if args.dry_parse_file is not None:
        rows = parse_simulation_output(
            args.dry_parse_file, composition, args.dry_parse_column
        )
        print(render_markdown_summary({args.dry_parse_column: rows}, ex2_reference=EX2_REF))
        return 0

    if args.mode != "hosts":
        print(
            "algorithms mode is supported by the design but not needed for "
            "project A baselines; implement when needed.",
            file=sys.stderr,
        )
        return 2

    build_dir = args.build_dir.resolve()
    algorithm = args.algorithm or (
        build_dir / "Algorithm" / OUR_ALGO
    )
    our_mc = args.our_mc or (build_dir / "MissionControl" / OUR_MC)
    foreign_mc = args.foreign_mc or (
        build_dir / "Simulator" / "tests" / "fixtures" / FOREIGN_MC
    )

    for required in (algorithm, our_mc):
        if not required.is_file():
            print(f"missing plugin: {required}", file=sys.stderr)
            return 2

    columns = [c.strip() for c in args.columns.split(",") if c.strip()]
    scratch = args.scratch or (
        REPO_ROOT / "tmp" / f"benchmark_{datetime.now(timezone.utc):%Y%m%dT%H%M%SZ}"
    )
    scratch.mkdir(parents=True, exist_ok=True)

    simulator = _find_simulator(build_dir)
    all_rows: list[Row] = []
    rows_by_column: dict[str, list[Row]] = {}
    failures: list[str] = []

    for column in columns:
        if column in ("ex2_comparable", "honest"):
            mc = our_mc
            if column == "honest":
                print(
                    "[benchmark] warning: honest column uses current MC binary; "
                    "meaningful only after project B de-batches scans.",
                    flush=True,
                )
        elif column == "adversarial":
            if not foreign_mc.is_file():
                failures.append(f"missing foreign MC: {foreign_mc}")
                continue
            mc = foreign_mc
        else:
            failures.append(f"unknown column: {column}")
            continue

        try:
            rows = run_hosts_column(
                simulator=simulator,
                composition_path=composition_path,
                composition=composition,
                algorithm=algorithm,
                mc_plugin=mc,
                column=column,
                scratch=scratch,
                num_threads=args.num_threads,
            )
            rows_by_column[column] = rows
            all_rows.extend(rows)
        except Exception as exc:  # noqa: BLE001 — record and continue columns
            failures.append(f"{column}: {exc}")
            print(f"[benchmark] FAILED {column}: {exc}", file=sys.stderr, flush=True)

    if not all_rows:
        print("[benchmark] no rows produced", file=sys.stderr)
        return 1

    baseline_rows = read_csv(args.baseline) if args.baseline and args.baseline.is_file() else None
    if args.baseline and baseline_rows is None:
        print(f"[benchmark] warning: baseline missing, skipping diff: {args.baseline}")

    summary = render_markdown_summary(
        rows_by_column,
        ex2_reference=EX2_REF if EX2_REF.is_file() else None,
        baseline=baseline_rows,
    )

    if args.quick:
        print(summary)
        print("[benchmark] --quick: not writing docs/benchmarks outputs")
    else:
        stamp = datetime.now(timezone.utc).strftime("%Y-%m-%d")
        out_dir = args.out_dir.resolve()
        out_dir.mkdir(parents=True, exist_ok=True)
        csv_path = out_dir / f"{stamp}-{args.label}.csv"
        md_path = out_dir / f"{stamp}-{args.label}.md"
        write_csv(all_rows, csv_path)
        md_path.write_text(summary + "\n", encoding="utf-8")
        print(f"[benchmark] wrote {csv_path}")
        print(f"[benchmark] wrote {md_path}")
        print(summary)

    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
