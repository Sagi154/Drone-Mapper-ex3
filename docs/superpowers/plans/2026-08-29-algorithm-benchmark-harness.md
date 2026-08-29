# Algorithm Benchmark Harness Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Ship a Python sweep tool under `scripts/benchmark/` that runs the existing simulator CLI, labels the 24 composition cells, and commits score/steps/status CSVs so later algorithm work can beat ex2's recorded bands.

**Architecture:** Pure-Python harness that shells out to `simulator_207190406_209543255` (no production C++ changes). Modules: `cell_labels` (index → name), `report_parser` (YAML → rows), `summarize` (totals / ex2 bands / baseline diff), `run_benchmark` (CLI + staging + invoke). Unit-test parsing/labelling only; full matrix sweeps are manual/Docker.

**Tech Stack:** Python 3.11+, venv + pip, PyYAML, pytest; Docker image `drone-mapper-ex3-dev` for builds and full sweeps (Linux `.so` artifacts).

## Global Constraints

- **No production C++ changes** — do not modify `Algorithm/`, `MissionControl/`, or `Simulator/` sources (project A scope).
- **Never `git commit` without explicit human approval** in this chat; stage and propose the message, then wait.
- **Python deps:** project-local venv only (`scripts/benchmark/.venv`); never global `pip install`. Follow `managing-python-dependencies` (venv + requirements.txt).
- **Index semantics (pinned to C++):** `config_indices.simulation` = group index into `simulation_compositions.simulations`; `config_indices.mission` = index within that entry's `mission_configs`; `drone` / `lidar` index the top-level lists. Source: `RunMatrixOrchestrator::expand` (`Simulator/src/RunMatrixOrchestrator.cpp:38-60`) and `main.cpp` mapping `group_index` → `simulation_index`.
- **Status strings** in YAML are `COMPLETED`, `MAX_STEPS`, `ERROR` (uppercase; from `missionRunStatusLabel`).
- **CSV columns:** `column,cell,score,steps,status` exactly as in the design spec.
- **Cell label format:** `{sim_stem}+{mission_stem}|{drone_stem}|{lidar_stem}` where stem is the YAML filename without path/extension (e.g. `house_simulation+house_mission_full|drone_large|lidar_short`).
- **Full 24-row invariant** only when composition is `inputs/sim_compose.yaml`; otherwise assert `expected_cell_count(composition)`.
- **Quick sweeps** (`--quick` / non-default composition) are for iteration; do **not** commit them under `docs/benchmarks/`.
- **Project A populates only** `ex2_comparable` and `adversarial` columns; `honest` is supported by the CLI but first measured after project B.
- Branch: continue on `algorithm-benchmark-harness` (already created from `main`).

---

### Task 1: Scaffold, venv ignore, ex2 reference CSV

**Files:**
- Create: `scripts/benchmark/requirements.txt`
- Create: `scripts/benchmark/README.md`
- Create: `docs/benchmarks/ex2-reference.csv`
- Create: `docs/benchmarks/.gitkeep` (optional if CSV alone is enough — skip `.gitkeep` if CSV exists)
- Modify: `.gitignore` (add `scripts/benchmark/.venv/` and `.venv/`)

**Interfaces:**
- Consumes: none
- Produces: pinned deps list; ex2 band table schema `group,low,high,citation`

- [ ] **Step 1: Add venv paths to `.gitignore`**

Append:

```
# Python local envs
.venv/
scripts/benchmark/.venv/
```

- [ ] **Step 2: Write `scripts/benchmark/requirements.txt`**

```
PyYAML==6.0.2
pytest==8.3.5
```

- [ ] **Step 3: Write `docs/benchmarks/ex2-reference.csv`**

```csv
group,low,high,citation
house_lower,100,100,"Drone-Mapper-ex2/docs/HLD.md:387-402; Jul 5 2026"
house_full,56,62,"Drone-Mapper-ex2/docs/HLD.md:387-402; Jul 5 2026"
large_out,80,88,"Drone-Mapper-ex2/docs/HLD.md:387-402; Jul 5 2026"
large_room,92,96,"Drone-Mapper-ex2/docs/HLD.md:387-402; Jul 5 2026"
small_out,75,89,"Drone-Mapper-ex2/docs/HLD.md:387-402; Jul 5 2026"
small_room,87,90,"Drone-Mapper-ex2/docs/HLD.md:387-402; Jul 5 2026"
```

- [ ] **Step 4: Write `scripts/benchmark/README.md`**

Document: create venv, install requirements, run pytest, run `--quick`, run full hosts sweep for `ex2_comparable` / `adversarial` via Docker. Point at the design spec.

- [ ] **Step 5: Propose commit (do not commit until human says yes)**

```
docs: scaffold benchmark harness deps and ex2 reference bands
```

Files: `.gitignore`, `scripts/benchmark/requirements.txt`, `scripts/benchmark/README.md`, `docs/benchmarks/ex2-reference.csv`

---

### Task 2: `cell_labels` + expected cell count (TDD)

**Files:**
- Create: `scripts/benchmark/cell_labels.py`
- Create: `scripts/benchmark/tests/test_cell_labels.py`
- Create: `scripts/benchmark/tests/__init__.py` (empty)

**Interfaces:**
- Consumes: composition YAML path / loaded dict
- Produces:
  - `stem(path: str) -> str`
  - `expected_cell_count(composition: dict) -> int`
  - `label_for_indices(composition: dict, simulation: int, mission: int, drone: int, lidar: int) -> str`
  - `scenario_group_for_label(cell: str) -> str` — maps label to one of `house_lower|house_full|large_out|large_room|small_out|small_room` by inspecting mission/sim stems

- [ ] **Step 1: Create venv and install deps**

From repo root (PowerShell):

```powershell
python -m venv scripts/benchmark/.venv
scripts/benchmark/.venv/Scripts/python -m pip install -r scripts/benchmark/requirements.txt
```

On Linux/Docker: `scripts/benchmark/.venv/bin/pip install -r scripts/benchmark/requirements.txt`

- [ ] **Step 2: Write failing tests `scripts/benchmark/tests/test_cell_labels.py`**

```python
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
    # simulations[0] = house; mission_configs[1] = house_mission_full
    # drones[1] = drone_large; lidars[1] = lidar_short
    label = label_for_indices(_load(), 0, 1, 1, 1)
    assert label == "house_simulation+house_mission_full|drone_large|lidar_short"


def test_scenario_group_mapping():
    assert scenario_group_for_label(
        "house_simulation+house_mission_lower|drone_small|lidar_long"
    ) == "house_lower"
    assert scenario_group_for_label(
        "large_simulation_room+large_mission_room|drone_small|lidar_short"
    ) == "large_room"
```

- [ ] **Step 3: Run tests — expect FAIL (import error)**

```powershell
cd scripts/benchmark
.venv/Scripts/python -m pytest tests/test_cell_labels.py -v
```

Expected: FAIL — `ModuleNotFoundError: cell_labels`

- [ ] **Step 4: Implement `scripts/benchmark/cell_labels.py`**

```python
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
        n += len(group["mission_configs"]) * len(root["drone_configs"]) * len(
            root["lidar_configs"]
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
    left = cell.split("|", 1)[0]  # sim+mission
    sim, mission = left.split("+", 1)
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
```

- [ ] **Step 5: Run tests — expect PASS**

Same pytest command. Expected: 4 passed.

- [ ] **Step 6: Propose commit**

```
feat: add composition cell labelling for benchmark harness
```

---

### Task 3: `report_parser` + sample fixture (TDD)

**Files:**
- Create: `scripts/benchmark/report_parser.py`
- Create: `scripts/benchmark/tests/fixtures/sample_simulation_output.yaml`
- Create: `scripts/benchmark/tests/test_report_parser.py`

**Interfaces:**
- Consumes: `cell_labels.label_for_indices`, `expected_cell_count`
- Produces:
  - `@dataclass Row: column: str, cell: str, score: float, steps: int, status: str, errors: list`
  - `parse_simulation_output(path, composition, column: str) -> list[Row]`
  - Raises `ValueError` if row count ≠ expected or duplicate index tuples

- [ ] **Step 1: Write fixture `scripts/benchmark/tests/fixtures/sample_simulation_output.yaml`**

Three runs (COMPLETED / MAX_STEPS / ERROR) with `drone` indices 0,1,2 so expected cell count is 3 under a 1×1×3×1 composition:

```yaml
score_report:
  composition_file: "tiny_for_parser.yaml"
  metric: maps_comparison_score_0_100
  score_range: [0.0, 100.0]
  error_score: -1
  runs:
    - run_id: 0
      mission_score: 88.75
      config_indices: {simulation: 0, mission: 0, drone: 0, lidar: 0}
      mission_results: [{status: COMPLETED, steps: 42, errors: []}]
    - run_id: 1
      mission_score: 40.0
      config_indices: {simulation: 0, mission: 0, drone: 1, lidar: 0}
      mission_results: [{status: MAX_STEPS, steps: 500, errors: []}]
    - run_id: 2
      mission_score: -1.0
      config_indices: {simulation: 0, mission: 0, drone: 2, lidar: 0}
      mission_results:
        - status: ERROR
          steps: 3
          errors: [{code: DRONE_STEP_FAILED, message: boom}]
```

- [ ] **Step 2: Write failing tests `scripts/benchmark/tests/test_report_parser.py`**

```python
from pathlib import Path

import pytest
import yaml

from report_parser import parse_simulation_output

FIX = Path(__file__).parent / "fixtures" / "sample_simulation_output.yaml"

COMPOSE_3 = {
    "simulation_compositions": {
        "simulations": [{
            "simulation_config": "simulation/small_simulation_room.yaml",
            "mission_configs": ["mission/small_mission_room.yaml"],
        }],
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
```

- [ ] **Step 3: Run tests — expect FAIL**

- [ ] **Step 4: Implement `scripts/benchmark/report_parser.py`**

```python
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
        raise ValueError(
            f"{path}: expected {expected} runs, got {len(runs)}"
        )

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
```

Ensure `tests/` can import sibling modules: run pytest with `PYTHONPATH=scripts/benchmark` or add empty `scripts/benchmark/conftest.py` that inserts the parent on `sys.path`:

```python
# scripts/benchmark/conftest.py
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
```

- [ ] **Step 5: Run tests — expect PASS**

```powershell
cd scripts/benchmark
.venv/Scripts/python -m pytest tests/ -v
```

- [ ] **Step 6: Propose commit**

```
feat: parse simulation_output.yaml into labelled benchmark rows
```

---

### Task 4: `summarize` (totals, ex2 bands, baseline diff)

**Files:**
- Create: `scripts/benchmark/summarize.py`
- Create: `scripts/benchmark/tests/test_summarize.py`

**Interfaces:**
- Consumes: `Row`, `scenario_group_for_label`, `docs/benchmarks/ex2-reference.csv`
- Produces:
  - `write_csv(rows, path)`
  - `read_csv(path) -> list[Row]`
  - `column_totals(rows) -> dict`
  - `ex2_band_report(rows, reference_csv) -> list[dict]` with keys `group, mean_score, band_low, band_high, verdict` where verdict ∈ `below|inside|above`
  - `baseline_diff(current, baseline) -> (deltas, regressions)`
  - `render_markdown_summary(...) -> str`

- [ ] **Step 1: Write failing tests for band verdict and regression list**

```python
from summarize import baseline_diff, ex2_band_report, write_csv, read_csv
from report_parser import Row
from pathlib import Path

def test_ex2_band_inside(tmp_path):
    ref = Path(__file__).resolve().parents[3] / "docs" / "benchmarks" / "ex2-reference.csv"
    rows = [
        Row("ex2_comparable", "small_simulation_room+small_mission_room|drone_small|lidar_short", 88.0, 10, "COMPLETED"),
        Row("ex2_comparable", "small_simulation_room+small_mission_room|drone_small|lidar_long", 89.0, 11, "COMPLETED"),
        Row("ex2_comparable", "small_simulation_room+small_mission_room|drone_large|lidar_short", 87.5, 12, "COMPLETED"),
        Row("ex2_comparable", "small_simulation_room+small_mission_room|drone_large|lidar_long", 90.0, 13, "COMPLETED"),
    ]
    report = ex2_band_report(rows, ref)
    small = next(r for r in report if r["group"] == "small_room")
    assert small["verdict"] == "inside"


def test_baseline_diff_flags_score_drop():
    base = [Row("c", "cell_a", 90.0, 10, "COMPLETED")]
    cur = [Row("c", "cell_a", 80.0, 12, "COMPLETED")]
    deltas, regressions = baseline_diff(cur, base)
    assert any("cell_a" in r for r in regressions)
```

- [ ] **Step 2: Implement `summarize.py`** with CSV via `csv` stdlib; mean score per group over rows whose `scenario_group_for_label(cell)` matches; verdict: `below` if mean < low, `above` if mean > high, else `inside`. Skip `-1` scores from mean (count them as errored separately in totals).

- [ ] **Step 3: pytest PASS**

- [ ] **Step 4: Propose commit**

```
feat: summarize benchmark columns against ex2 bands and baselines
```

---

### Task 5: `run_benchmark` CLI orchestrator

**Files:**
- Create: `scripts/benchmark/run_benchmark.py`

**Interfaces:**
- CLI:
  - `--mode hosts|algorithms` (default `hosts`)
  - `--build-dir` (default `build/default`)
  - `--composition` (default `inputs/sim_compose.yaml`)
  - `--quick` → composition `Simulator/tests/fixtures/tiny_compose.yaml`
  - `--columns` comma list subset of `ex2_comparable,adversarial,honest` (default `ex2_comparable,adversarial` for project A)
  - `--algorithm` path to Algorithm `.so`
  - `--our-mc` path to our MissionControl `.so`
  - `--foreign-mc` path to `foreign_hits_only_mission_control_plugin.so`
  - `--num-threads` (default `8`)
  - `--scratch` temp dir
  - `--label` suffix for output filenames
  - `--baseline` optional CSV
  - `--out-dir` default `docs/benchmarks`
  - `--dry-parse DIR` — skip simulator; parse existing `*_simulation_output.yaml` from DIR (useful for tests)

**Column → plugin staging (`hosts` mode):**
- `ex2_comparable` / `honest`: stage a copy of `--our-mc` into scratch subdir `mc_ex2_comparable/` or `mc_honest/` (same binary pre-B; column name still distinguishes intent). For A, only run `ex2_comparable` for our MC (do not duplicate the same binary under `honest` until B lands — if user passes `honest` pre-B, warn and still label column `honest`).
- `adversarial`: stage `--foreign-mc` into `mc_adversarial/`

**Invoke (hosts):** one comparative run **per column** (separate `mission_control_folder` each), so plugin filename → column mapping stays unambiguous:

```text
simulator -comparative simulation=<compose> mission_control_folder=<staged> algorithm=<algo> num_threads=<N>
```

Find newest `comparative_results_*` under the staged folder after the run. Map `<plugin>_simulation_output.yaml` → rows with that column name.

**On simulator non-zero exit:** record column failed, continue other columns, exit 1 at end.

**Outputs:** write `{date}-{label}.csv` and `{date}-{label}.md` under `--out-dir` unless `--quick`.

- [ ] **Step 1: Implement `run_benchmark.py`** with `argparse` + `subprocess.run` + `shutil.copy2` for staging.

- [ ] **Step 2: Smoke with `--dry-parse`** on the sample fixture directory (copy fixture as `MissionControl_dummy_simulation_output.yaml` or add `--dry-parse-file`).

Add `--dry-parse-file path --column name` for unit smoke without full compose.

- [ ] **Step 3: Propose commit**

```
feat: add run_benchmark CLI for comparative host sweeps
```

---

### Task 6: Capture `ex2_comparable` + `adversarial` baselines

**Files:**
- Create: `docs/benchmarks/2026-08-29-ex2_comparable-adversarial.csv` (date may differ)
- Create: `docs/benchmarks/2026-08-29-ex2_comparable-adversarial.md`
- Modify: `docs/superpowers/specs/2026-08-29-mapping-algorithm-roadmap.md` — fill A's "when A lands" numbers

**Prereq:** Docker image `drone-mapper-ex3-dev` and `cmake --preset default && cmake --build --preset default` producing:
- `build/default/Simulator/simulator_207190406_209543255`
- `build/default/Algorithm/Algorithm_207190406_209543255.so`
- `build/default/MissionControl/MissionControl_207190406_209543255.so`
- `build/default/Simulator/tests/fixtures/foreign_hits_only_mission_control_plugin.so`

- [ ] **Step 1: Build in Docker** (from repo root; adjust volume mount for Windows):

```bash
docker run --rm -e VCPKG_ROOT=/usr/local/vcpkg -v "${PWD}:/work" -w /work drone-mapper-ex3-dev \
  bash -lc 'cmake --preset default && cmake --build --preset default -j$(nproc)'
```

- [ ] **Step 2: Install harness deps in container and run full sweep**

```bash
docker run --rm -e VCPKG_ROOT=/usr/local/vcpkg -v "${PWD}:/work" -w /work drone-mapper-ex3-dev \
  bash -lc '
    python3 -m venv scripts/benchmark/.venv
    scripts/benchmark/.venv/bin/pip install -r scripts/benchmark/requirements.txt
    scripts/benchmark/.venv/bin/python scripts/benchmark/run_benchmark.py \
      --mode hosts \
      --build-dir build/default \
      --columns ex2_comparable,adversarial \
      --label pre_b_baseline \
      --num-threads 8
  '
```

Expect ~2×276s wall clock. Do not commit quick runs.

- [ ] **Step 3: Sanity-check CSV** — 24 rows per column; statuses mostly COMPLETED for `ex2_comparable`; adversarial may differ. Summary must include ex2 band verdicts.

- [ ] **Step 4: Update roadmap Project A status** with measured group means / below|inside|above.

- [ ] **Step 5: Propose commit**

```
docs: record pre-B algorithm score baseline vs ex2 bands
```

- [ ] **Step 6: Confirm `scripts/` does not break zip layout** — skim `.cursor/skills/pre-submission-review/SKILL.md` ZIP checks; `scripts/` is outside the five submission folders and should be fine. Note confirmation in the PR/commit body.

---

## Spec coverage checklist

| Spec requirement | Task |
|------------------|------|
| `scripts/benchmark/` layout | 1–5 |
| `docs/benchmarks/ex2-reference.csv` | 1 |
| CSV + markdown outputs | 4–5 |
| Index labelling + 24-row assert | 2–3 |
| hosts / algorithms modes | 5 (algorithms mode: implement CLI path; A uses hosts) |
| `--quick` / `--composition` | 5 |
| `--baseline` diff | 4–5 |
| Error handling (continue columns, -1 preserved) | 3, 5 |
| Capture ex2_comparable + adversarial | 6 |
| pytest for parser/labels | 2–4 |
| No production C++ changes | Global Constraints |
| `honest` column deferred | 5 defaults |

## Plan self-review

- No TBD placeholders in task steps; fixture revision is explicit in Task 3.
- Types consistent: `Row` defined in Task 3, used in 4–5.
- Commit steps always "propose / wait for human" per Global Constraints.

---

**Execution:** User requested Subagent-Driven Development. After this plan is committed, execute Tasks 1→6 with a fresh implementer per task, task review after each, **stopping for human approval before every `git commit`**.
