# Algorithm benchmark harness

Developer tool for project A (`docs/superpowers/specs/2026-08-29-algorithm-benchmark-harness-design.md`).
Runs the existing `simulator_207190406_209543255` CLI, labels composition cells, and writes
score/steps/status CSVs under `docs/benchmarks/`.

## Setup

```bash
python3 -m venv scripts/benchmark/.venv
# Windows:
scripts/benchmark/.venv/Scripts/pip install -r scripts/benchmark/requirements.txt
# Linux / Docker:
scripts/benchmark/.venv/bin/pip install -r scripts/benchmark/requirements.txt
```

## Unit tests (no simulator)

```bash
cd scripts/benchmark
# Windows
.venv/Scripts/python -m pytest tests/ -v
# Linux
.venv/bin/python -m pytest tests/ -v
```

## Quick smoke (1 cell)

Requires a built tree under `build/default` (use Docker on Windows — `.so` targets are Linux).

```bash
python scripts/benchmark/run_benchmark.py --quick --columns ex2_comparable --label smoke
```

## Full baseline (project A)

On the host (venv already created):

```bash
python scripts/benchmark/run_benchmark.py \
  --mode hosts \
  --columns ex2_comparable,adversarial \
  --label pre_b_baseline \
  --num-threads 8
```

Inside `drone-mapper-ex3-dev` the image may lack `python3-venv`; install once per container:

```bash
apt-get update -qq && apt-get install -y -qq python3-venv python3-pip
python3 -m venv scripts/benchmark/.venv
scripts/benchmark/.venv/bin/pip install -r scripts/benchmark/requirements.txt
```

Do not commit `--quick` outputs. Full-matrix CSVs under `docs/benchmarks/` are the deliverable.
