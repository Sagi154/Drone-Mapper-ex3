#!/usr/bin/env bash
# CLI-03: all arguments can appear in any order (black-box).
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
BUILD_DIR="${1:-${REPO_ROOT}/build/default}"
SIM="${BUILD_DIR}/Simulator/simulator_207190406_209543255"
COMPOSE="${REPO_ROOT}/Simulator/tests/fixtures/tiny_compose.yaml"
SCRATCH="/tmp/ex3_verify/arg_order"
ALGO="${BUILD_DIR}/Algorithm/Algorithm_207190406_209543255.so"
MC="${BUILD_DIR}/MissionControl/MissionControl_207190406_209543255.so"

rm -rf "${SCRATCH}"
mkdir -p "${SCRATCH}/mission_controls"
cp "$MC" "${SCRATCH}/mission_controls/"

"$SIM" \
    algorithm="${ALGO}" \
    -verbose \
    mission_control_folder="${SCRATCH}/mission_controls" \
    -comparative \
    num_threads=2 \
    simulation="${COMPOSE}"

dir=$(find "${SCRATCH}/mission_controls" -maxdepth 1 -type d -name 'comparative_results_*' | sort | tail -n1)
test -n "$dir" || { echo "FAIL: no comparative_results_* after scrambled argv" >&2; exit 1; }
test -f "${dir}/comparative_report.yaml" \
  || { echo "FAIL: missing comparative_report.yaml" >&2; exit 1; }
echo "PASS: scrambled argument order accepted"
