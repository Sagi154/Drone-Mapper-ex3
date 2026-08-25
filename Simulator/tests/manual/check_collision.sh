#!/usr/bin/env bash
# Confirms two runs launched immediately after each other never reuse an output directory.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
BUILD_DIR="${1:-${REPO_ROOT}/build/default}"
SIM="${BUILD_DIR}/Simulator/simulator_207190406_209543255"
SCRATCH="/tmp/ex3_verify/collision"

rm -rf "${SCRATCH}"
mkdir -p "${SCRATCH}/mission_controls"
cp "${BUILD_DIR}/MissionControl/MissionControl_207190406_209543255.so" "${SCRATCH}/mission_controls/"

"$SIM" -comparative simulation="${REPO_ROOT}/inputs/sim_compose.yaml" \
    mission_control_folder="${SCRATCH}/mission_controls" \
    algorithm="${BUILD_DIR}/Algorithm/Algorithm_207190406_209543255.so"
"$SIM" -comparative simulation="${REPO_ROOT}/inputs/sim_compose.yaml" \
    mission_control_folder="${SCRATCH}/mission_controls" \
    algorithm="${BUILD_DIR}/Algorithm/Algorithm_207190406_209543255.so"

after=$(find "${SCRATCH}/mission_controls" -maxdepth 1 -type d -name 'comparative_results_*' | sort)
count=$(echo "$after" | grep -c 'comparative_results_' || true)

echo "--- result directories ---"
echo "$after"
if [ "$count" -lt 2 ]; then
    echo "FAIL: expected 2 distinct comparative_results_* directories, found ${count}" >&2
    exit 1
fi
echo "PASS: ${count} distinct directories, no collision"
