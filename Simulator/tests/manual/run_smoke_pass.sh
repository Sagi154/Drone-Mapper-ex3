#!/usr/bin/env bash
# Runs both CLI modes once against the real composition and checks the output shape.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
BUILD_DIR="${1:-${REPO_ROOT}/build/default}"
SIM="${BUILD_DIR}/Simulator/simulator_207190406_209543255"
SCRATCH="/tmp/ex3_verify"

# mission_control_folder / algorithms_folder must be writable — the simulator creates the
# results directory *inside* them, so copy the fixture folders into per-run scratch copies.
rm -rf "${SCRATCH}/mission_controls" "${SCRATCH}/algorithms"
cp -r "${SCRATCH}/plugins/mission_controls" "${SCRATCH}/mission_controls"
cp -r "${SCRATCH}/plugins/algorithms" "${SCRATCH}/algorithms"

echo "=== comparative mode ==="
set +e
"$SIM" -comparative \
    simulation="${REPO_ROOT}/inputs/sim_compose.yaml" \
    mission_control_folder="${SCRATCH}/mission_controls" \
    algorithm="${SCRATCH}/algorithms/Algorithm_207190406_209543255.so"
cmp_code=$?
set -e
echo "exit code: ${cmp_code}"

echo "=== competition mode ==="
set +e
"$SIM" -competition \
    simulation="${REPO_ROOT}/inputs/sim_compose.yaml" \
    mission_control="${SCRATCH}/mission_controls/MissionControl_207190406_209543255.so" \
    algorithms_folder="${SCRATCH}/algorithms"
comp_code=$?
set -e
echo "exit code: ${comp_code}"

echo "=== comparative results dir(s) ==="
find "${SCRATCH}/mission_controls" -maxdepth 1 -type d -name 'comparative_results_*'
echo "=== competition results dir(s) ==="
find "${SCRATCH}/algorithms" -maxdepth 1 -type d -name 'competition_*'

if [ "${cmp_code}" -ne 0 ] || [ "${comp_code}" -ne 0 ]; then
    echo "FAIL: simulator did not exit 0" >&2
    exit 1
fi
