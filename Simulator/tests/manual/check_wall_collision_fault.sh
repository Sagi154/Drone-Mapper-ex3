#!/usr/bin/env bash
# FAULT-02: faulty algorithm commands a wall hit; MockMovement throws; process must not crash.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
BUILD_DIR="${1:-${REPO_ROOT}/build/default}"
SIM="${BUILD_DIR}/Simulator/simulator_207190406_209543255"
FAULT_SO="${BUILD_DIR}/Simulator/tests/fixtures/faulty_wall_algorithm_plugin.so"
MC="${BUILD_DIR}/MissionControl/MissionControl_207190406_209543255.so"
COMPOSE="${REPO_ROOT}/Simulator/tests/fixtures/tiny_compose.yaml"
SCRATCH="/tmp/ex3_verify/wall_fault"

[ -f "$FAULT_SO" ] || { echo "FAIL: missing $FAULT_SO" >&2; exit 1; }

rm -rf "${SCRATCH}"
mkdir -p "${SCRATCH}/algorithms"
cp "$FAULT_SO" "${SCRATCH}/algorithms/"

set +e
"$SIM" -competition \
    simulation="${COMPOSE}" \
    mission_control="${MC}" \
    algorithms_folder="${SCRATCH}/algorithms"
code=$?
set -e

if [ "$code" -ge 128 ]; then
    echo "FAIL: simulator crashed on wall-collision fault (signal $((code - 128)))" >&2
    exit 1
fi
echo "PASS: wall-collision fault did not crash simulator (exit ${code})"
