#!/usr/bin/env bash
# Confirms MissionControl verbose output appears iff -verbose is passed.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
BUILD_DIR="${1:-${REPO_ROOT}/build/default}"
SIM="${BUILD_DIR}/Simulator/simulator_207190406_209543255"
SCRATCH="/tmp/ex3_verify/verbose"

rm -rf "${SCRATCH}"
mkdir -p "${SCRATCH}/off/mc" "${SCRATCH}/on/mc"
cp "${BUILD_DIR}/MissionControl/MissionControl_207190406_209543255.so" "${SCRATCH}/off/mc/"
cp "${BUILD_DIR}/MissionControl/MissionControl_207190406_209543255.so" "${SCRATCH}/on/mc/"

"$SIM" -comparative simulation="${REPO_ROOT}/inputs/sim_compose.yaml" \
    mission_control_folder="${SCRATCH}/off/mc" \
    algorithm="${BUILD_DIR}/Algorithm/Algorithm_207190406_209543255.so"

"$SIM" -comparative simulation="${REPO_ROOT}/inputs/sim_compose.yaml" \
    mission_control_folder="${SCRATCH}/on/mc" \
    algorithm="${BUILD_DIR}/Algorithm/Algorithm_207190406_209543255.so" \
    -verbose

echo "--- files without -verbose ---"
find "${SCRATCH}/off/mc"/comparative_results_* -type f | sed "s#${SCRATCH}/off/mc/##" | sort
echo "--- files with -verbose ---"
find "${SCRATCH}/on/mc"/comparative_results_* -type f | sed "s#${SCRATCH}/on/mc/##" | sort
