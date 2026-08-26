#!/usr/bin/env bash
# Folder full of .so files that do not register: still write aggregate errors: [...]
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
BUILD_DIR="${1:-${REPO_ROOT}/build/default}"
SIM="${BUILD_DIR}/Simulator/simulator_207190406_209543255"
UNREG="${BUILD_DIR}/Simulator/tests/fixtures/unregistered_plugin.so"
ALGO="${BUILD_DIR}/Algorithm/Algorithm_207190406_209543255.so"
MC="${BUILD_DIR}/MissionControl/MissionControl_207190406_209543255.so"
SCRATCH="/tmp/ex3_verify/all_folder_fail"
COMPOSE="${REPO_ROOT}/inputs/sim_compose.yaml"

rm -rf "${SCRATCH}"
mkdir -p "${SCRATCH}/mc_bad" "${SCRATCH}/algo_bad"
cp "${UNREG}" "${SCRATCH}/mc_bad/bad_mc.so"
cp "${UNREG}" "${SCRATCH}/algo_bad/bad_algo.so"

"$SIM" -comparative simulation="${COMPOSE}" \
    mission_control_folder="${SCRATCH}/mc_bad" \
    algorithm="${ALGO}"

REPORT=$(find "${SCRATCH}/mc_bad"/comparative_results_* -name comparative_report.yaml | head -n 1)
test -n "${REPORT}" || { echo "FAIL: no comparative_report.yaml"; exit 1; }
grep -q "bad_mc.so" "${REPORT}" || { echo "FAIL: errors missing bad_mc.so"; exit 1; }

"$SIM" -competition simulation="${COMPOSE}" \
    mission_control="${MC}" \
    algorithms_folder="${SCRATCH}/algo_bad"

REPORT=$(find "${SCRATCH}/algo_bad"/competition_* -name competitive_report.yaml | head -n 1)
test -n "${REPORT}" || { echo "FAIL: no competitive_report.yaml"; exit 1; }
grep -q "bad_algo.so" "${REPORT}" || { echo "FAIL: errors missing bad_algo.so"; exit 1; }

echo "PASS: all-folder plugin load failure wrote errors in both reports"
