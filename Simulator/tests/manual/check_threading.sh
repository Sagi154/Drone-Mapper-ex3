#!/usr/bin/env bash
# Confirms report contents are identical regardless of num_threads (absent/1/2/8),
# apart from the generated_at_utc timestamp.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
BUILD_DIR="${1:-${REPO_ROOT}/build/default}"
SIM="${BUILD_DIR}/Simulator/simulator_207190406_209543255"
SCRATCH="/tmp/ex3_verify/threading"

rm -rf "${SCRATCH}"

run_case() {
    local label="$1"; shift
    mkdir -p "${SCRATCH}/${label}/mc"
    cp "${BUILD_DIR}/MissionControl/MissionControl_207190406_209543255.so" "${SCRATCH}/${label}/mc/"
    "$SIM" -comparative simulation="${REPO_ROOT}/inputs/sim_compose.yaml" \
        mission_control_folder="${SCRATCH}/${label}/mc" \
        algorithm="${BUILD_DIR}/Algorithm/Algorithm_207190406_209543255.so" "$@"
    report=$(find "${SCRATCH}/${label}/mc"/comparative_results_* -maxdepth 1 -name 'comparative_report.yaml')
    grep -vE 'generated_at_utc|mission_control_folder' "$report" > "${SCRATCH}/${label}.stripped.yaml"
}

run_case absent
run_case t1 num_threads=1
run_case t2 num_threads=2
run_case t8 num_threads=8

for pair in t1 t2 t8; do
    echo "--- diff absent vs ${pair} ---"
    if diff -u "${SCRATCH}/absent.stripped.yaml" "${SCRATCH}/${pair}.stripped.yaml"; then
        echo "PASS: absent == ${pair}"
    else
        echo "FAIL: absent != ${pair}" >&2
        exit 1
    fi
done
