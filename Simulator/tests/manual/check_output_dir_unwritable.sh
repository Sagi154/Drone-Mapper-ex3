#!/usr/bin/env bash
# CLI-08: results directory cannot be created → error to screen, no crash.
set -euo pipefail

if [ "$(id -u)" -eq 0 ] && command -v runuser >/dev/null 2>&1 && getent passwd vscode >/dev/null 2>&1; then
    exec runuser -u vscode -- bash "$0" "$@"
fi

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
BUILD_DIR="${1:-${REPO_ROOT}/build/default}"
SIM="${BUILD_DIR}/Simulator/simulator_207190406_209543255"
COMPOSE="${REPO_ROOT}/Simulator/tests/fixtures/tiny_compose.yaml"
SCRATCH="/tmp/ex3_verify/unwritable"
ALGO="${BUILD_DIR}/Algorithm/Algorithm_207190406_209543255.so"
MC="${BUILD_DIR}/MissionControl/MissionControl_207190406_209543255.so"

rm -rf "${SCRATCH}"
mkdir -p "${SCRATCH}/mission_controls" "${SCRATCH}/algorithms"
cp "$MC" "${SCRATCH}/mission_controls/"
cp "$ALGO" "${SCRATCH}/algorithms/"

# Drop write on the parent folders so create_directories(comparative_results_*) fails.
chmod a-w "${SCRATCH}/mission_controls" "${SCRATCH}/algorithms"

cleanup() {
    chmod u+w "${SCRATCH}/mission_controls" "${SCRATCH}/algorithms" 2>/dev/null || true
}
trap cleanup EXIT

if touch "${SCRATCH}/mission_controls/probe" 2>/dev/null; then
    rm -f "${SCRATCH}/mission_controls/probe"
    echo "FAIL: mission_controls still writable after chmod (run as non-root)" >&2
    exit 1
fi
if touch "${SCRATCH}/algorithms/probe" 2>/dev/null; then
    rm -f "${SCRATCH}/algorithms/probe"
    echo "FAIL: algorithms still writable after chmod (run as non-root)" >&2
    exit 1
fi

set +e
out=$("$SIM" -comparative \
    simulation="${COMPOSE}" \
    mission_control_folder="${SCRATCH}/mission_controls" \
    algorithm="${ALGO}" 2>&1)
code=$?
set -e
echo "$out"
if [ "$code" -ge 128 ]; then
    echo "FAIL: crashed creating comparative output dir" >&2
    exit 1
fi
echo "$out" | grep -Fq "could not create output directory" \
  || { echo "FAIL: missing create-dir error text" >&2; exit 1; }

set +e
out=$("$SIM" -competition \
    simulation="${COMPOSE}" \
    mission_control="${MC}" \
    algorithms_folder="${SCRATCH}/algorithms" 2>&1)
code=$?
set -e
echo "$out"
if [ "$code" -ge 128 ]; then
    echo "FAIL: crashed creating competition output dir" >&2
    exit 1
fi
echo "$out" | grep -Fq "could not create output directory" \
  || { echo "FAIL: missing create-dir error text (competition)" >&2; exit 1; }

echo "PASS: non-writable parent reported without crash"
