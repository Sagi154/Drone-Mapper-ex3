#!/usr/bin/env bash
# OUT-02: competition results live under algorithms_folder as competition_<time>.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
BUILD_DIR="${1:-${REPO_ROOT}/build/default}"
SIM="${BUILD_DIR}/Simulator/simulator_207190406_209543255"
COMPOSE="${REPO_ROOT}/Simulator/tests/fixtures/tiny_compose.yaml"
SCRATCH="/tmp/ex3_verify/competition_outdir"
ALGO="${BUILD_DIR}/Algorithm/Algorithm_207190406_209543255.so"
MC="${BUILD_DIR}/MissionControl/MissionControl_207190406_209543255.so"

rm -rf "${SCRATCH}"
mkdir -p "${SCRATCH}/algorithms"
cp "$ALGO" "${SCRATCH}/algorithms/"

"$SIM" -competition \
    simulation="${COMPOSE}" \
    mission_control="${MC}" \
    algorithms_folder="${SCRATCH}/algorithms"
"$SIM" -competition \
    simulation="${COMPOSE}" \
    mission_control="${MC}" \
    algorithms_folder="${SCRATCH}/algorithms"

after=$(find "${SCRATCH}/algorithms" -maxdepth 1 -type d -name 'competition_*' | sort)
count=$(echo "$after" | grep -c 'competition_' || true)
echo "--- result directories ---"
echo "$after"
if [ "$count" -lt 2 ]; then
    echo "FAIL: expected 2 distinct competition_* directories, found ${count}" >&2
    exit 1
fi
# Guard against accidental comparative_ prefix under algorithms_folder:
if find "${SCRATCH}/algorithms" -maxdepth 1 -type d -name 'comparative_results_*' | grep -q .; then
    echo "FAIL: competition mode must not create comparative_results_* under algorithms_folder" >&2
    exit 1
fi
echo "PASS: ${count} distinct competition_* directories"
