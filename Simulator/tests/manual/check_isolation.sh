#!/usr/bin/env bash
# 1. Confirms each plugin exports nothing surprising besides the registration constructor.
# 2. Loads our Algorithm .so plus the distinct valid_algorithm_plugin.so fixture in the
#    same process and confirms both run independently under RTLD_LOCAL instead of colliding.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
BUILD_DIR="${1:-${REPO_ROOT}/build/default}"
SIM="${BUILD_DIR}/Simulator/simulator_207190406_209543255"
COMPOSE="${REPO_ROOT}/Simulator/tests/fixtures/tiny_compose.yaml"
SCRATCH="/tmp/ex3_verify/plugins"

echo "=== undefined symbols in Algorithm .so (expect only Registration + libc/libstdc++) ==="
nm -DC --undefined-only "${BUILD_DIR}/Algorithm/Algorithm_207190406_209543255.so" | grep -i registration || true
nm -DC --undefined-only "${BUILD_DIR}/Algorithm/Algorithm_207190406_209543255.so"

echo "=== undefined symbols in MissionControl .so ==="
nm -DC --undefined-only "${BUILD_DIR}/MissionControl/MissionControl_207190406_209543255.so" | grep -i registration || true
nm -DC --undefined-only "${BUILD_DIR}/MissionControl/MissionControl_207190406_209543255.so"

echo "=== running competition mode with our Algorithm .so + valid_algorithm_plugin.so fixture ==="
"$SIM" -competition \
    simulation="${COMPOSE}" \
    mission_control="${SCRATCH}/mission_controls/MissionControl_207190406_209543255.so" \
    algorithms_folder="${SCRATCH}/algorithms"

report=$(find "${SCRATCH}/algorithms"/competition_* -maxdepth 1 -name 'competitive_report.yaml' | sort | tail -n1)
echo "--- ${report} ---"
cat "$report"

grep -q 'valid_algorithm_plugin.so' "$report" \
  || { echo "FAIL: competitive_report must mention valid_algorithm_plugin.so (distinct fixture)" >&2; exit 1; }
grep -q 'Algorithm_207190406_209543255.so' "$report" \
  || { echo "FAIL: competitive_report must mention our Algorithm .so" >&2; exit 1; }
if grep -q '_copy2' "$report"; then
  echo "FAIL: isolation must not use _copy2 clone of our own .so" >&2
  exit 1
fi
