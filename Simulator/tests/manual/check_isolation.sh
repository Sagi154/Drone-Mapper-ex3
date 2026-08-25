#!/usr/bin/env bash
# 1. Confirms each plugin exports nothing surprising besides the registration constructor.
# 2. Loads a renamed copy of our own Algorithm .so under a second filename in the same
#    process (approximating "another team's plugin" per pre-submission-review §7 note,
#    since we don't have a real second team's .so) and confirms both instances run
#    independently under RTLD_LOCAL instead of colliding.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
BUILD_DIR="${1:-${REPO_ROOT}/build/default}"
SIM="${BUILD_DIR}/Simulator/simulator_207190406_209543255"
SCRATCH="/tmp/ex3_verify/plugins"

echo "=== undefined symbols in Algorithm .so (expect only Registration + libc/libstdc++) ==="
nm -DC --undefined-only "${BUILD_DIR}/Algorithm/Algorithm_207190406_209543255.so" | grep -i registration || true
nm -DC --undefined-only "${BUILD_DIR}/Algorithm/Algorithm_207190406_209543255.so"

echo "=== undefined symbols in MissionControl .so ==="
nm -DC --undefined-only "${BUILD_DIR}/MissionControl/MissionControl_207190406_209543255.so" | grep -i registration || true
nm -DC --undefined-only "${BUILD_DIR}/MissionControl/MissionControl_207190406_209543255.so"

echo "=== running competition mode with original + renamed-copy Algorithm .so loaded together ==="
"$SIM" -competition \
    simulation="${REPO_ROOT}/inputs/sim_compose.yaml" \
    mission_control="${SCRATCH}/mission_controls/MissionControl_207190406_209543255.so" \
    algorithms_folder="${SCRATCH}/algorithms"

report=$(find "${SCRATCH}/algorithms"/competition_* -maxdepth 1 -name 'competitive_report.yaml' | sort | tail -n1)
echo "--- ${report} ---"
cat "$report"
