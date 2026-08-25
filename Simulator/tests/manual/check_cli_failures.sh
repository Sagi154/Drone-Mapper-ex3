#!/usr/bin/env bash
# Confirms every documented CLI failure mode degrades gracefully (never a crash) and
# names every problem, not just the first one found.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
BUILD_DIR="${1:-${REPO_ROOT}/build/default}"
SIM="${BUILD_DIR}/Simulator/simulator_207190406_209543255"

run_and_report() {
    local label="$1"; shift
    echo "=== ${label} ==="
    set +e
    "$@" 2>&1
    local code=$?
    set -e
    if [ "$code" -ge 128 ]; then
        echo "FAIL: ${label} crashed (signal $((code - 128)))" >&2
        exit 1
    fi
    echo "(exit code: ${code})"
    echo
}

# 1. Unsupported argument
run_and_report "unsupported argument" "$SIM" -comparative \
    simulation="${REPO_ROOT}/inputs/sim_compose.yaml" \
    mission_control_folder=/tmp/ex3_verify/plugins/mission_controls \
    algorithm="${BUILD_DIR}/Algorithm/Algorithm_207190406_209543255.so" \
    bogus_arg=1

# 2. Two missing arguments together (omit mission_control_folder and algorithm)
run_and_report "two missing arguments" "$SIM" -comparative \
    simulation="${REPO_ROOT}/inputs/sim_compose.yaml"

# 3. Nonexistent file argument
run_and_report "nonexistent simulation file" "$SIM" -comparative \
    simulation=/tmp/ex3_verify/does_not_exist.yaml \
    mission_control_folder=/tmp/ex3_verify/plugins/mission_controls \
    algorithm="${BUILD_DIR}/Algorithm/Algorithm_207190406_209543255.so"

# 4. Empty folder (zero .so files)
run_and_report "empty mission_control_folder" "$SIM" -comparative \
    simulation="${REPO_ROOT}/inputs/sim_compose.yaml" \
    mission_control_folder=/tmp/ex3_verify/plugins/mission_controls_empty \
    algorithm="${BUILD_DIR}/Algorithm/Algorithm_207190406_209543255.so"

# 5. Missing '=' (scrambled/malformed argument)
run_and_report "malformed argument (no =)" "$SIM" -comparative \
    simulation"${REPO_ROOT}/inputs/sim_compose.yaml" \
    mission_control_folder=/tmp/ex3_verify/plugins/mission_controls \
    algorithm="${BUILD_DIR}/Algorithm/Algorithm_207190406_209543255.so"

echo "All CLI failure cases finished without a crash."
