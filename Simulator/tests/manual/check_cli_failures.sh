#!/usr/bin/env bash
# Confirms every documented CLI failure mode degrades gracefully (never a crash) and
# names every problem, not just the first one found.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
BUILD_DIR="${1:-${REPO_ROOT}/build/default}"
SIM="${BUILD_DIR}/Simulator/simulator_207190406_209543255"

"${REPO_ROOT}/Simulator/tests/manual/make_fixture_dirs.sh" "${BUILD_DIR}"

run_and_assert() {
    local label="$1"; shift
    local -a needles=()
    while [[ "${1:-}" == --contains ]]; do
        shift
        needles+=("$1")
        shift
    done
    echo "=== ${label} ==="
    set +e
    local out
    out=$("$@" 2>&1)
    local code=$?
    set -e
    echo "$out"
    if [ "$code" -ge 128 ]; then
        echo "FAIL: ${label} crashed (signal $((code - 128)))" >&2
        exit 1
    fi
    local n
    for n in "${needles[@]}"; do
        echo "$out" | grep -Fq "$n" \
          || { echo "FAIL: ${label}: output missing '${n}'" >&2; exit 1; }
    done
    echo "(exit code: ${code})"
    echo
}

# 1. Unsupported argument
run_and_assert "unsupported argument" \
    --contains "unsupported argument(s):" --contains "bogus_arg" \
    "$SIM" -comparative \
    simulation="${REPO_ROOT}/inputs/sim_compose.yaml" \
    mission_control_folder=/tmp/ex3_verify/plugins/mission_controls \
    algorithm="${BUILD_DIR}/Algorithm/Algorithm_207190406_209543255.so" \
    bogus_arg=1

# 2. Two unsupported arguments together
run_and_assert "two unsupported arguments" \
    --contains "unsupported argument(s):" --contains "foo" --contains "bar" \
    "$SIM" -comparative \
    simulation="${REPO_ROOT}/inputs/sim_compose.yaml" \
    mission_control_folder=/tmp/ex3_verify/plugins/mission_controls \
    algorithm="${BUILD_DIR}/Algorithm/Algorithm_207190406_209543255.so" \
    foo=1 bar=2

# 3. Two missing arguments together (omit mission_control_folder and algorithm)
run_and_assert "two missing arguments" \
    --contains "missing argument(s):" --contains "mission_control_folder" --contains "algorithm" \
    "$SIM" -comparative \
    simulation="${REPO_ROOT}/inputs/sim_compose.yaml"

# 4. Nonexistent file argument
run_and_assert "nonexistent simulation file" \
    --contains "simulation" --contains "missing or unopenable" \
    "$SIM" -comparative \
    simulation=/tmp/ex3_verify/does_not_exist.yaml \
    mission_control_folder=/tmp/ex3_verify/plugins/mission_controls \
    algorithm="${BUILD_DIR}/Algorithm/Algorithm_207190406_209543255.so"

# 5. Empty folder (zero .so files)
run_and_assert "empty mission_control_folder" \
    --contains "mission_control_folder" --contains "no .so files" \
    "$SIM" -comparative \
    simulation="${REPO_ROOT}/inputs/sim_compose.yaml" \
    mission_control_folder=/tmp/ex3_verify/plugins/mission_controls_empty \
    algorithm="${BUILD_DIR}/Algorithm/Algorithm_207190406_209543255.so"

# 6. Missing '=' (scrambled/malformed argument)
run_and_assert "malformed argument (no =)" \
    --contains "unsupported argument(s):" --contains "simulation" \
    "$SIM" -comparative \
    simulation"${REPO_ROOT}/inputs/sim_compose.yaml" \
    mission_control_folder=/tmp/ex3_verify/plugins/mission_controls \
    algorithm="${BUILD_DIR}/Algorithm/Algorithm_207190406_209543255.so"

echo "All CLI failure cases finished without a crash."
