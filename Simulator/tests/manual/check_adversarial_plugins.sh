#!/usr/bin/env bash
# VAR-03: adversarial Algorithm/MissionControl fixtures — no crash under our simulator.
# Uses tiny_compose_adversarial.yaml (max_steps=25) so never_finish/bad_scan stay bounded.
# Each invocation is wrapped in `timeout` so a lidar/algo hang fails the check instead of
# wedging run_all.sh (seen with extreme scan_orientation vs MockLidar).
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
BUILD_DIR="${1:-${REPO_ROOT}/build/default}"
SIM="${BUILD_DIR}/Simulator/simulator_207190406_209543255"
FIXTURES="${BUILD_DIR}/Simulator/tests/fixtures"
MC_OURS="${BUILD_DIR}/MissionControl/MissionControl_207190406_209543255.so"
ALGO_OURS="${BUILD_DIR}/Algorithm/Algorithm_207190406_209543255.so"
COMPOSE="${REPO_ROOT}/Simulator/tests/fixtures/tiny_compose_adversarial.yaml"
SCRATCH="/tmp/ex3_verify/adversarial"
RUN_TIMEOUT_SEC="${ADVERSARIAL_TIMEOUT_SEC:-30}"

[ -x "$SIM" ] || { echo "FAIL: missing $SIM" >&2; exit 1; }
[ -f "$MC_OURS" ] || { echo "FAIL: missing $MC_OURS" >&2; exit 1; }
[ -f "$ALGO_OURS" ] || { echo "FAIL: missing $ALGO_OURS" >&2; exit 1; }

assert_no_crash() {
  local code="$1" label="$2"
  if [ "$code" -eq 124 ]; then
    echo "FAIL: ${label} timed out after ${RUN_TIMEOUT_SEC}s (likely hang)" >&2
    exit 1
  fi
  if [ "$code" -ge 128 ]; then
    echo "FAIL: ${label} crashed (signal $((code - 128)))" >&2
    exit 1
  fi
}

run_competition_with_algo() {
  local so="$1" label="$2"
  echo "=== adversarial algo: ${label} (${so}) ==="
  [ -f "${FIXTURES}/${so}" ] || { echo "FAIL: missing ${FIXTURES}/${so}" >&2; exit 1; }
  rm -rf "${SCRATCH}/${label}"
  mkdir -p "${SCRATCH}/${label}/algorithms"
  cp "${FIXTURES}/${so}" "${SCRATCH}/${label}/algorithms/"
  set +e
  timeout "${RUN_TIMEOUT_SEC}" "$SIM" -competition \
    simulation="${COMPOSE}" \
    mission_control="${MC_OURS}" \
    algorithms_folder="${SCRATCH}/${label}/algorithms"
  local code=$?
  set -e
  assert_no_crash "$code" "$label"
  echo "ok: ${label} exit=${code}"
}

run_comparative_with_mc() {
  local so="$1" label="$2"
  echo "=== adversarial mc: ${label} (${so}) ==="
  [ -f "${FIXTURES}/${so}" ] || { echo "FAIL: missing ${FIXTURES}/${so}" >&2; exit 1; }
  rm -rf "${SCRATCH}/${label}"
  mkdir -p "${SCRATCH}/${label}/mission_controls"
  cp "${FIXTURES}/${so}" "${SCRATCH}/${label}/mission_controls/"
  set +e
  timeout "${RUN_TIMEOUT_SEC}" "$SIM" -comparative \
    simulation="${COMPOSE}" \
    mission_control_folder="${SCRATCH}/${label}/mission_controls" \
    algorithm="${ALGO_OURS}"
  local code=$?
  set -e
  assert_no_crash "$code" "$label"
  report=$(find "${SCRATCH}/${label}/mission_controls"/comparative_results_* \
    -maxdepth 1 -name 'comparative_report.yaml' 2>/dev/null | sort | tail -n1 || true)
  [ -n "$report" ] || { echo "FAIL: ${label}: missing comparative_report" >&2; exit 1; }
  echo "ok: ${label} exit=${code} report=${report}"
}

run_competition_with_algo adversarial_throw_algorithm_plugin.so throw_algo
run_competition_with_algo adversarial_never_finish_algorithm_plugin.so never_finish
run_competition_with_algo adversarial_into_occupied_algorithm_plugin.so into_occ

run_comparative_with_mc adversarial_throw_mission_control_plugin.so throw_mc
run_comparative_with_mc adversarial_empty_mission_control_plugin.so empty_mc
run_comparative_with_mc adversarial_implausible_steps_mission_control_plugin.so implausible_mc

# bad_scan last: extreme angles currently hang MockLidar (trig on ~1e12 deg).
run_competition_with_algo adversarial_bad_scan_orientation_algorithm_plugin.so bad_scan

echo "PASS: check_adversarial_plugins"