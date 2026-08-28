#!/usr/bin/env bash
# VAR-02: our Algorithm under foreign hits-only MissionControl (competition path).
# Diagnostic: dumps coverage evidence; does NOT auto-fail on low score.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
BUILD_DIR="${1:-${REPO_ROOT}/build/default}"
SIM="${BUILD_DIR}/Simulator/simulator_207190406_209543255"
HOST="${BUILD_DIR}/Simulator/skeleton_host"
ALGO="${BUILD_DIR}/Algorithm/Algorithm_207190406_209543255.so"
FOREIGN_MC="${BUILD_DIR}/Simulator/tests/fixtures/foreign_hits_only_mission_control_plugin.so"
INPUTS="${REPO_ROOT}/inputs"
COMPOSE="${REPO_ROOT}/Simulator/tests/fixtures/tiny_compose.yaml"
SCRATCH="/tmp/ex3_verify/foreign_mc"
FINDINGS="/tmp/ex3_verify/foreign_mc_findings.txt"

[ -f "$FOREIGN_MC" ] || { echo "FAIL: missing $FOREIGN_MC" >&2; exit 1; }
[ -x "$SIM" ] || { echo "FAIL: missing $SIM" >&2; exit 1; }
[ -f "$ALGO" ] || { echo "FAIL: missing $ALGO" >&2; exit 1; }

rm -rf "${SCRATCH}"
mkdir -p "${SCRATCH}/algorithms"
cp "$ALGO" "${SCRATCH}/algorithms/"

set +e
"$SIM" -competition \
  simulation="${COMPOSE}" \
  mission_control="${FOREIGN_MC}" \
  algorithms_folder="${SCRATCH}/algorithms" >"${SCRATCH}/sim_stdout.txt" 2>"${SCRATCH}/sim_stderr.txt"
sim_code=$?
set -e

if [ "$sim_code" -ge 128 ]; then
  echo "FAIL: simulator crashed under foreign MC" >&2
  exit 1
fi

report=$(find "${SCRATCH}/algorithms"/competition_* -maxdepth 1 -name 'competitive_report.yaml' 2>/dev/null | sort | tail -n1 || true)
{
  echo "=== VAR-02 findings $(date -u +%Y-%m-%dT%H:%M:%SZ) ==="
  echo "sim_exit=${sim_code}"
  echo "report=${report}"
  [ -n "$report" ] && cat "$report"
  echo "--- stderr ---"
  cat "${SCRATCH}/sim_stderr.txt"
  echo "--- stdout ---"
  cat "${SCRATCH}/sim_stdout.txt"
  echo "--- host cross-check (if skeleton_host present) ---"
  if [ -x "$HOST" ]; then
    "$HOST" \
      --algorithm="$ALGO" \
      --mission-control="$FOREIGN_MC" \
      --simulation="${INPUTS}/simulation/small_simulation_room.yaml" \
      --mission="${INPUTS}/mission/small_mission_room.yaml" \
      --drone="${INPUTS}/drone/drone_small.yaml" \
      --lidar="${INPUTS}/lidar/lidar_short.yaml" || true
  fi
} | tee "$FINDINGS"

echo "FINDINGS written to ${FINDINGS}"
echo "STOP: present findings to human before any Algorithm/MissionControl production fix."
echo "PASS: check_foreign_mission_control (diagnostic completed; crash-free)"