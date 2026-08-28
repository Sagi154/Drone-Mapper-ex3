#!/usr/bin/env bash
# VAR-01: our plugins under blind skeleton_host on staff inputs maps.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
BUILD_DIR="${1:-${REPO_ROOT}/build/default}"
HOST="${BUILD_DIR}/Simulator/skeleton_host"
ALGO="${BUILD_DIR}/Algorithm/Algorithm_207190406_209543255.so"
MC="${BUILD_DIR}/MissionControl/MissionControl_207190406_209543255.so"
INPUTS="${REPO_ROOT}/inputs"

[ -x "$HOST" ] || { echo "FAIL: missing $HOST" >&2; exit 1; }
[ -f "$ALGO" ] || { echo "FAIL: missing $ALGO" >&2; exit 1; }
[ -f "$MC" ] || { echo "FAIL: missing $MC" >&2; exit 1; }

run_one() {
  local name="$1" sim="$2" mission="$3" drone="$4" lidar="$5"
  echo "=== foreign host scenario: ${name} ==="
  set +e
  out=$("$HOST" \
    --algorithm="$ALGO" \
    --mission-control="$MC" \
    --simulation="$sim" \
    --mission="$mission" \
    --drone="$drone" \
    --lidar="$lidar" 2>&1)
  code=$?
  set -e
  echo "$out"
  if [ "$code" -ge 128 ]; then
    echo "FAIL: skeleton_host crashed on ${name} (signal $((code - 128)))" >&2
    exit 1
  fi
  echo "$out" | grep -q '^HOST_STATUS=' \
    || { echo "FAIL: ${name}: missing HOST_STATUS" >&2; exit 1; }
  echo "$out" | grep -q '^HOST_ILLEGAL_MOVE_ATTEMPTS=0$' \
    || { echo "FAIL: ${name}: illegal move into Occupied (HOST_ILLEGAL_MOVE_ATTEMPTS != 0)" >&2; exit 1; }
  status=$(echo "$out" | sed -n 's/^HOST_STATUS=//p' | tail -n1)
  case "$status" in
    Completed|MaxSteps|Error|CrashContained) ;;
    *) echo "FAIL: ${name}: unexpected HOST_STATUS=${status}" >&2; exit 1 ;;
  esac
}

run_one small_room \
  "${INPUTS}/simulation/small_simulation_room.yaml" \
  "${INPUTS}/mission/small_mission_room.yaml" \
  "${INPUTS}/drone/drone_small.yaml" \
  "${INPUTS}/lidar/lidar_short.yaml"

run_one house_lower \
  "${INPUTS}/simulation/house_simulation.yaml" \
  "${INPUTS}/mission/house_mission_lower.yaml" \
  "${INPUTS}/drone/drone_small.yaml" \
  "${INPUTS}/lidar/lidar_short.yaml"

echo "PASS: check_foreign_host"
