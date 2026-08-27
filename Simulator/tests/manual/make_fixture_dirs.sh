#!/usr/bin/env bash
# Builds scratch plugin folders for manual verification runs. Safe to re-run (idempotent).
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
BUILD_DIR="${1:-${REPO_ROOT}/build/default}"
SCRATCH="/tmp/ex3_verify"

FIXTURES="${BUILD_DIR}/Simulator/tests/fixtures"
ALGO_SO="${BUILD_DIR}/Algorithm/Algorithm_207190406_209543255.so"
MC_SO="${BUILD_DIR}/MissionControl/MissionControl_207190406_209543255.so"
VALID_ALGO="${FIXTURES}/valid_algorithm_plugin.so"
VALID_MC="${FIXTURES}/valid_mission_control_plugin.so"

for f in "$ALGO_SO" "$MC_SO" "$VALID_ALGO" "$VALID_MC"; do
    [ -f "$f" ] || { echo "missing built plugin: $f (build first)" >&2; exit 1; }
done

rm -rf "${SCRATCH}/plugins"
mkdir -p "${SCRATCH}/plugins/algorithms" "${SCRATCH}/plugins/mission_controls" \
         "${SCRATCH}/plugins/algorithms_empty" "${SCRATCH}/plugins/mission_controls_empty"

cp "$ALGO_SO" "${SCRATCH}/plugins/algorithms/Algorithm_207190406_209543255.so"
cp "$VALID_ALGO" "${SCRATCH}/plugins/algorithms/valid_algorithm_plugin.so"
cp "$MC_SO" "${SCRATCH}/plugins/mission_controls/MissionControl_207190406_209543255.so"
cp "$VALID_MC" "${SCRATCH}/plugins/mission_controls/valid_mission_control_plugin.so"

echo "fixture folders ready under ${SCRATCH}/plugins"
