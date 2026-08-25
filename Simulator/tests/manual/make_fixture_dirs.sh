#!/usr/bin/env bash
# Builds scratch plugin folders for manual verification runs. Safe to re-run (idempotent).
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
BUILD_DIR="${1:-${REPO_ROOT}/build/default}"
SCRATCH="/tmp/ex3_verify"

ALGO_SO="${BUILD_DIR}/Algorithm/Algorithm_207190406_209543255.so"
MC_SO="${BUILD_DIR}/MissionControl/MissionControl_207190406_209543255.so"

for f in "$ALGO_SO" "$MC_SO"; do
    [ -f "$f" ] || { echo "missing built plugin: $f (build first)" >&2; exit 1; }
done

rm -rf "${SCRATCH}/plugins"
mkdir -p "${SCRATCH}/plugins/algorithms" "${SCRATCH}/plugins/mission_controls" \
         "${SCRATCH}/plugins/algorithms_empty" "${SCRATCH}/plugins/mission_controls_empty"

cp "$ALGO_SO" "${SCRATCH}/plugins/algorithms/Algorithm_207190406_209543255.so"
cp "$ALGO_SO" "${SCRATCH}/plugins/algorithms/Algorithm_207190406_209543255_copy2.so"
cp "$MC_SO" "${SCRATCH}/plugins/mission_controls/MissionControl_207190406_209543255.so"
cp "$MC_SO" "${SCRATCH}/plugins/mission_controls/MissionControl_207190406_209543255_copy2.so"

echo "fixture folders ready under ${SCRATCH}/plugins"
