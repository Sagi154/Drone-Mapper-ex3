#!/usr/bin/env bash
# VAR-04: our Algorithm + baseline lawnmower in competition — both named in report, no crash.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
BUILD_DIR="${1:-${REPO_ROOT}/build/default}"
SIM="${BUILD_DIR}/Simulator/simulator_207190406_209543255"
MC="${BUILD_DIR}/MissionControl/MissionControl_207190406_209543255.so"
ALGO="${BUILD_DIR}/Algorithm/Algorithm_207190406_209543255.so"
BASE="${BUILD_DIR}/Simulator/tests/fixtures/baseline_lawnmower_algorithm_plugin.so"
COMPOSE="${REPO_ROOT}/Simulator/tests/fixtures/tiny_compose.yaml"
SCRATCH="/tmp/ex3_verify/baseline"

[ -x "$SIM" ] || { echo "FAIL: missing $SIM" >&2; exit 1; }
[ -f "$MC" ] || { echo "FAIL: missing $MC" >&2; exit 1; }
[ -f "$ALGO" ] || { echo "FAIL: missing $ALGO" >&2; exit 1; }
[ -f "$BASE" ] || { echo "FAIL: missing $BASE" >&2; exit 1; }

rm -rf "$SCRATCH"
mkdir -p "${SCRATCH}/algorithms"
cp "$ALGO" "$BASE" "${SCRATCH}/algorithms/"

set +e
"$SIM" -competition \
  simulation="${COMPOSE}" \
  mission_control="${MC}" \
  algorithms_folder="${SCRATCH}/algorithms"
code=$?
set -e

if [ "$code" -ge 128 ]; then
  echo "FAIL: simulator crashed under baseline competition (signal $((code - 128)))" >&2
  exit 1
fi

report=$(find "${SCRATCH}/algorithms"/competition_* -maxdepth 1 -name 'competitive_report.yaml' 2>/dev/null | sort | tail -n1 || true)
[ -n "$report" ] || { echo "FAIL: missing competitive_report.yaml" >&2; exit 1; }

grep -q 'Algorithm_207190406_209543255.so' "$report" \
  || { echo "FAIL: report missing our Algorithm.so" >&2; exit 1; }
grep -q 'baseline_lawnmower_algorithm_plugin.so' "$report" \
  || { echo "FAIL: report missing baseline_lawnmower_algorithm_plugin.so" >&2; exit 1; }

# Both plugins should get a per-algorithm simulation_output.yaml (names vary by stem).
out_dir=$(dirname "$report")
our_out=$(find "$out_dir" -maxdepth 1 -name '*Algorithm_207190406_209543255*simulation_output.yaml' | head -n1 || true)
base_out=$(find "$out_dir" -maxdepth 1 -name '*baseline_lawnmower*simulation_output.yaml' | head -n1 || true)
[ -n "$our_out" ] || { echo "FAIL: missing our algorithm simulation_output.yaml under $out_dir" >&2; ls -la "$out_dir" >&2; exit 1; }
[ -n "$base_out" ] || { echo "FAIL: missing baseline simulation_output.yaml under $out_dir" >&2; ls -la "$out_dir" >&2; exit 1; }

echo "PASS: check_baseline_algorithm"