#!/usr/bin/env bash
# PLUGIN-01 / PLUGIN-02 / YAML-OUT-03: two genuinely distinct loadable plugins per mode;
# each successful plugin gets <name>_simulation_output.yaml in the results dir.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
BUILD_DIR="${1:-${REPO_ROOT}/build/default}"
SIM="${BUILD_DIR}/Simulator/simulator_207190406_209543255"
COMPOSE="${REPO_ROOT}/Simulator/tests/fixtures/tiny_compose.yaml"
SCRATCH="/tmp/ex3_verify/multi_plugin"

"${REPO_ROOT}/Simulator/tests/manual/make_fixture_dirs.sh" "${BUILD_DIR}"

rm -rf "${SCRATCH}"
mkdir -p "${SCRATCH}/mission_controls" "${SCRATCH}/algorithms"
cp /tmp/ex3_verify/plugins/mission_controls/*.so "${SCRATCH}/mission_controls/"
cp /tmp/ex3_verify/plugins/algorithms/*.so "${SCRATCH}/algorithms/"

"$SIM" -comparative \
    simulation="${COMPOSE}" \
    mission_control_folder="${SCRATCH}/mission_controls" \
    algorithm="${SCRATCH}/algorithms/Algorithm_207190406_209543255.so"

cmp_dir=$(find "${SCRATCH}/mission_controls" -maxdepth 1 -type d -name 'comparative_results_*' | sort | tail -n1)
test -n "${cmp_dir}" || { echo "FAIL: no comparative_results_* dir" >&2; exit 1; }
test -f "${cmp_dir}/MissionControl_207190406_209543255.so_simulation_output.yaml" \
  || { echo "FAIL: missing our MC simulation_output.yaml" >&2; exit 1; }
test -f "${cmp_dir}/valid_mission_control_plugin.so_simulation_output.yaml" \
  || { echo "FAIL: missing fixture MC simulation_output.yaml" >&2; exit 1; }
grep -E 'MissionControl_207190406_209543255\.so|valid_mission_control_plugin\.so' \
  "${cmp_dir}/comparative_report.yaml" >/dev/null \
  || { echo "FAIL: comparative_report missing plugin filenames" >&2; exit 1; }

"$SIM" -competition \
    simulation="${COMPOSE}" \
    mission_control="${SCRATCH}/mission_controls/MissionControl_207190406_209543255.so" \
    algorithms_folder="${SCRATCH}/algorithms"

comp_dir=$(find "${SCRATCH}/algorithms" -maxdepth 1 -type d -name 'competition_*' | sort | tail -n1)
test -n "${comp_dir}" || { echo "FAIL: no competition_* dir" >&2; exit 1; }
test -f "${comp_dir}/Algorithm_207190406_209543255.so_simulation_output.yaml" \
  || { echo "FAIL: missing our Algorithm simulation_output.yaml" >&2; exit 1; }
test -f "${comp_dir}/valid_algorithm_plugin.so_simulation_output.yaml" \
  || { echo "FAIL: missing fixture Algorithm simulation_output.yaml" >&2; exit 1; }

echo "PASS: multi-plugin outputs present for comparative and competition"
