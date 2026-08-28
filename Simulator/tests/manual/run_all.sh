#!/usr/bin/env bash
# Entry point: run fixture setup plus every default-preset verification script.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${1:-}"

if [ -z "${BUILD_DIR}" ]; then
    REPO_ROOT="$(cd "${ROOT}/../../.." && pwd)"
    BUILD_DIR="${REPO_ROOT}/build/default"
fi

"${ROOT}/make_fixture_dirs.sh" "${BUILD_DIR}"
"${ROOT}/run_smoke_pass.sh" "${BUILD_DIR}"
"${ROOT}/check_output_dir_collision.sh" "${BUILD_DIR}"
"${ROOT}/check_competition_output_dir.sh" "${BUILD_DIR}"
"${ROOT}/check_output_dir_unwritable.sh" "${BUILD_DIR}"
"${ROOT}/check_wall_collision_fault.sh" "${BUILD_DIR}"
"${ROOT}/check_verbose.sh" "${BUILD_DIR}"
"${ROOT}/check_threading.sh" "${BUILD_DIR}"
"${ROOT}/check_cli_failures.sh" "${BUILD_DIR}"
"${ROOT}/check_cli_argument_order.sh" "${BUILD_DIR}"
"${ROOT}/check_all_folder_plugins_fail.sh" "${BUILD_DIR}"
"${ROOT}/check_isolation.sh" "${BUILD_DIR}"
"${ROOT}/check_multi_plugin_outputs.sh" "${BUILD_DIR}"

echo "run_all.sh: all default-preset checks finished"
