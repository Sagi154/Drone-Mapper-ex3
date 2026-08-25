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
"${ROOT}/check_collision.sh" "${BUILD_DIR}"
"${ROOT}/check_verbose.sh" "${BUILD_DIR}"
"${ROOT}/check_threading.sh" "${BUILD_DIR}"
"${ROOT}/check_cli_failures.sh" "${BUILD_DIR}"
"${ROOT}/check_isolation.sh" "${BUILD_DIR}"

echo "run_all.sh: all default-preset checks finished"
