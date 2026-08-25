#!/usr/bin/env bash
# Run a command inside the course-like Ubuntu 24.04 + vcpkg image.
# Usage: ./run_in_docker.sh [command...]
# Default command: cmake --preset default && cmake --build --preset default
set -euo pipefail

IMAGE="${EX3_DOCKER_IMAGE:-drone-mapper-ex3-dev}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"

if [ "$#" -eq 0 ]; then
  set -- bash -lc 'export VCPKG_ROOT=/usr/local/vcpkg; cmake --preset default && cmake --build --preset default'
fi

exec docker run --rm \
  -e VCPKG_ROOT=/usr/local/vcpkg \
  -v "${REPO_ROOT}:/work" \
  -w /work \
  "${IMAGE}" \
  "$@"
