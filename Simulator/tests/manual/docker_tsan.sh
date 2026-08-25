#!/usr/bin/env bash
# Build and run ThreadSanitizer on the container's own filesystem (not the Windows bind mount).
set -euo pipefail
export VCPKG_ROOT=/usr/local/vcpkg

rm -rf /tmp/ex3
mkdir -p /tmp/ex3
tar -C /work --exclude=build --exclude=.git -cf - . | tar -C /tmp/ex3 -xf -
cd /tmp/ex3
# TSan on recent kernels (Docker Desktop) needs reduced ASLR entropy.
sysctl -w vm.mmap_rnd_bits=28 >/dev/null 2>&1 || true
sed -i 's/\r$//' Simulator/tests/manual/*.sh || true

echo "=== tsan configure+build (timeout 12m) ==="
timeout 720 cmake --preset tsan
timeout 720 cmake --build --preset tsan --parallel 2 --target \
  simulator_207190406_209543255 Algorithm_207190406_209543255 MissionControl_207190406_209543255

SIM=/tmp/ex3/build/tsan/Simulator/simulator_207190406_209543255
ALGO=/tmp/ex3/build/tsan/Algorithm/Algorithm_207190406_209543255.so
MC=/tmp/ex3/build/tsan/MissionControl/MissionControl_207190406_209543255.so
LOG=/work/tmp/tsan_output.log
mkdir -p /work/tmp /tmp/ex3_verify/tsan/mc
cp "$MC" /tmp/ex3_verify/tsan/mc/
: > "$LOG"
export TSAN_OPTIONS="halt_on_error=0:history_size=7"

run_cmp() {
  echo "=== tsan comparative $* ===" | tee -a "$LOG"
  timeout 300 "$SIM" -comparative \
    simulation=/tmp/ex3/inputs/sim_compose.yaml \
    mission_control_folder=/tmp/ex3_verify/tsan/mc \
    algorithm="$ALGO" \
    "$@" 2>&1 | tee -a "$LOG"
}

run_cmp num_threads=8
run_cmp num_threads=2
run_cmp

echo "=== ThreadSanitizer warning count ===" | tee -a "$LOG"
grep -c "WARNING: ThreadSanitizer" "$LOG" || echo 0
