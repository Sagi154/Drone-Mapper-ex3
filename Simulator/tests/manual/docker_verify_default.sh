#!/usr/bin/env bash
# Default-preset artifact checks + all manual verification scripts (Docker).
set -euo pipefail
export VCPKG_ROOT=/usr/local/vcpkg
cd /work
sed -i 's/\r$//' Simulator/tests/manual/*.sh || true
chmod +x Simulator/tests/manual/*.sh

ls -l build/default/Simulator/simulator_207190406_209543255
ls -l build/default/Algorithm/Algorithm_207190406_209543255.so
ls -l build/default/MissionControl/MissionControl_207190406_209543255.so
nm -D build/default/Simulator/simulator_207190406_209543255 | grep -c "MappingAlgorithmRegistration\|MissionControlRegistration"

./Simulator/tests/manual/run_all.sh /work/build/default
