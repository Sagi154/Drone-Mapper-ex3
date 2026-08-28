#pragma once

#include "HostMap3D.h"

#include <Common/Types.h>
#include <Simulator/SimulationTypes.h>

#include <filesystem>

namespace skeleton_host {

HostMap3D loadHiddenMapFromNpy(const std::filesystem::path& npy_path,
                               const simulator::types::SimulationConfigData& simulation);

HostMap3D makeEmptyOutputMap(const common::types::MissionConfigData& mission,
                             const simulator::types::SimulationConfigData& simulation);

}  // namespace skeleton_host
