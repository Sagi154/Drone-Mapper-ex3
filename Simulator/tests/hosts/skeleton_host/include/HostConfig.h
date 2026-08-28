#pragma once

#include <Common/Types.h>
#include <Simulator/SimulationTypes.h>

#include <filesystem>

namespace skeleton_host {

struct LoadedConfigs {
    simulator::types::SimulationConfigData simulation{};
    common::types::MissionConfigData mission{};
    common::types::DroneConfigData drone{};
    common::types::LidarConfigData lidar{};
    std::filesystem::path map_path{};
};

// Loads staff YAML key shapes (see inputs/). Throws std::runtime_error on
// missing files or unreadable YAML.
LoadedConfigs loadStaffConfigs(const std::filesystem::path& simulation_yaml,
                               const std::filesystem::path& mission_yaml,
                               const std::filesystem::path& drone_yaml,
                               const std::filesystem::path& lidar_yaml);

}  // namespace skeleton_host
