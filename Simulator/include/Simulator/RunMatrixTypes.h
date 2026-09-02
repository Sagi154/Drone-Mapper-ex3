#pragma once

#include <Simulator/ISimulationRunFactory.h>
#include <Simulator/SimulationTypes.h>

#include <cstddef>
#include <functional>
#include <string>
#include <vector>

namespace simulator {

struct MatrixCell {
    std::size_t group_index = 0;
    std::size_t mission_index = 0;
    std::size_t drone_index = 0;
    std::size_t lidar_index = 0;
    const types::SimulationConfigData* simulation = nullptr;
    const common::types::MissionConfigData* mission = nullptr;
    const common::types::DroneConfigData* drone = nullptr;
    const common::types::LidarConfigData* lidar = nullptr;
};

struct PluginMatrixBinding {
    std::string plugin_filename;
    std::reference_wrapper<ISimulationRunFactory> factory;
};

struct PluginMatrixResult {
    std::string plugin_filename;
    std::vector<types::SimulationResult> results; // size == composition cell count
};

} // namespace simulator
