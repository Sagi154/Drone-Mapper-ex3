#pragma once

#include <Simulator/ISimulationRunFactory.h>
#include <Simulator/SimulationTypes.h>

#include <cstddef>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

namespace simulator {

/// Score assigned to a run that failed before or during mission execution
/// (startup error, uncaught exception, or an ErrorRef-producing scenario).
inline constexpr double kErrorScore = -1.0;

/// Thrown when a plugin factory fails during ISimulationRun construction
/// (as opposed to a mid-run throw that still scores -1 in results_summary).
class PluginConstructionError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

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
    bool never_started = false; // factory threw; not a started run that scored -1
};

} // namespace simulator
