// SimulationOutputYamlWriter.h — per-plugin ex2-style "Simulation Result Output
// File", ported from ../Drone-Mapper-ex2/src/io/SimulationOutputYamlWriter.cpp.

#pragma once

#include <Simulator/SimulationTypes.h>

#include <cstddef>
#include <filesystem>
#include <vector>

namespace simulator::io {

struct SimulationRunYamlEntry {
    int run_id = 0;
    std::size_t simulation_index = 0;
    std::size_t mission_index = 0;
    std::size_t drone_index = 0;
    std::size_t lidar_index = 0;
    types::SimulationResult result{};
};

/// Writes one ex2-style `simulation_output.yaml`-shaped file (root key
/// `score_report`) to `output_path` (a full file path, not a directory).
void writeSimulationOutputYaml(const std::filesystem::path& output_path,
                               const types::SimulationManagerReport& report,
                               const std::vector<SimulationRunYamlEntry>& entries);

} // namespace simulator::io
