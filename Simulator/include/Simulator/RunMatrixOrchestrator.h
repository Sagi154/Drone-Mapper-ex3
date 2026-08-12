// RunMatrixOrchestrator.h — expand a composition into a run matrix and execute
// each cell via ISimulationRunFactory / ISimulationRun.
// Not wired into main() yet (needs Yoav's concrete factory).

#pragma once

#include <Simulator/ISimulationRunFactory.h>
#include <Simulator/SimulationTypes.h>
#include <Simulator/WorkDistributor.h>

#include <cstddef>
#include <filesystem>
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
    ISimulationRunFactory* factory = nullptr; // non-owning; must outlive run()
};

struct PluginMatrixResult {
    std::string plugin_filename;
    std::vector<types::SimulationResult> results; // size == composition cell count
};

class RunMatrixOrchestrator {
public:
    /// Flat expansion: for each group, each mission in that group, each drone, each lidar.
    [[nodiscard]] static std::vector<MatrixCell> expand(
        const types::SimulationCompositionData& composition);

    [[nodiscard]] static std::size_t cellCount(
        const types::SimulationCompositionData& composition) {
        return expand(composition).size();
    }

    /// Run the full matrix for every plugin binding.
    /// Result table is pre-sized to plugins × cells; each slot is written exactly once.
    /// `num_threads` follows WorkDistributor semantics (≤1 → main only).
    /// Per-run output paths are `{output_root}/{plugin}_run_<NNNN>_output_map.npy`
    /// (NNNN = zero-padded flat cell index, unique across the whole run matrix).
    [[nodiscard]] static std::vector<PluginMatrixResult> run(
        const std::vector<PluginMatrixBinding>& plugins,
        const types::SimulationCompositionData& composition,
        const std::filesystem::path& output_root,
        unsigned num_threads);
};

} // namespace simulator
