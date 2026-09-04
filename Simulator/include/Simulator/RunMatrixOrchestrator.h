// RunMatrixOrchestrator.h — expand a composition into a run matrix and execute
// each cell via ISimulationRunFactory / ISimulationRun.

#pragma once

#include <Simulator/RunMatrixTypes.h>

#include <filesystem>
#include <vector>

namespace simulator {

[[nodiscard]] std::size_t countRunMatrixCells(const types::SimulationCompositionData& composition);

/// Flat expansion: for each group, each mission in that group, each drone, each lidar.
[[nodiscard]] std::vector<MatrixCell> expandRunMatrix(
    const types::SimulationCompositionData& composition);

/// Run the full matrix for every plugin binding.
/// Result table is pre-sized to plugins × cells; each slot is written exactly once.
/// `num_threads` follows distributeWork semantics (≤1 → main only).
/// Per-run output paths are `{output_root}/{plugin}_run_<NNNN>_output_map.npy`
/// (NNNN = zero-padded flat cell index, unique across the whole run matrix).
[[nodiscard]] std::vector<PluginMatrixResult> runPluginMatrix(
    const std::vector<PluginMatrixBinding>& plugins,
    const types::SimulationCompositionData& composition,
    const std::filesystem::path& output_root,
    unsigned num_threads);

} // namespace simulator
