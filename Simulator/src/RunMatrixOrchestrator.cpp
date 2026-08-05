#include <Simulator/RunMatrixOrchestrator.h>

#include <Simulator/ISimulationRun.h>

#include <iostream>
#include <utility>

namespace simulator {
namespace {

[[nodiscard]] types::SimulationResult makeFailureResult(
    const MatrixCell& cell,
    const std::filesystem::path& output_path) {
    types::SimulationResult result;
    if (cell.simulation != nullptr) {
        result.simulation_config = *cell.simulation;
    }
    if (cell.mission != nullptr) {
        result.mission_config = *cell.mission;
    }
    result.output_map_file = output_path;
    result.mission_score   = -1.0;
    return result;
}

} // namespace

std::vector<MatrixCell> RunMatrixOrchestrator::expand(
    const types::SimulationCompositionData& composition) {
    std::vector<MatrixCell> cells;
    for (std::size_t g = 0; g < composition.simulation_mission_groups.size(); ++g) {
        const auto& [simulation, missions] = composition.simulation_mission_groups[g];
        for (std::size_t m = 0; m < missions.size(); ++m) {
            for (std::size_t d = 0; d < composition.drone_configs.size(); ++d) {
                for (std::size_t l = 0; l < composition.lidar_configs.size(); ++l) {
                    MatrixCell cell;
                    cell.group_index   = g;
                    cell.mission_index = m;
                    cell.drone_index   = d;
                    cell.lidar_index   = l;
                    cell.simulation    = &simulation;
                    cell.mission       = &missions[m];
                    cell.drone         = &composition.drone_configs[d];
                    cell.lidar         = &composition.lidar_configs[l];
                    cells.push_back(cell);
                }
            }
        }
    }
    return cells;
}

std::vector<PluginMatrixResult> RunMatrixOrchestrator::run(
    const std::vector<PluginMatrixBinding>& plugins,
    const types::SimulationCompositionData& composition,
    const std::filesystem::path& output_root,
    unsigned num_threads) {
    const std::vector<MatrixCell> cells = expand(composition);
    const std::size_t cell_count        = cells.size();
    const std::size_t plugin_count      = plugins.size();

    std::vector<PluginMatrixResult> table(plugin_count);
    for (std::size_t p = 0; p < plugin_count; ++p) {
        table[p].plugin_filename = plugins[p].plugin_filename;
        table[p].results.resize(cell_count);
    }

    if (plugin_count == 0 || cell_count == 0) {
        return table;
    }

    const std::size_t total = plugin_count * cell_count;

    (void)WorkDistributor::distribute(
        total, num_threads,
        [&](std::size_t flat_index) {
            const std::size_t plugin_index = flat_index / cell_count;
            const std::size_t cell_index   = flat_index % cell_count;
            const MatrixCell& cell        = cells[cell_index];
            PluginMatrixBinding binding   = plugins[plugin_index];

            const std::filesystem::path run_out =
                output_root / binding.plugin_filename / std::to_string(cell_index);

            if (binding.factory == nullptr || cell.simulation == nullptr ||
                cell.mission == nullptr || cell.drone == nullptr || cell.lidar == nullptr) {
                table[plugin_index].results[cell_index] = makeFailureResult(cell, run_out);
                return;
            }

            auto run = binding.factory->create(*cell.simulation, *cell.mission, *cell.drone,
                                               *cell.lidar, run_out);
            if (!run) {
                table[plugin_index].results[cell_index] = makeFailureResult(cell, run_out);
                return;
            }
            table[plugin_index].results[cell_index] = run->run();
        },
        [&](std::size_t flat_index) {
            const std::size_t plugin_index = flat_index / cell_count;
            const std::size_t cell_index   = flat_index % cell_count;
            std::cerr << "error: matrix cell plugin=" << plugins[plugin_index].plugin_filename
                      << " cell=" << cell_index << " threw\n";
            table[plugin_index].results[cell_index] =
                makeFailureResult(cells[cell_index],
                                  output_root / plugins[plugin_index].plugin_filename /
                                      std::to_string(cell_index));
        });

    return table;
}

} // namespace simulator
