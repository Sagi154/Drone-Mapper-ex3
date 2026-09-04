#pragma once

#include <Simulator/ISimulationRun.h>

#include <Common/IGPS.h>
#include <Common/IDroneMovement.h>
#include <Common/ILidar.h>
#include <Common/IMappingAlgorithm.h>
#include <Common/IMissionControl.h>
#include <Common/IMutableMap3D.h>

#include <filesystem>
#include <memory>
#include <vector>

namespace simulator {

/// Holds all per-run dependencies and drives one simulation run.
///
/// Completed/MaxSteps runs are scored with MapsComparison (0–100 BFS).
/// Error (including caught runMission()/save failures) stays at mission_score = -1.0.
class SimulationRunImpl final : public ISimulationRun {
public:
    SimulationRunImpl(std::unique_ptr<common::IMap3D>          hidden_map,
                      std::unique_ptr<common::IMutableMap3D>   output_map,
                      std::unique_ptr<common::IGPS>            gps,
                      std::unique_ptr<common::IDroneMovement>  movement,
                      std::unique_ptr<common::ILidar>          lidar,
                      std::unique_ptr<common::IMappingAlgorithm> mapping_algorithm,
                      std::unique_ptr<common::IMissionControl> mission_control,
                      const types::SimulationConfigData&       simulation_config,
                      const common::types::MissionConfigData&  mission_config,
                      std::filesystem::path                    output_map_file,
                      const std::vector<common::types::ErrorRef>& startup_errors);

    [[nodiscard]] types::SimulationResult run() override;

private:
    std::unique_ptr<common::IMap3D>            hidden_map_;
    std::unique_ptr<common::IMutableMap3D>     output_map_;
    std::unique_ptr<common::IGPS>              gps_;
    std::unique_ptr<common::IDroneMovement>    movement_;
    std::unique_ptr<common::ILidar>            lidar_;
    std::unique_ptr<common::IMappingAlgorithm> mapping_algorithm_;
    std::unique_ptr<common::IMissionControl>   mission_control_;
    types::SimulationConfigData                simulation_config_;
    common::types::MissionConfigData           mission_config_;
    std::filesystem::path                      output_map_file_;
    std::vector<common::types::ErrorRef>       startup_errors_;
};

} // namespace simulator
