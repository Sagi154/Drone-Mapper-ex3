#pragma once

#include <Simulator/ISimulationRunFactory.h>

#include <Common/MappingAlgorithmFactory.h>
#include <Common/MissionControlFactory.h>

namespace simulator {

/// Builds a fully-wired ISimulationRun for one (simulation, mission, drone, lidar) cell.
///
/// Constructor signature is frozen so the Simulator orchestrator (which holds the
/// loaded plugin factories) can instantiate this without knowing the plugin details.
class SimulationRunFactoryImpl final : public ISimulationRunFactory {
public:
    SimulationRunFactoryImpl(const common::MappingAlgorithmFactory& algorithm_factory,
                              const common::MissionControlFactory&   mission_control_factory,
                              bool                                   verbose);

    [[nodiscard]] std::unique_ptr<ISimulationRun> create(
        const types::SimulationConfigData&    simulation_config,
        const common::types::MissionConfigData& mission_config,
        const common::types::DroneConfigData&   drone_config,
        const common::types::LidarConfigData&   lidar_config,
        const std::filesystem::path&            output_path) override;

private:
    common::MappingAlgorithmFactory  algorithm_factory_;
    common::MissionControlFactory    mission_control_factory_;
    bool                             verbose_ = false;
};

} // namespace simulator
