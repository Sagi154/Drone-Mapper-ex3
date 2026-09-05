// SimulationImpl.h — in-process ISimulation for one algorithm/MC factory pair.
// CLI comparative/competitive orchestration stays in main.cpp.

#pragma once

#include <Simulator/ISimulation.h>

#include <string>

namespace simulator {

class ISimulationRunFactory;

class SimulationImpl final : public ISimulation {
public:
    SimulationImpl(ISimulationRunFactory& factory, std::string plugin_filename,
                   unsigned num_threads);

    [[nodiscard]] types::SimulationManagerReport run(
        const types::SimulationCompositionData& composition,
        const std::filesystem::path& output_path) override;

private:
    ISimulationRunFactory& factory_;
    std::string plugin_filename_;
    unsigned num_threads_ = 1;
};

} // namespace simulator
