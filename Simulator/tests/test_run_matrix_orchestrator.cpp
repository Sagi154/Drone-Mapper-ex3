// test_run_matrix_orchestrator.cpp — fake factory + hand-built composition literal.

#include <Simulator/ISimulationRun.h>
#include <Simulator/ISimulationRunFactory.h>
#include <Simulator/RunMatrixOrchestrator.h>
#include <Simulator/SimulationTypes.h>

#include <gtest/gtest.h>

#include <atomic>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

namespace {

class FakeRun final : public simulator::ISimulationRun {
public:
    explicit FakeRun(simulator::types::SimulationResult result) : result_(std::move(result)) {}

    [[nodiscard]] simulator::types::SimulationResult run() override { return result_; }

private:
    simulator::types::SimulationResult result_;
};

class FakeRunFactory final : public simulator::ISimulationRunFactory {
public:
    std::atomic<int> create_calls{0};

    [[nodiscard]] std::unique_ptr<simulator::ISimulationRun> create(
        const simulator::types::SimulationConfigData& simulation_config,
        const common::types::MissionConfigData& mission_config,
        const common::types::DroneConfigData& /*drone_config*/,
        const common::types::LidarConfigData& /*lidar_config*/,
        const std::filesystem::path& output_path) override {
        create_calls.fetch_add(1, std::memory_order_relaxed);

        simulator::types::SimulationResult result;
        result.simulation_config = simulation_config;
        result.mission_config    = mission_config;
        result.output_map_file   = output_path;
        result.mission_score =
            static_cast<double>(mission_config.max_steps); // encode identity for asserts
        return std::make_unique<FakeRun>(std::move(result));
    }
};

class ThrowingRun final : public simulator::ISimulationRun {
public:
    [[nodiscard]] simulator::types::SimulationResult run() override {
        throw std::runtime_error("fake run boom");
    }
};

class ThrowingFactory final : public simulator::ISimulationRunFactory {
public:
    [[nodiscard]] std::unique_ptr<simulator::ISimulationRun> create(
        const simulator::types::SimulationConfigData& /*simulation_config*/,
        const common::types::MissionConfigData& /*mission_config*/,
        const common::types::DroneConfigData& /*drone_config*/,
        const common::types::LidarConfigData& /*lidar_config*/,
        const std::filesystem::path& /*output_path*/) override {
        return std::make_unique<ThrowingRun>();
    }
};

[[nodiscard]] simulator::types::SimulationCompositionData makeLiteralComposition() {
    // 2 groups × 1 mission each × 2 drones × 3 lidars = 12 cells
    // (matches workplan verify wording when each group has one mission:
    //  groups × drone_configs × lidar_configs)
    simulator::types::SimulationCompositionData composition;
    composition.composition_file = "hand_built.yaml";

    simulator::types::SimulationConfigData sim_a;
    sim_a.map_filename = "map_a.npy";
    common::types::MissionConfigData mission_a;
    mission_a.max_steps = 10;

    simulator::types::SimulationConfigData sim_b;
    sim_b.map_filename = "map_b.npy";
    common::types::MissionConfigData mission_b;
    mission_b.max_steps = 20;

    composition.simulation_mission_groups.push_back({sim_a, {mission_a}});
    composition.simulation_mission_groups.push_back({sim_b, {mission_b}});

    composition.drone_configs.resize(2);
    composition.lidar_configs.resize(3);
    return composition;
}

} // namespace

TEST(RunMatrixOrchestrator, ExpandSizeIsGroupsTimesDronesTimesLidars) {
    const auto composition = makeLiteralComposition();
    const auto cells       = simulator::RunMatrixOrchestrator::expand(composition);

    constexpr std::size_t kGroups = 2;
    constexpr std::size_t kDrones = 2;
    constexpr std::size_t kLidars = 3;
    EXPECT_EQ(cells.size(), kGroups * kDrones * kLidars);
    EXPECT_EQ(simulator::RunMatrixOrchestrator::cellCount(composition), 12U);
}

TEST(RunMatrixOrchestrator, EverySlotWrittenOnceForOnePlugin) {
    const auto composition = makeLiteralComposition();
    FakeRunFactory factory;

    const std::vector<simulator::PluginMatrixBinding> plugins = {
        {"plugin_a.so", std::ref(factory)},
    };

    const auto table = simulator::RunMatrixOrchestrator::run(
        plugins, composition, std::filesystem::path{"/tmp/orch_test"}, /*num_threads=*/1);

    ASSERT_EQ(table.size(), 1U);
    EXPECT_EQ(table[0].plugin_filename, "plugin_a.so");
    ASSERT_EQ(table[0].results.size(), 12U);
    EXPECT_EQ(factory.create_calls.load(), 12);

    // Scores encode mission max_steps from the fake: first group 10, second 20.
    for (std::size_t i = 0; i < 6; ++i) {
        EXPECT_EQ(table[0].results[i].mission_score, 10.0) << "cell " << i;
    }
    for (std::size_t i = 6; i < 12; ++i) {
        EXPECT_EQ(table[0].results[i].mission_score, 20.0) << "cell " << i;
    }
}

TEST(RunMatrixOrchestrator, TwoPluginsGetIndependentResultRows) {
    const auto composition = makeLiteralComposition();
    FakeRunFactory factory_a;
    FakeRunFactory factory_b;

    const std::vector<simulator::PluginMatrixBinding> plugins = {
        {"a.so", std::ref(factory_a)},
        {"b.so", std::ref(factory_b)},
    };

    const auto table = simulator::RunMatrixOrchestrator::run(
        plugins, composition, std::filesystem::path{"/tmp/orch_test2"}, /*num_threads=*/4);

    ASSERT_EQ(table.size(), 2U);
    EXPECT_EQ(table[0].results.size(), 12U);
    EXPECT_EQ(table[1].results.size(), 12U);
    EXPECT_EQ(factory_a.create_calls.load(), 12);
    EXPECT_EQ(factory_b.create_calls.load(), 12);
}

TEST(RunMatrixOrchestrator, ThrowingRunWritesFailureSentinelWithoutAbortingMatrix) {
    simulator::types::SimulationCompositionData composition;
    composition.simulation_mission_groups.push_back(
        {simulator::types::SimulationConfigData{}, {common::types::MissionConfigData{}}});
    composition.drone_configs.resize(1);
    composition.lidar_configs.resize(2); // 2 cells

    ThrowingFactory throwing;
    FakeRunFactory ok_factory;

    // Two plugins: first throws on every cell, second succeeds.
    const std::vector<simulator::PluginMatrixBinding> plugins = {
        {"bad.so", std::ref(throwing)},
        {"good.so", std::ref(ok_factory)},
    };

    const auto table = simulator::RunMatrixOrchestrator::run(
        plugins, composition, std::filesystem::path{"/tmp/orch_throw"}, /*num_threads=*/2);

    ASSERT_EQ(table.size(), 2U);
    ASSERT_EQ(table[0].results.size(), 2U);
    ASSERT_EQ(table[1].results.size(), 2U);
    EXPECT_EQ(table[0].results[0].mission_score, -1.0);
    EXPECT_EQ(table[0].results[1].mission_score, -1.0);
    EXPECT_EQ(ok_factory.create_calls.load(), 2);
}
