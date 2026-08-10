// test_simulation_run_factory.cpp
// Covers SimulationRunFactoryImpl.
//
// Key invariant: the output MapConfig's Z bounds must be shifted by
// map_axes_offset (house scenario: height_offset = 150 cm).
// Without the offset shift, scans write outside the output map and
// the score collapses — this is the ex2 regression that must not recur.

#include <Simulator/SimulationRunFactoryImpl.h>
#include <Simulator/SimulationTypes.h>

#include <Common/IDroneMovement.h>
#include <Common/IGPS.h>
#include <Common/ILidar.h>
#include <Common/IMap3D.h>
#include <Common/IMappingAlgorithm.h>
#include <Common/IMissionControl.h>
#include <Common/IMutableMap3D.h>
#include <Common/MappingAlgorithmFactory.h>
#include <Common/MissionControlFactory.h>

#include <TinyNPY.h>

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <stdexcept>

namespace fs = std::filesystem;

// ---------------------------------------------------------------------------
// Minimal fakes
// ---------------------------------------------------------------------------

struct FakeAlgorithm final : public common::IMappingAlgorithm {
    using IMappingAlgorithm::IMappingAlgorithm;

    [[nodiscard]] common::types::MappingStepCommand nextStep(
        const common::types::DroneState&,
        const common::types::LidarScanResult*) override {
        return common::types::MappingStepCommand{
            .movement         = std::nullopt,
            .scan_orientation = std::nullopt,
            .status           = common::types::AlgorithmStatus::Finished,
        };
    }
};

struct FakeMissionControl final : public common::IMissionControl {
    explicit FakeMissionControl(common::MissionControlDependencies) {}

    [[nodiscard]] common::types::MissionRunResult runMission() override {
        return common::types::MissionRunResult{common::types::MissionRunStatus::Completed, 0, {}};
    }
};

// ---------------------------------------------------------------------------
// Factory helpers
// ---------------------------------------------------------------------------

[[nodiscard]] static common::MappingAlgorithmFactory makeAlgorithmFactory() {
    return [](common::MappingAlgorithmDependencies deps) -> std::unique_ptr<common::IMappingAlgorithm> {
        return std::make_unique<FakeAlgorithm>(std::move(deps));
    };
}

[[nodiscard]] static common::MissionControlFactory makeMissionControlFactory() {
    return [](common::MissionControlDependencies deps) -> std::unique_ptr<common::IMissionControl> {
        return std::make_unique<FakeMissionControl>(deps);
    };
}

// ---------------------------------------------------------------------------
// Helpers to create a tiny .npy hidden map on disk
// ---------------------------------------------------------------------------

[[nodiscard]] static fs::path writeTinyHiddenMap(const fs::path& dir,
                                                   std::size_t nx, std::size_t ny, std::size_t nz) {
    auto arr = std::make_shared<NpyArray>(NpyArray::shape_t{nx, ny, nz},
                                          sizeof(std::int8_t),
                                          NpyArray::GetTypeChar(typeid(std::int8_t)));
    arr->Allocate();
    std::fill_n(arr->Data<std::int8_t>(), arr->NumValue(), std::int8_t{0});

    fs::create_directories(dir);
    const fs::path path = dir / "test_hidden.npy";
    const LPCSTR err = arr->SaveNPY(path.string());
    if (err != nullptr) {
        throw std::runtime_error(std::string("writeTinyHiddenMap: ") + err);
    }
    return path;
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

TEST(SimulationRunFactory, CreateReturnsNonNull) {
    const fs::path tmp = fs::temp_directory_path() / "srf_test_basic";
    const fs::path map_path = writeTinyHiddenMap(tmp, 10, 10, 10);

    simulator::types::SimulationConfigData sim{};
    sim.map_filename   = map_path;
    sim.map_resolution = 10.0 * common::cm;

    common::types::MissionConfigData mission{};
    mission.max_steps                        = 1;
    mission.gps_resolution                   = 10.0 * common::cm;
    mission.output_mapping_resolution_factor = 1.0;

    simulator::SimulationRunFactoryImpl factory{makeAlgorithmFactory(), makeMissionControlFactory()};
    auto run = factory.create(sim, mission, {}, {}, tmp / "output.npy");
    EXPECT_NE(run, nullptr);

    std::error_code ec;
    fs::remove_all(tmp, ec);
}

TEST(SimulationRunFactory, OutputMapConfigReflectsOffset_HouseScenarioShape) {
    // Simulate the house scenario: map_axes_offset.height_offset = 150 cm.
    // Mission boundaries are local: min_height=0, max_height=60.
    // Expected output map: min_height=150, max_height=210 (world Z).

    const fs::path tmp = fs::temp_directory_path() / "srf_test_house";
    const fs::path map_path = writeTinyHiddenMap(tmp, 29, 30, 31);

    simulator::types::SimulationConfigData sim{};
    sim.map_filename   = map_path;
    sim.map_resolution = 10.0 * common::cm;
    sim.map_offset     = common::Position3D{
        0.0  * common::x_extent[common::cm],
        0.0  * common::y_extent[common::cm],
        150.0 * common::z_extent[common::cm],
    };
    sim.initial_drone_position = common::Position3D{
        50.0 * common::x_extent[common::cm],
        50.0 * common::y_extent[common::cm],
        10.0 * common::z_extent[common::cm],
    };

    common::types::MissionConfigData mission{};
    mission.max_steps                        = 1;
    mission.gps_resolution                   = 10.0 * common::cm;
    mission.output_mapping_resolution_factor = 1.0;
    mission.mission_bounds.min_x      =   0.0 * common::x_extent[common::cm];
    mission.mission_bounds.max_x      = 280.0 * common::x_extent[common::cm];
    mission.mission_bounds.min_y      =   0.0 * common::y_extent[common::cm];
    mission.mission_bounds.max_y      = 290.0 * common::y_extent[common::cm];
    mission.mission_bounds.min_height =   0.0 * common::z_extent[common::cm];
    mission.mission_bounds.max_height =  60.0 * common::z_extent[common::cm];

    simulator::SimulationRunFactoryImpl factory{makeAlgorithmFactory(), makeMissionControlFactory()};
    auto run = factory.create(sim, mission, {}, {}, tmp / "output.npy");
    ASSERT_NE(run, nullptr);

    simulator::types::SimulationResult result = run->run();

    // The output map config must reflect world-space Z bounds (shifted by 150 cm).
    EXPECT_DOUBLE_EQ(result.output_map_config.boundaries.min_height.numerical_value_in(common::cm), 150.0)
        << "min_height should be 0 (local) + 150 (offset) = 150 cm";
    EXPECT_DOUBLE_EQ(result.output_map_config.boundaries.max_height.numerical_value_in(common::cm), 210.0)
        << "max_height should be 60 (local) + 150 (offset) = 210 cm";
    EXPECT_DOUBLE_EQ(result.output_map_config.offset.z.numerical_value_in(common::cm), 150.0)
        << "output map origin Z should equal min_height (150 cm)";

    std::error_code ec;
    fs::remove_all(tmp, ec);
}

TEST(SimulationRunFactory, MissingMapFileStillReturnsNonNullRun) {
    simulator::types::SimulationConfigData sim{};
    sim.map_filename   = "/no/such/map.npy";
    sim.map_resolution = 10.0 * common::cm;

    simulator::SimulationRunFactoryImpl factory{makeAlgorithmFactory(), makeMissionControlFactory()};
    auto run = factory.create(sim, {}, {}, {}, "/tmp/srf_missing.npy");
    EXPECT_NE(run, nullptr);

    simulator::types::SimulationResult result = run->run();
    ASSERT_EQ(result.mission_results.size(), 1u);
    EXPECT_EQ(result.mission_results.front().status, common::types::MissionRunStatus::Error);
}
