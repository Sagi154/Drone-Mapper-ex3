// test_simulation_run_impl.cpp
// Per-run error-log naming: SimulationRunImpl mirrors ErrorRefs into
// <plugin>_run_NNNN_error.log derived from the output map path.

#include <Simulator/io/SimulatorPaths.h>
#include <Simulator/SimulationRunImpl.h>
#include <Simulator/SimulationTypes.h>

#include <Common/IDroneMovement.h>
#include <Common/IGPS.h>
#include <Common/ILidar.h>
#include <Common/IMappingAlgorithm.h>
#include <Common/IMissionControl.h>
#include <Common/IMutableMap3D.h>

#include <gtest/gtest.h>

#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using common::types::ErrorRef;
using common::types::MissionRunResult;
using common::types::MissionRunStatus;
using common::types::VoxelOccupancy;

class FakeMap3D final : public common::IMutableMap3D {
public:
    explicit FakeMap3D(bool throw_on_save = false) : throw_on_save_(throw_on_save) {}

    [[nodiscard]] VoxelOccupancy atVoxel(const common::Position3D&) const override {
        return VoxelOccupancy::Unmapped;
    }
    [[nodiscard]] common::types::MapConfig getMapConfig() const override { return {}; }
    [[nodiscard]] bool isInBounds(const common::Position3D&) const override { return true; }
    void set(const common::Position3D&, VoxelOccupancy) override {}
    void save(const std::filesystem::path&) const override {
        if (throw_on_save_) {
            throw std::runtime_error("disk full");
        }
    }

private:
    bool throw_on_save_ = false;
};

struct FakeGps final : public common::IGPS {
    [[nodiscard]] common::Position3D position() const override { return {}; }
    [[nodiscard]] common::Orientation heading() const override { return {}; }
};

struct FakeMovement final : public common::IDroneMovement {
    common::types::MovementResult rotate(common::types::RotationDirection,
                                         common::HorizontalAngle) override {
        return {};
    }
    common::types::MovementResult advance(common::PhysicalLength) override { return {}; }
    common::types::MovementResult elevate(common::PhysicalLength) override { return {}; }
};

struct FakeLidar final : public common::ILidar {
    [[nodiscard]] common::types::LidarScanResult scan(common::Orientation) const override {
        return {};
    }
    [[nodiscard]] common::types::LidarConfigData config() const override { return {}; }
};

struct FakeAlgorithm final : public common::IMappingAlgorithm {
    using IMappingAlgorithm::IMappingAlgorithm;

    [[nodiscard]] common::types::MappingStepCommand nextStep(
        const common::types::DroneState&,
        const common::types::LidarScanResult*) override {
        return common::types::MappingStepCommand{
            .status = common::types::AlgorithmStatus::Finished,
        };
    }
};

struct FakeMissionControl final : public common::IMissionControl {
    explicit FakeMissionControl(MissionRunResult result) : result_(std::move(result)) {}

    [[nodiscard]] MissionRunResult runMission() override { return result_; }

private:
    MissionRunResult result_;
};

struct ThrowingMissionControl final : public common::IMissionControl {
    [[nodiscard]] MissionRunResult runMission() override {
        throw std::runtime_error("wall collision");
    }
};

[[nodiscard]] std::vector<std::string> readAllLines(const std::filesystem::path& path) {
    std::ifstream input(path);
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(input, line)) {
        lines.push_back(line);
    }
    return lines;
}

struct RunBundle {
    std::unique_ptr<simulator::SimulationRunImpl> run;
    std::filesystem::path map_path;
    std::filesystem::path log_path;
};

[[nodiscard]] RunBundle makeRun(std::unique_ptr<common::IMissionControl> mission_control,
                                std::vector<ErrorRef> startup_errors = {},
                                bool throw_on_save = false) {
    const std::filesystem::path dir =
        std::filesystem::temp_directory_path() / "sim_run_error_log_tests";
    std::filesystem::create_directories(dir);
    const auto map_path = dir / "plugin.so_run_0007_output_map.npy";
    const auto log_path = simulator::io::errorLogPathFromOutputMap(map_path);
    std::error_code ec;
    std::filesystem::remove(log_path, ec);

    auto hidden = std::make_unique<FakeMap3D>();
    auto output = std::make_unique<FakeMap3D>(throw_on_save);

    common::types::MissionConfigData mission{};
    common::types::LidarConfigData lidar{};
    common::types::DroneConfigData drone{};
    auto algorithm = std::make_unique<FakeAlgorithm>(common::MappingAlgorithmDependencies{
        mission, lidar, drone, *output});

    RunBundle bundle;
    bundle.map_path = map_path;
    bundle.log_path = log_path;
    bundle.run = std::make_unique<simulator::SimulationRunImpl>(
        std::move(hidden),
        std::move(output),
        std::make_unique<FakeGps>(),
        std::make_unique<FakeMovement>(),
        std::make_unique<FakeLidar>(),
        std::move(algorithm),
        std::move(mission_control),
        simulator::types::SimulationConfigData{},
        mission,
        map_path,
        std::move(startup_errors));
    return bundle;
}

} // namespace

TEST(SimulationRunImpl, WritesErrorLogOnStartupErrors) {
    auto bundle = makeRun(std::make_unique<FakeMissionControl>(MissionRunResult{}),
                          {ErrorRef{"SPAWN_NOT_PASSABLE", "blocked spawn"}});

    const auto result = bundle.run->run();
    EXPECT_EQ(result.mission_score, -1.0);
    ASSERT_TRUE(std::filesystem::exists(bundle.log_path));
    const auto lines = readAllLines(bundle.log_path);
    ASSERT_EQ(lines.size(), 1U);
    EXPECT_NE(lines.front().find("SPAWN_NOT_PASSABLE"), std::string::npos);
    EXPECT_NE(lines.front().find("blocked spawn"), std::string::npos);

    std::error_code ec;
    std::filesystem::remove(bundle.log_path, ec);
}

TEST(SimulationRunImpl, WritesErrorLogWhenMissionReturnsErrors) {
    MissionRunResult mission_result{MissionRunStatus::Error, 3,
                                    {ErrorRef{"MISSION_FAILED", "stuck"}}};
    auto bundle = makeRun(std::make_unique<FakeMissionControl>(std::move(mission_result)));

    const auto result = bundle.run->run();
    EXPECT_EQ(result.mission_score, -1.0);
    const auto lines = readAllLines(bundle.log_path);
    ASSERT_EQ(lines.size(), 1U);
    EXPECT_NE(lines.front().find("MISSION_FAILED"), std::string::npos);
    EXPECT_NE(lines.front().find("stuck"), std::string::npos);

    std::error_code ec;
    std::filesystem::remove(bundle.log_path, ec);
}

TEST(SimulationRunImpl, WritesErrorLogWhenRunMissionThrows) {
    auto bundle = makeRun(std::make_unique<ThrowingMissionControl>());

    const auto result = bundle.run->run();
    EXPECT_EQ(result.mission_score, -1.0);
    const auto lines = readAllLines(bundle.log_path);
    ASSERT_EQ(lines.size(), 1U);
    EXPECT_NE(lines.front().find("MISSION_EXCEPTION"), std::string::npos);
    EXPECT_NE(lines.front().find("wall collision"), std::string::npos);

    std::error_code ec;
    std::filesystem::remove(bundle.log_path, ec);
}

TEST(SimulationRunImpl, WritesErrorLogWhenMapSaveFails) {
    auto bundle =
        makeRun(std::make_unique<FakeMissionControl>(
                    MissionRunResult{MissionRunStatus::Completed, 1, {}}),
                {}, true);

    const auto result = bundle.run->run();
    EXPECT_EQ(result.mission_score, -1.0);
    const auto lines = readAllLines(bundle.log_path);
    ASSERT_EQ(lines.size(), 1U);
    EXPECT_NE(lines.front().find("MAP_SAVE_FAILED"), std::string::npos);

    std::error_code ec;
    std::filesystem::remove(bundle.log_path, ec);
}

TEST(SimulationRunImpl, CreatesEmptyErrorLogWhenRunHasNoErrors) {
    auto bundle = makeRun(std::make_unique<FakeMissionControl>(
        MissionRunResult{MissionRunStatus::Completed, 0, {}}));

    (void)bundle.run->run();
    EXPECT_TRUE(std::filesystem::exists(bundle.log_path));
    EXPECT_TRUE(readAllLines(bundle.log_path).empty());

    std::error_code ec;
    std::filesystem::remove(bundle.log_path, ec);
}
