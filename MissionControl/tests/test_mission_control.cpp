#include <MissionControl/MissionControlImpl.h>
#include <MissionControl/IDroneControl.h>
#include <MissionControl/MissionRunLoop.h>

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using common::Orientation;
using common::PhysicalLength;
using common::Position3D;
using common::altitude_angle;
using common::cm;
using common::deg;
using common::horizontal_angle;
using common::x_extent;
using common::y_extent;
using common::z_extent;

class FakeMap3D final : public common::IMutableMap3D {
public:
    explicit FakeMap3D(common::types::MapConfig config) : config_(std::move(config)) {}

    [[nodiscard]] common::types::VoxelOccupancy atVoxel(const Position3D& /*pos*/) const override {
        return common::types::VoxelOccupancy::Unmapped;
    }

    [[nodiscard]] common::types::MapConfig getMapConfig() const override { return config_; }

    [[nodiscard]] bool isInBounds(const Position3D& /*pos*/) const override { return true; }

    void set(const Position3D& /*pos*/, common::types::VoxelOccupancy /*value*/) override {}

    void save(const std::filesystem::path& path) const override {
        if (path.has_parent_path()) {
            std::filesystem::create_directories(path.parent_path());
        }
        std::ofstream out(path);
        out << "saved\n";
        ++save_count_;
    }

    mutable int save_count_ = 0;

private:
    common::types::MapConfig config_;
};

class FakeGPS final : public common::IGPS {
public:
    [[nodiscard]] Position3D position() const override { return {}; }
    [[nodiscard]] Orientation heading() const override { return {}; }
};

class FakeLidar final : public common::ILidar {
public:
    explicit FakeLidar(common::types::LidarConfigData config) : config_(std::move(config)) {}

    [[nodiscard]] common::types::LidarScanResult scan(Orientation /*scan_orientation*/) const override {
        return {};
    }

    [[nodiscard]] common::types::LidarConfigData config() const override { return config_; }

private:
    common::types::LidarConfigData config_;
};

class FakeMovement final : public common::IDroneMovement {
public:
    common::types::MovementResult rotate(common::types::RotationDirection,
                                         common::HorizontalAngle) override {
        return {true, {}};
    }
    common::types::MovementResult advance(PhysicalLength) override { return {true, {}}; }
    common::types::MovementResult elevate(PhysicalLength) override { return {true, {}}; }
};

class ScriptedAlgorithm final : public common::IMappingAlgorithm {
public:
    ScriptedAlgorithm(common::MappingAlgorithmDependencies deps,
                      std::vector<common::types::MappingStepCommand> script)
        : IMappingAlgorithm(std::move(deps)), script_(std::move(script)) {}

    [[nodiscard]] common::types::MappingStepCommand nextStep(
        const common::types::DroneState&,
        const common::types::LidarScanResult*) override {
        if (call_index_ >= script_.size()) {
            return {.status = common::types::AlgorithmStatus::Finished};
        }
        return script_[call_index_++];
    }

    std::size_t call_index_ = 0;
    std::vector<common::types::MappingStepCommand> script_;
};

[[nodiscard]] common::types::MapConfig makeMapConfig() {
    common::types::MapConfig config{};
    config.resolution = 10.0 * cm;
    config.boundaries = {
        0.0 * x_extent[cm], 100.0 * x_extent[cm], 0.0 * y_extent[cm], 100.0 * y_extent[cm],
        0.0 * z_extent[cm], 100.0 * z_extent[cm],
    };
    return config;
}

[[nodiscard]] common::types::MissionConfigData makeMission(std::size_t max_steps) {
    return {max_steps, 10.0 * cm, 1.0, {}};
}

[[nodiscard]] common::types::DroneConfigData defaultDrone() {
    return {5.0 * cm, 90.0 * horizontal_angle[deg], 20.0 * cm, 20.0 * cm};
}

[[nodiscard]] common::types::LidarConfigData defaultLidar() {
    return {20.0 * cm, 120.0 * cm, 2.5 * cm, 3};
}

} // namespace

class FakeDroneControl final : public mission_control::IDroneControl {
public:
    explicit FakeDroneControl(std::vector<common::types::DroneStepResult> script)
        : script_(std::move(script)) {}

    [[nodiscard]] common::types::DroneStepResult step() override {
        ++step_calls_;
        if (call_index_ >= script_.size()) {
            return {common::types::DroneStepStatus::Completed, {}};
        }
        return script_[call_index_++];
    }

    [[nodiscard]] common::types::DroneState state() const override { return {}; }

    std::size_t step_calls_ = 0;

private:
    std::vector<common::types::DroneStepResult> script_;
    std::size_t call_index_ = 0;
};

TEST(MissionControl, CompletesWhenAlgorithmFinishes) {
    FakeMap3D stand_in{makeMapConfig()};
    FakeMap3D output{makeMapConfig()};
    FakeGPS gps;
    FakeLidar lidar{defaultLidar()};
    FakeMovement movement;

    const auto mission = makeMission(10);
    const auto drone = defaultDrone();
    const auto lidar_cfg = defaultLidar();
    ScriptedAlgorithm algorithm{
        {mission, lidar_cfg, drone, stand_in},
        {{.status = common::types::AlgorithmStatus::Finished}},
    };

    const auto output_file =
        std::filesystem::temp_directory_path() / "mc_completed_output.npy";
    mission_control_207190406_209543255::MissionControlImpl_207190406_209543255 control{
        common::MissionControlDependencies{
            mission, drone, lidar, gps, movement, output, algorithm, output_file, false},
    };

    const auto result = control.runMission();
    EXPECT_EQ(result.status, common::types::MissionRunStatus::Completed);
    EXPECT_EQ(result.steps, 1U);
    // Map save is owned by SimulationRunImpl, not MissionControlImpl.
    EXPECT_EQ(output.save_count_, 0);
    EXPECT_FALSE(std::filesystem::exists(std::filesystem::path(output_file.string() + ".verbose.txt")));

    std::error_code ec;
    std::filesystem::remove(output_file, ec);
}

TEST(MissionControl, HitsMaxSteps) {
    FakeMap3D stand_in{makeMapConfig()};
    FakeMap3D output{makeMapConfig()};
    FakeGPS gps;
    FakeLidar lidar{defaultLidar()};
    FakeMovement movement;

    const auto mission = makeMission(2);
    const auto drone = defaultDrone();
    const auto lidar_cfg = defaultLidar();
    ScriptedAlgorithm algorithm{
        {mission, lidar_cfg, drone, stand_in},
        {
            {.status = common::types::AlgorithmStatus::Working},
            {.status = common::types::AlgorithmStatus::Working},
            {.status = common::types::AlgorithmStatus::Working},
        },
    };

    const auto output_file = std::filesystem::temp_directory_path() / "mc_max_steps.npy";
    mission_control_207190406_209543255::MissionControlImpl_207190406_209543255 control{
        common::MissionControlDependencies{
            mission, drone, lidar, gps, movement, output, algorithm, output_file, false},
    };

    const auto result = control.runMission();
    EXPECT_EQ(result.status, common::types::MissionRunStatus::MaxSteps);
    EXPECT_EQ(result.steps, 2U);

    std::error_code ec;
    std::filesystem::remove(output_file, ec);
}

TEST(MissionControl, VerboseWritesExtraFile) {
    FakeMap3D stand_in{makeMapConfig()};
    FakeMap3D output{makeMapConfig()};
    FakeGPS gps;
    FakeLidar lidar{defaultLidar()};
    FakeMovement movement;

    const auto mission = makeMission(5);
    const auto drone = defaultDrone();
    const auto lidar_cfg = defaultLidar();
    ScriptedAlgorithm algorithm{
        {mission, lidar_cfg, drone, stand_in},
        {{.status = common::types::AlgorithmStatus::Finished}},
    };

    const auto output_file = std::filesystem::temp_directory_path() / "mc_verbose_output.npy";
    const auto verbose_file = std::filesystem::path(output_file.string() + ".verbose.txt");
    mission_control_207190406_209543255::MissionControlImpl_207190406_209543255 control{
        common::MissionControlDependencies{
            mission, drone, lidar, gps, movement, output, algorithm, output_file, true},
    };

    (void)control.runMission();
    EXPECT_TRUE(std::filesystem::exists(verbose_file));

    std::error_code ec;
    std::filesystem::remove(output_file, ec);
    std::filesystem::remove(verbose_file, ec);
}

TEST(MissionControl, VerboseOffWritesNoExtraFile) {
    FakeMap3D stand_in{makeMapConfig()};
    FakeMap3D output{makeMapConfig()};
    FakeGPS gps;
    FakeLidar lidar{defaultLidar()};
    FakeMovement movement;

    const auto mission = makeMission(5);
    const auto drone = defaultDrone();
    const auto lidar_cfg = defaultLidar();
    ScriptedAlgorithm algorithm{
        {mission, lidar_cfg, drone, stand_in},
        {{.status = common::types::AlgorithmStatus::Finished}},
    };

    const auto output_file = std::filesystem::temp_directory_path() / "mc_quiet_output.npy";
    const auto verbose_file = std::filesystem::path(output_file.string() + ".verbose.txt");
    std::error_code ec;
    std::filesystem::remove(verbose_file, ec);

    mission_control_207190406_209543255::MissionControlImpl_207190406_209543255 control{
        common::MissionControlDependencies{
            mission, drone, lidar, gps, movement, output, algorithm, output_file, false},
    };

    (void)control.runMission();
    EXPECT_FALSE(std::filesystem::exists(verbose_file));

    std::filesystem::remove(output_file, ec);
}

TEST(MissionControl, OversizeAdvanceCompletesWithoutErrors) {
    FakeMap3D stand_in{makeMapConfig()};
    FakeMap3D output{makeMapConfig()};
    FakeGPS gps;
    FakeLidar lidar{defaultLidar()};
    FakeMovement movement;

    const auto mission = makeMission(10);
    const auto drone = defaultDrone();
    const auto lidar_cfg = defaultLidar();
    ScriptedAlgorithm algorithm{
        {mission, lidar_cfg, drone, stand_in},
        {{
            .movement =
                common::types::MovementCommand{
                    .type = common::types::MovementCommandType::Advance,
                    .distance = 50.0 * cm,
                },
            .status = common::types::AlgorithmStatus::Working,
        }},
    };

    const auto output_file =
        std::filesystem::temp_directory_path() / "mc_oversize_split_output.npy";
    mission_control_207190406_209543255::MissionControlImpl_207190406_209543255 control{
        common::MissionControlDependencies{
            mission, drone, lidar, gps, movement, output, algorithm, output_file, false},
    };

    const auto result = control.runMission();
    EXPECT_EQ(result.status, common::types::MissionRunStatus::Completed);
    EXPECT_TRUE(result.errors.empty());

    std::error_code ec;
    std::filesystem::remove(output_file, ec);
}

TEST(MissionControl, InvalidCommandsThrowAfterRetries) {
    FakeMap3D stand_in{makeMapConfig()};
    FakeMap3D output{makeMapConfig()};
    FakeGPS gps;
    FakeLidar lidar{defaultLidar()};
    FakeMovement movement;

    const auto mission = makeMission(5);
    const auto drone = defaultDrone();
    const auto lidar_cfg = defaultLidar();
    const auto bad = common::types::MovementCommand{
        .type = static_cast<common::types::MovementCommandType>(99),
    };
    ScriptedAlgorithm algorithm{
        {mission, lidar_cfg, drone, stand_in},
        {
            {.movement = bad, .status = common::types::AlgorithmStatus::Working},
            {.movement = bad, .status = common::types::AlgorithmStatus::Working},
            {.movement = bad, .status = common::types::AlgorithmStatus::Working},
        },
    };

    const auto output_file =
        std::filesystem::temp_directory_path() / "mc_invalid_command_output.npy";
    mission_control_207190406_209543255::MissionControlImpl_207190406_209543255 control{
        common::MissionControlDependencies{
            mission, drone, lidar, gps, movement, output, algorithm, output_file, false},
    };

    EXPECT_THROW(
        { (void)control.runMission(); },
        std::runtime_error);

    std::error_code ec;
    std::filesystem::remove(output_file, ec);
}

TEST(MissionControl, StepErrorIsLoggedAndLoopContinuesUntilFinished) {
    FakeDroneControl drone({
        {common::types::DroneStepStatus::Error, "Movement command exceeds drone limits."},
        {common::types::DroneStepStatus::Completed, {}},
    });
    const auto result = mission_control_207190406_209543255::runMissionSteps(
        drone, 5, {}, false);
    EXPECT_EQ(result.status, common::types::MissionRunStatus::Completed);
    EXPECT_EQ(result.steps, 2U);
    EXPECT_EQ(drone.step_calls_, 2U);
    ASSERT_EQ(result.errors.size(), 1U);
    EXPECT_EQ(result.errors.front().code, "DRONE_STEP_FAILED");
    EXPECT_EQ(result.errors.front().message, "Drone step failed.");
}

TEST(MissionControl, PersistentStepErrorRunsToMaxSteps) {
    FakeDroneControl drone({
        {common::types::DroneStepStatus::Error, "bad"},
        {common::types::DroneStepStatus::Error, "bad"},
        {common::types::DroneStepStatus::Error, "bad"},
        {common::types::DroneStepStatus::Error, "bad"},
    });
    const auto result = mission_control_207190406_209543255::runMissionSteps(
        drone, 3, {}, false);
    EXPECT_EQ(result.status, common::types::MissionRunStatus::MaxSteps);
    EXPECT_EQ(result.steps, 3U);
    EXPECT_EQ(result.errors.size(), 3U);
    EXPECT_EQ(result.errors[0].code, "DRONE_STEP_FAILED");
    EXPECT_EQ(result.errors[1].code, "DRONE_STEP_FAILED");
    EXPECT_EQ(result.errors[2].code, "DRONE_STEP_FAILED");
}
