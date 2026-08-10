#include <MissionControl/DroneControlImpl.h>

#include <gtest/gtest.h>

#include <filesystem>
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
        return occupancy_;
    }

    [[nodiscard]] common::types::MapConfig getMapConfig() const override { return config_; }

    [[nodiscard]] bool isInBounds(const Position3D& /*pos*/) const override { return in_bounds_; }

    void set(const Position3D& /*pos*/, common::types::VoxelOccupancy value) override {
        occupancy_ = value;
        ++set_count_;
    }

    void save(const std::filesystem::path& /*path*/) const override {}

    void setInBounds(bool value) { in_bounds_ = value; }

    int set_count_ = 0;

private:
    common::types::MapConfig config_;
    common::types::VoxelOccupancy occupancy_ = common::types::VoxelOccupancy::Unmapped;
    bool in_bounds_ = true;
};

class FakeGPS final : public common::IGPS {
public:
    [[nodiscard]] Position3D position() const override { return position_; }
    [[nodiscard]] Orientation heading() const override { return heading_; }

    Position3D position_{};
    Orientation heading_{};
};

class FakeLidar final : public common::ILidar {
public:
    explicit FakeLidar(common::types::LidarConfigData config) : config_(std::move(config)) {}

    [[nodiscard]] common::types::LidarScanResult scan(Orientation /*scan_orientation*/) const override {
        ++scan_count_;
        common::types::LidarScanResult result;
        result.push_back(common::types::LidarHit{
            50.0 * cm,
            Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},
        });
        return result;
    }

    [[nodiscard]] common::types::LidarConfigData config() const override { return config_; }

    mutable int scan_count_ = 0;

private:
    common::types::LidarConfigData config_;
};

class FakeMovement final : public common::IDroneMovement {
public:
    common::types::MovementResult rotate(common::types::RotationDirection /*direction*/,
                                         common::HorizontalAngle /*angle*/) override {
        return {true, {}};
    }

    common::types::MovementResult advance(PhysicalLength /*distance*/) override {
        if (throw_on_advance_) {
            throw std::runtime_error("wall collision");
        }
        if (!advance_ok_) {
            return {false, "Movement failed."};
        }
        return {true, {}};
    }

    common::types::MovementResult elevate(PhysicalLength /*distance*/) override {
        return {true, {}};
    }

    bool throw_on_advance_ = false;
    bool advance_ok_ = true;
};

class ScriptedAlgorithm final : public common::IMappingAlgorithm {
public:
    ScriptedAlgorithm(common::MappingAlgorithmDependencies deps,
                      std::vector<common::types::MappingStepCommand> script)
        : IMappingAlgorithm(std::move(deps)), script_(std::move(script)) {}

    [[nodiscard]] common::types::MappingStepCommand nextStep(
        const common::types::DroneState& /*state*/,
        const common::types::LidarScanResult* latest_scan) override {
        latest_scan_was_null_.push_back(latest_scan == nullptr);
        if (call_index_ >= script_.size()) {
            return common::types::MappingStepCommand{
                .status = common::types::AlgorithmStatus::Finished,
            };
        }
        return script_[call_index_++];
    }

    std::size_t call_index_ = 0;
    std::vector<common::types::MappingStepCommand> script_;
    std::vector<bool> latest_scan_was_null_;
};

[[nodiscard]] common::types::MapConfig makeMapConfig() {
    common::types::MapConfig config{};
    config.resolution = 10.0 * cm;
    config.boundaries = {
        0.0 * x_extent[cm],
        100.0 * x_extent[cm],
        0.0 * y_extent[cm],
        100.0 * y_extent[cm],
        0.0 * z_extent[cm],
        100.0 * z_extent[cm],
    };
    return config;
}

[[nodiscard]] common::types::MissionConfigData defaultMission() {
    return common::types::MissionConfigData{100, 10.0 * cm, 1.0, {}};
}

[[nodiscard]] common::types::DroneConfigData defaultDrone() {
    return common::types::DroneConfigData{
        5.0 * cm,
        90.0 * horizontal_angle[deg],
        20.0 * cm,
        20.0 * cm,
    };
}

[[nodiscard]] common::types::LidarConfigData defaultLidar() {
    return common::types::LidarConfigData{20.0 * cm, 120.0 * cm, 2.5 * cm, 3};
}

struct Fixture {
    FakeMap3D stand_in_map{makeMapConfig()};
    FakeMap3D output_map{makeMapConfig()};
    FakeGPS gps{};
    FakeLidar lidar{defaultLidar()};
    FakeMovement movement{};
};

} // namespace

TEST(DroneControl, FirstStepPassesNullScanToAlgorithm) {
    Fixture fixture;
    const auto mission = defaultMission();
    const auto lidar_cfg = defaultLidar();
    const auto drone_cfg = defaultDrone();
    ScriptedAlgorithm algorithm{
        common::MappingAlgorithmDependencies{mission, lidar_cfg, drone_cfg, fixture.stand_in_map},
        {common::types::MappingStepCommand{
            .status = common::types::AlgorithmStatus::Working,
        }},
    };

    MissionControl_207190406_209543255::DroneControlImpl control{
        defaultDrone(),
        defaultMission(),
        defaultLidar(),
        fixture.lidar,
        fixture.gps,
        fixture.movement,
        fixture.output_map,
        algorithm,
    };

    const auto result = control.step();
    EXPECT_EQ(result.status, common::types::DroneStepStatus::Continue);
    ASSERT_FALSE(algorithm.latest_scan_was_null_.empty());
    EXPECT_TRUE(algorithm.latest_scan_was_null_.front());
}

TEST(DroneControl, ReturnsCompletedWhenAlgorithmFinishes) {
    Fixture fixture;
    const auto mission = defaultMission();
    const auto lidar_cfg = defaultLidar();
    const auto drone_cfg = defaultDrone();
    ScriptedAlgorithm algorithm{
        common::MappingAlgorithmDependencies{mission, lidar_cfg, drone_cfg, fixture.stand_in_map},
        {common::types::MappingStepCommand{
            .status = common::types::AlgorithmStatus::Finished,
        }},
    };

    MissionControl_207190406_209543255::DroneControlImpl control{
        defaultDrone(),
        defaultMission(),
        defaultLidar(),
        fixture.lidar,
        fixture.gps,
        fixture.movement,
        fixture.output_map,
        algorithm,
    };

    EXPECT_EQ(control.step().status, common::types::DroneStepStatus::Completed);
}

TEST(DroneControl, ReturnsErrorWhenMovementExceedsDroneLimits) {
    Fixture fixture;
    const auto mission = defaultMission();
    const auto lidar_cfg = defaultLidar();
    const auto drone_cfg = defaultDrone();
    ScriptedAlgorithm algorithm{
        common::MappingAlgorithmDependencies{mission, lidar_cfg, drone_cfg, fixture.stand_in_map},
        {common::types::MappingStepCommand{
            .movement =
                common::types::MovementCommand{
                    .type = common::types::MovementCommandType::Advance,
                    .distance = 500.0 * cm,
                },
            .status = common::types::AlgorithmStatus::Working,
        }},
    };

    MissionControl_207190406_209543255::DroneControlImpl control{
        defaultDrone(),
        defaultMission(),
        defaultLidar(),
        fixture.lidar,
        fixture.gps,
        fixture.movement,
        fixture.output_map,
        algorithm,
    };

    const auto result = control.step();
    EXPECT_EQ(result.status, common::types::DroneStepStatus::Error);
    EXPECT_NE(result.message.find("limits"), std::string::npos);
}

TEST(DroneControl, CollisionExceptionEscapesStep) {
    Fixture fixture;
    fixture.movement.throw_on_advance_ = true;

    const auto mission = defaultMission();
    const auto lidar_cfg = defaultLidar();
    const auto drone_cfg = defaultDrone();
    ScriptedAlgorithm algorithm{
        common::MappingAlgorithmDependencies{mission, lidar_cfg, drone_cfg, fixture.stand_in_map},
        {common::types::MappingStepCommand{
            .movement =
                common::types::MovementCommand{
                    .type = common::types::MovementCommandType::Advance,
                    .distance = 10.0 * cm,
                },
            .status = common::types::AlgorithmStatus::Working,
        }},
    };

    MissionControl_207190406_209543255::DroneControlImpl control{
        defaultDrone(),
        defaultMission(),
        defaultLidar(),
        fixture.lidar,
        fixture.gps,
        fixture.movement,
        fixture.output_map,
        algorithm,
    };

    EXPECT_THROW(
        { (void)control.step(); },
        std::runtime_error);
}

TEST(DroneControl, ExecutesScanThenContinues) {
    Fixture fixture;
    const auto mission = defaultMission();
    const auto lidar_cfg = defaultLidar();
    const auto drone_cfg = defaultDrone();
    ScriptedAlgorithm algorithm{
        common::MappingAlgorithmDependencies{mission, lidar_cfg, drone_cfg, fixture.stand_in_map},
        {
            common::types::MappingStepCommand{
                .scan_orientation =
                    Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},
                .status = common::types::AlgorithmStatus::Working,
            },
            common::types::MappingStepCommand{
                .status = common::types::AlgorithmStatus::Working,
            },
        },
    };

    MissionControl_207190406_209543255::DroneControlImpl control{
        defaultDrone(),
        defaultMission(),
        defaultLidar(),
        fixture.lidar,
        fixture.gps,
        fixture.movement,
        fixture.output_map,
        algorithm,
    };

    EXPECT_EQ(control.step().status, common::types::DroneStepStatus::Continue);
    EXPECT_EQ(fixture.lidar.scan_count_, 1);
    EXPECT_GE(fixture.output_map.set_count_, 1);
}
