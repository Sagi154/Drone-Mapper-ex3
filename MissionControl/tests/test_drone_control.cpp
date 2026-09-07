#include <MissionControl/DroneControlImpl.h>

#include <Common/IDroneMovement.h>
#include <Common/IGPS.h>
#include <Common/ILidar.h>
#include <Common/IMappingAlgorithm.h>
#include <Common/IMutableMap3D.h>

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

    [[nodiscard]] bool isInBounds(const Position3D& pos) const override {
        const auto& b = config_.boundaries;
        return pos.x >= b.min_x && pos.x <= b.max_x && pos.y >= b.min_y &&
               pos.y <= b.max_y && pos.z >= b.min_height && pos.z <= b.max_height;
    }

    void set(const Position3D& /*pos*/, common::types::VoxelOccupancy value) override {
        occupancy_ = value;
        ++set_count_;
    }

    void save(const std::filesystem::path& /*path*/) const override {}

    int set_count_ = 0;

private:
    common::types::MapConfig config_;
    common::types::VoxelOccupancy occupancy_ = common::types::VoxelOccupancy::Unmapped;
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
                                         common::HorizontalAngle angle) override {
        ++rotate_count_;
        last_angle_ = angle;
        return {true, {}};
    }

    common::types::MovementResult advance(PhysicalLength distance) override {
        ++advance_count_;
        last_advance_ = distance;
        if (throw_on_advance_) {
            throw std::runtime_error(advance_throw_message_);
        }
        if (!advance_ok_) {
            return {false, advance_fail_message_};
        }
        return {true, {}};
    }

    common::types::MovementResult elevate(PhysicalLength distance) override {
        last_elevate_ = distance;
        return {true, {}};
    }

    bool throw_on_advance_ = false;
    bool advance_ok_ = true;
    int advance_count_ = 0;
    int rotate_count_ = 0;
    PhysicalLength last_advance_{};
    PhysicalLength last_elevate_{};
    common::HorizontalAngle last_angle_{};
    std::string advance_fail_message_ = "Movement failed.";
    std::string advance_throw_message_ =
        "advance: destination blocked by obstacle or map boundary";
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

    mission_control_207190406_209543255::DroneControlImpl control{
        defaultDrone(),
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

    mission_control_207190406_209543255::DroneControlImpl control{
        defaultDrone(),
        defaultLidar(),
        fixture.lidar,
        fixture.gps,
        fixture.movement,
        fixture.output_map,
        algorithm,
    };

    EXPECT_EQ(control.step().status, common::types::DroneStepStatus::Completed);
}

TEST(DroneControl, OversizeAdvanceNoLongerErrorsOnFirstStep) {
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

    mission_control_207190406_209543255::DroneControlImpl control{
        defaultDrone(),
        defaultLidar(),
        fixture.lidar,
        fixture.gps,
        fixture.movement,
        fixture.output_map,
        algorithm,
    };

    const auto result = control.step();
    EXPECT_EQ(result.status, common::types::DroneStepStatus::Continue);
    EXPECT_EQ(fixture.movement.advance_count_, 1);
    EXPECT_DOUBLE_EQ(fixture.movement.last_advance_.numerical_value_in(cm), 20.0);
}

TEST(DroneControl, OversizeAdvanceSplitsAcrossStepsWithoutExtraNextStep) {
    Fixture fixture;
    ScriptedAlgorithm algorithm{
        common::MappingAlgorithmDependencies{
            defaultMission(), defaultLidar(), defaultDrone(), fixture.stand_in_map},
        {common::types::MappingStepCommand{
            .movement =
                common::types::MovementCommand{
                    .type = common::types::MovementCommandType::Advance,
                    .distance = 50.0 * cm,
                },
            .scan_orientation =
                Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},
            .status = common::types::AlgorithmStatus::Working,
        }},
    };
    mission_control_207190406_209543255::DroneControlImpl control{
        defaultDrone(), defaultLidar(), fixture.lidar, fixture.gps,
        fixture.movement, fixture.output_map, algorithm,
    };
    // max_advance is 20 cm → fragments 20, 20, 10
    EXPECT_EQ(control.step().status, common::types::DroneStepStatus::Continue);
    EXPECT_EQ(algorithm.call_index_, 1U);
    EXPECT_EQ(fixture.movement.advance_count_, 1);
    EXPECT_DOUBLE_EQ(fixture.movement.last_advance_.numerical_value_in(cm), 20.0);
    EXPECT_EQ(fixture.lidar.scan_count_, 1);
    EXPECT_EQ(control.state().step_index, 1U);

    EXPECT_EQ(control.step().status, common::types::DroneStepStatus::Continue);
    EXPECT_EQ(algorithm.call_index_, 1U);
    EXPECT_EQ(fixture.movement.advance_count_, 2);
    EXPECT_EQ(fixture.lidar.scan_count_, 1);
    EXPECT_EQ(control.state().step_index, 2U);

    EXPECT_EQ(control.step().status, common::types::DroneStepStatus::Continue);
    EXPECT_EQ(algorithm.call_index_, 1U);
    EXPECT_EQ(fixture.movement.advance_count_, 3);
    EXPECT_DOUBLE_EQ(fixture.movement.last_advance_.numerical_value_in(cm), 10.0);
    EXPECT_EQ(control.state().step_index, 3U);
}

TEST(DroneControl, OversizeRotateSplits) {
    Fixture fixture;
    ScriptedAlgorithm algorithm{
        common::MappingAlgorithmDependencies{
            defaultMission(), defaultLidar(), defaultDrone(), fixture.stand_in_map},
        {common::types::MappingStepCommand{
            .movement =
                common::types::MovementCommand{
                    .type = common::types::MovementCommandType::Rotate,
                    .rotation = common::types::RotationDirection::Left,
                    .angle = 180.0 * horizontal_angle[deg],
                },
            .status = common::types::AlgorithmStatus::Working,
        }},
    };
    // FakeMovement::rotate must ++rotate_count_ and last_angle_
    mission_control_207190406_209543255::DroneControlImpl control{
        defaultDrone(), defaultLidar(), fixture.lidar, fixture.gps,
        fixture.movement, fixture.output_map, algorithm,
    };
    EXPECT_EQ(control.step().status, common::types::DroneStepStatus::Continue);
    EXPECT_EQ(control.step().status, common::types::DroneStepStatus::Continue);
    EXPECT_EQ(fixture.movement.rotate_count_, 2);
    EXPECT_EQ(algorithm.call_index_, 1U);
}

TEST(DroneControl, InvalidCommandRetriesThenExecutesValid) {
    Fixture fixture;
    const auto bad = common::types::MovementCommand{
        .type = static_cast<common::types::MovementCommandType>(99),
    };
    const auto ok = common::types::MovementCommand{
        .type = common::types::MovementCommandType::Advance,
        .distance = 10.0 * cm,
    };
    ScriptedAlgorithm algorithm{
        common::MappingAlgorithmDependencies{
            defaultMission(), defaultLidar(), defaultDrone(), fixture.stand_in_map},
        {
            {.movement = bad, .status = common::types::AlgorithmStatus::Working},
            {.movement = bad, .status = common::types::AlgorithmStatus::Working},
            {.movement = ok, .status = common::types::AlgorithmStatus::Working},
        },
    };
    mission_control_207190406_209543255::DroneControlImpl control{
        defaultDrone(), defaultLidar(), fixture.lidar, fixture.gps,
        fixture.movement, fixture.output_map, algorithm,
    };
    EXPECT_EQ(control.step().status, common::types::DroneStepStatus::Continue);
    EXPECT_EQ(algorithm.call_index_, 3U);
    EXPECT_EQ(fixture.movement.advance_count_, 1);
    EXPECT_EQ(control.state().step_index, 1U);
}

TEST(DroneControl, InvalidCommandThrowsAfterMaxRetries) {
    Fixture fixture;
    const auto bad = common::types::MovementCommand{
        .type = static_cast<common::types::MovementCommandType>(99),
    };
    ScriptedAlgorithm algorithm{
        common::MappingAlgorithmDependencies{
            defaultMission(), defaultLidar(), defaultDrone(), fixture.stand_in_map},
        {
            {.movement = bad, .status = common::types::AlgorithmStatus::Working},
            {.movement = bad, .status = common::types::AlgorithmStatus::Working},
            {.movement = bad, .status = common::types::AlgorithmStatus::Working},
            {.movement = bad, .status = common::types::AlgorithmStatus::Working},
        },
    };
    mission_control_207190406_209543255::DroneControlImpl control{
        defaultDrone(), defaultLidar(), fixture.lidar, fixture.gps,
        fixture.movement, fixture.output_map, algorithm,
    };
    EXPECT_THROW(
        { (void)control.step(); },
        std::runtime_error);
    EXPECT_EQ(algorithm.call_index_, 3U);
    EXPECT_EQ(fixture.movement.advance_count_, 0);
}

TEST(DroneControl, OversizeIsNotInvalidRetry) {
    Fixture fixture;
    ScriptedAlgorithm algorithm{
        common::MappingAlgorithmDependencies{
            defaultMission(), defaultLidar(), defaultDrone(), fixture.stand_in_map},
        {common::types::MappingStepCommand{
            .movement =
                common::types::MovementCommand{
                    .type = common::types::MovementCommandType::Advance,
                    .distance = 500.0 * cm,
                },
            .status = common::types::AlgorithmStatus::Working,
        }},
    };
    mission_control_207190406_209543255::DroneControlImpl control{
        defaultDrone(), defaultLidar(), fixture.lidar, fixture.gps,
        fixture.movement, fixture.output_map, algorithm,
    };
    const auto result = control.step();
    EXPECT_EQ(result.status, common::types::DroneStepStatus::Continue);
    EXPECT_EQ(algorithm.call_index_, 1U);
    EXPECT_EQ(fixture.movement.advance_count_, 1);
}

TEST(DroneControl, CollisionBlockedThrowContinues) {
    Fixture fixture;
    fixture.movement.throw_on_advance_ = true;
    fixture.movement.advance_throw_message_ =
        "advance: destination blocked by obstacle or map boundary";

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

    mission_control_207190406_209543255::DroneControlImpl control{
        defaultDrone(),
        defaultLidar(),
        fixture.lidar,
        fixture.gps,
        fixture.movement,
        fixture.output_map,
        algorithm,
    };

    const auto result = control.step();
    EXPECT_EQ(result.status, common::types::DroneStepStatus::Continue);
}

TEST(DroneControl, UnsuccessfulMovementResultIsRecoverableWithoutStringMatch) {
    Fixture fixture;
    fixture.movement.advance_ok_ = false;
    fixture.movement.advance_fail_message_ = "no-keywords";

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

    mission_control_207190406_209543255::DroneControlImpl control{
        defaultDrone(),
        defaultLidar(),
        fixture.lidar,
        fixture.gps,
        fixture.movement,
        fixture.output_map,
        algorithm,
    };

    const auto result = control.step();
    EXPECT_EQ(result.status, common::types::DroneStepStatus::Continue);
}

TEST(DroneControl, MovementExceptionRecoversWithoutMessageMatch) {
    Fixture fixture;
    fixture.movement.throw_on_advance_ = true;
    fixture.movement.advance_throw_message_ = "wall";

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

    mission_control_207190406_209543255::DroneControlImpl control{
        defaultDrone(),
        defaultLidar(),
        fixture.lidar,
        fixture.gps,
        fixture.movement,
        fixture.output_map,
        algorithm,
    };

    const auto result = control.step();
    EXPECT_EQ(result.status, common::types::DroneStepStatus::Continue);
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

    mission_control_207190406_209543255::DroneControlImpl control{
        defaultDrone(),
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

TEST(DroneControl, ExecutesMovementAndScanInOneStep) {
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
                    .distance = 10.0 * cm,
                },
            .scan_orientation =
                Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},
            .status = common::types::AlgorithmStatus::Working,
        }},
    };

    mission_control_207190406_209543255::DroneControlImpl control{
        defaultDrone(),
        defaultLidar(),
        fixture.lidar,
        fixture.gps,
        fixture.movement,
        fixture.output_map,
        algorithm,
    };

    EXPECT_EQ(control.step().status, common::types::DroneStepStatus::Continue);
    EXPECT_EQ(fixture.movement.advance_count_, 1);
    EXPECT_EQ(fixture.lidar.scan_count_, 1);
    EXPECT_EQ(algorithm.call_index_, 1U);
    EXPECT_EQ(control.state().step_index, 1U);
}

TEST(DroneControl, RecoverableBlockedStillScans) {
    Fixture fixture;
    fixture.movement.throw_on_advance_ = true;
    fixture.movement.advance_throw_message_ =
        "advance: destination blocked by obstacle or map boundary";

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
            .scan_orientation =
                Orientation{45.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},
            .status = common::types::AlgorithmStatus::Working,
        }},
    };

    mission_control_207190406_209543255::DroneControlImpl control{
        defaultDrone(),
        defaultLidar(),
        fixture.lidar,
        fixture.gps,
        fixture.movement,
        fixture.output_map,
        algorithm,
    };

    EXPECT_EQ(control.step().status, common::types::DroneStepStatus::Continue);
    EXPECT_EQ(fixture.lidar.scan_count_, 1);
    EXPECT_EQ(control.state().step_index, 1U);
}

TEST(DroneControl, UnsuccessfulMovementResultStillScans) {
    Fixture fixture;
    fixture.movement.advance_ok_ = false;
    fixture.movement.advance_fail_message_ = "actuator fault";

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
            .scan_orientation =
                Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},
            .status = common::types::AlgorithmStatus::Working,
        }},
    };

    mission_control_207190406_209543255::DroneControlImpl control{
        defaultDrone(),
        defaultLidar(),
        fixture.lidar,
        fixture.gps,
        fixture.movement,
        fixture.output_map,
        algorithm,
    };

    EXPECT_EQ(control.step().status, common::types::DroneStepStatus::Continue);
    EXPECT_EQ(fixture.lidar.scan_count_, 1);
    EXPECT_EQ(control.state().step_index, 1U);
}

TEST(DroneControl, AlwaysScanAlgorithmScansOncePerStep) {
    Fixture fixture;
    const auto mission = defaultMission();
    const auto lidar_cfg = defaultLidar();
    const auto drone_cfg = defaultDrone();
    std::vector<common::types::MappingStepCommand> script;
    for (int i = 0; i < 5; ++i) {
        script.push_back(common::types::MappingStepCommand{
            .scan_orientation =
                Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},
            .status = common::types::AlgorithmStatus::Working,
        });
    }
    ScriptedAlgorithm algorithm{
        common::MappingAlgorithmDependencies{mission, lidar_cfg, drone_cfg, fixture.stand_in_map},
        std::move(script),
    };

    mission_control_207190406_209543255::DroneControlImpl control{
        defaultDrone(),
        defaultLidar(),
        fixture.lidar,
        fixture.gps,
        fixture.movement,
        fixture.output_map,
        algorithm,
    };

    for (int i = 0; i < 5; ++i) {
        EXPECT_EQ(control.step().status, common::types::DroneStepStatus::Continue);
    }
    EXPECT_EQ(fixture.lidar.scan_count_, 5);
    EXPECT_EQ(algorithm.call_index_, 5U);
}

TEST(DroneControl, WorldOutOfBoundsAdvanceIsIgnored) {
    Fixture fixture;
    fixture.gps.position_ = Position3D{
        95.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]};
    // heading default 0 => +X; dest 115 is outside map 0..100

    ScriptedAlgorithm algorithm{
        common::MappingAlgorithmDependencies{
            defaultMission(), defaultLidar(), defaultDrone(), fixture.stand_in_map},
        {common::types::MappingStepCommand{
            .movement =
                common::types::MovementCommand{
                    .type = common::types::MovementCommandType::Advance,
                    .distance = 20.0 * cm,
                },
            .status = common::types::AlgorithmStatus::Working,
        }},
    };

    mission_control_207190406_209543255::DroneControlImpl control{
        defaultDrone(), defaultLidar(), fixture.lidar, fixture.gps,
        fixture.movement, fixture.output_map, algorithm,
    };

    const auto result = control.step();
    EXPECT_EQ(result.status, common::types::DroneStepStatus::Continue);
    EXPECT_EQ(fixture.movement.advance_count_, 0);
}

TEST(DroneControl, InBoundsAdvanceStillReachesMovement) {
    Fixture fixture;
    fixture.gps.position_ = Position3D{
        50.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]};
    ScriptedAlgorithm algorithm{
        common::MappingAlgorithmDependencies{
            defaultMission(), defaultLidar(), defaultDrone(), fixture.stand_in_map},
        {common::types::MappingStepCommand{
            .movement =
                common::types::MovementCommand{
                    .type = common::types::MovementCommandType::Advance,
                    .distance = 10.0 * cm,
                },
            .status = common::types::AlgorithmStatus::Working,
        }},
    };
    mission_control_207190406_209543255::DroneControlImpl control{
        defaultDrone(), defaultLidar(), fixture.lidar, fixture.gps,
        fixture.movement, fixture.output_map, algorithm,
    };
    EXPECT_EQ(control.step().status, common::types::DroneStepStatus::Continue);
    EXPECT_EQ(fixture.movement.advance_count_, 1);
}

TEST(DroneControl, MissionBoundsAdvanceIsClamped) {
    Fixture fixture;
    fixture.gps.position_ = Position3D{
        70.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]};
    auto mission = defaultMission();
    mission.mission_bounds = {
        0.0 * x_extent[cm], 80.0 * x_extent[cm], 0.0 * y_extent[cm], 100.0 * y_extent[cm],
        0.0 * z_extent[cm], 100.0 * z_extent[cm],
    };
    ScriptedAlgorithm algorithm{
        common::MappingAlgorithmDependencies{
            mission, defaultLidar(), defaultDrone(), fixture.stand_in_map},
        {common::types::MappingStepCommand{
            .movement =
                common::types::MovementCommand{
                    .type = common::types::MovementCommandType::Advance,
                    .distance = 20.0 * cm,
                },
            .status = common::types::AlgorithmStatus::Working,
        }},
    };
    mission_control_207190406_209543255::DroneControlImpl control{
        defaultDrone(), defaultLidar(), fixture.lidar, fixture.gps,
        fixture.movement, fixture.output_map, algorithm, mission.mission_bounds,
    };
    EXPECT_EQ(control.step().status, common::types::DroneStepStatus::Continue);
    EXPECT_EQ(fixture.movement.advance_count_, 1);
    EXPECT_DOUBLE_EQ(fixture.movement.last_advance_.numerical_value_in(cm), 10.0);
}

TEST(DroneControl, MissionLargerThanMapStillIgnoresWorldOob) {
    Fixture fixture;
    fixture.gps.position_ = Position3D{
        95.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]};
    auto mission = defaultMission();
    mission.mission_bounds = {
        0.0 * x_extent[cm], 200.0 * x_extent[cm], 0.0 * y_extent[cm], 100.0 * y_extent[cm],
        0.0 * z_extent[cm], 100.0 * z_extent[cm],
    };
    ScriptedAlgorithm algorithm{
        common::MappingAlgorithmDependencies{
            mission, defaultLidar(), defaultDrone(), fixture.stand_in_map},
        {common::types::MappingStepCommand{
            .movement =
                common::types::MovementCommand{
                    .type = common::types::MovementCommandType::Advance,
                    .distance = 20.0 * cm,
                },
            .status = common::types::AlgorithmStatus::Working,
        }},
    };
    mission_control_207190406_209543255::DroneControlImpl control{
        defaultDrone(), defaultLidar(), fixture.lidar, fixture.gps,
        fixture.movement, fixture.output_map, algorithm, mission.mission_bounds,
    };
    EXPECT_EQ(control.step().status, common::types::DroneStepStatus::Continue);
    EXPECT_EQ(fixture.movement.advance_count_, 0);
}

TEST(DroneControl, ElevateIsClampedToMissionHeight) {
    Fixture fixture;
    fixture.gps.position_ = Position3D{
        50.0 * x_extent[cm], 50.0 * y_extent[cm], 90.0 * z_extent[cm]};
    auto mission = defaultMission();
    mission.mission_bounds = {
        0.0 * x_extent[cm], 100.0 * x_extent[cm], 0.0 * y_extent[cm], 100.0 * y_extent[cm],
        0.0 * z_extent[cm], 100.0 * z_extent[cm],
    };
    ScriptedAlgorithm algorithm{
        common::MappingAlgorithmDependencies{
            mission, defaultLidar(), defaultDrone(), fixture.stand_in_map},
        {common::types::MappingStepCommand{
            .movement =
                common::types::MovementCommand{
                    .type = common::types::MovementCommandType::Elevate,
                    .distance = 20.0 * cm,
                },
            .status = common::types::AlgorithmStatus::Working,
        }},
    };
    mission_control_207190406_209543255::DroneControlImpl control{
        defaultDrone(), defaultLidar(), fixture.lidar, fixture.gps,
        fixture.movement, fixture.output_map, algorithm, mission.mission_bounds,
    };
    EXPECT_EQ(control.step().status, common::types::DroneStepStatus::Continue);
    EXPECT_DOUBLE_EQ(fixture.movement.last_elevate_.numerical_value_in(cm), 10.0);
}
