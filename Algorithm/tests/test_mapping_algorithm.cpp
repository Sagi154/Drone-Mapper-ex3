// test_mapping_algorithm.cpp — MappingAlgorithm.*
// Tests MappingAlgorithmImpl nextStep state machine and command emission.
// Ported from Drone-Mapper-ex2; uses FakeMap3D instead of Map3DImpl+TinyNPY.

#include "FakeMap3D.h"

#include <Algorithm/MappingAlgorithmImpl.h>

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <set>
#include <tuple>

namespace Algo = Algorithm_207190406_209543255;
using Impl    = Algo::MappingAlgorithmImpl_207190406_209543255;
using Map     = AlgorithmTest::FakeMap3D;

namespace {

using common::Orientation;
using common::Position3D;
using common::cm;
using common::deg;
using common::x_extent;
using common::y_extent;
using common::z_extent;
namespace ct = common::types;

[[nodiscard]] ct::MapConfig makeCorridorConfig() {
    ct::MapConfig config{};
    config.resolution            = 10.0 * cm;
    config.offset                = Position3D{};
    config.boundaries.min_x      = 0.0 * x_extent[cm];
    config.boundaries.max_x      = 100.0 * x_extent[cm];
    config.boundaries.min_y      = 0.0 * y_extent[cm];
    config.boundaries.max_y      = 100.0 * y_extent[cm];
    config.boundaries.min_height = 0.0 * z_extent[cm];
    config.boundaries.max_height = 100.0 * z_extent[cm];
    return config;
}

[[nodiscard]] ct::DroneConfigData makeDroneConfig() {
    ct::DroneConfigData cfg{};
    cfg.radius      = 5.0 * cm;
    cfg.max_rotate  = 45.0 * deg;
    cfg.max_advance = 10.0 * cm;
    cfg.max_elevate = 10.0 * cm;
    return cfg;
}

[[nodiscard]] ct::LidarConfigData makeLidarConfig() {
    ct::LidarConfigData cfg{};
    cfg.z_min       = 1.0 * cm;
    cfg.z_max       = 200.0 * cm;
    cfg.d           = 1.0 * cm;
    cfg.fov_circles = 1;
    return cfg;
}

[[nodiscard]] ct::MissionConfigData makeMissionConfig() {
    ct::MissionConfigData cfg{};
    cfg.max_steps      = 10000;
    cfg.gps_resolution = 1.0 * cm;
    return cfg;
}

[[nodiscard]] Position3D gridPoint(int x, int y, int z, const ct::MapConfig& config) {
    const double step = config.resolution.force_numerical_value_in(cm);
    const double ox   = config.offset.x.force_numerical_value_in(cm);
    const double oy   = config.offset.y.force_numerical_value_in(cm);
    const double oz   = config.offset.z.force_numerical_value_in(cm);
    return Position3D{
        (ox + static_cast<double>(x) * step) * x_extent[cm],
        (oy + static_cast<double>(y) * step) * y_extent[cm],
        (oz + static_cast<double>(z) * step) * z_extent[cm],
    };
}

void fillEmptyBox(Map& map, int x0, int x1, int y0, int y1, int z0, int z1,
                  const ct::MapConfig& config) {
    for (int x = x0; x <= x1; ++x) {
        for (int y = y0; y <= y1; ++y) {
            for (int z = z0; z <= z1; ++z) {
                map.set(gridPoint(x, y, z, config), ct::VoxelOccupancy::Empty);
            }
        }
    }
}

[[nodiscard]] std::optional<ct::MappingStepCommand>
firstCommandMatching(Impl& algorithm, const ct::DroneState& state, int max_steps,
                     const std::function<bool(const ct::MappingStepCommand&)>& predicate) {
    for (int step = 0; step < max_steps; ++step) {
        const ct::MappingStepCommand cmd = algorithm.nextStep(state, nullptr);
        if (predicate(cmd)) {
            return cmd;
        }
        if (cmd.status != ct::AlgorithmStatus::Working) {
            return std::nullopt;
        }
    }
    return std::nullopt;
}

[[nodiscard]] std::set<std::tuple<double, double>>
collectScanOrientationsUntilNonScan(Impl& algorithm, const ct::DroneState& state,
                                    int max_steps) {
    std::set<std::tuple<double, double>> orientations;
    for (int step = 0; step < max_steps; ++step) {
        const ct::MappingStepCommand cmd = algorithm.nextStep(state, nullptr);
        if (!cmd.scan_orientation.has_value()) {
            break;
        }
        const double az = cmd.scan_orientation->horizontal.force_numerical_value_in(deg);
        const double el = cmd.scan_orientation->altitude.force_numerical_value_in(deg);
        orientations.insert({az, el});
        if (cmd.status != ct::AlgorithmStatus::Working) {
            break;
        }
    }
    return orientations;
}

void fillStartBubble(Map& map, int cx, int cy, int cz, const ct::MapConfig& config) {
    map.set(gridPoint(cx, cy, cz, config), ct::VoxelOccupancy::Empty);
    fillEmptyBox(map, cx - 1, cx + 1, cy, cy, cz, cz, config);
    fillEmptyBox(map, cx, cx, cy - 1, cy + 1, cz, cz, config);
    fillEmptyBox(map, cx, cx, cy, cy, cz - 1, cz + 1, config);
}

[[nodiscard]] ct::AlgorithmStatus runUntilTerminal(Impl& algorithm,
                                                    const ct::DroneState& state,
                                                    int max_steps) {
    ct::AlgorithmStatus status = ct::AlgorithmStatus::Working;
    for (int step = 0; step < max_steps; ++step) {
        status = algorithm.nextStep(state, nullptr).status;
        if (status != ct::AlgorithmStatus::Working) {
            return status;
        }
    }
    return status;
}

} // namespace

// What: first nextStep on a fresh mission with no prior scan.
// Expected: algorithm requests a scan orientation (latest_scan is nullptr).
TEST(MappingAlgorithm, FirstStepRequestsScanWithNullLatestScan) {
    const ct::MapConfig config = makeCorridorConfig();
    Map output_map{{11, 11, 11}, config};
    fillEmptyBox(output_map, 4, 6, 4, 6, 4, 6, config);

    const auto mc = makeMissionConfig();
    const auto lc = makeLidarConfig();
    const auto dc = makeDroneConfig();
    Impl algorithm{common::MappingAlgorithmDependencies{mc, lc, dc, output_map}};

    const ct::DroneState state{gridPoint(5, 5, 5, config), Orientation{0.0 * deg, 0.0 * deg}, 0};

    const ct::MappingStepCommand cmd = algorithm.nextStep(state, nullptr);
    EXPECT_EQ(cmd.status, ct::AlgorithmStatus::Working);
    ASSERT_TRUE(cmd.scan_orientation.has_value());
    EXPECT_FALSE(cmd.movement.has_value());
}

// What: fully mapped empty volume with no adjacent unknown cells.
// Expected: after exhausting scan sweep, algorithm reports Finished.
TEST(MappingAlgorithm, FinishesWhenNoFrontierRemains) {
    const ct::MapConfig config = makeCorridorConfig();
    Map output_map{{3, 3, 3}, config};
    fillEmptyBox(output_map, 0, 2, 0, 2, 0, 2, config);

    const auto mc = makeMissionConfig();
    const auto lc = makeLidarConfig();
    const auto dc = makeDroneConfig();
    Impl algorithm{common::MappingAlgorithmDependencies{mc, lc, dc, output_map}};

    const ct::DroneState state{
        gridPoint(1, 1, 1, config), Orientation{0.0 * deg, 0.0 * deg}, 0};

    const ct::AlgorithmStatus last_status = runUntilTerminal(algorithm, state, 5000);
    EXPECT_EQ(last_status, ct::AlgorithmStatus::Finished);
}

// What: empty corridor with unknown space beyond +X; start sphere is passable.
// Expected: planning finds a frontier and nextStep emits movement toward it.
TEST(MappingAlgorithm, EmitsMovementTowardFrontier) {
    const ct::MapConfig config = makeCorridorConfig();
    Map output_map{{11, 3, 3}, config};
    fillEmptyBox(output_map, 0, 4, 0, 2, 0, 2, config);

    const auto mc = makeMissionConfig();
    const auto lc = makeLidarConfig();
    const auto dc = makeDroneConfig();
    Impl algorithm{common::MappingAlgorithmDependencies{mc, lc, dc, output_map}};

    const ct::DroneState state{
        gridPoint(2, 1, 1, config), Orientation{0.0 * deg, 0.0 * deg}, 0};

    const auto cmd = firstCommandMatching(
        algorithm, state, 500, [](const ct::MappingStepCommand& step) {
            return step.movement.has_value() &&
                   step.movement->type != ct::MovementCommandType::Hover;
        });

    ASSERT_TRUE(cmd.has_value());
    EXPECT_EQ(cmd->status, ct::AlgorithmStatus::Working);
}

// What: algorithm begins in scanning phase on a fresh mission.
// Expected: multiple consecutive steps request scan orientations before movement.
TEST(MappingAlgorithm, ScanSweepEmitsMultipleOrientationsBeforeMovement) {
    const ct::MapConfig config = makeCorridorConfig();
    Map output_map{{11, 3, 3}, config};
    fillEmptyBox(output_map, 0, 4, 0, 2, 0, 2, config);

    const auto mc = makeMissionConfig();
    const auto lc = makeLidarConfig();
    const auto dc = makeDroneConfig();
    Impl algorithm{common::MappingAlgorithmDependencies{mc, lc, dc, output_map}};

    const ct::DroneState state{
        gridPoint(2, 1, 1, config), Orientation{0.0 * deg, 0.0 * deg}, 0};

    std::size_t scan_steps = 0;
    bool saw_movement      = false;
    for (int step = 0; step < 500; ++step) {
        const ct::MappingStepCommand cmd = algorithm.nextStep(state, nullptr);
        if (cmd.scan_orientation.has_value()) {
            ++scan_steps;
        }
        if (cmd.movement.has_value() &&
            cmd.movement->type != ct::MovementCommandType::Hover) {
            saw_movement = true;
            break;
        }
        if (cmd.status != ct::AlgorithmStatus::Working) {
            break;
        }
    }

    EXPECT_GT(scan_steps, 1U);
    EXPECT_TRUE(saw_movement);
}

// What: exploration completes on a fully known map.
// Expected: subsequent nextStep calls keep returning Finished.
TEST(MappingAlgorithm, ReturnsFinishedOnSubsequentCallsAfterCompletion) {
    const ct::MapConfig config = makeCorridorConfig();
    Map output_map{{3, 3, 3}, config};
    fillEmptyBox(output_map, 0, 2, 0, 2, 0, 2, config);

    const auto mc = makeMissionConfig();
    const auto lc = makeLidarConfig();
    const auto dc = makeDroneConfig();
    Impl algorithm{common::MappingAlgorithmDependencies{mc, lc, dc, output_map}};

    const ct::DroneState state{
        gridPoint(1, 1, 1, config), Orientation{0.0 * deg, 0.0 * deg}, 0};

    ASSERT_EQ(runUntilTerminal(algorithm, state, 5000), ct::AlgorithmStatus::Finished);

    for (int i = 0; i < 3; ++i) {
        const ct::MappingStepCommand cmd = algorithm.nextStep(state, nullptr);
        EXPECT_EQ(cmd.status, ct::AlgorithmStatus::Finished);
        EXPECT_FALSE(cmd.scan_orientation.has_value());
        EXPECT_FALSE(cmd.movement.has_value());
    }
}

// What: drone heading is 90° while the frontier path lies along +X.
// Expected: first movement command is a Rotate toward the path.
TEST(MappingAlgorithm, EmitsRotateWhenHeadingMisalignedWithPath) {
    const ct::MapConfig config = makeCorridorConfig();
    Map output_map{{11, 3, 3}, config};
    fillEmptyBox(output_map, 0, 4, 0, 2, 0, 2, config);

    const auto mc = makeMissionConfig();
    const auto lc = makeLidarConfig();
    const auto dc = makeDroneConfig();
    Impl algorithm{common::MappingAlgorithmDependencies{mc, lc, dc, output_map}};

    const ct::DroneState state{gridPoint(2, 1, 1, config),
                               Orientation{90.0 * deg, 0.0 * deg}, 0};

    const auto cmd = firstCommandMatching(
        algorithm, state, 500,
        [](const ct::MappingStepCommand& step) { return step.movement.has_value(); });

    ASSERT_TRUE(cmd.has_value());
    ASSERT_TRUE(cmd->movement.has_value());
    EXPECT_EQ(cmd->movement->type, ct::MovementCommandType::Rotate);
}

// What: drone heading already faces +X along the corridor path.
// Expected: first movement command is Advance (or Hover if already at waypoint).
TEST(MappingAlgorithm, EmitsAdvanceWhenHeadingAlignedWithPath) {
    const ct::MapConfig config = makeCorridorConfig();
    Map output_map{{11, 3, 3}, config};
    fillEmptyBox(output_map, 0, 4, 0, 2, 0, 2, config);

    const auto mc = makeMissionConfig();
    const auto lc = makeLidarConfig();
    const auto dc = makeDroneConfig();
    Impl algorithm{common::MappingAlgorithmDependencies{mc, lc, dc, output_map}};

    const ct::DroneState state{
        gridPoint(2, 1, 1, config), Orientation{0.0 * deg, 0.0 * deg}, 0};

    const auto cmd = firstCommandMatching(
        algorithm, state, 500,
        [](const ct::MappingStepCommand& step) { return step.movement.has_value(); });

    ASSERT_TRUE(cmd.has_value());
    ASSERT_TRUE(cmd->movement.has_value());
    EXPECT_EQ(cmd->movement->type, ct::MovementCommandType::Advance);
}

// What: waypoint is at a higher z than the drone on the same column.
// Expected: first movement command elevates toward the waypoint.
TEST(MappingAlgorithm, EmitsElevateWhenWaypointHeightDiffers) {
    const ct::MapConfig config = makeCorridorConfig();
    Map output_map{{5, 3, 5}, config};
    fillEmptyBox(output_map, 2, 2, 1, 1, 0, 3, config);

    const auto mc = makeMissionConfig();
    const auto lc = makeLidarConfig();
    const auto dc = makeDroneConfig();
    Impl algorithm{common::MappingAlgorithmDependencies{mc, lc, dc, output_map}};

    const ct::DroneState state{
        gridPoint(2, 1, 0, config), Orientation{0.0 * deg, 0.0 * deg}, 0};

    const auto cmd = firstCommandMatching(
        algorithm, state, 800, [](const ct::MappingStepCommand& step) {
            return step.movement.has_value() &&
                   step.movement->type == ct::MovementCommandType::Elevate;
        });

    ASSERT_TRUE(cmd.has_value());
    EXPECT_GT(cmd->movement->distance.force_numerical_value_in(cm), 0.0);
}

// What: each emitted command during scanning/planning/moving.
// Expected: algorithm never bundles scan and movement in the same step.
TEST(MappingAlgorithm, DoesNotCombineScanAndMovementInOneStep) {
    const ct::MapConfig config = makeCorridorConfig();
    Map output_map{{11, 3, 3}, config};
    fillEmptyBox(output_map, 0, 4, 0, 2, 0, 2, config);

    const auto mc = makeMissionConfig();
    const auto lc = makeLidarConfig();
    const auto dc = makeDroneConfig();
    Impl algorithm{common::MappingAlgorithmDependencies{mc, lc, dc, output_map}};

    const ct::DroneState state{
        gridPoint(2, 1, 1, config), Orientation{0.0 * deg, 0.0 * deg}, 0};

    for (int step = 0; step < 200; ++step) {
        const ct::MappingStepCommand cmd = algorithm.nextStep(state, nullptr);
        EXPECT_FALSE(cmd.scan_orientation.has_value() && cmd.movement.has_value());
        if (cmd.status != ct::AlgorithmStatus::Working) {
            break;
        }
    }
}

// What: isolated empty cell surrounded by Unmapped with no frontier progress possible.
// Expected: algorithm eventually stops with FinishedWithUnmappableVoxels while unknown remains.
TEST(MappingAlgorithm, FinishesWithUnmappableWhenStartNotSpherePassable) {
    const ct::MapConfig config = makeCorridorConfig();
    Map output_map{{11, 11, 11}, config};
    output_map.set(gridPoint(5, 5, 5, config), ct::VoxelOccupancy::Empty);

    const auto mc = makeMissionConfig();
    const auto lc = makeLidarConfig();
    const auto dc = makeDroneConfig();
    Impl algorithm{common::MappingAlgorithmDependencies{mc, lc, dc, output_map}};

    const ct::DroneState state{
        gridPoint(5, 5, 5, config), Orientation{0.0 * deg, 0.0 * deg}, 0};

    const ct::AlgorithmStatus last_status = runUntilTerminal(algorithm, state, 8000);
    EXPECT_EQ(last_status, ct::AlgorithmStatus::FinishedWithUnmappableVoxels);
}

// What: scan pass zero on a fresh mission.
// Expected: first scan sweep includes all six axis-aligned orientations.
TEST(MappingAlgorithm, ScanPassZeroIncludesAxisAlignedOrientations) {
    const ct::MapConfig config = makeCorridorConfig();
    Map output_map{{11, 11, 11}, config};
    output_map.set(gridPoint(5, 5, 5, config), ct::VoxelOccupancy::Empty);

    const auto mc = makeMissionConfig();
    const auto lc = makeLidarConfig();
    const auto dc = makeDroneConfig();
    Impl algorithm{common::MappingAlgorithmDependencies{mc, lc, dc, output_map}};

    const ct::DroneState state{
        gridPoint(5, 5, 5, config), Orientation{0.0 * deg, 0.0 * deg}, 0};

    const std::set<std::tuple<double, double>> orientations =
        collectScanOrientationsUntilNonScan(algorithm, state, 200);

    const std::set<std::tuple<double, double>> expected = {
        {0.0, 0.0}, {180.0, 0.0}, {90.0, 0.0}, {270.0, 0.0}, {0.0, 90.0}, {0.0, -90.0},
    };
    for (const auto& axis : expected) {
        EXPECT_TRUE(orientations.contains(axis)) << "missing axis-aligned orientation";
    }
}

// What: start position with a passable local bubble (center + six face neighbors Empty).
// Expected: planning finds a frontier and emits movement.
TEST(MappingAlgorithm, DroneNavigatesFromStartWhenAdjacentCellsAreEmpty) {
    const ct::MapConfig config = makeCorridorConfig();
    Map output_map{{11, 3, 3}, config};
    fillStartBubble(output_map, 2, 1, 1, config);

    const auto mc = makeMissionConfig();
    const auto lc = makeLidarConfig();
    const auto dc = makeDroneConfig();
    Impl algorithm{common::MappingAlgorithmDependencies{mc, lc, dc, output_map}};

    const ct::DroneState state{
        gridPoint(2, 1, 1, config), Orientation{0.0 * deg, 0.0 * deg}, 0};

    const auto cmd = firstCommandMatching(
        algorithm, state, 500, [](const ct::MappingStepCommand& step) {
            return step.movement.has_value() &&
                   step.movement->type != ct::MovementCommandType::Hover;
        });

    ASSERT_TRUE(cmd.has_value());
    EXPECT_EQ(cmd->status, ct::AlgorithmStatus::Working);
}

// What: latest_scan pointer is accepted but scanning phase is driven internally.
// Expected: providing a non-null latest_scan on step zero still begins with a scan request.
TEST(MappingAlgorithm, AcceptsNonNullLatestScanWithoutChangingFirstScanRequest) {
    const ct::MapConfig config = makeCorridorConfig();
    Map output_map{{11, 11, 11}, config};
    fillEmptyBox(output_map, 4, 6, 4, 6, 4, 6, config);

    const auto mc = makeMissionConfig();
    const auto lc = makeLidarConfig();
    const auto dc = makeDroneConfig();
    Impl algorithm{common::MappingAlgorithmDependencies{mc, lc, dc, output_map}};

    const ct::DroneState state{
        gridPoint(5, 5, 5, config), Orientation{0.0 * deg, 0.0 * deg}, 0};

    ct::LidarScanResult fake_scan{{ct::LidarHit{100.0 * cm, Orientation{}}}};
    const ct::MappingStepCommand cmd = algorithm.nextStep(state, &fake_scan);

    EXPECT_EQ(cmd.status, ct::AlgorithmStatus::Working);
    ASSERT_TRUE(cmd.scan_orientation.has_value());
}

// NOTE: NoFusionMaxEmittedInScanCommand is intentionally omitted — the
// MappingStepCommand in ex3 has no fusion_max field.

// What: corridor with unknown space beyond the fused empty region.
// Expected: algorithm keeps working through scan + explore/rescan instead of quitting early.
TEST(MappingAlgorithm, DoesNotTerminatePrematurely) {
    const ct::MapConfig config = makeCorridorConfig();
    Map output_map{{11, 3, 3}, config};
    fillEmptyBox(output_map, 0, 4, 0, 2, 0, 2, config);

    const auto mc = makeMissionConfig();
    const auto lc = makeLidarConfig();
    const auto dc = makeDroneConfig();
    Impl algorithm{common::MappingAlgorithmDependencies{mc, lc, dc, output_map}};

    const ct::DroneState state{
        gridPoint(2, 1, 1, config), Orientation{0.0 * deg, 0.0 * deg}, 0};

    int working_steps = 0;
    for (int step = 0; step < 100; ++step) {
        const ct::MappingStepCommand cmd = algorithm.nextStep(state, nullptr);
        if (cmd.status == ct::AlgorithmStatus::Working) {
            ++working_steps;
            continue;
        }
        if (cmd.status == ct::AlgorithmStatus::Finished ||
            cmd.status == ct::AlgorithmStatus::FinishedWithUnmappableVoxels) {
            break;
        }
    }

    EXPECT_GT(working_steps, 26);
}

// What: nextStep receives a non-null LidarScanResult after the initial scan phase.
// Expected: algorithm stays Working and does not finish prematurely.
TEST(MappingAlgorithm, HandlesNonNullScanResultWithoutCrashOrPrematureFinish) {
    const ct::MapConfig config = makeCorridorConfig();
    Map output_map{{11, 3, 3}, config};
    fillEmptyBox(output_map, 0, 4, 0, 2, 0, 2, config);

    const auto mc = makeMissionConfig();
    const auto lc = makeLidarConfig();
    const auto dc = makeDroneConfig();
    Impl algorithm{common::MappingAlgorithmDependencies{mc, lc, dc, output_map}};

    const ct::DroneState state{
        gridPoint(2, 1, 1, config), Orientation{0.0 * deg, 0.0 * deg}, 0};

    const ct::MappingStepCommand first_cmd = algorithm.nextStep(state, nullptr);
    EXPECT_EQ(first_cmd.status, ct::AlgorithmStatus::Working);
    ASSERT_TRUE(first_cmd.scan_orientation.has_value());

    ct::LidarScanResult fake_scan{
        {ct::LidarHit{std::numeric_limits<double>::max() * cm, Orientation{}}}};
    const ct::MappingStepCommand second_cmd = algorithm.nextStep(state, &fake_scan);

    EXPECT_EQ(second_cmd.status, ct::AlgorithmStatus::Working);
    EXPECT_NE(second_cmd.status, ct::AlgorithmStatus::Finished);
}
