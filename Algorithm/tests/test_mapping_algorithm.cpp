// test_mapping_algorithm.cpp — MappingAlgorithm.*
// Tests MappingAlgorithmImpl nextStep state machine and command emission.
// Ported from Drone-Mapper-ex2; uses FakeMap3D instead of Map3DImpl+TinyNPY.

#include "FakeMap3D.h"

#include <Algorithm/MappingAlgorithmImpl.h>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
#include <vector>

namespace Algo = algorithm_207190406_209543255;
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
    auto lc = makeLidarConfig();
    lc.z_max = 20.0 * cm;  // Unmapped pocket starts 25 cm past start; stay-in-place must lose.
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
    auto lc = makeLidarConfig();
    lc.z_max = 20.0 * cm;  // Unmapped pocket starts 25 cm past start; stay-in-place must lose.
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
    auto lc = makeLidarConfig();
    lc.z_max = 20.0 * cm;  // Unmapped pocket starts 25 cm past start; stay-in-place must lose.
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
    // Empty slab at z=0..1 so the start sphere is passable (z=0 no longer clips
    // OutOfBounds) and nearby XY is known. Shaft up to z=3; Unmapped sits at z=4.
    fillEmptyBox(output_map, 1, 3, 0, 2, 0, 1, config);
    fillEmptyBox(output_map, 2, 2, 1, 1, 2, 3, config);

    const auto mc = makeMissionConfig();
    auto lc = makeLidarConfig();
    lc.z_max = 8.0 * cm;  // Start cannot see Unmapped ~14 cm away; z=2 shaft pose can.
    const auto dc = makeDroneConfig();
    Impl algorithm{common::MappingAlgorithmDependencies{mc, lc, dc, output_map}};

    const ct::DroneState state{
        gridPoint(2, 1, 1, config), Orientation{0.0 * deg, 0.0 * deg}, 0};

    const auto cmd = firstCommandMatching(
        algorithm, state, 800, [](const ct::MappingStepCommand& step) {
            return step.movement.has_value() &&
                   step.movement->type == ct::MovementCommandType::Elevate;
        });

    ASSERT_TRUE(cmd.has_value());
    EXPECT_GT(cmd->movement->distance.force_numerical_value_in(cm), 0.0);
}

// What: isolated empty cell surrounded by Unmapped with no frontier progress possible.
// Expected: algorithm eventually stops with FinishedWithUnmappableVoxels while unknown remains.
TEST(MappingAlgorithm, FinishesWithUnmappableWhenStartNotSpherePassable) {
    const ct::MapConfig config = makeCorridorConfig();
    Map output_map{{11, 11, 11}, config};
    output_map.set(gridPoint(5, 5, 5, config), ct::VoxelOccupancy::Empty);
    // Unmapped is traversable, so Empty-in-Unmapped is still sphere-passable and
    // the fixture cone sees diagonal Unmapped between face walls. Occupy the
    // full 3³ neighbourhood so start is not passable and beams stop immediately.
    // Brief recovery then FinishedWithUnmappableVoxels. Fixture lidar kept.
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dz = -1; dz <= 1; ++dz) {
                if (dx == 0 && dy == 0 && dz == 0) {
                    continue;
                }
                output_map.set(gridPoint(5 + dx, 5 + dy, 5 + dz, config),
                               ct::VoxelOccupancy::Occupied);
            }
        }
    }

    const auto mc = makeMissionConfig();
    const auto lc = makeLidarConfig();
    const auto dc = makeDroneConfig();
    Impl algorithm{common::MappingAlgorithmDependencies{mc, lc, dc, output_map}};

    const ct::DroneState state{
        gridPoint(5, 5, 5, config), Orientation{0.0 * deg, 0.0 * deg}, 0};

    const ct::AlgorithmStatus last_status = runUntilTerminal(algorithm, state, 8000);
    EXPECT_EQ(last_status, ct::AlgorithmStatus::FinishedWithUnmappableVoxels);
}

// What: start position with a passable local bubble (center + six face neighbors Empty).
// Expected: planning finds a frontier and emits movement.
TEST(MappingAlgorithm, DroneNavigatesFromStartWhenAdjacentCellsAreEmpty) {
    const ct::MapConfig config = makeCorridorConfig();
    Map output_map{{11, 3, 3}, config};
    fillStartBubble(output_map, 2, 1, 1, config);

    const auto mc = makeMissionConfig();
    auto lc = makeLidarConfig();
    lc.z_max = 6.0 * cm;  // Corner Unmapped is ~9 cm; 10 cm still let stay-in-place win.
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

    EXPECT_GT(working_steps, 0);
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

// What: drone fails to advance toward a waypoint for kMaxMovingStallTicks steps.
// Expected: stall recovery only records blocked_cells; output map voxels stay unchanged.
TEST(MappingAlgorithm, StallPathDoesNotMutateMap) {
    const ct::MapConfig config = makeCorridorConfig();
    constexpr std::array<int, 3> dims{11, 3, 3};
    Map output_map{dims, config};
    fillEmptyBox(output_map, 0, 4, 0, 2, 0, 2, config);

    const auto snapshotOccupancy = [&]() {
        std::vector<ct::VoxelOccupancy> cells;
        cells.reserve(static_cast<std::size_t>(dims[0] * dims[1] * dims[2]));
        for (int x = 0; x < dims[0]; ++x) {
            for (int y = 0; y < dims[1]; ++y) {
                for (int z = 0; z < dims[2]; ++z) {
                    cells.push_back(output_map.atVoxel(gridPoint(x, y, z, config)));
                }
            }
        }
        return cells;
    };

    const auto mc = makeMissionConfig();
    auto lc = makeLidarConfig();
    lc.z_max = 20.0 * cm;  // Unmapped pocket starts 25 cm past start; must travel to stall.
    const auto dc = makeDroneConfig();
    Impl algorithm{common::MappingAlgorithmDependencies{mc, lc, dc, output_map}};

    const ct::DroneState state{
        gridPoint(2, 1, 1, config), Orientation{0.0 * deg, 0.0 * deg}, 0};

    // Reach Moving and emit a non-hover movement (establishes last_position).
    const auto move_cmd = firstCommandMatching(
        algorithm, state, 500, [](const ct::MappingStepCommand& step) {
            return step.movement.has_value() &&
                   step.movement->type != ct::MovementCommandType::Hover;
        });
    ASSERT_TRUE(move_cmd.has_value());

    const auto before = snapshotOccupancy();

    // Hold position long enough to trip stall recovery (kMaxMovingStallTicks == 2).
    constexpr int kStallTicks = 2;
    for (int tick = 0; tick < kStallTicks + 2; ++tick) {
        (void)algorithm.nextStep(state, nullptr);
    }

    const auto after = snapshotOccupancy();
    ASSERT_EQ(before.size(), after.size());
    for (std::size_t i = 0; i < before.size(); ++i) {
        EXPECT_EQ(before[i], after[i]) << "voxel " << i << " mutated by stall path";
    }
}

// What: a travel step should carry a scan as well as a movement.
// Expected: at least one emitted command has both fields set.
TEST(MappingAlgorithm, EmitsMovementAndScanInTheSameCommand) {
    const ct::MapConfig config = makeCorridorConfig();
    Map output_map{{11, 11, 11}, config};
    fillEmptyBox(output_map, 0, 8, 0, 10, 0, 10, config);  // unknown wall beyond x=8

    const auto mc = makeMissionConfig();
    auto lc = makeLidarConfig();
    lc.z_max = 60.0 * cm;  // Start cannot see the wall (65 cm); one 10 cm step can.
    const auto dc = makeDroneConfig();
    Impl algorithm{common::MappingAlgorithmDependencies{mc, lc, dc, output_map}};

    const ct::DroneState state{gridPoint(2, 5, 5, config), Orientation{0.0 * deg, 0.0 * deg}, 0};

    const auto cmd = firstCommandMatching(
        algorithm, state, 500, [](const ct::MappingStepCommand& step) {
            return step.movement.has_value() && step.scan_orientation.has_value();
        });

    ASSERT_TRUE(cmd.has_value()) << "no command carried both a movement and a scan";
    EXPECT_EQ(cmd->status, ct::AlgorithmStatus::Working);
}

// What: unresolved space remains and the budget is nearly untouched.
// Expected: the algorithm keeps working instead of declaring itself finished.
//
// This is the house_full failure as a unit test: the retired policy set finished=true
// on the first planning cycle whose fallbacks all missed.
TEST(MappingAlgorithm, DoesNotFinishWhileUnresolvedSpaceRemainsAndBudgetIsLarge) {
    const ct::MapConfig config = makeCorridorConfig();
    Map output_map{{11, 11, 11}, config};
    fillEmptyBox(output_map, 0, 8, 0, 10, 0, 10, config);

    const auto mc = makeMissionConfig();
    const auto lc = makeLidarConfig();
    const auto dc = makeDroneConfig();
    Impl algorithm{common::MappingAlgorithmDependencies{mc, lc, dc, output_map}};

    // Short window: still Working (must not repeat the 3-step house_full quit).
    // A frozen map that never drops Unmapped is required to stop later — see
    // FinishesWhenUnmappedCountDoesNotDropAcrossReplans.
    for (int step = 0; step < 50; ++step) {
        const ct::DroneState state{gridPoint(2, 5, 5, config),
                                   Orientation{0.0 * deg, 0.0 * deg},
                                   static_cast<std::size_t>(step)};
        const ct::MappingStepCommand cmd = algorithm.nextStep(state, nullptr);
        ASSERT_EQ(cmd.status, ct::AlgorithmStatus::Working) << "gave up at step " << step;
    }
}

// What: Unmapped remains and the planner still finds a high predicted rate (large
// Empty surface), but MissionControl never classifies new cells so Unmapped
// never drops.
// Expected: FinishedWithUnmappableVoxels after several full replan intervals,
// not a 10k-step burn and not a 3-step quit.
TEST(MappingAlgorithm, FinishesWhenUnmappedCountDoesNotDropAcrossReplans) {
    const ct::MapConfig config = makeCorridorConfig();
    Map output_map{{11, 11, 11}, config};
    fillEmptyBox(output_map, 0, 8, 0, 10, 0, 10, config);

    const auto mc = makeMissionConfig();
    const auto lc = makeLidarConfig();
    const auto dc = makeDroneConfig();
    Impl algorithm{common::MappingAlgorithmDependencies{mc, lc, dc, output_map}};

    int finished_at = -1;
    ct::AlgorithmStatus last = ct::AlgorithmStatus::Working;
    for (int step = 0; step < 400; ++step) {
        const ct::DroneState state{gridPoint(2, 5, 5, config),
                                   Orientation{0.0 * deg, 0.0 * deg},
                                   static_cast<std::size_t>(step)};
        last = algorithm.nextStep(state, nullptr).status;
        if (last != ct::AlgorithmStatus::Working) {
            finished_at = step;
            break;
        }
    }
    EXPECT_EQ(last, ct::AlgorithmStatus::FinishedWithUnmappableVoxels);
    ASSERT_GE(finished_at, 50) << "quit before the keep-working window";
    EXPECT_LT(finished_at, 450) << "must not burn the mission budget";
}

// What: same large Empty surface as the stall fixture, but Unmapped drops every
// step (as if MissionControl fused new cells).
// Expected: still Working well past the stall horizon.
TEST(MappingAlgorithm, KeepsWorkingWhenUnmappedCountKeepsDropping) {
    const ct::MapConfig config = makeCorridorConfig();
    Map output_map{{11, 11, 11}, config};
    fillEmptyBox(output_map, 0, 8, 0, 10, 0, 10, config);

    const auto mc = makeMissionConfig();
    const auto lc = makeLidarConfig();
    const auto dc = makeDroneConfig();
    Impl algorithm{common::MappingAlgorithmDependencies{mc, lc, dc, output_map}};

    const auto classify_one = [&]() {
        for (int x = 0; x < 11; ++x) {
            for (int y = 0; y < 11; ++y) {
                for (int z = 0; z < 11; ++z) {
                    const Position3D p = gridPoint(x, y, z, config);
                    if (output_map.atVoxel(p) == ct::VoxelOccupancy::Unmapped) {
                        output_map.set(p, ct::VoxelOccupancy::Empty);
                        return;
                    }
                }
            }
        }
    };

    for (int step = 0; step < 200; ++step) {
        if (step > 0) {
            classify_one();
        }
        const ct::DroneState state{gridPoint(2, 5, 5, config),
                                   Orientation{0.0 * deg, 0.0 * deg},
                                   static_cast<std::size_t>(step)};
        const ct::MappingStepCommand cmd = algorithm.nextStep(state, nullptr);
        ASSERT_EQ(cmd.status, ct::AlgorithmStatus::Working) << "gave up at step " << step;
    }
}

// What: nothing is Unmapped anywhere in bounds.
// Expected: Finished (true completion), not FinishedWithUnmappableVoxels.
TEST(MappingAlgorithm, FinishesCleanlyWhenNothingIsUnmapped) {
    const ct::MapConfig config = makeCorridorConfig();
    Map output_map{{3, 3, 3}, config};
    fillEmptyBox(output_map, 0, 2, 0, 2, 0, 2, config);

    const auto mc = makeMissionConfig();
    const auto lc = makeLidarConfig();
    const auto dc = makeDroneConfig();
    Impl algorithm{common::MappingAlgorithmDependencies{mc, lc, dc, output_map}};

    const ct::DroneState state{gridPoint(1, 1, 1, config), Orientation{0.0 * deg, 0.0 * deg}, 0};

    EXPECT_EQ(runUntilTerminal(algorithm, state, 5000), ct::AlgorithmStatus::Finished);
}

// What: budget exhausted (step_index == max_steps), unresolved space still present.
// Expected: no candidate is affordable, so the policy terminates instead of spinning.
TEST(MappingAlgorithm, TerminatesWhenBudgetIsExhausted) {
    const ct::MapConfig config = makeCorridorConfig();
    Map output_map{{11, 11, 11}, config};
    fillEmptyBox(output_map, 0, 8, 0, 10, 0, 10, config);

    const auto mc = makeMissionConfig();  // max_steps = 10000
    const auto lc = makeLidarConfig();
    const auto dc = makeDroneConfig();
    Impl algorithm{common::MappingAlgorithmDependencies{mc, lc, dc, output_map}};

    const ct::DroneState state{gridPoint(2, 5, 5, config),
                              Orientation{0.0 * deg, 0.0 * deg},
                              10000};

    ct::AlgorithmStatus status = ct::AlgorithmStatus::Working;
    for (int step = 0; step < 20 && status == ct::AlgorithmStatus::Working; ++step) {
        status = algorithm.nextStep(state, nullptr).status;
    }
    EXPECT_NE(status, ct::AlgorithmStatus::Working);
}

TEST(MappingAlgorithm, VisitsMultipleDisjointCrumbsAndFinishes) {
    const ct::MapConfig config = makeCorridorConfig();
    Map output_map{{11, 11, 11}, config};
    fillEmptyBox(output_map, 0, 10, 0, 10, 0, 10, config);
    // Three small, separated Unmapped crumbs so the first replan ranks >1 cluster.
    const std::array<std::array<int, 3>, 3> crumbs{{
        {2, 2, 5},
        {8, 2, 5},
        {5, 8, 5},
    }};
    for (const auto& crumb : crumbs) {
        output_map.set(gridPoint(crumb[0], crumb[1], crumb[2], config),
                       ct::VoxelOccupancy::Unmapped);
    }

    const auto mc = makeMissionConfig();
    const auto lc = makeLidarConfig();
    const auto dc = makeDroneConfig();
    Impl algorithm{common::MappingAlgorithmDependencies{mc, lc, dc, output_map}};

    ct::DroneState state{
        gridPoint(5, 5, 5, config), Orientation{0.0 * deg, 0.0 * deg}, 0};
    const auto apply_movement = [](ct::DroneState& current,
                                   const ct::MovementCommand& movement) {
        switch (movement.type) {
        case ct::MovementCommandType::Hover:
            break;
        case ct::MovementCommandType::Rotate: {
            const double delta =
                movement.angle.force_numerical_value_in(deg) *
                ((movement.rotation == ct::RotationDirection::Left) ? 1.0 : -1.0);
            current.heading.horizontal =
                (current.heading.horizontal.force_numerical_value_in(deg) + delta) * deg;
            break;
        }
        case ct::MovementCommandType::Advance: {
            const double distance = movement.distance.force_numerical_value_in(cm);
            const double heading_radians =
                current.heading.horizontal.force_numerical_value_in(deg) *
                (std::numbers::pi / 180.0);
            current.position = Position3D{
                (current.position.x.force_numerical_value_in(cm) +
                 distance * std::cos(heading_radians)) *
                    x_extent[cm],
                (current.position.y.force_numerical_value_in(cm) +
                 distance * std::sin(heading_radians)) *
                    y_extent[cm],
                current.position.z,
            };
            break;
        }
        case ct::MovementCommandType::Elevate:
            current.position = Position3D{
                current.position.x,
                current.position.y,
                (current.position.z.force_numerical_value_in(cm) +
                 movement.distance.force_numerical_value_in(cm)) *
                    z_extent[cm],
            };
            break;
        }
    };

    ct::AlgorithmStatus last = ct::AlgorithmStatus::Working;
    for (int step = 0; step < 400 && last == ct::AlgorithmStatus::Working; ++step) {
        const ct::MappingStepCommand command = algorithm.nextStep(state, nullptr);
        last = command.status;
        if (command.movement) {
            apply_movement(state, *command.movement);
        }
        state.step_index = static_cast<std::size_t>(step + 1);

        const double resolution = config.resolution.force_numerical_value_in(cm);
        const int drone_x = static_cast<int>(
            std::lround(state.position.x.force_numerical_value_in(cm) / resolution));
        const int drone_y = static_cast<int>(
            std::lround(state.position.y.force_numerical_value_in(cm) / resolution));
        const int drone_z = static_cast<int>(
            std::lround(state.position.z.force_numerical_value_in(cm) / resolution));
        for (const auto& crumb : crumbs) {
            const Position3D crumb_position =
                gridPoint(crumb[0], crumb[1], crumb[2], config);
            const int manhattan_distance = std::abs(drone_x - crumb[0]) +
                                           std::abs(drone_y - crumb[1]) +
                                           std::abs(drone_z - crumb[2]);
            if (manhattan_distance <= 1 &&
                output_map.atVoxel(crumb_position) == ct::VoxelOccupancy::Unmapped) {
                output_map.set(crumb_position, ct::VoxelOccupancy::Empty);
            }
        }
    }
    EXPECT_NE(last, ct::AlgorithmStatus::Working) << "did not finish within budget";
    for (const auto& crumb : crumbs) {
        EXPECT_EQ(output_map.atVoxel(gridPoint(crumb[0], crumb[1], crumb[2], config)),
                  ct::VoxelOccupancy::Empty);
    }
}

// What: several disjoint Unmapped crumbs so the first replan queues low-rate runner-ups.
// Expected: adopting those queued plans must not trip kLowRateReplans; still Working
// after more than three plan exhaustions.
TEST(MappingAlgorithm, QueuedLowRateRunnerUpsDoNotFinishEarly) {
    const ct::MapConfig config = makeCorridorConfig();
    // Occupied default so each crumb's Empty shell is a single approach cell;
    // cell_count=1 keeps expected_rate < kMinInformationRate even on a 1-step hop.
    Map output_map{{11, 11, 11}, config, ct::VoxelOccupancy::Occupied};
    fillEmptyBox(output_map, 3, 7, 3, 7, 3, 7, config);
    const std::array<std::array<int, 3>, 5> crumbs{{
        {8, 5, 5},
        {2, 5, 5},
        {5, 8, 5},
        {5, 2, 5},
        {5, 5, 8},
    }};
    for (const auto& crumb : crumbs) {
        output_map.set(gridPoint(crumb[0], crumb[1], crumb[2], config),
                       ct::VoxelOccupancy::Unmapped);
    }

    Impl algorithm{common::MappingAlgorithmDependencies{
        makeMissionConfig(), makeLidarConfig(), makeDroneConfig(), output_map}};

    ct::DroneState state{
        gridPoint(5, 5, 5, config), Orientation{0.0 * deg, 0.0 * deg}, 0};
    const auto apply_movement = [](ct::DroneState& current,
                                   const ct::MovementCommand& movement) {
        switch (movement.type) {
        case ct::MovementCommandType::Hover:
            break;
        case ct::MovementCommandType::Rotate: {
            const double delta =
                movement.angle.force_numerical_value_in(deg) *
                ((movement.rotation == ct::RotationDirection::Left) ? 1.0 : -1.0);
            current.heading.horizontal =
                (current.heading.horizontal.force_numerical_value_in(deg) + delta) * deg;
            break;
        }
        case ct::MovementCommandType::Advance: {
            const double distance = movement.distance.force_numerical_value_in(cm);
            const double heading_radians =
                current.heading.horizontal.force_numerical_value_in(deg) *
                (std::numbers::pi / 180.0);
            current.position = Position3D{
                (current.position.x.force_numerical_value_in(cm) +
                 distance * std::cos(heading_radians)) *
                    x_extent[cm],
                (current.position.y.force_numerical_value_in(cm) +
                 distance * std::sin(heading_radians)) *
                    y_extent[cm],
                current.position.z,
            };
            break;
        }
        case ct::MovementCommandType::Elevate:
            current.position = Position3D{
                current.position.x,
                current.position.y,
                (current.position.z.force_numerical_value_in(cm) +
                 movement.distance.force_numerical_value_in(cm)) *
                    z_extent[cm],
            };
            break;
        }
    };

    constexpr int kStepsPastLowRateLimit = 20;
    for (int step = 0; step < kStepsPastLowRateLimit; ++step) {
        const ct::MappingStepCommand command = algorithm.nextStep(state, nullptr);
        ASSERT_EQ(command.status, ct::AlgorithmStatus::Working)
            << "finished at step " << step << " after queued low-rate runner-ups";
        if (command.movement) {
            apply_movement(state, *command.movement);
        }
        state.step_index = static_cast<std::size_t>(step + 1);
    }
}

TEST(MappingAlgorithm, FinishesAfterConsecutiveLowRateReplans) {
    const ct::MapConfig config = makeCorridorConfig();
    Map output_map{{11, 11, 11}, config, ct::VoxelOccupancy::Empty};
    // Isolated leftover far from start so travel+reserve keeps rate below 0.25.
    output_map.set(gridPoint(9, 9, 9, config), ct::VoxelOccupancy::Unmapped);
    fillStartBubble(output_map, 1, 1, 1, config);

    const auto mc = makeMissionConfig();
    const auto lc = makeLidarConfig();
    const auto dc = makeDroneConfig();
    Impl algorithm{common::MappingAlgorithmDependencies{mc, lc, dc, output_map}};

    const ct::DroneState state{gridPoint(1, 1, 1, config), Orientation{0.0 * deg, 0.0 * deg}, 0};
    const ct::AlgorithmStatus status = runUntilTerminal(algorithm, state, 200);
    EXPECT_EQ(status, ct::AlgorithmStatus::FinishedWithUnmappableVoxels);
}

TEST(MappingAlgorithm, AbandonsPlanWhenTargetClusterIsResolved) {
    const ct::MapConfig config = makeCorridorConfig();
    Map output_map{{11, 11, 11}, config, ct::VoxelOccupancy::Empty};
    fillEmptyBox(output_map, 0, 8, 0, 10, 0, 10, config);
    output_map.set(gridPoint(10, 5, 5, config), ct::VoxelOccupancy::Unmapped);

    auto lc = makeLidarConfig();
    lc.z_max = 40.0 * cm;
    Impl algorithm{common::MappingAlgorithmDependencies{
        makeMissionConfig(), lc, makeDroneConfig(), output_map}};

    const ct::DroneState state{gridPoint(2, 5, 5, config), Orientation{0.0 * deg, 0.0 * deg}, 0};
    const auto first = algorithm.nextStep(state, nullptr);
    ASSERT_EQ(first.status, ct::AlgorithmStatus::Working);

    // Resolve the only unknown. The stored cluster is no longer a frontier.
    output_map.set(gridPoint(10, 5, 5, config), ct::VoxelOccupancy::Empty);

    ct::AlgorithmStatus status = ct::AlgorithmStatus::Working;
    for (int i = 0; i < 30 && status == ct::AlgorithmStatus::Working; ++i) {
        status = algorithm.nextStep(state, nullptr).status;
    }
    EXPECT_NE(status, ct::AlgorithmStatus::Working);
}
