// test_nbv_planner.cpp — NBV objective, feasibility filter, determinism.

#include "FakeMap3D.h"
#include "NbvPlanner.h"

#include <user_common_207190406_209543255/LidarCone.h>

#include <gtest/gtest.h>

#include <cstdint>
#include <unordered_set>

namespace detail = algorithm_207190406_209543255::detail;
namespace lc = user_common_207190406_209543255::lidar_cone;
using Map = AlgorithmTest::FakeMap3D;

namespace {

using common::Orientation;
using common::Position3D;
using common::cm;
using common::deg;
using common::x_extent;
using common::y_extent;
using common::z_extent;
namespace ct = common::types;

[[nodiscard]] ct::MapConfig makeConfig() {
    ct::MapConfig config{};
    config.resolution = 10.0 * cm;
    config.offset = Position3D{};
    config.boundaries.min_x = 0.0 * x_extent[cm];
    config.boundaries.max_x = 200.0 * x_extent[cm];
    config.boundaries.min_y = 0.0 * y_extent[cm];
    config.boundaries.max_y = 200.0 * y_extent[cm];
    config.boundaries.min_height = 0.0 * z_extent[cm];
    config.boundaries.max_height = 200.0 * z_extent[cm];
    return config;
}

[[nodiscard]] ct::LidarConfigData makeLidar() {
    ct::LidarConfigData cfg{};
    cfg.z_min = 20.0 * cm;
    cfg.z_max = 80.0 * cm;
    cfg.d = 2.5 * cm;
    cfg.fov_circles = 4;
    return cfg;
}

[[nodiscard]] ct::DroneConfigData makeDrone() {
    ct::DroneConfigData cfg{};
    cfg.radius = 4.0 * cm;
    cfg.max_rotate = 90.0 * deg;
    cfg.max_advance = 30.0 * cm;
    cfg.max_elevate = 20.0 * cm;
    return cfg;
}

[[nodiscard]] Position3D at(double x, double y, double z) {
    return Position3D{x * x_extent[cm], y * y_extent[cm], z * z_extent[cm]};
}

[[nodiscard]] ct::DroneState stateAt(const Position3D& p, std::size_t step_index = 0) {
    return ct::DroneState{p, Orientation{0.0 * deg, 0.0 * deg}, step_index};
}

} // namespace

TEST(NbvPlanner, PlansTowardTheUnexploredSideOfTheMap) {
    Map map{{21, 21, 21}, makeConfig(), ct::VoxelOccupancy::Empty};
    // Unmapped pocket far along +X only.
    for (int z = 0; z <= 2; ++z) {
        for (int y = 8; y <= 12; ++y) {
            map.set(at(180.0, y * 10.0, z * 10.0), ct::VoxelOccupancy::Unmapped);
        }
    }

    const detail::NbvPlanner planner;
    const ct::DroneState state = stateAt(at(50.0, 100.0, 0.0));
    const detail::BlockedCells blocked;
    const detail::ExplorationPlan plan = planner.plan(
        {map, state, makeLidar(), makeDrone(), 1000, blocked, false});

    ASSERT_TRUE(plan.valid);
    ASSERT_FALSE(plan.waypoints.empty());
    EXPECT_GT(plan.expected_gain, 0.0);
    // Movement is toward +X, where the only unresolved space is.
    EXPECT_GT(plan.waypoints.back().x.force_numerical_value_in(cm), 50.0);
}

TEST(NbvPlanner, ScansInPlaceWhenTheCurrentPoseHasGain) {
    Map map{{21, 21, 21}, makeConfig(), ct::VoxelOccupancy::Unmapped};
    // Everything unknown, as at spawn: the best viewpoint is right here.
    const detail::NbvPlanner planner;
    const ct::DroneState state = stateAt(at(100.0, 100.0, 100.0));
    const detail::BlockedCells blocked;
    const detail::ExplorationPlan plan = planner.plan(
        {map, state, makeLidar(), makeDrone(), 1000, blocked, false});

    ASSERT_TRUE(plan.valid);
    EXPECT_TRUE(plan.waypoints.empty());
    EXPECT_FALSE(plan.terminal_scans.empty());
    EXPECT_GT(plan.expected_gain, 0.0);
}

TEST(NbvPlanner, DiscardsCandidatesTheRemainingBudgetCannotAfford) {
    Map map{{21, 21, 21}, makeConfig(), ct::VoxelOccupancy::Empty};
    for (int y = 8; y <= 12; ++y) {
        map.set(at(200.0, y * 10.0, 0.0), ct::VoxelOccupancy::Unmapped);
    }

    const detail::NbvPlanner planner;
    const ct::DroneState state = stateAt(at(0.0, 100.0, 0.0));
    const detail::BlockedCells blocked;

    const detail::ExplorationPlan rich = planner.plan(
        {map, state, makeLidar(), makeDrone(), 1000, blocked, false});
    const detail::ExplorationPlan poor = planner.plan(
        {map, state, makeLidar(), makeDrone(), 2, blocked, false});

    EXPECT_TRUE(rich.valid);
    // Two steps cannot reach 200 cm away, so no travelling plan survives the filter.
    if (poor.valid) {
        EXPECT_TRUE(poor.waypoints.empty());
    }
}

TEST(NbvPlanner, IsDeterministicAcrossIdenticalCalls) {
    Map map{{21, 21, 21}, makeConfig(), ct::VoxelOccupancy::Empty};
    for (int y = 8; y <= 12; ++y) {
        map.set(at(180.0, y * 10.0, 0.0), ct::VoxelOccupancy::Unmapped);
    }

    const detail::NbvPlanner planner;
    const ct::DroneState state = stateAt(at(50.0, 100.0, 0.0));
    const detail::BlockedCells blocked;
    const detail::ExplorationPlan a = planner.plan(
        {map, state, makeLidar(), makeDrone(), 1000, blocked, false});
    const detail::ExplorationPlan b = planner.plan(
        {map, state, makeLidar(), makeDrone(), 1000, blocked, false});

    ASSERT_EQ(a.waypoints.size(), b.waypoints.size());
    for (std::size_t i = 0; i < a.waypoints.size(); ++i) {
        EXPECT_DOUBLE_EQ(a.waypoints[i].x.force_numerical_value_in(cm),
                         b.waypoints[i].x.force_numerical_value_in(cm));
        EXPECT_DOUBLE_EQ(a.waypoints[i].y.force_numerical_value_in(cm),
                         b.waypoints[i].y.force_numerical_value_in(cm));
        EXPECT_DOUBLE_EQ(a.waypoints[i].z.force_numerical_value_in(cm),
                         b.waypoints[i].z.force_numerical_value_in(cm));
    }
    EXPECT_DOUBLE_EQ(a.expected_gain, b.expected_gain);
}

TEST(NbvPlanner, ReportsInvalidWhenNothingIsUnresolved) {
    Map map{{21, 21, 21}, makeConfig(), ct::VoxelOccupancy::Empty};
    const detail::NbvPlanner planner;
    const ct::DroneState state = stateAt(at(100.0, 100.0, 100.0));
    const detail::BlockedCells blocked;
    const detail::ExplorationPlan plan = planner.plan(
        {map, state, makeLidar(), makeDrone(), 1000, blocked, false});

    EXPECT_FALSE(plan.valid);
    EXPECT_DOUBLE_EQ(plan.expected_gain, 0.0);
}

TEST(NbvPlanner, InPlaceExpectedGainUsesOnlyAffordableScanPrefix) {
    Map map{{21, 21, 21}, makeConfig(), ct::VoxelOccupancy::Unmapped};
    const detail::NbvPlanner planner;
    const Position3D origin = at(100.0, 100.0, 100.0);
    const ct::DroneState state = stateAt(origin);
    const detail::BlockedCells blocked;
    const ct::LidarConfigData lidar = makeLidar();
    constexpr std::size_t remaining_steps = 1;

    const detail::ExplorationPlan plan =
        planner.plan({map, state, lidar, makeDrone(), remaining_steps, blocked, false});

    ASSERT_TRUE(plan.valid);
    EXPECT_TRUE(plan.waypoints.empty());
    ASSERT_EQ(plan.terminal_scans.size(), remaining_steps);

    const Orientation world_heading{};
    std::unordered_set<std::int64_t> seen;
    double prefix_gain = 0.0;
    for (const Orientation& dir : plan.terminal_scans) {
        prefix_gain += static_cast<double>(
            lc::countUnresolvedVoxels(map, origin, world_heading, dir, lidar, seen));
    }
    EXPECT_DOUBLE_EQ(plan.expected_gain, prefix_gain);

    const double full_gain = detail::NbvPlanner::gainAt(map, origin, lidar, nullptr);
    EXPECT_LT(plan.expected_gain, full_gain);
}

TEST(NbvPlanner, IgnoreBlockedRecoversWhenTheBlockedSetSealsTheDrone) {
    Map map{{21, 21, 21}, makeConfig(), ct::VoxelOccupancy::Empty};
    for (int y = 8; y <= 12; ++y) {
        map.set(at(180.0, y * 10.0, 0.0), ct::VoxelOccupancy::Unmapped);
    }
    const Position3D start = at(100.0, 100.0, 0.0);

    // Block every 6-neighbour of the start cell.
    detail::BlockedCells blocked;
    const ct::MapConfig config = makeConfig();
    for (const Position3D& nb : {at(110.0, 100.0, 0.0), at(90.0, 100.0, 0.0),
                                 at(100.0, 110.0, 0.0), at(100.0, 90.0, 0.0),
                                 at(100.0, 100.0, 10.0), at(100.0, 100.0, -10.0)}) {
        blocked.insert(detail::quantizePosition(nb, config));
    }

    const detail::NbvPlanner planner;
    const ct::DroneState state = stateAt(start);
    const detail::ExplorationPlan sealed = planner.plan(
        {map, state, makeLidar(), makeDrone(), 1000, blocked, false});
    const detail::ExplorationPlan recovered = planner.plan(
        {map, state, makeLidar(), makeDrone(), 1000, blocked, true});

    EXPECT_TRUE(sealed.waypoints.empty());
    EXPECT_TRUE(recovered.valid);
}
