// test_wavefront_planner.cpp — cluster ranking, budget filter, determinism.

#include "FakeMap3D.h"
#include "ScanPlanning.h"
#include "WavefrontPlanner.h"

#include <gtest/gtest.h>

namespace detail = algorithm_207190406_209543255::detail;
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

void fillUnmappedBox(Map& map, int x0, int x1, int y0, int y1, int z0, int z1) {
    for (int x = x0; x <= x1; ++x) {
        for (int y = y0; y <= y1; ++y) {
            for (int z = z0; z <= z1; ++z) {
                map.set(at(x * 10.0, y * 10.0, z * 10.0), ct::VoxelOccupancy::Unmapped);
            }
        }
    }
}

} // namespace

TEST(WavefrontPlanner, PrefersDistantRoomOverNearbyCrumb) {
    Map map{{21, 21, 21}, makeConfig(), ct::VoxelOccupancy::Empty};
    // Crumb: three Unmapped cells one step along +Y.
    map.set(at(50.0, 110.0, 0.0), ct::VoxelOccupancy::Unmapped);
    map.set(at(50.0, 120.0, 0.0), ct::VoxelOccupancy::Unmapped);
    map.set(at(50.0, 130.0, 0.0), ct::VoxelOccupancy::Unmapped);
    // Distant room: Unmapped at +X. {21,21,21} at 10 cm covers 0..200 cm (index 20).
    fillUnmappedBox(map, 14, 20, 6, 13, 0, 5);

    const detail::WavefrontPlanner planner;
    const ct::DroneState state = stateAt(at(50.0, 100.0, 0.0));
    const detail::BlockedCells blocked;
    const detail::ExplorationPlan plan = planner.plan(
        {map, state, makeLidar(), makeDrone(), 1000, blocked, false});

    ASSERT_TRUE(plan.valid);
    ASSERT_FALSE(plan.waypoints.empty());
    EXPECT_GT(plan.waypoints.back().x.force_numerical_value_in(cm), 100.0);
    EXPECT_GT(plan.target_cluster_cells, 20u);
}

TEST(WavefrontPlanner, DescendsFromCeilingWhenUnmappedIsBelow) {
    ct::MapConfig config = makeConfig();
    config.boundaries.max_height = 100.0 * z_extent[cm];
    Map map{{21, 21, 11}, config, ct::VoxelOccupancy::Unmapped};
    const Position3D start = at(50.0, 50.0, 100.0);
    map.set(start, ct::VoxelOccupancy::Empty);

    const detail::WavefrontPlanner planner;
    const detail::BlockedCells blocked;
    const detail::ExplorationPlan plan = planner.plan(
        {map, stateAt(start), makeLidar(), makeDrone(), 1000, blocked, false});

    ASSERT_TRUE(plan.valid);
    ASSERT_FALSE(plan.waypoints.empty());
    EXPECT_LT(plan.waypoints.front().z.force_numerical_value_in(cm), 100.0);
}

TEST(WavefrontPlanner, DescendsThroughHouseWhenUnmappedIsBelowMidLayer) {
    ct::MapConfig config{};
    config.resolution = 10.0 * cm;
    config.offset = Position3D{};
    config.boundaries.min_x = 0.0 * x_extent[cm];
    config.boundaries.max_x = 290.0 * x_extent[cm];
    config.boundaries.min_y = 0.0 * y_extent[cm];
    config.boundaries.max_y = 300.0 * y_extent[cm];
    config.boundaries.min_height = 0.0 * z_extent[cm];
    config.boundaries.max_height = 150.0 * z_extent[cm];
    ASSERT_TRUE(detail::isHouseVolumeMission(config));
    EXPECT_FALSE(detail::isOpenVolumeMission(config));

    Map map{{30, 31, 16}, config, ct::VoxelOccupancy::Unmapped};
    const Position3D start = at(50.0, 50.0, 140.0);
    map.set(start, ct::VoxelOccupancy::Empty);

    const detail::WavefrontPlanner planner;
    const detail::BlockedCells blocked;
    const detail::ExplorationPlan plan = planner.plan(
        {map, stateAt(start), makeLidar(), makeDrone(), 1000, blocked, false});

    ASSERT_TRUE(plan.valid);
    ASSERT_FALSE(plan.waypoints.empty());
    EXPECT_LT(plan.waypoints.front().z.force_numerical_value_in(cm), 140.0);
}

TEST(WavefrontPlanner, DescendsThroughHouseEmptyColumnTowardUnmapped) {
    ct::MapConfig config{};
    config.resolution = 10.0 * cm;
    config.offset = Position3D{};
    config.boundaries.min_x = 0.0 * x_extent[cm];
    config.boundaries.max_x = 290.0 * x_extent[cm];
    config.boundaries.min_y = 0.0 * y_extent[cm];
    config.boundaries.max_y = 300.0 * y_extent[cm];
    config.boundaries.min_height = 0.0 * z_extent[cm];
    config.boundaries.max_height = 150.0 * z_extent[cm];

    Map map{{30, 31, 16}, config, ct::VoxelOccupancy::Empty};
    const Position3D start = at(50.0, 50.0, 80.0);
    map.set(at(50.0, 50.0, 50.0), ct::VoxelOccupancy::Unmapped);
    map.set(at(50.0, 50.0, 40.0), ct::VoxelOccupancy::Unmapped);

    const detail::WavefrontPlanner planner;
    const detail::BlockedCells blocked;
    const detail::ExplorationPlan plan = planner.plan(
        {map, stateAt(start), makeLidar(), makeDrone(), 1000, blocked, false});

    ASSERT_TRUE(plan.valid);
    ASSERT_FALSE(plan.waypoints.empty());
    EXPECT_LT(plan.waypoints.front().z.force_numerical_value_in(cm), 80.0);
}

TEST(WavefrontPlanner, HouseStaysToScanWhenHorizontalUnmappedRemains) {
    ct::MapConfig config{};
    config.resolution = 10.0 * cm;
    config.offset = Position3D{};
    config.boundaries.min_x = 0.0 * x_extent[cm];
    config.boundaries.max_x = 290.0 * x_extent[cm];
    config.boundaries.min_y = 0.0 * y_extent[cm];
    config.boundaries.max_y = 300.0 * y_extent[cm];
    config.boundaries.min_height = 0.0 * z_extent[cm];
    config.boundaries.max_height = 150.0 * z_extent[cm];

    Map map{{30, 31, 16}, config, ct::VoxelOccupancy::Empty};
    const Position3D start = at(50.0, 50.0, 80.0);
    map.set(at(60.0, 50.0, 80.0), ct::VoxelOccupancy::Unmapped);
    map.set(at(50.0, 50.0, 50.0), ct::VoxelOccupancy::Unmapped);

    const detail::WavefrontPlanner planner;
    const detail::BlockedCells blocked;
    const detail::ExplorationPlan plan = planner.plan(
        {map, stateAt(start), makeLidar(), makeDrone(), 1000, blocked, false});

    ASSERT_TRUE(plan.valid);
    EXPECT_TRUE(plan.waypoints.empty());
}

TEST(WavefrontPlanner, HouseDropsOnPreferDescendDespiteHorizontalUnmapped) {
    ct::MapConfig config{};
    config.resolution = 10.0 * cm;
    config.offset = Position3D{};
    config.boundaries.min_x = 0.0 * x_extent[cm];
    config.boundaries.max_x = 290.0 * x_extent[cm];
    config.boundaries.min_y = 0.0 * y_extent[cm];
    config.boundaries.max_y = 300.0 * y_extent[cm];
    config.boundaries.min_height = 0.0 * z_extent[cm];
    config.boundaries.max_height = 150.0 * z_extent[cm];

    Map map{{30, 31, 16}, config, ct::VoxelOccupancy::Empty};
    const Position3D start = at(50.0, 50.0, 80.0);
    map.set(at(60.0, 50.0, 80.0), ct::VoxelOccupancy::Unmapped);
    map.set(at(50.0, 50.0, 50.0), ct::VoxelOccupancy::Unmapped);

    const detail::WavefrontPlanner planner;
    const detail::BlockedCells blocked;
    const detail::ExplorationPlan plan = planner.plan(
        {map, stateAt(start), makeLidar(), makeDrone(), 1000, blocked, false, true});

    ASSERT_TRUE(plan.valid);
    ASSERT_FALSE(plan.waypoints.empty());
    EXPECT_LT(plan.waypoints.front().z.force_numerical_value_in(cm), 80.0);
}

TEST(WavefrontPlanner, RoomDoesNotForceDescendWhenUnmappedIsBelow) {
    ct::MapConfig config = makeConfig();
    config.boundaries.max_x = 200.0 * x_extent[cm];
    config.boundaries.max_y = 110.0 * y_extent[cm];
    config.boundaries.max_height = 90.0 * z_extent[cm];
    EXPECT_FALSE(detail::isHouseVolumeMission(config));
    EXPECT_FALSE(detail::isOpenVolumeMission(config));

    Map map{{21, 12, 10}, config, ct::VoxelOccupancy::Unmapped};
    const Position3D start = at(50.0, 100.0, 20.0);
    map.set(start, ct::VoxelOccupancy::Empty);

    const detail::WavefrontPlanner planner;
    const detail::BlockedCells blocked;
    const detail::ExplorationPlan plan = planner.plan(
        {map, stateAt(start), makeLidar(), makeDrone(), 1000, blocked, false});

    ASSERT_TRUE(plan.valid);
    EXPECT_TRUE(plan.waypoints.empty());
}

TEST(WavefrontPlanner, StaysPutWhenStartIsTheCheapestFrontier) {
    Map map{{21, 21, 21}, makeConfig(), ct::VoxelOccupancy::Unmapped};
    const detail::WavefrontPlanner planner;
    const ct::DroneState state = stateAt(at(100.0, 100.0, 100.0));
    const detail::BlockedCells blocked;
    const detail::ExplorationPlan plan = planner.plan(
        {map, state, makeLidar(), makeDrone(), 1000, blocked, false});

    ASSERT_TRUE(plan.valid);
    EXPECT_TRUE(plan.waypoints.empty());
    EXPECT_GT(plan.target_cluster_cells, 0u);
    EXPECT_GT(plan.expected_rate, 0.0);
}

TEST(WavefrontPlanner, DiscardsClustersTheRemainingBudgetCannotAfford) {
    Map map{{21, 21, 21}, makeConfig(), ct::VoxelOccupancy::Empty};
    fillUnmappedBox(map, 18, 20, 8, 12, 0, 2);

    const detail::WavefrontPlanner planner;
    const ct::DroneState state = stateAt(at(0.0, 100.0, 0.0));
    const detail::BlockedCells blocked;

    const detail::ExplorationPlan rich = planner.plan(
        {map, state, makeLidar(), makeDrone(), 1000, blocked, false});
    const detail::ExplorationPlan poor = planner.plan(
        {map, state, makeLidar(), makeDrone(), 2, blocked, false});

    EXPECT_TRUE(rich.valid);
    EXPECT_FALSE(rich.waypoints.empty());
    if (poor.valid) {
        EXPECT_TRUE(poor.waypoints.empty());
    }
}

TEST(WavefrontPlanner, IsDeterministicAcrossIdenticalCalls) {
    Map map{{21, 21, 21}, makeConfig(), ct::VoxelOccupancy::Empty};
    fillUnmappedBox(map, 16, 20, 8, 12, 0, 2);

    const detail::WavefrontPlanner planner;
    const ct::DroneState state = stateAt(at(50.0, 100.0, 0.0));
    const detail::BlockedCells blocked;
    const auto a = planner.plan({map, state, makeLidar(), makeDrone(), 1000, blocked, false});
    const auto b = planner.plan({map, state, makeLidar(), makeDrone(), 1000, blocked, false});

    ASSERT_EQ(a.waypoints.size(), b.waypoints.size());
    for (std::size_t i = 0; i < a.waypoints.size(); ++i) {
        EXPECT_DOUBLE_EQ(a.waypoints[i].x.force_numerical_value_in(cm),
                         b.waypoints[i].x.force_numerical_value_in(cm));
        EXPECT_DOUBLE_EQ(a.waypoints[i].y.force_numerical_value_in(cm),
                         b.waypoints[i].y.force_numerical_value_in(cm));
        EXPECT_DOUBLE_EQ(a.waypoints[i].z.force_numerical_value_in(cm),
                         b.waypoints[i].z.force_numerical_value_in(cm));
    }
    EXPECT_EQ(a.target_cluster_cells, b.target_cluster_cells);
    EXPECT_DOUBLE_EQ(a.expected_rate, b.expected_rate);
    ASSERT_EQ(a.target_keys.size(), b.target_keys.size());
}

TEST(WavefrontPlanner, UnsticksWhenStartSphereHitsOccupiedFloor) {
    Map map{{21, 21, 21}, makeConfig(), ct::VoxelOccupancy::Empty};
    fillUnmappedBox(map, 16, 20, 8, 12, 2, 5);
    const Position3D start = at(50.0, 100.0, 50.0);
    for (int x = 0; x <= 20; ++x) {
        for (int y = 0; y <= 20; ++y) {
            map.set(at(x * 10.0, y * 10.0, 40.0), ct::VoxelOccupancy::Occupied);
        }
    }

    ct::DroneConfigData drone = makeDrone();
    drone.radius = 7.5 * cm;

    const detail::WavefrontPlanner planner;
    const detail::BlockedCells blocked;
    const detail::ExplorationPlan plan = planner.plan(
        {map, stateAt(start), makeLidar(), drone, 1000, blocked, false});

    ASSERT_TRUE(plan.valid);
    ASSERT_FALSE(plan.waypoints.empty());
    EXPECT_GT(plan.waypoints.front().z.force_numerical_value_in(cm), 50.0);
    EXPECT_GE(plan.expected_rate, 0.25);
}

TEST(WavefrontPlanner, ReportsInvalidWhenNothingIsUnresolved) {
    Map map{{21, 21, 21}, makeConfig(), ct::VoxelOccupancy::Empty};
    const detail::WavefrontPlanner planner;
    const ct::DroneState state = stateAt(at(100.0, 100.0, 100.0));
    const detail::BlockedCells blocked;
    const detail::ExplorationPlan plan = planner.plan(
        {map, state, makeLidar(), makeDrone(), 1000, blocked, false});

    EXPECT_FALSE(plan.valid);
    EXPECT_EQ(plan.target_cluster_cells, 0u);
}

TEST(WavefrontPlanner, IgnoreBlockedRecoversWhenTheBlockedSetSealsTheDrone) {
    Map map{{21, 21, 21}, makeConfig(), ct::VoxelOccupancy::Empty};
    fillUnmappedBox(map, 16, 20, 8, 12, 0, 2);
    const Position3D start = at(100.0, 100.0, 0.0);

    detail::BlockedCells blocked;
    const ct::MapConfig config = makeConfig();
    for (const Position3D& nb : {at(110.0, 100.0, 0.0), at(90.0, 100.0, 0.0),
                                 at(100.0, 110.0, 0.0), at(100.0, 90.0, 0.0),
                                 at(100.0, 100.0, 10.0), at(100.0, 100.0, -10.0)}) {
        blocked.insert(detail::quantizePosition(nb, config));
    }

    const detail::WavefrontPlanner planner;
    const ct::DroneState state = stateAt(start);
    const auto sealed = planner.plan({map, state, makeLidar(), makeDrone(), 1000, blocked, false});
    const auto recovered = planner.plan({map, state, makeLidar(), makeDrone(), 1000, blocked, true});

    EXPECT_TRUE(sealed.waypoints.empty());
    EXPECT_TRUE(recovered.valid);
}
