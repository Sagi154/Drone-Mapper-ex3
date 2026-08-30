// test_scan_planning.cpp — frontier mask, sweep order, pass-2 gate.

#include "FakeMap3D.h"
#include "MappingAlgorithmFrontier.h"
#include "ScanPlanning.h"

#include <user_common_207190406_209543255/ConeTemplate.h>

#include <gtest/gtest.h>

namespace detail = algorithm_207190406_209543255::detail;
namespace ctpl = user_common_207190406_209543255::cone_template;
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
    config.boundaries.max_x = 100.0 * x_extent[cm];
    config.boundaries.min_y = 0.0 * y_extent[cm];
    config.boundaries.max_y = 100.0 * y_extent[cm];
    config.boundaries.min_height = 0.0 * z_extent[cm];
    config.boundaries.max_height = 100.0 * z_extent[cm];
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

[[nodiscard]] Position3D at(double x, double y, double z) {
    return Position3D{x * x_extent[cm], y * y_extent[cm], z * z_extent[cm]};
}

} // namespace

TEST(ScanPlanning, MaskRejectsUnmappedBehindOccupied) {
    [[maybe_unused]] const ct::MapConfig config = makeConfig();
    const detail::GridKey free{5, 5, 5};
    const detail::GridKey wall{6, 5, 5};
    const detail::GridKey behind{7, 5, 5};
    detail::FrontierCells frontier;
    frontier.insert(free);
    EXPECT_TRUE(detail::isGainMasked(free, frontier));
    EXPECT_TRUE(detail::isGainMasked(wall, frontier));  // face-adjacent
    EXPECT_FALSE(detail::isGainMasked(behind, frontier));
}

TEST(ScanPlanning, SweepOrdersByIndependentGainNotEnumerationOrder) {
    const ct::MapConfig config = makeConfig();
    Map map{{11, 11, 11}, config, ct::VoxelOccupancy::Empty};
    // Unmapped only along +Z, so +Z must rank first even though axes[+Z] is index 4.
    for (int z = 6; z <= 10; ++z) {
        map.set(at(50.0, 50.0, z * 10.0), ct::VoxelOccupancy::Unmapped);
    }
    const Position3D origin = at(50.0, 50.0, 50.0);
    map.set(origin, ct::VoxelOccupancy::Empty);

    const detail::MappingAlgorithmFrontier frontier;
    const auto reach = frontier.exploreReachable(
        map, origin, 4.0 * cm, {}, 1, detail::maxExpansionsForMap(map));

    ctpl::ConeTemplateCache cache;
    const auto& templates = cache.get(makeLidar(), config.resolution);
    ctpl::VoxelStamp stamp;
    const auto dirs = detail::buildSweepDirections(
        map, origin, makeLidar(), reach.frontier_cells, templates, stamp);

    ASSERT_FALSE(dirs.empty());
    EXPECT_NEAR(dirs.front().altitude.force_numerical_value_in(deg), 90.0, 1e-6);
}

TEST(ScanPlanning, MarginalPassDropsFullyClaimedDirections) {
    const ct::MapConfig config = makeConfig();
    Map map{{11, 11, 11}, config, ct::VoxelOccupancy::Empty};
    map.set(at(60.0, 50.0, 50.0), ct::VoxelOccupancy::Unmapped);
    const Position3D origin = at(50.0, 50.0, 50.0);

    const detail::MappingAlgorithmFrontier frontier;
    const auto reach = frontier.exploreReachable(
        map, origin, 4.0 * cm, {}, 1, detail::maxExpansionsForMap(map));

    ctpl::ConeTemplateCache cache;
    const auto& templates = cache.get(makeLidar(), config.resolution);
    ctpl::VoxelStamp stamp;
    const auto dirs = detail::buildSweepDirections(
        map, origin, makeLidar(), reach.frontier_cells, templates, stamp);

    EXPECT_FALSE(dirs.empty());
    EXPECT_LT(dirs.size(), templates.size());
}

TEST(ScanPlanning, TravelScanRejectsDirectionWithOccupiedNearField) {
    const ct::MapConfig config = makeConfig();
    Map map{{11, 11, 11}, config, ct::VoxelOccupancy::Empty};
    const Position3D origin = at(50.0, 50.0, 50.0);
    const Position3D ahead = at(60.0, 50.0, 50.0);
    map.set(ahead, ct::VoxelOccupancy::Occupied);

    detail::FrontierCells frontier;
    frontier.insert(detail::quantizePosition(origin, config));

    ctpl::ConeTemplateCache cache;
    const auto& templates = cache.get(makeLidar(), config.resolution);
    ctpl::VoxelStamp stamp;
    ct::DroneState predicted{origin, Orientation{0.0 * deg, 0.0 * deg}, 0};
    const auto scan = detail::bestTravelScan(
        map, predicted, at(80.0, 50.0, 50.0), makeLidar(), frontier, templates, stamp);

    // +X is blocked in the near field; ±Z have no masked Unmapped. Omit the scan.
    EXPECT_FALSE(scan.has_value());
}
