// test_lidar_cone.cpp — UserCommon lidar_cone helpers.

#include "FakeMap3D.h"

#include <user_common_207190406_209543255/LidarCone.h>

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

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

[[nodiscard]] ct::LidarConfigData makeShortLidar() {
    ct::LidarConfigData cfg{};
    cfg.z_min = 20.0 * cm;
    cfg.z_max = 80.0 * cm;
    cfg.d = 2.5 * cm;
    cfg.fov_circles = 4;
    return cfg;
}

[[nodiscard]] ct::LidarConfigData makeLongLidar() {
    ct::LidarConfigData cfg{};
    cfg.z_min = 20.0 * cm;
    cfg.z_max = 150.0 * cm;
    cfg.d = 2.5 * cm;
    cfg.fov_circles = 3;
    return cfg;
}

[[nodiscard]] ct::MapConfig makeSmallMapConfig() {
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

} // namespace

TEST(LidarCone, HalfAngleMatchesMockLidarShortAndLong) {
    EXPECT_NEAR(lc::coneHalfAngleRad(makeShortLidar()), std::atan2(7.5, 20.0), 1e-9);
    EXPECT_NEAR(lc::coneHalfAngleRad(makeLongLidar()), std::atan2(5.0, 20.0), 1e-9);
}

TEST(LidarCone, DirectionCountsDifferBetweenShortAndLong) {
    const std::size_t n_short =
        lc::directionCountForHalfAngle(lc::coneHalfAngleRad(makeShortLidar()));
    const std::size_t n_long =
        lc::directionCountForHalfAngle(lc::coneHalfAngleRad(makeLongLidar()));
    EXPECT_GE(n_short, 6u);
    EXPECT_LE(n_short, 64u);
    EXPECT_GE(n_long, 6u);
    EXPECT_LE(n_long, 64u);
    EXPECT_NE(n_short, n_long);
    EXPECT_NE(n_short, 26u);
}

TEST(LidarCone, FibonacciSizeAndDeterminism) {
    const auto a = lc::fibonacciSphereOrientations(17);
    const auto b = lc::fibonacciSphereOrientations(17);
    ASSERT_EQ(a.size(), 17u);
    ASSERT_EQ(b.size(), 17u);
    for (std::size_t i = 0; i < a.size(); ++i) {
        EXPECT_DOUBLE_EQ(a[i].horizontal.numerical_value_in(deg),
                         b[i].horizontal.numerical_value_in(deg));
        EXPECT_DOUBLE_EQ(a[i].altitude.numerical_value_in(deg),
                         b[i].altitude.numerical_value_in(deg));
    }
    // First six are axis-aligned.
    EXPECT_DOUBLE_EQ(a[0].horizontal.numerical_value_in(deg), 0.0);
    EXPECT_DOUBLE_EQ(a[0].altitude.numerical_value_in(deg), 0.0);
}

TEST(LidarCone, ConeCoversUnresolvedWhenUnmappedAlongBeam) {
    const ct::MapConfig config = makeSmallMapConfig();
    Map map{{11, 11, 11}, config, ct::VoxelOccupancy::Empty};
    const Position3D origin{50.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]};
    map.set(Position3D{70.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]},
            ct::VoxelOccupancy::Unmapped);

    ct::LidarConfigData cfg = makeShortLidar();
    cfg.z_max = 40.0 * cm;
    cfg.fov_circles = 1; // centre beam only

    const Orientation heading{};
    const Orientation scan{}; // +X in world when heading is zero
    EXPECT_TRUE(lc::coneCoversUnresolved(map, origin, heading, scan, cfg));
}

TEST(LidarCone, ConeDoesNotCoverWhenFullyResolved) {
    const ct::MapConfig config = makeSmallMapConfig();
    Map map{{11, 11, 11}, config, ct::VoxelOccupancy::Empty};
    const Position3D origin{50.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]};

    ct::LidarConfigData cfg = makeShortLidar();
    cfg.z_max = 40.0 * cm;
    cfg.fov_circles = 1;

    const Orientation heading{};
    const Orientation scan{};
    EXPECT_FALSE(lc::coneCoversUnresolved(map, origin, heading, scan, cfg));
}
