// test_cone_template.cpp — ConeTemplate walk equals the trig oracle.

#include "FakeMap3D.h"

#include <user_common_207190406_209543255/ConeTemplate.h>
#include <user_common_207190406_209543255/LidarCone.h>

#include <gtest/gtest.h>

#include <cstdint>
#include <unordered_set>
#include <vector>

namespace lc = user_common_207190406_209543255::lidar_cone;
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

[[nodiscard]] ct::LidarConfigData makeShortLidar() {
    ct::LidarConfigData cfg{};
    cfg.z_min = 20.0 * cm;
    cfg.z_max = 80.0 * cm;
    cfg.d = 2.5 * cm;
    cfg.fov_circles = 4;
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

[[nodiscard]] std::unordered_set<std::int64_t> trigKeys(
    const Map& map, const Position3D& origin, const Orientation& dir,
    const ct::LidarConfigData& lidar) {
    std::unordered_set<std::int64_t> seen;
    (void)lc::countUnresolvedVoxels(map, origin, Orientation{}, dir, lidar, seen);
    return seen;
}

[[nodiscard]] std::unordered_set<std::int64_t> templateKeys(
    const ctpl::detail::ConeTemplate& cone, const Map& map, const Position3D& origin) {
    ctpl::VoxelStamp stamp;
    stamp.begin(map.getMapConfig(), origin, 80.0 * cm);
    std::unordered_set<std::int64_t> keys;
    const auto config = map.getMapConfig();
    (void)ctpl::walkTemplate(cone, map, origin, stamp, [&](const Position3D& p) {
        keys.insert(lc::voxelKey(config, p));
        return true;
    });
    return keys;
}

} // namespace

TEST(ConeTemplate, WalkMatchesTrigVoxelSetOnAlignedOrigin) {
    const ct::MapConfig config = makeSmallMapConfig();
    Map map{{11, 11, 11}, config, ct::VoxelOccupancy::Unmapped};
    const Position3D origin{50.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]};
    const ct::LidarConfigData lidar = makeShortLidar();

    ctpl::ConeTemplateCache cache;
    const auto& templates = cache.get(lidar, config.resolution);
    ASSERT_FALSE(templates.empty());

    const auto trig = trigKeys(map, origin, templates.front().direction, lidar);
    const auto tmpl = templateKeys(templates.front(), map, origin);
    EXPECT_EQ(trig, tmpl);
}

TEST(ConeTemplate, WalkMatchesTrigVoxelSetOnNonAlignedOrigin) {
    const ct::MapConfig config = makeSmallMapConfig();
    Map map{{11, 11, 11}, config, ct::VoxelOccupancy::Unmapped};
    const Position3D origin{53.0 * x_extent[cm], 47.0 * y_extent[cm], 51.0 * z_extent[cm]};
    const ct::LidarConfigData lidar = makeShortLidar();

    ctpl::ConeTemplateCache cache;
    const auto& templates = cache.get(lidar, config.resolution);
    ASSERT_GE(templates.size(), 6u);

    for (std::size_t i = 0; i < 6; ++i) {
        EXPECT_EQ(trigKeys(map, origin, templates[i].direction, lidar),
                  templateKeys(templates[i], map, origin))
            << "axis " << i;
    }
}

TEST(ConeTemplate, WalkStopsAtOccupied) {
    const ct::MapConfig config = makeSmallMapConfig();
    Map map{{11, 11, 11}, config, ct::VoxelOccupancy::Empty};
    const Position3D origin{50.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]};
    map.set(Position3D{60.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]},
            ct::VoxelOccupancy::Occupied);
    map.set(Position3D{70.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]},
            ct::VoxelOccupancy::Unmapped);

    ct::LidarConfigData lidar = makeShortLidar();
    lidar.fov_circles = 1;
    lidar.z_max = 40.0 * cm;

    ctpl::ConeTemplateCache cache;
    const auto& templates = cache.get(lidar, config.resolution);
    ASSERT_FALSE(templates.empty());

    EXPECT_EQ(trigKeys(map, origin, Orientation{}, lidar),
              templateKeys(templates.front(), map, origin));
    EXPECT_TRUE(templateKeys(templates.front(), map, origin).empty());
}

TEST(ConeTemplate, StampDeduplicatesAcrossBeamsAndGenerations) {
    const ct::MapConfig config = makeSmallMapConfig();
    Map map{{11, 11, 11}, config, ct::VoxelOccupancy::Unmapped};
    const Position3D origin{50.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]};
    const ct::LidarConfigData lidar = makeShortLidar();

    ctpl::ConeTemplateCache cache;
    const auto& templates = cache.get(lidar, config.resolution);
    ctpl::VoxelStamp stamp;
    stamp.begin(config, origin, lidar.z_max);

    std::size_t first = 0;
    (void)ctpl::walkTemplate(templates.front(), map, origin, stamp,
                             [&](const Position3D&) {
                                 ++first;
                                 return true;
                             });
    std::size_t second = 0;
    (void)ctpl::walkTemplate(templates.front(), map, origin, stamp,
                             [&](const Position3D&) {
                                 ++second;
                                 return true;
                             });
    EXPECT_GT(first, 0u);
    EXPECT_EQ(second, 0u);

    stamp.begin(config, origin, lidar.z_max);
    std::size_t third = 0;
    (void)ctpl::walkTemplate(templates.front(), map, origin, stamp,
                             [&](const Position3D&) {
                                 ++third;
                                 return true;
                             });
    EXPECT_EQ(third, first);
}

TEST(ConeTemplate, NearFieldSamplesCoverExactlyInsideZMin) {
    const ct::LidarConfigData lidar = makeShortLidar();
    const ct::MapConfig config = makeSmallMapConfig();
    ctpl::ConeTemplateCache cache;
    const auto& templates = cache.get(lidar, config.resolution);
    ASSERT_FALSE(templates.empty());

    const double step = templates.front().step.numerical_value_in(common::cm);
    ASSERT_GT(step, 0.0);
    std::size_t expected = 0;
    for (double dist = step; dist < 20.0 - 1e-9; dist += step) {
        ++expected;
    }
    EXPECT_EQ(templates.front().near_field_samples, expected);
}

TEST(ConeTemplate, CacheReturnsSameTemplatesForSameLidarAndResolution) {
    const ct::LidarConfigData lidar = makeShortLidar();
    ctpl::ConeTemplateCache cache;
    const auto& a = cache.get(lidar, 10.0 * cm);
    const auto& b = cache.get(lidar, 10.0 * cm);
    EXPECT_EQ(&a, &b);
    EXPECT_EQ(a.size(), lc::fibonacciSphereOrientations(
                            lc::directionCountForHalfAngle(lc::coneHalfAngleRad(lidar)))
                            .size());
}

TEST(ConeTemplate, NearFieldContainsSolidWhenOccupiedInsideZMin) {
    const ct::MapConfig config = makeSmallMapConfig();
    Map map{{11, 11, 11}, config, ct::VoxelOccupancy::Empty};
    const Position3D origin{50.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]};
    map.set(Position3D{60.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]},
            ct::VoxelOccupancy::Occupied);  // 10 cm, inside z_min = 20 cm

    ct::LidarConfigData lidar = makeShortLidar();
    lidar.fov_circles = 1;
    ctpl::ConeTemplateCache cache;
    const auto& templates = cache.get(lidar, config.resolution);
    EXPECT_TRUE(ctpl::nearFieldContainsSolid(templates.front(), map, origin));
}
