// test_mapping_algorithm_frontier.cpp — MappingAlgorithm.*
// Direct tests for internal MappingAlgorithmFrontier BFS.
// Ported from Drone-Mapper-ex2; uses FakeMap3D instead of Map3DImpl+TinyNPY.

#include "FakeMap3D.h"

#include <gtest/gtest.h>

// Internal header — Algorithm/src is added to include dirs by CMake.
#include "MappingAlgorithmFrontier.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>

namespace detail = algorithm_207190406_209543255::detail;
using Map        = AlgorithmTest::FakeMap3D;

namespace {

using common::Position3D;
using common::cm;
using common::x_extent;
using common::y_extent;
using common::z_extent;
namespace ct = common::types;

// wide config: 101×101×101 voxels (x∈[-50,50], y∈[-50,50], z∈[0,100], step=1cm)
[[nodiscard]] ct::MapConfig makeWideConfig() {
    ct::MapConfig config{};
    config.resolution            = 1.0 * cm;
    config.offset                = Position3D{};
    config.boundaries.min_x      = -50.0 * x_extent[cm];
    config.boundaries.max_x      = 50.0 * x_extent[cm];
    config.boundaries.min_y      = -50.0 * y_extent[cm];
    config.boundaries.max_y      = 50.0 * y_extent[cm];
    config.boundaries.min_height = 0.0 * z_extent[cm];
    config.boundaries.max_height = 100.0 * z_extent[cm];
    return config;
}

// narrow corridor: 11×3×3 voxels (x∈[0,10], y∈[0,2], z∈[0,2], step=1cm)
[[nodiscard]] ct::MapConfig makeNarrowCorridorConfig() {
    ct::MapConfig config{};
    config.resolution            = 1.0 * cm;
    config.offset                = Position3D{};
    config.boundaries.min_x      = 0.0 * x_extent[cm];
    config.boundaries.max_x      = 10.0 * x_extent[cm];
    config.boundaries.min_y      = 0.0 * y_extent[cm];
    config.boundaries.max_y      = 2.0 * y_extent[cm];
    config.boundaries.min_height = 0.0 * z_extent[cm];
    config.boundaries.max_height = 2.0 * z_extent[cm];
    return config;
}

[[nodiscard]] Position3D pointCm(double x, double y, double z) {
    return Position3D{x * x_extent[cm], y * y_extent[cm], z * z_extent[cm]};
}

void fillEmptyBox(Map& map, int x0, int x1, int y0, int y1, int z0, int z1,
                  const ct::MapConfig& config) {
    const double step = config.resolution.force_numerical_value_in(cm);
    const double ox   = config.offset.x.force_numerical_value_in(cm);
    const double oy   = config.offset.y.force_numerical_value_in(cm);
    const double oz   = config.offset.z.force_numerical_value_in(cm);
    for (int x = x0; x <= x1; ++x) {
        for (int y = y0; y <= y1; ++y) {
            for (int z = z0; z <= z1; ++z) {
                map.set(Position3D{(ox + x * step) * x_extent[cm],
                                   (oy + y * step) * y_extent[cm],
                                   (oz + z * step) * z_extent[cm]},
                        ct::VoxelOccupancy::Empty);
            }
        }
    }
}

void fillEmptyCube(Map& map, int cx, int cy, int cz, int half,
                   const ct::MapConfig& config) {
    fillEmptyBox(map, cx - half, cx + half, cy - half, cy + half, cz - half, cz + half, config);
}

} // namespace

// What: start cell is Empty with Unmapped neighbours in the drone sphere.
// Expected: start is sphere-passable (Unmapped probes do not block navigation).
TEST(MappingAlgorithm, FrontierStartPassableWhenSphereHasUnmapped) {
    const ct::MapConfig config = makeWideConfig();
    // wide config: x∈[-50,50] (offset=0 → index = x+50), 101 slots each axis
    Map map{{101, 101, 101}, config};
    const Position3D start = pointCm(10, 10, 50);
    map.set(start, ct::VoxelOccupancy::Empty);

    const detail::MappingAlgorithmFrontier frontier;
    EXPECT_TRUE(frontier.exploreReachable(map, start, 5.0 * cm, {}, 1).start_passable);
}

// What: centre is Empty but the drone sphere overlaps Occupied voxels.
// Expected: start is not passable — confirmed walls block navigation.
TEST(MappingAlgorithm, FrontierStartNotPassableWhenSphereOverlapsOccupied) {
    const ct::MapConfig config = makeWideConfig();
    Map map{{101, 101, 101}, config};
    map.set(pointCm(0, 0, 50), ct::VoxelOccupancy::Empty);
    for (int dx = -5; dx <= 5; ++dx) {
        for (int dy = -5; dy <= 5; ++dy) {
            for (int dz = -5; dz <= 5; ++dz) {
                if (dx == 0 && dy == 0 && dz == 0) {
                    continue;
                }
                if (dx * dx + dy * dy + dz * dz > 25) {
                    continue;
                }
                map.set(pointCm(dx, dy, 50 + dz), ct::VoxelOccupancy::Occupied);
            }
        }
    }

    const detail::MappingAlgorithmFrontier frontier;
    EXPECT_FALSE(frontier.exploreReachable(map, pointCm(0, 0, 50), 5.0 * cm, {}, 1)
                     .start_passable);
}

// What: centre cell is Occupied.
// Expected: start is not passable.
TEST(MappingAlgorithm, FrontierStartNotPassableWhenCentreOccupied) {
    const ct::MapConfig config = makeWideConfig();
    Map map{{101, 101, 101}, config};
    map.set(pointCm(0, 0, 50), ct::VoxelOccupancy::Occupied);

    const detail::MappingAlgorithmFrontier frontier;
    EXPECT_FALSE(frontier.exploreReachable(map, pointCm(0, 0, 50), 5.0 * cm, {}, 1)
                     .start_passable);
}

// What: straight empty corridor ending before unknown space along +X.
// Expected: exploreReachable finds frontier-adjacent cells at the empty/unmapped interface.
TEST(MappingAlgorithm, FrontierFindsPathAlongEmptyCorridor) {
    const ct::MapConfig config = makeNarrowCorridorConfig();
    Map map{{11, 3, 3}, config};
    fillEmptyBox(map, 0, 2, 0, 2, 0, 2, config);

    const detail::MappingAlgorithmFrontier frontier;
    const detail::ReachabilityResult result = frontier.exploreReachable(
        map, pointCm(0, 0, 1), 0.0 * cm, {}, detail::maxExpansionsForMap(map));

    EXPECT_TRUE(result.start_passable);
    ASSERT_FALSE(result.clusters.empty());
    EXPECT_FALSE(result.frontier_cells.empty());
    bool reached_interface = false;
    for (const detail::FrontierCluster& cluster : result.clusters) {
        if (cluster.approach_position.x.force_numerical_value_in(cm) >= 2.0) {
            reached_interface = true;
            break;
        }
    }
    EXPECT_TRUE(reached_interface);
}

// What: large empty cube with a 5 cm drone radius.
// Expected: exploreReachable finds frontier-adjacent cells on the cube face.
TEST(MappingAlgorithm, FrontierFindsFrontierInsideEmptyCube) {
    const ct::MapConfig config = makeWideConfig();
    Map map{{101, 101, 101}, config};
    fillEmptyCube(map, 20, 20, 50, 8, config);

    const Position3D start = pointCm(20, 20, 50);
    const detail::MappingAlgorithmFrontier frontier;
    // Face is 8 cells from centre; do not walk the whole 101³ Unmapped volume.
    const detail::ReachabilityResult result =
        frontier.exploreReachable(map, start, 5.0 * cm, {}, 5000);

    EXPECT_TRUE(result.start_passable);
    ASSERT_FALSE(result.clusters.empty());
    EXPECT_FALSE(result.frontier_cells.empty());
    bool on_face = false;
    for (const detail::FrontierCluster& cluster : result.clusters) {
        const double dx = std::abs(cluster.approach_position.x.force_numerical_value_in(cm) - 20.0);
        const double dy = std::abs(cluster.approach_position.y.force_numerical_value_in(cm) - 20.0);
        const double dz = std::abs(cluster.approach_position.z.force_numerical_value_in(cm) - 50.0);
        if (dx >= 7.0 || dy >= 7.0 || dz >= 7.0) {
            on_face = true;
            break;
        }
    }
    EXPECT_TRUE(on_face);
}

// What: unstored cells exist inside the probe sphere.
// Expected: hasNotMappedInSphere returns true.
TEST(MappingAlgorithm, FrontierHasUnmappedInSphereWhenUnknownExists) {
    const ct::MapConfig config = makeWideConfig();
    Map map{{101, 101, 101}, config};
    map.set(pointCm(0, 0, 50), ct::VoxelOccupancy::Empty);

    EXPECT_TRUE(detail::hasNotMappedInSphere(map, pointCm(0, 0, 50), 2.0 * cm));
}

// What: every cell in the probe sphere is explicitly Empty.
// Expected: hasNotMappedInSphere returns false.
TEST(MappingAlgorithm, FrontierHasNoUnmappedInSphereWhenFullyKnown) {
    const ct::MapConfig config = makeWideConfig();
    Map map{{101, 101, 101}, config};
    fillEmptyCube(map, 0, 0, 50, 2, config);

    EXPECT_FALSE(detail::hasNotMappedInSphere(map, pointCm(0, 0, 50), 2.0 * cm));
}

// What: long corridor with unknown beyond fused range.
// Expected: exploreReachable reports frontier-adjacent cells toward unknown space.
TEST(MappingAlgorithm, FrontierFindExplorePathMovesTowardUnknown) {
    const ct::MapConfig config = makeNarrowCorridorConfig();
    Map map{{11, 3, 3}, config};
    fillEmptyBox(map, 0, 8, 0, 2, 0, 2, config);

    const detail::MappingAlgorithmFrontier frontier;
    const detail::ReachabilityResult result = frontier.exploreReachable(
        map, pointCm(0, 0, 1), 0.0 * cm, {}, detail::maxExpansionsForMap(map));

    EXPECT_TRUE(result.start_passable);
    ASSERT_FALSE(result.clusters.empty());
    EXPECT_FALSE(result.frontier_cells.empty());
    bool toward_unknown = false;
    for (const detail::FrontierCluster& cluster : result.clusters) {
        if (cluster.approach_position.x.force_numerical_value_in(cm) > 0.0) {
            toward_unknown = true;
            break;
        }
    }
    EXPECT_TRUE(toward_unknown);
}

// What: map still contains Unmapped voxels inside mission bounds.
// Expected: hasAnyNotMappedInBounds returns true.
TEST(MappingAlgorithm, FrontierDetectsUnmappedCellsInBounds) {
    const ct::MapConfig config = makeWideConfig();
    Map map{{101, 101, 101}, config};
    map.set(pointCm(0, 0, 50), ct::VoxelOccupancy::Empty);

    EXPECT_TRUE(detail::hasAnyNotMappedInBounds(map));
}

// What: every in-bounds cell is marked Empty.
// Expected: hasAnyNotMappedInBounds returns false.
TEST(MappingAlgorithm, FrontierNoUnmappedWhenFullyMappedEmpty) {
    const ct::MapConfig config = makeWideConfig();
    // Use a small 5×5×5 sub-region for speed; bounds must match so
    // countUnmappedInBounds only iterates those 125 voxels.
    ct::MapConfig small_cfg  = config;
    small_cfg.boundaries.min_x      = 0.0 * x_extent[cm];
    small_cfg.boundaries.max_x      = 4.0 * x_extent[cm];
    small_cfg.boundaries.min_y      = 0.0 * y_extent[cm];
    small_cfg.boundaries.max_y      = 4.0 * y_extent[cm];
    small_cfg.boundaries.min_height = 0.0 * z_extent[cm];
    small_cfg.boundaries.max_height = 4.0 * z_extent[cm];
    Map map{{5, 5, 5}, small_cfg};
    fillEmptyBox(map, 0, 4, 0, 4, 0, 4, small_cfg);

    EXPECT_FALSE(detail::hasAnyNotMappedInBounds(map));
}

// What: start-to-goal routing with a short Unmapped corridor and a longer Empty detour.
// Expected: weighted path search prefers the Empty route over the Unmapped shortcut.
TEST(MappingAlgorithm, FrontierPrefersEmptyOverUnmappedPath) {
    const ct::MapConfig config = makeNarrowCorridorConfig();
    Map map{{11, 3, 3}, config};

    // Empty detours on y=0 and y=2; middle row y=1 stays Unmapped except endpoints.
    fillEmptyBox(map, 0, 10, 0, 0, 0, 2, config);
    fillEmptyBox(map, 0, 10, 2, 2, 0, 2, config);
    const Position3D start = pointCm(0, 1, 1);
    const Position3D goal  = pointCm(5, 1, 1);
    map.set(start, ct::VoxelOccupancy::Empty);
    map.set(goal, ct::VoxelOccupancy::Empty);

    const detail::MappingAlgorithmFrontier frontier;
    const detail::FrontierPathResult result =
        frontier.findPathTo(map, start, goal, 0.0 * cm);

    ASSERT_TRUE(result.found);
    ASSERT_FALSE(result.path.empty());
    EXPECT_EQ(result.path.back().x.force_numerical_value_in(cm), 5.0);

    for (const Position3D& waypoint : result.path) {
        const double y = waypoint.y.force_numerical_value_in(cm);
        const double x = waypoint.x.force_numerical_value_in(cm);
        if (y >= 0.9 && y <= 1.1 && x >= 1.0 && x <= 4.0) {
            EXPECT_NE(map.atVoxel(waypoint), ct::VoxelOccupancy::Unmapped)
                << "path should avoid the Unmapped corridor at y=1";
        }
    }
}

[[nodiscard]] ct::MapConfig makeCm10Config() {
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

// What: 10 cm grid, radius 7.5 cm, Occupied face neighbour (nearest box dist 5 ≤ 7.5).
// Expected: start not passable — the old centre-distance gate silently skipped this probe.
TEST(MappingAlgorithm, FrontierRejectsOccupiedFaceNeighbourOnCm10Grid) {
    const ct::MapConfig config = makeCm10Config();
    Map map{{11, 11, 11}, config};
    const Position3D centre = pointCm(50, 50, 50);
    const Position3D face   = pointCm(60, 50, 50);
    map.set(centre, ct::VoxelOccupancy::Empty);
    map.set(face, ct::VoxelOccupancy::Occupied);

    const detail::MappingAlgorithmFrontier frontier;
    EXPECT_FALSE(frontier.exploreReachable(map, centre, 7.5 * cm, {}, 1).start_passable);
}

// What: same Occupied face neighbour but radius 4 cm (nearest 5 > 4).
// Expected: still passable — sphere does not reach the neighbour box.
TEST(MappingAlgorithm, FrontierAllowsOccupiedFaceNeighbourWhenRadiusTooSmall) {
    const ct::MapConfig config = makeCm10Config();
    Map map{{11, 11, 11}, config};
    const Position3D centre = pointCm(50, 50, 50);
    const Position3D face   = pointCm(60, 50, 50);
    map.set(centre, ct::VoxelOccupancy::Empty);
    map.set(face, ct::VoxelOccupancy::Occupied);

    const detail::MappingAlgorithmFrontier frontier;
    EXPECT_TRUE(frontier.exploreReachable(map, centre, 4.0 * cm, {}, 1).start_passable);
}

// What: face neighbour Unmapped, centre Empty, radius 7.5 on 10 cm grid.
// Expected: hasNotMappedInSphere sees the face cell (same geometry as passability).
TEST(MappingAlgorithm, FrontierHasUnmappedFaceNeighbourOnCm10Grid) {
    const ct::MapConfig config = makeCm10Config();
    Map map{{11, 11, 11}, config, ct::VoxelOccupancy::Empty};
    const Position3D centre = pointCm(50, 50, 50);
    const Position3D face   = pointCm(60, 50, 50);
    map.set(centre, ct::VoxelOccupancy::Empty);
    map.set(face, ct::VoxelOccupancy::Unmapped);

    EXPECT_TRUE(detail::hasNotMappedInSphere(map, centre, 7.5 * cm));
    EXPECT_FALSE(detail::hasNotMappedInSphere(map, centre, 4.0 * cm));
}

TEST(MappingAlgorithm, ExploreReachableFindsFrontierAdjacentCandidates) {
    const ct::MapConfig config = makeCm10Config();
    Map map{{11, 11, 11}, config, ct::VoxelOccupancy::Empty};
    map.set(pointCm(50, 50, 50), ct::VoxelOccupancy::Unmapped);

    const detail::MappingAlgorithmFrontier frontier;
    const Position3D start = pointCm(0, 0, 0);
    const detail::ReachabilityResult result = frontier.exploreReachable(
        map, start, 4.0 * cm, {}, detail::maxExpansionsForMap(map));

    EXPECT_TRUE(result.start_passable);
    EXPECT_FALSE(result.truncated);
    ASSERT_FALSE(result.clusters.empty());
    EXPECT_FALSE(result.frontier_cells.empty());
    for (const detail::FrontierCluster& cluster : result.clusters) {
        EXPECT_GT(cluster.cell_count, 0u);
        EXPECT_GT(cluster.approach_cost, 0);
    }
}

TEST(MappingAlgorithm, ExploreReachableRespectsExpansionCap) {
    const ct::MapConfig config = makeCm10Config();
    Map map{{11, 11, 11}, config, ct::VoxelOccupancy::Empty};
    const detail::MappingAlgorithmFrontier frontier;
    const Position3D start = pointCm(50, 50, 50);

    const detail::ReachabilityResult result =
        frontier.exploreReachable(map, start, 4.0 * cm, {}, 5);

    EXPECT_TRUE(result.truncated);
}

TEST(MappingAlgorithm, ExploreReachableReportsStartPassabilityWithCapOfOne) {
    // A cap of 1 makes this an O(1) start-passability probe — the replacement for
    // diagnose().start_passable, which task 6 deletes.
    const ct::MapConfig config = makeCm10Config();
    Map map{{11, 11, 11}, config};
    map.set(pointCm(50, 50, 50), ct::VoxelOccupancy::Empty);
    map.set(pointCm(60, 50, 50), ct::VoxelOccupancy::Occupied);

    const detail::MappingAlgorithmFrontier frontier;
    EXPECT_FALSE(frontier.exploreReachable(map, pointCm(50, 50, 50), 7.5 * cm, {}, 1)
                     .start_passable);
    EXPECT_TRUE(frontier.exploreReachable(map, pointCm(50, 50, 50), 4.0 * cm, {}, 1)
                    .start_passable);
}

TEST(MappingAlgorithm, LineOfSightBlockedByOccupiedVoxel) {
    const ct::MapConfig config = makeCm10Config();
    Map map{{11, 11, 11}, config, ct::VoxelOccupancy::Empty};
    const Position3D from = pointCm(0, 0, 0);
    const Position3D to = pointCm(50, 0, 0);

    EXPECT_TRUE(detail::hasClearLineOfSight(map, from, to, 4.0 * cm));

    map.set(pointCm(30, 0, 0), ct::VoxelOccupancy::Occupied);
    EXPECT_FALSE(detail::hasClearLineOfSight(map, from, to, 4.0 * cm));
}

TEST(MappingAlgorithm, MaxExpansionsCoversMapVolume) {
    const ct::MapConfig config = makeCm10Config();
    Map map{{11, 11, 11}, config, ct::VoxelOccupancy::Empty};
    // 0..100 cm inclusive at 10 cm on three axes = 11^3.
    EXPECT_GE(detail::maxExpansionsForMap(map), 11u * 11u * 11u);
}

TEST(MappingAlgorithm, ExploreReachableTerminatesWithoutOccupancyBound) {
    ct::MapConfig config = makeCm10Config();
    // Huge bounds, nothing Occupied: only the expansion cap can stop the walk.
    config.boundaries.max_x = 100000.0 * x_extent[cm];
    config.boundaries.max_y = 100000.0 * y_extent[cm];
    config.boundaries.max_height = 100000.0 * z_extent[cm];
    Map map{{10000, 10000, 10000}, config, ct::VoxelOccupancy::Unmapped};

    const detail::MappingAlgorithmFrontier frontier;
    const Position3D start = pointCm(0, 0, 0);
    const detail::ReachabilityResult result =
        frontier.exploreReachable(map, start, 4.0 * cm, {}, 20000);

    EXPECT_TRUE(result.truncated);
}

TEST(MappingAlgorithm, FindUnstickPathStepsToAdjacentEmpty) {
    const ct::MapConfig config = makeCm10Config();
    Map map{{11, 11, 11}, config, ct::VoxelOccupancy::Occupied};
    const Position3D start = pointCm(50, 50, 50);
    const Position3D neighbour = pointCm(60, 50, 50);
    map.set(start, ct::VoxelOccupancy::Occupied);
    map.set(neighbour, ct::VoxelOccupancy::Empty);

    const detail::MappingAlgorithmFrontier frontier;
    // Radius 4 cm: nearest Occupied box is 5 cm away, so the Empty neighbour is passable.
    const detail::FrontierPathResult result = frontier.findUnstickPath(map, start, 4.0 * cm);

    ASSERT_TRUE(result.found);
    ASSERT_EQ(result.path.size(), 1u);
    EXPECT_EQ(result.path.front().x.force_numerical_value_in(cm), 60.0);
    EXPECT_EQ(result.path.front().y.force_numerical_value_in(cm), 50.0);
    EXPECT_EQ(result.path.front().z.force_numerical_value_in(cm), 50.0);
}

TEST(MappingAlgorithm, FindUnstickPathBoxedInByOccupied) {
    const ct::MapConfig config = makeCm10Config();
    Map map{{11, 11, 11}, config, ct::VoxelOccupancy::Occupied};
    const Position3D start = pointCm(50, 50, 50);
    map.set(start, ct::VoxelOccupancy::Occupied);

    const detail::MappingAlgorithmFrontier frontier;
    const detail::FrontierPathResult result = frontier.findUnstickPath(map, start, 4.0 * cm);

    EXPECT_FALSE(result.found);
    EXPECT_TRUE(result.path.empty());
}

TEST(MappingAlgorithm, FindUnstickPathDoesNotTeleportThroughOccupied) {
    const ct::MapConfig config = makeCm10Config();
    Map map{{11, 11, 11}, config, ct::VoxelOccupancy::Occupied};
    const Position3D start = pointCm(50, 50, 50);
    const Position3D between = pointCm(60, 50, 50);
    const Position3D far = pointCm(70, 50, 50);
    map.set(start, ct::VoxelOccupancy::Occupied);
    map.set(between, ct::VoxelOccupancy::Occupied);
    map.set(far, ct::VoxelOccupancy::Empty);

    const detail::MappingAlgorithmFrontier frontier;
    const detail::FrontierPathResult result = frontier.findUnstickPath(map, start, 4.0 * cm);

    EXPECT_FALSE(result.found);
    EXPECT_TRUE(result.path.empty());
}

TEST(MappingAlgorithm, FindPathToIsExpansionBounded) {
    ct::MapConfig config = makeCm10Config();
    // FakeMap3D dims stay huge so occupancy / OutOfBounds cannot stop the walk.
    // Mission bounds stay at 0..100 cm (11^3 cells): findPathTo caps on
    // maxExpansionsForMap, unlike exploreReachable which takes an explicit cap.
    // Exploding max_* here would make the cap ~10^12 and the test would still hang.
    Map map{{10000, 10000, 10000}, config, ct::VoxelOccupancy::Unmapped};

    const detail::MappingAlgorithmFrontier frontier;
    const Position3D start{0.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]};
    const Position3D unreachable{99000.0 * x_extent[cm], 99000.0 * y_extent[cm],
                                 99000.0 * z_extent[cm]};

    // Must return, not hang. Result may be found or not; termination is the assertion.
    const detail::FrontierPathResult result =
        frontier.findPathTo(map, start, unreachable, 4.0 * cm, {});
    (void)result;
    SUCCEED();
}

TEST(MappingAlgorithm, ExploreReachableClustersTwoRoomsSeparatedByAWall) {
    const ct::MapConfig config = makeCm10Config();
    Map map{{11, 11, 11}, config, ct::VoxelOccupancy::Empty};
    // Occupied wall at x=50 splits the volume.
    for (int y = 0; y <= 10; ++y) {
        for (int z = 0; z <= 10; ++z) {
            map.set(pointCm(50, y * 10, z * 10), ct::VoxelOccupancy::Occupied);
        }
    }
    // Unknown pocket on the start side (x=20) and one beyond the wall (x=80).
    // The far pocket is unreachable (wall), so it must not join the near cluster.
    map.set(pointCm(20, 50, 50), ct::VoxelOccupancy::Unmapped);
    map.set(pointCm(80, 50, 50), ct::VoxelOccupancy::Unmapped);

    const detail::MappingAlgorithmFrontier frontier;
    const detail::ReachabilityResult result = frontier.exploreReachable(
        map, pointCm(0, 0, 0), 4.0 * cm, {}, detail::maxExpansionsForMap(map));

    EXPECT_TRUE(result.start_passable);
    ASSERT_FALSE(result.clusters.empty());
    // Only the near pocket is reachable. One cluster, not two.
    EXPECT_EQ(result.clusters.size(), 1u);
    EXPECT_GT(result.clusters.front().cell_count, 0u);
    EXPECT_EQ(result.clusters.front().cell_count,
              result.clusters.front().keys.size());
}

TEST(MappingAlgorithm, ExploreReachableClusterCountEqualsFrontierSet) {
    const ct::MapConfig config = makeCm10Config();
    Map map{{11, 11, 11}, config, ct::VoxelOccupancy::Empty};
    map.set(pointCm(50, 50, 50), ct::VoxelOccupancy::Unmapped);

    const detail::MappingAlgorithmFrontier frontier;
    const detail::ReachabilityResult result = frontier.exploreReachable(
        map, pointCm(0, 0, 0), 4.0 * cm, {}, detail::maxExpansionsForMap(map));

    std::size_t summed = 0;
    for (const detail::FrontierCluster& c : result.clusters) {
        summed += c.cell_count;
        EXPECT_EQ(c.cell_count, c.keys.size());
    }
    EXPECT_EQ(summed, result.frontier_cells.size());
    EXPECT_FALSE(result.frontier_cells.empty());
}

TEST(MappingAlgorithm, ExploreReachableApproachKeyIsLowestCostMember) {
    const ct::MapConfig config = makeCm10Config();
    Map map{{11, 11, 11}, config, ct::VoxelOccupancy::Empty};
    map.set(pointCm(80, 50, 50), ct::VoxelOccupancy::Unmapped);

    const detail::MappingAlgorithmFrontier frontier;
    const Position3D start = pointCm(0, 50, 50);
    const detail::ReachabilityResult result = frontier.exploreReachable(
        map, start, 4.0 * cm, {}, detail::maxExpansionsForMap(map));

    ASSERT_FALSE(result.clusters.empty());
    const detail::FrontierCluster& cluster = result.clusters.front();
    int min_cost = cluster.approach_cost;
    for (const detail::GridKey& key : cluster.keys) {
        const auto it = result.parent_of.find(key);
        ASSERT_TRUE(it != result.parent_of.end() || key == result.start_key);
        (void)it;
    }
    // The approach cell is on the near side of the pocket, not past it.
    EXPECT_LE(cluster.approach_position.x.force_numerical_value_in(cm), 80.0);
    EXPECT_GE(min_cost, 0);
}

TEST(MappingAlgorithm, FrontierFlagIgnoresDiagonalOnlyUnmapped) {
    const ct::MapConfig config = makeCm10Config();
    Map map{{11, 11, 11}, config, ct::VoxelOccupancy::Empty};
    // Only a diagonal neighbour is Unmapped — not a 6-face frontier.
    map.set(pointCm(60, 60, 60), ct::VoxelOccupancy::Unmapped);

    const detail::MappingAlgorithmFrontier frontier;
    const detail::ReachabilityResult result = frontier.exploreReachable(
        map, pointCm(50, 50, 50), 4.0 * cm, {}, detail::maxExpansionsForMap(map));

    // Start's only Unmapped neighbour is diagonal, so start is not a 6-face frontier.
    // The pocket's face neighbours (and the reachable Unmapped cell) still are.
    EXPECT_FALSE(result.frontier_cells.contains(result.start_key));
    EXPECT_FALSE(result.frontier_cells.empty());
    EXPECT_FALSE(result.clusters.empty());
}

TEST(MappingAlgorithm, ExploreReachableIncludesStartWhenItBordersUnmapped) {
    const ct::MapConfig config = makeCm10Config();
    Map map{{11, 11, 11}, config, ct::VoxelOccupancy::Unmapped};
    const Position3D start = pointCm(50, 50, 50);
    map.set(start, ct::VoxelOccupancy::Empty);

    const detail::MappingAlgorithmFrontier frontier;
    const detail::ReachabilityResult result = frontier.exploreReachable(
        map, start, 4.0 * cm, {}, detail::maxExpansionsForMap(map));

    ASSERT_FALSE(result.clusters.empty());
    bool start_is_approach = false;
    for (const detail::FrontierCluster& c : result.clusters) {
        if (c.approach_key == result.start_key) {
            start_is_approach = true;
        }
    }
    EXPECT_TRUE(start_is_approach);
    EXPECT_TRUE(result.frontier_cells.contains(result.start_key));
}

TEST(MappingAlgorithm, ExploreReachableClusteringIsDeterministic) {
    const ct::MapConfig config = makeCm10Config();
    Map map{{11, 11, 11}, config, ct::VoxelOccupancy::Empty};
    map.set(pointCm(80, 50, 50), ct::VoxelOccupancy::Unmapped);
    map.set(pointCm(80, 60, 50), ct::VoxelOccupancy::Unmapped);

    const detail::MappingAlgorithmFrontier frontier;
    const auto a = frontier.exploreReachable(
        map, pointCm(0, 0, 0), 4.0 * cm, {}, detail::maxExpansionsForMap(map));
    const auto b = frontier.exploreReachable(
        map, pointCm(0, 0, 0), 4.0 * cm, {}, detail::maxExpansionsForMap(map));

    ASSERT_EQ(a.clusters.size(), b.clusters.size());
    for (std::size_t i = 0; i < a.clusters.size(); ++i) {
        EXPECT_EQ(a.clusters[i].cell_count, b.clusters[i].cell_count);
        EXPECT_EQ(a.clusters[i].approach_cost, b.clusters[i].approach_cost);
        EXPECT_EQ(a.clusters[i].approach_key, b.clusters[i].approach_key);
    }
}
