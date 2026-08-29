// test_mapping_algorithm_frontier.cpp — MappingAlgorithm.*
// Direct tests for internal MappingAlgorithmFrontier BFS.
// Ported from Drone-Mapper-ex2; uses FakeMap3D instead of Map3DImpl+TinyNPY.

#include "FakeMap3D.h"

#include <gtest/gtest.h>

// Internal header — Algorithm/src is added to include dirs by CMake.
#include "MappingAlgorithmFrontier.h"

#include <algorithm>
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
    const detail::PlanningDiagnostics diag = frontier.diagnose(map, start, 5.0 * cm);
    EXPECT_TRUE(diag.start_passable);
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
    const detail::PlanningDiagnostics diag =
        frontier.diagnose(map, pointCm(0, 0, 50), 5.0 * cm);
    EXPECT_FALSE(diag.start_passable);
}

// What: centre cell is Occupied.
// Expected: start is not passable.
TEST(MappingAlgorithm, FrontierStartNotPassableWhenCentreOccupied) {
    const ct::MapConfig config = makeWideConfig();
    Map map{{101, 101, 101}, config};
    map.set(pointCm(0, 0, 50), ct::VoxelOccupancy::Occupied);

    const detail::MappingAlgorithmFrontier frontier;
    const detail::PlanningDiagnostics diag =
        frontier.diagnose(map, pointCm(0, 0, 50), 5.0 * cm);
    EXPECT_FALSE(diag.start_passable);
}

// What: straight empty corridor ending before unknown space along +X.
// Expected: path reaches a frontier; density scoring may prefer a farther target than nearest.
TEST(MappingAlgorithm, FrontierFindsPathAlongEmptyCorridor) {
    const ct::MapConfig config = makeNarrowCorridorConfig();
    Map map{{11, 3, 3}, config};
    fillEmptyBox(map, 0, 2, 0, 2, 0, 2, config);

    const detail::MappingAlgorithmFrontier frontier;
    const detail::FrontierPathResult result =
        frontier.findPath(map, pointCm(0, 0, 1), 0.0 * cm);

    ASSERT_TRUE(result.found);
    ASSERT_GE(result.path.size(), 2U);
    EXPECT_GE(result.path.back().x.force_numerical_value_in(cm), 2.0);
}

// What: large empty cube with a 5 cm drone radius.
// Expected: BFS reaches a frontier on the cube face (path may be longer than nearest).
TEST(MappingAlgorithm, FrontierFindsFrontierInsideEmptyCube) {
    const ct::MapConfig config = makeWideConfig();
    Map map{{101, 101, 101}, config};
    fillEmptyCube(map, 20, 20, 50, 8, config);

    const Position3D start = pointCm(20, 20, 50);
    const detail::MappingAlgorithmFrontier frontier;
    const detail::FrontierPathResult result = frontier.findPath(map, start, 5.0 * cm);

    ASSERT_TRUE(result.found);
    ASSERT_FALSE(result.path.empty());
    EXPECT_GE(result.path.back().x.force_numerical_value_in(cm), 15.0);
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
// Expected: findExplorePath returns a non-empty path toward unknown space.
TEST(MappingAlgorithm, FrontierFindExplorePathMovesTowardUnknown) {
    const ct::MapConfig config = makeNarrowCorridorConfig();
    Map map{{11, 3, 3}, config};
    fillEmptyBox(map, 0, 8, 0, 2, 0, 2, config);

    const detail::MappingAlgorithmFrontier frontier;
    const detail::FrontierPathResult explore =
        frontier.findExplorePath(map, pointCm(0, 0, 1), 0.0 * cm);

    ASSERT_TRUE(explore.found);
    ASSERT_FALSE(explore.path.empty());
    EXPECT_GT(explore.path.back().x.force_numerical_value_in(cm), 0.0);
}

// What: corridor with no reachable frontier at zero radius but distant unknown.
// Expected: diagnose reports connectivity and explore_path_available.
TEST(MappingAlgorithm, FrontierDiagnoseReportsConnectivityMetrics) {
    const ct::MapConfig config = makeNarrowCorridorConfig();
    Map map{{11, 3, 3}, config};
    fillEmptyBox(map, 0, 8, 0, 2, 0, 2, config);

    const detail::MappingAlgorithmFrontier frontier;
    const detail::PlanningDiagnostics diag =
        frontier.diagnose(map, pointCm(0, 0, 1), 0.0 * cm);

    EXPECT_TRUE(diag.start_passable);
    EXPECT_GT(diag.passable_reached, 0U);
    EXPECT_GE(diag.nearest_unknown_steps, 0);
    EXPECT_TRUE(diag.explore_path_available);
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
    const detail::PlanningDiagnostics diag = frontier.diagnose(map, centre, 7.5 * cm);
    EXPECT_FALSE(diag.start_passable);
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
    const detail::PlanningDiagnostics diag = frontier.diagnose(map, centre, 4.0 * cm);
    EXPECT_TRUE(diag.start_passable);
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
