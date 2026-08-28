// test_simulator_mocks.cpp
// Covers MockGPS, MockMovement, and MockLidar.
//
// Key mandatory scenario verified here:
//   MockMovement::advance/elevate throws std::runtime_error when the
//   destination sphere overlaps an Occupied voxel (or leaves map bounds).

#include <Simulator/MockGPS.h>
#include <Simulator/MockLidar.h>
#include <Simulator/MockMovement.h>

#include <Common/IMap3D.h>

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <vector>

using namespace common;
using namespace common::types;

// ---------------------------------------------------------------------------
// Minimal IMap3D implementation for testing
// ---------------------------------------------------------------------------

/// 3-D boolean grid, origin at (0,0,0), uniform resolution.
struct GridMap final : public IMap3D {
    int width_cells;
    int height_cells;
    int depth_cells;
    double res_cm;
    std::vector<bool> occupied;

    GridMap(int w, int h, int d, double res)
        : width_cells(w), height_cells(h), depth_cells(d), res_cm(res),
          occupied(static_cast<std::size_t>(w * h * d), false) {}

    void setOccupied(int xi, int yi, int zi) {
        const auto idx = static_cast<std::size_t>(zi * height_cells * width_cells +
                                                   yi * width_cells + xi);
        occupied[idx] = true;
    }

    [[nodiscard]] bool isInBounds(const Position3D& pos) const override {
        const double x = pos.x.numerical_value_in(cm);
        const double y = pos.y.numerical_value_in(cm);
        const double z = pos.z.numerical_value_in(cm);
        return x >= 0.0 && x < static_cast<double>(width_cells)  * res_cm &&
               y >= 0.0 && y < static_cast<double>(height_cells) * res_cm &&
               z >= 0.0 && z < static_cast<double>(depth_cells)  * res_cm;
    }

    [[nodiscard]] VoxelOccupancy atVoxel(const Position3D& pos) const override {
        if (!isInBounds(pos)) {
            return VoxelOccupancy::OutOfBounds;
        }
        const int xi = static_cast<int>(pos.x.numerical_value_in(cm) / res_cm);
        const int yi = static_cast<int>(pos.y.numerical_value_in(cm) / res_cm);
        const int zi = static_cast<int>(pos.z.numerical_value_in(cm) / res_cm);
        const auto idx = static_cast<std::size_t>(zi * height_cells * width_cells +
                                                   yi * width_cells + xi);
        return occupied[idx] ? VoxelOccupancy::Occupied : VoxelOccupancy::Empty;
    }

    [[nodiscard]] MapConfig getMapConfig() const override {
        return MapConfig{
            .boundaries = {},
            .offset     = {},
            .resolution = res_cm * cm,
        };
    }
};

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static DroneConfigData makeDroneConfig(double radius_cm = 5.0,
                                       double max_advance_cm = 200.0,
                                       double max_elevate_cm = 200.0,
                                       double max_rotate_deg = 360.0) {
    return DroneConfigData{
        .radius     = radius_cm * cm,
        .max_rotate = max_rotate_deg * deg,
        .max_advance = max_advance_cm * cm,
        .max_elevate = max_elevate_cm * cm,
    };
}

// ---------------------------------------------------------------------------
// MockGPS tests
// ---------------------------------------------------------------------------

TEST(MockGPS, ReturnsInitialPositionAndHeading) {
    const Position3D  pos{50.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]};
    const Orientation hdg{10.0 * deg, 0.0 * deg};
    simulator::MockGPS gps{pos, hdg, 10.0 * cm};

    EXPECT_DOUBLE_EQ(gps.position().x.numerical_value_in(cm), 50.0);
    EXPECT_DOUBLE_EQ(gps.heading().horizontal.numerical_value_in(deg), 10.0);
}

TEST(MockGPS, SetPositionSnapsToResolution) {
    const Position3D  pos{0.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]};
    simulator::MockGPS gps{pos, {}, 10.0 * cm};

    // 14 cm → snaps to 10 cm (nearest multiple of 10)
    gps.setPosition(Position3D{14.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]});
    EXPECT_DOUBLE_EQ(gps.position().x.numerical_value_in(cm), 10.0);

    // 16 cm → snaps to 20 cm
    gps.setPosition(Position3D{16.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]});
    EXPECT_DOUBLE_EQ(gps.position().x.numerical_value_in(cm), 20.0);
}

TEST(MockGPS, ZeroResolutionDoesNotSnap) {
    simulator::MockGPS gps{{}, {}, 0.0 * cm};
    gps.setPosition(Position3D{13.7 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]});
    EXPECT_DOUBLE_EQ(gps.position().x.numerical_value_in(cm), 13.7);
}

// ---------------------------------------------------------------------------
// MockMovement tests
// ---------------------------------------------------------------------------

TEST(MockMovement, AdvanceLegalMoveSucceeds) {
    // 10x10x10 grid, 10 cm/cell, no walls — advance 50 cm forward (+X)
    GridMap map{10, 10, 10, 10.0};
    simulator::MockGPS gps{
        Position3D{50.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]},
        Orientation{0.0 * deg, 0.0 * deg},
        10.0 * cm};
    simulator::MockMovement mv{gps, map, makeDroneConfig(5.0, 200.0)};

    const auto result = mv.advance(30.0 * cm);
    EXPECT_TRUE(result.success);
    EXPECT_NEAR(gps.position().x.numerical_value_in(cm), 80.0, 1e-9);
}

TEST(MockMovement, AdvanceIntoWallThrows) {
    // 10x10x10 grid, 10 cm/cell; place wall at cell (9, 5, 5) = x∈[90,100)
    GridMap map{10, 10, 10, 10.0};
    map.setOccupied(9, 5, 5);

    simulator::MockGPS gps{
        Position3D{50.0 * x_extent[cm], 55.0 * y_extent[cm], 55.0 * z_extent[cm]},
        Orientation{0.0 * deg, 0.0 * deg},
        10.0 * cm};
    simulator::MockMovement mv{gps, map, makeDroneConfig(5.0, 200.0)};

    // Advance 50 cm from x=50 → would land at x=100, which overlaps the wall
    EXPECT_THROW(mv.advance(50.0 * cm), std::runtime_error);
}

TEST(MockMovement, AdvanceAdjacentToWallDoesNotThrow) {
    // Wall at x=90 cell; start at x=50, advance only 30 cm → stops at x=80, clear
    GridMap map{10, 10, 10, 10.0};
    map.setOccupied(9, 5, 5);

    simulator::MockGPS gps{
        Position3D{50.0 * x_extent[cm], 55.0 * y_extent[cm], 55.0 * z_extent[cm]},
        Orientation{0.0 * deg, 0.0 * deg},
        10.0 * cm};
    simulator::MockMovement mv{gps, map, makeDroneConfig(5.0, 200.0)};

    EXPECT_NO_THROW(mv.advance(30.0 * cm));
}

TEST(MockMovement, ElevateIntoWallThrows) {
    GridMap map{10, 10, 10, 10.0};
    map.setOccupied(5, 5, 9); // ceiling at z=90..100

    // 50.0 cm snaps exactly to cell 5 at 10 cm resolution — stays on the wall column
    simulator::MockGPS gps{
        Position3D{50.0 * x_extent[cm], 50.0 * y_extent[cm], 40.0 * z_extent[cm]},
        Orientation{0.0 * deg, 0.0 * deg},
        10.0 * cm};
    simulator::MockMovement mv{gps, map, makeDroneConfig(5.0, 200.0, 200.0)};

    EXPECT_THROW(mv.elevate(50.0 * cm), std::runtime_error);
}

TEST(MockMovement, AdvanceExceedsLimitReturnsFalse) {
    GridMap map{20, 20, 20, 10.0};
    simulator::MockGPS gps{
        Position3D{100.0 * x_extent[cm], 100.0 * y_extent[cm], 100.0 * z_extent[cm]},
        Orientation{0.0 * deg, 0.0 * deg},
        10.0 * cm};
    simulator::MockMovement mv{gps, map, makeDroneConfig(5.0, 50.0)};

    // 60 cm exceeds max_advance=50 cm
    const auto result = mv.advance(60.0 * cm);
    EXPECT_FALSE(result.success);
}

TEST(MockMovement, RotateLeftUpdatesHeading) {
    GridMap map{10, 10, 10, 10.0};
    simulator::MockGPS gps{
        Position3D{50.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]},
        Orientation{0.0 * deg, 0.0 * deg},
        10.0 * cm};
    simulator::MockMovement mv{gps, map, makeDroneConfig()};

    mv.rotate(RotationDirection::Left, 45.0 * deg);
    EXPECT_NEAR(gps.heading().horizontal.numerical_value_in(deg), 45.0, 1e-9);
}

TEST(MockMovement, RotateRightUpdatesHeading) {
    GridMap map{10, 10, 10, 10.0};
    simulator::MockGPS gps{
        Position3D{50.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]},
        Orientation{0.0 * deg, 0.0 * deg},
        10.0 * cm};
    simulator::MockMovement mv{gps, map, makeDroneConfig()};

    mv.rotate(RotationDirection::Right, 30.0 * deg);
    EXPECT_NEAR(gps.heading().horizontal.numerical_value_in(deg), -30.0, 1e-9);
}

// ---------------------------------------------------------------------------
// MockLidar tests
// ---------------------------------------------------------------------------

TEST(MockLidar, ZeroCirclesReturnsEmptyResult) {
    GridMap map{10, 10, 10, 10.0};
    simulator::MockGPS gps{
        Position3D{50.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]},
        {},
        10.0 * cm};
    LidarConfigData cfg{.z_min = 10.0 * cm, .z_max = 200.0 * cm, .d = 5.0 * cm, .fov_circles = 0};
    simulator::MockLidar lidar{cfg, map, gps};

    const auto result = lidar.scan({});
    EXPECT_TRUE(result.empty());
}

TEST(MockLidar, SingleCircleReturnsCenterBeam) {
    GridMap map{20, 20, 20, 10.0};
    simulator::MockGPS gps{
        Position3D{100.0 * x_extent[cm], 100.0 * y_extent[cm], 100.0 * z_extent[cm]},
        {},
        10.0 * cm};
    // Wall straight ahead at x=190
    map.setOccupied(18, 10, 10);

    LidarConfigData cfg{.z_min = 10.0 * cm, .z_max = 500.0 * cm, .d = 5.0 * cm, .fov_circles = 1};
    simulator::MockLidar lidar{cfg, map, gps};

    const auto result = lidar.scan(Orientation{0.0 * deg, 0.0 * deg});
    ASSERT_EQ(result.size(), 1u);
    // Distance to wall at x=180 (cell centre) from x=100 ≈ 80 cm
    EXPECT_GT(result[0].distance.numerical_value_in(cm), 0.0);
}

TEST(MockLidar, ConfigGetterMatchesConstructorArg) {
    GridMap map{10, 10, 10, 10.0};
    simulator::MockGPS gps{{}, {}, 10.0 * cm};
    LidarConfigData cfg{.z_min = 5.0 * cm, .z_max = 300.0 * cm, .d = 10.0 * cm, .fov_circles = 3};
    simulator::MockLidar lidar{cfg, map, gps};

    const auto got = lidar.config();
    EXPECT_DOUBLE_EQ(got.z_min.numerical_value_in(cm), 5.0);
    EXPECT_DOUBLE_EQ(got.z_max.numerical_value_in(cm), 300.0);
    EXPECT_EQ(got.fov_circles, 3u);
}

TEST(MockLidar, ExtremeScanOrientationCompletes) {
    // VAR-03 adversarial angles must not hang MockLidar (trig on ~1e12 deg).
    GridMap map{10, 10, 10, 10.0};
    simulator::MockGPS gps{
        Position3D{50.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]},
        {},
        10.0 * cm};
    LidarConfigData cfg{.z_min = 10.0 * cm, .z_max = 80.0 * cm, .d = 2.5 * cm, .fov_circles = 2};
    simulator::MockLidar lidar{cfg, map, gps};

    constexpr double kExtreme = 1.0e12;
    const auto result = lidar.scan(Orientation{kExtreme * deg, -kExtreme * deg});
    EXPECT_FALSE(result.empty());
}
