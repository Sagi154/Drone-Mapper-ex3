// test_path_shaping.cpp — string-pulling and the command-level step-cost model.

#include "FakeMap3D.h"
#include "PathShaping.h"

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
    config.boundaries.max_x = 100.0 * x_extent[cm];
    config.boundaries.min_y = 0.0 * y_extent[cm];
    config.boundaries.max_y = 100.0 * y_extent[cm];
    config.boundaries.min_height = 0.0 * z_extent[cm];
    config.boundaries.max_height = 100.0 * z_extent[cm];
    return config;
}

[[nodiscard]] Position3D at(double x, double y, double z) {
    return Position3D{x * x_extent[cm], y * y_extent[cm], z * z_extent[cm]};
}

// drone_small: 30 cm advance, 20 cm elevate, 90 deg rotate.
[[nodiscard]] detail::MovementLimits smallDrone() {
    return {30.0 * cm, 20.0 * cm, 90.0 * deg};
}

// drone_large: 50 cm advance, 40 cm elevate, 45 deg rotate.
[[nodiscard]] detail::MovementLimits largeDrone() {
    return {50.0 * cm, 40.0 * cm, 45.0 * deg};
}

} // namespace

TEST(PathShaping, StringPullCollapsesStaircaseAtConstantAltitude) {
    Map map{{11, 11, 11}, makeConfig(), ct::VoxelOccupancy::Empty};
    const std::vector<Position3D> staircase{
        at(0, 0, 0), at(10, 0, 0), at(10, 10, 0), at(20, 10, 0), at(20, 20, 0),
    };

    const std::vector<Position3D> pulled =
        detail::stringPullConstantAltitude(map, staircase, 4.0 * cm);

    EXPECT_LT(pulled.size(), staircase.size());
    EXPECT_EQ(pulled.front().x.force_numerical_value_in(cm), 0.0);
    EXPECT_EQ(pulled.back().x.force_numerical_value_in(cm), 20.0);
    EXPECT_EQ(pulled.back().y.force_numerical_value_in(cm), 20.0);
}

TEST(PathShaping, StringPullDoesNotCutThroughOccupiedVoxel) {
    Map map{{11, 11, 11}, makeConfig(), ct::VoxelOccupancy::Empty};
    map.set(at(10, 10, 0), ct::VoxelOccupancy::Occupied);
    const std::vector<Position3D> around{
        at(0, 0, 0), at(0, 10, 0), at(0, 20, 0), at(10, 20, 0), at(20, 20, 0),
    };

    const std::vector<Position3D> pulled =
        detail::stringPullConstantAltitude(map, around, 4.0 * cm);

    for (const Position3D& wp : pulled) {
        EXPECT_NE(map.atVoxel(wp), ct::VoxelOccupancy::Occupied);
    }
    // The diagonal shortcut passes through the occupied cell, so the corner survives.
    EXPECT_GE(pulled.size(), 3u);
}

TEST(PathShaping, StringPullKeepsAltitudeChangesSplit) {
    Map map{{11, 11, 11}, makeConfig(), ct::VoxelOccupancy::Empty};
    const std::vector<Position3D> climb{
        at(0, 0, 0), at(10, 0, 0), at(10, 0, 10), at(20, 0, 10),
    };

    const std::vector<Position3D> pulled =
        detail::stringPullConstantAltitude(map, climb, 4.0 * cm);

    // The altitude change at index 2 must remain its own waypoint.
    bool has_transition = false;
    for (const Position3D& wp : pulled) {
        if (wp.z.force_numerical_value_in(cm) == 10.0 &&
            wp.x.force_numerical_value_in(cm) == 10.0) {
            has_transition = true;
        }
    }
    EXPECT_TRUE(has_transition);
}

TEST(PathShaping, StepCostChargesTwoRotationsForRightAngleOnLargeDrone) {
    // Facing +X, target straight ahead: advance only, no rotation.
    const std::size_t ahead = detail::stepCostForPath(
        {at(50, 0, 0)}, at(0, 0, 0), Orientation{0.0 * deg, 0.0 * deg}, largeDrone());
    EXPECT_EQ(ahead, 1u);  // 50 cm at 50 cm per advance

    // Facing +X, target to the left: 90 deg turn at 45 deg per step = 2 rotations.
    const std::size_t sideways = detail::stepCostForPath(
        {at(0, 50, 0)}, at(0, 0, 0), Orientation{0.0 * deg, 0.0 * deg}, largeDrone());
    EXPECT_EQ(sideways, 3u);  // 2 rotate + 1 advance
}

TEST(PathShaping, StepCostChargesOneRotationForRightAngleOnSmallDrone) {
    const std::size_t sideways = detail::stepCostForPath(
        {at(0, 30, 0)}, at(0, 0, 0), Orientation{0.0 * deg, 0.0 * deg}, smallDrone());
    EXPECT_EQ(sideways, 2u);  // 1 rotate (90 deg) + 1 advance (30 cm)
}

TEST(PathShaping, StepCostChargesElevationSeparately) {
    const std::size_t climb = detail::stepCostForPath(
        {at(0, 0, 60)}, at(0, 0, 0), Orientation{0.0 * deg, 0.0 * deg}, smallDrone());
    EXPECT_EQ(climb, 3u);  // 60 cm at 20 cm per elevate, no rotation or advance
}

TEST(PathShaping, StepCostOfEmptyPathIsZero) {
    EXPECT_EQ(detail::stepCostForPath({}, at(0, 0, 0), Orientation{}, smallDrone()), 0u);
}
