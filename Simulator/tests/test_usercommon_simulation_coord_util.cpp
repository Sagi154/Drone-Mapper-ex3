#include <UserCommon_207190406_209543255/SimulationCoordUtil.h>

#include <gtest/gtest.h>

#include <cmath>

namespace {

using common::cm;
using common::PhysicalLength;
using common::Position3D;
using common::x_extent;
using common::y_extent;
using common::z_extent;
using common::types::MapConfig;
using common::types::VoxelOccupancy;

class FakeMap3D final : public common::IMap3D {
public:
    FakeMap3D(MapConfig config, VoxelOccupancy fill)
        : config_(std::move(config)), fill_(fill) {}

    [[nodiscard]] VoxelOccupancy atVoxel(const Position3D& /*pos*/) const override {
        return fill_;
    }

    [[nodiscard]] MapConfig getMapConfig() const override { return config_; }

    [[nodiscard]] bool isInBounds(const Position3D& pos) const override {
        const auto& b = config_.boundaries;
        return pos.x >= b.min_x && pos.x < b.max_x && pos.y >= b.min_y && pos.y < b.max_y &&
               pos.z >= b.min_height && pos.z < b.max_height;
    }

private:
    MapConfig config_;
    VoxelOccupancy fill_;
};

[[nodiscard]] MapConfig emptyRoomConfig() {
    MapConfig cfg{};
    cfg.resolution = 10.0 * cm;
    cfg.boundaries = {
        0.0 * x_extent[cm],
        100.0 * x_extent[cm],
        0.0 * y_extent[cm],
        100.0 * y_extent[cm],
        0.0 * z_extent[cm],
        100.0 * z_extent[cm],
    };
    return cfg;
}

} // namespace

TEST(SimulationCoordUtil, WorldInitialDronePosition_HouseScenarioOffset) {
    // house_simulation.yaml: height_cm: 10, height_offset: 150 → world z 160
    simulator::types::SimulationConfigData simulation{};
    simulation.initial_drone_position =
        Position3D{150.0 * x_extent[cm], 200.0 * y_extent[cm], 10.0 * z_extent[cm]};
    simulation.map_offset =
        Position3D{0.0 * x_extent[cm], 0.0 * y_extent[cm], 150.0 * z_extent[cm]};

    const Position3D world =
        UserCommon_207190406_209543255::worldInitialDronePosition(simulation);
    EXPECT_DOUBLE_EQ(world.x.numerical_value_in(cm), 150.0);
    EXPECT_DOUBLE_EQ(world.y.numerical_value_in(cm), 200.0);
    EXPECT_DOUBLE_EQ(world.z.numerical_value_in(cm), 160.0);
}

TEST(SimulationCoordUtil, WorldInitialDronePosition_UnchangedWhenOffsetZero) {
    simulator::types::SimulationConfigData simulation{};
    simulation.initial_drone_position =
        Position3D{10.0 * x_extent[cm], 20.0 * y_extent[cm], 30.0 * z_extent[cm]};
    simulation.map_offset =
        Position3D{0.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]};

    const Position3D world =
        UserCommon_207190406_209543255::worldInitialDronePosition(simulation);
    EXPECT_EQ(world.x, simulation.initial_drone_position.x);
    EXPECT_EQ(world.y, simulation.initial_drone_position.y);
    EXPECT_EQ(world.z, simulation.initial_drone_position.z);
}

TEST(SimulationCoordUtil, IsDroneSpawnPassable_EmptyMapInBounds) {
    FakeMap3D map{emptyRoomConfig(), VoxelOccupancy::Empty};
    const Position3D center{50.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]};
    const PhysicalLength radius = 5.0 * cm;
    EXPECT_TRUE(UserCommon_207190406_209543255::isDroneSpawnPassable(map, radius, center));
}

TEST(SimulationCoordUtil, IsDroneSpawnPassable_OccupiedRejects) {
    FakeMap3D map{emptyRoomConfig(), VoxelOccupancy::Occupied};
    const Position3D center{50.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]};
    const PhysicalLength radius = 5.0 * cm;
    EXPECT_FALSE(UserCommon_207190406_209543255::isDroneSpawnPassable(map, radius, center));
}

TEST(SimulationCoordUtil, IsDroneSpawnPassable_OutOfBoundsRejects) {
    FakeMap3D map{emptyRoomConfig(), VoxelOccupancy::Empty};
    // Near the edge so the sphere samples outside bounds
    const Position3D center{1.0 * x_extent[cm], 1.0 * y_extent[cm], 1.0 * z_extent[cm]};
    const PhysicalLength radius = 20.0 * cm;
    EXPECT_FALSE(UserCommon_207190406_209543255::isDroneSpawnPassable(map, radius, center));
}
