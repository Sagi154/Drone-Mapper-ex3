#include <Simulator/Map3DImpl.h>
#include <Simulator/io/YamlConfigParsers.h>
#include <user_common_207190406_209543255/SimulationCoordUtil.h>
#include <user_common_207190406_209543255/RunErrorLog.h>

#include <TinyNPY.h>

#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <memory>

namespace fs = std::filesystem;

namespace {

using common::cm;
using common::PhysicalLength;
using common::Position3D;
using common::x_extent;
using common::y_extent;
using common::z_extent;
using common::types::MapConfig;
using common::types::VoxelOccupancy;
using simulator::Map3DImpl;

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

struct NullLog final : user_common_207190406_209543255::IRunErrorLog {
    void log(const common::types::ErrorRef&) override {}
};

[[nodiscard]] fs::path inputsDir() {
#ifdef SIMULATOR_INPUTS_DIR
    return fs::path{SIMULATOR_INPUTS_DIR};
#else
    fs::path p = fs::current_path();
    for (int i = 0; i < 6; ++i) {
        if (fs::exists(p / "inputs" / "sim_compose.yaml")) {
            return p / "inputs";
        }
        p = p.parent_path();
    }
    return fs::path{"inputs"};
#endif
}

[[nodiscard]] MapConfig hiddenMapConfig(const simulator::types::SimulationConfigData& sim) {
    return MapConfig{
        .boundaries = {},
        .offset     = sim.map_offset,
        .resolution = sim.map_resolution,
    };
}

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
    // house_simulation.yaml: height_cm: 150, height_offset: 150 → world z 300
    simulator::types::SimulationConfigData simulation{};
    simulation.initial_drone_position =
        Position3D{150.0 * x_extent[cm], 200.0 * y_extent[cm], 150.0 * z_extent[cm]};
    simulation.map_offset =
        Position3D{0.0 * x_extent[cm], 0.0 * y_extent[cm], 150.0 * z_extent[cm]};

    const Position3D world =
        user_common_207190406_209543255::worldInitialDronePosition(simulation);
    EXPECT_DOUBLE_EQ(world.x.numerical_value_in(cm), 150.0);
    EXPECT_DOUBLE_EQ(world.y.numerical_value_in(cm), 200.0);
    EXPECT_DOUBLE_EQ(world.z.numerical_value_in(cm), 300.0);
}

TEST(SimulationCoordUtil, WorldInitialDronePosition_UnchangedWhenOffsetZero) {
    simulator::types::SimulationConfigData simulation{};
    simulation.initial_drone_position =
        Position3D{10.0 * x_extent[cm], 20.0 * y_extent[cm], 30.0 * z_extent[cm]};
    simulation.map_offset =
        Position3D{0.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]};

    const Position3D world =
        user_common_207190406_209543255::worldInitialDronePosition(simulation);
    EXPECT_EQ(world.x, simulation.initial_drone_position.x);
    EXPECT_EQ(world.y, simulation.initial_drone_position.y);
    EXPECT_EQ(world.z, simulation.initial_drone_position.z);
}

TEST(SimulationCoordUtil, IsDroneSpawnPassable_EmptyMapInBounds) {
    FakeMap3D map{emptyRoomConfig(), VoxelOccupancy::Empty};
    const Position3D center{50.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]};
    const PhysicalLength radius = 5.0 * cm;
    EXPECT_TRUE(user_common_207190406_209543255::isDroneSpawnPassable(map, radius, center));
}

TEST(SimulationCoordUtil, IsDroneSpawnPassable_OccupiedRejects) {
    FakeMap3D map{emptyRoomConfig(), VoxelOccupancy::Occupied};
    const Position3D center{50.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]};
    const PhysicalLength radius = 5.0 * cm;
    EXPECT_FALSE(user_common_207190406_209543255::isDroneSpawnPassable(map, radius, center));
}

TEST(SimulationCoordUtil, IsDroneSpawnPassable_OutOfBoundsRejects) {
    FakeMap3D map{emptyRoomConfig(), VoxelOccupancy::Empty};
    // Near the edge so the sphere samples outside bounds
    const Position3D center{1.0 * x_extent[cm], 1.0 * y_extent[cm], 1.0 * z_extent[cm]};
    const PhysicalLength radius = 20.0 * cm;
    EXPECT_FALSE(user_common_207190406_209543255::isDroneSpawnPassable(map, radius, center));
}

TEST(SimulationCoordUtil, IsDroneSpawnPassable_HouseScenarioInstructorSpawn) {
    const fs::path sim_path = inputsDir() / "simulation" / "house_simulation.yaml";
    if (!fs::exists(sim_path)) {
        GTEST_SKIP() << "inputs not available";
    }

    NullLog log;
    const auto parse_result = simulator::io::parseSimulationConfig(sim_path, log);
    ASSERT_TRUE(parse_result.ok);

    const simulator::types::SimulationConfigData& simulation = parse_result.value;
    ASSERT_TRUE(fs::exists(simulation.map_filename)) << simulation.map_filename;

    auto map_array = std::make_shared<NpyArray>();
    const LPCSTR load_err = map_array->LoadNPY(simulation.map_filename.string());
    ASSERT_EQ(load_err, nullptr) << load_err;

    const Map3DImpl hidden_map{map_array, simulator::MapRole::Hidden, hiddenMapConfig(simulation)};

    const Position3D world_spawn =
        user_common_207190406_209543255::worldInitialDronePosition(simulation);
    EXPECT_DOUBLE_EQ(world_spawn.x.numerical_value_in(cm), 150.0);
    EXPECT_DOUBLE_EQ(world_spawn.y.numerical_value_in(cm), 200.0);
    EXPECT_DOUBLE_EQ(world_spawn.z.numerical_value_in(cm), 300.0);

    // Pre-fix spawn at world z 160 landed on occupied NPY z=1 (uint8 value 3); fixed spawn is z=300 (NPY z=15).
    const PhysicalLength drone_small_radius = 4.0 * cm;
    EXPECT_TRUE(user_common_207190406_209543255::isDroneSpawnPassable(
        hidden_map, drone_small_radius, world_spawn));
}
