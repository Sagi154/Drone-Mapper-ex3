// test_maps_comparison.cpp — ported BFS/reachability scoring cases (single-
// target signature). Uses a hand-written mutable IMap3D fake, no Map3DImpl.

#include <Simulator/MapsComparison.h>

#include <Common/IMap3D.h>
#include <Common/Types.h>
#include <Common/Units.h>

#include <gtest/gtest.h>

#include <map>
#include <tuple>

namespace {

using common::cm;
using common::Position3D;
using common::x_extent;
using common::y_extent;
using common::z_extent;
using common::types::MapConfig;
using common::types::VoxelOccupancy;

struct GridKey {
    int x = 0;
    int y = 0;
    int z = 0;

    [[nodiscard]] bool operator<(const GridKey& other) const {
        return std::tie(x, y, z) < std::tie(other.x, other.y, other.z);
    }
};

// Minimal mutable IMap3D fake: a resolution-10cm grid over the given bounds,
// backed by a sparse map (defaults to Unmapped).
class FakeMap3D final : public common::IMap3D {
public:
    explicit FakeMap3D(MapConfig config) : config_(std::move(config)) {}

    void set(const Position3D& pos, VoxelOccupancy value) { grid_[key(pos)] = value; }

    [[nodiscard]] VoxelOccupancy atVoxel(const Position3D& pos) const override {
        if (!isInBounds(pos)) {
            return VoxelOccupancy::OutOfBounds;
        }
        const auto it = grid_.find(key(pos));
        return it == grid_.end() ? VoxelOccupancy::Unmapped : it->second;
    }

    [[nodiscard]] MapConfig getMapConfig() const override { return config_; }

    [[nodiscard]] bool isInBounds(const Position3D& pos) const override {
        const auto& b = config_.boundaries;
        return pos.x >= b.min_x && pos.x <= b.max_x && pos.y >= b.min_y && pos.y <= b.max_y &&
               pos.z >= b.min_height && pos.z <= b.max_height;
    }

private:
    [[nodiscard]] GridKey key(const Position3D& pos) const {
        const double step = config_.resolution.numerical_value_in(cm);
        return GridKey{
            static_cast<int>(
                (pos.x.numerical_value_in(cm) - config_.offset.x.numerical_value_in(cm)) / step),
            static_cast<int>(
                (pos.y.numerical_value_in(cm) - config_.offset.y.numerical_value_in(cm)) / step),
            static_cast<int>(
                (pos.z.numerical_value_in(cm) - config_.offset.z.numerical_value_in(cm)) / step),
        };
    }

    MapConfig config_;
    std::map<GridKey, VoxelOccupancy> grid_;
};

[[nodiscard]] MapConfig makeThreeByThreeConfig() {
    MapConfig config{};
    config.resolution = 10.0 * cm;
    config.boundaries.min_x = 0.0 * x_extent[cm];
    config.boundaries.max_x = 20.0 * x_extent[cm];
    config.boundaries.min_y = 0.0 * y_extent[cm];
    config.boundaries.max_y = 20.0 * y_extent[cm];
    config.boundaries.min_height = 0.0 * z_extent[cm];
    config.boundaries.max_height = 20.0 * z_extent[cm];
    return config;
}

[[nodiscard]] Position3D cellCenter(int x, int y, int z) {
    return Position3D{static_cast<double>(x * 10) * x_extent[cm],
                      static_cast<double>(y * 10) * y_extent[cm],
                      static_cast<double>(z * 10) * z_extent[cm]};
}

void fillKnownGrid(FakeMap3D& map, VoxelOccupancy value) {
    for (int x = 0; x < 3; ++x) {
        for (int y = 0; y < 3; ++y) {
            for (int z = 0; z < 3; ++z) {
                map.set(cellCenter(x, y, z), value);
            }
        }
    }
}

[[nodiscard]] MapConfig makeSlabConfig() {
    MapConfig config{};
    config.resolution = 10.0 * cm;
    config.boundaries.min_x = 0.0 * x_extent[cm];
    config.boundaries.max_x = 20.0 * x_extent[cm];
    config.boundaries.min_y = 0.0 * y_extent[cm];
    config.boundaries.max_y = 0.0 * y_extent[cm];
    config.boundaries.min_height = 0.0 * z_extent[cm];
    config.boundaries.max_height = 20.0 * z_extent[cm];
    return config;
}

[[nodiscard]] Position3D slabPt(int x, int z) {
    return Position3D{static_cast<double>(x * 10) * x_extent[cm], 0.0 * y_extent[cm],
                      static_cast<double>(z * 10) * z_extent[cm]};
}

// x=0 Empty (reachable), x=1 Occupied wall, x=2 Empty (sealed room).
void fillSlabReference(FakeMap3D& ref) {
    for (int z = 0; z < 3; ++z) {
        ref.set(slabPt(0, z), VoxelOccupancy::Empty);
        ref.set(slabPt(1, z), VoxelOccupancy::Occupied);
        ref.set(slabPt(2, z), VoxelOccupancy::Empty);
    }
}

} // namespace

TEST(MapsComparison, IdenticalMapsScore100) {
    const MapConfig config = makeThreeByThreeConfig();
    FakeMap3D reference(config);
    FakeMap3D target(config);
    fillKnownGrid(reference, VoxelOccupancy::Empty);
    fillKnownGrid(target, VoxelOccupancy::Empty);
    reference.set(cellCenter(1, 1, 1), VoxelOccupancy::Occupied);
    target.set(cellCenter(1, 1, 1), VoxelOccupancy::Occupied);

    EXPECT_DOUBLE_EQ(simulator::MapsComparison::compare(reference, target), 100.0);
}

TEST(MapsComparison, EmptyUnionScore100) {
    const MapConfig config = makeThreeByThreeConfig();
    FakeMap3D reference(config);
    FakeMap3D target(config);

    EXPECT_DOUBLE_EQ(simulator::MapsComparison::compare(reference, target), 100.0);
}

TEST(MapsComparison, SingleMismatchReducesScore) {
    const MapConfig config = makeThreeByThreeConfig();
    FakeMap3D reference(config);
    FakeMap3D target(config);
    fillKnownGrid(reference, VoxelOccupancy::Occupied);
    fillKnownGrid(target, VoxelOccupancy::Occupied);
    target.set(cellCenter(0, 0, 0), VoxelOccupancy::Empty);

    const double score = simulator::MapsComparison::compare(reference, target);
    EXPECT_LT(score, 100.0);
    EXPECT_GT(score, 95.0);
}

TEST(MapsComparison, DistinctMapsScoreNearZero) {
    const MapConfig config = makeThreeByThreeConfig();
    FakeMap3D reference(config);
    FakeMap3D target(config);
    fillKnownGrid(reference, VoxelOccupancy::Occupied);
    fillKnownGrid(target, VoxelOccupancy::Empty);

    EXPECT_NEAR(simulator::MapsComparison::compare(reference, target), 0.0, 1e-6);
}

TEST(MapsComparison, ReferenceUnmappedCellsAreSkipped) {
    const MapConfig config = makeThreeByThreeConfig();
    FakeMap3D reference(config);
    FakeMap3D target(config);
    reference.set(cellCenter(1, 1, 1), VoxelOccupancy::Occupied);
    target.set(cellCenter(1, 1, 1), VoxelOccupancy::Occupied);

    EXPECT_DOUBLE_EQ(simulator::MapsComparison::compare(reference, target), 100.0);
}

TEST(MapsComparison, OutOfBoundsCellsAreSkipped) {
    const MapConfig config = makeThreeByThreeConfig();
    FakeMap3D reference(config);
    FakeMap3D target(config);
    fillKnownGrid(reference, VoxelOccupancy::Empty);
    fillKnownGrid(target, VoxelOccupancy::Empty);

    const Position3D outside{100.0 * x_extent[cm], 100.0 * y_extent[cm], 100.0 * z_extent[cm]};
    ASSERT_EQ(reference.atVoxel(outside), VoxelOccupancy::OutOfBounds);

    EXPECT_DOUBLE_EQ(simulator::MapsComparison::compare(reference, target), 100.0);
}

TEST(MapsComparison, SealedRoomPenalisesWithoutSpawn) {
    const MapConfig config = makeSlabConfig();
    FakeMap3D reference(config);
    FakeMap3D target(config);
    fillSlabReference(reference);
    for (int z = 0; z < 3; ++z) {
        target.set(slabPt(0, z), VoxelOccupancy::Empty);
        target.set(slabPt(1, z), VoxelOccupancy::Occupied);
    }

    EXPECT_LT(simulator::MapsComparison::compare(reference, target), 100.0);
}

TEST(MapsComparison, SealedRoomExcludedWhenSpawnProvided) {
    const MapConfig config = makeSlabConfig();
    FakeMap3D reference(config);
    FakeMap3D target(config);
    fillSlabReference(reference);
    for (int z = 0; z < 3; ++z) {
        target.set(slabPt(0, z), VoxelOccupancy::Empty);
        target.set(slabPt(1, z), VoxelOccupancy::Occupied);
    }

    EXPECT_DOUBLE_EQ(simulator::MapsComparison::compare(reference, target, slabPt(0, 0)), 100.0);
}

TEST(MapsComparison, ReachableUnmappedCellsStillPenalise) {
    const MapConfig config = makeSlabConfig();
    FakeMap3D reference(config);
    FakeMap3D target(config);
    fillSlabReference(reference);
    for (int z = 0; z < 3; ++z) {
        target.set(slabPt(1, z), VoxelOccupancy::Occupied);
    }

    EXPECT_LT(simulator::MapsComparison::compare(reference, target, slabPt(0, 0)), 100.0);
}

TEST(MapsComparison, SpawnAwareScoreNeverWorseForSealedRoom) {
    const MapConfig config = makeSlabConfig();
    FakeMap3D reference(config);
    FakeMap3D target(config);
    fillSlabReference(reference);
    for (int z = 0; z < 3; ++z) {
        target.set(slabPt(0, z), VoxelOccupancy::Empty);
        target.set(slabPt(1, z), VoxelOccupancy::Occupied);
    }

    const double raw = simulator::MapsComparison::compare(reference, target);
    const double with_spawn =
        simulator::MapsComparison::compare(reference, target, slabPt(0, 0));
    EXPECT_GE(with_spawn, raw);
}
