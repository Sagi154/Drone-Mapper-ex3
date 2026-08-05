// test_maps_comparison.cpp — stub returns the failure sentinel (-1).

#include <Simulator/MapsComparison.h>

#include <Common/IMap3D.h>
#include <Common/Types.h>

#include <gtest/gtest.h>

namespace {

class EmptyMap final : public common::IMap3D {
public:
    [[nodiscard]] common::types::VoxelOccupancy atVoxel(
        const common::Position3D& /*pos*/) const override {
        return common::types::VoxelOccupancy::Unmapped;
    }

    [[nodiscard]] common::types::MapConfig getMapConfig() const override {
        return common::types::MapConfig{};
    }

    [[nodiscard]] bool isInBounds(const common::Position3D& /*pos*/) const override {
        return false;
    }
};

} // namespace

TEST(MapsComparison, StubReturnsFailureSentinel) {
    EmptyMap origin;
    EmptyMap target;
    EXPECT_EQ(simulator::MapsComparison::compare(origin, target), -1.0);
    EXPECT_EQ(simulator::MapsComparison::compare(origin, target, common::Position3D{}), -1.0);
}
