// test_map3d_impl.cpp
// Covers Map3DImpl: role-based dtype dispatch, bounds, set/save/load,
// and the regression cases from docs/map3d-contract.md.

#include <Simulator/Map3DImpl.h>

#include <TinyNPY.h>

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <stdexcept>

using namespace common;
using namespace common::types;
using simulator::Map3DImpl;
using simulator::MapRole;

namespace {

[[nodiscard]] common::types::MapConfig makeConfig(double resolution_cm = 10.0,
                                                   double max_bound_cm = 40.0) {
    MapConfig cfg{};
    cfg.resolution = resolution_cm * cm;
    cfg.boundaries.min_x      = 0.0 * x_extent[cm];
    cfg.boundaries.max_x      = max_bound_cm * x_extent[cm];
    cfg.boundaries.min_y      = 0.0 * y_extent[cm];
    cfg.boundaries.max_y      = max_bound_cm * y_extent[cm];
    cfg.boundaries.min_height = 0.0 * z_extent[cm];
    cfg.boundaries.max_height = max_bound_cm * z_extent[cm];
    return cfg;
}

[[nodiscard]] std::shared_ptr<NpyArray> makeInt8Map(const NpyArray::shape_t& shape,
                                                     std::int8_t fill_value) {
    auto map = std::make_shared<NpyArray>(shape, sizeof(std::int8_t),
                                          NpyArray::GetTypeChar(typeid(std::int8_t)));
    map->Allocate();
    std::fill_n(map->Data<std::int8_t>(), map->NumValue(), fill_value);
    return map;
}

[[nodiscard]] std::shared_ptr<NpyArray> makeUint8Map(const NpyArray::shape_t& shape,
                                                      std::uint8_t fill_value) {
    auto map = std::make_shared<NpyArray>(shape, sizeof(std::uint8_t),
                                          NpyArray::GetTypeChar(typeid(std::uint8_t)));
    map->Allocate();
    std::fill_n(map->Data<std::uint8_t>(), map->NumValue(), fill_value);
    return map;
}

} // namespace

// ---------------------------------------------------------------------------
// Null pointer rejection
// ---------------------------------------------------------------------------

TEST(Map3DImpl, RejectsNullMapPointer) {
    EXPECT_THROW(Map3DImpl(std::shared_ptr<NpyArray>{}, MapRole::Hidden), std::invalid_argument);
}

// ---------------------------------------------------------------------------
// Hidden map — uint8 dtype: values > 1 must be Occupied
// (regression: scenario_house.npy has uint8 values 2, 3, 4, 18, 45)
// ---------------------------------------------------------------------------

TEST(Map3DImpl, Uint8HiddenMapValuesGt1AreOccupied) {
    const Position3D voxel{0.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]};
    for (const std::uint8_t v :
         {std::uint8_t{2}, std::uint8_t{3}, std::uint8_t{4}, std::uint8_t{18}, std::uint8_t{45}}) {
        const auto map = makeUint8Map({1, 1, 1}, v);
        const Map3DImpl impl{map, MapRole::Hidden, makeConfig()};
        EXPECT_EQ(impl.atVoxel(voxel), VoxelOccupancy::Occupied)
            << "stored uint8 value=" << static_cast<int>(v);
    }
}

TEST(Map3DImpl, Uint8HiddenMapValueZeroIsEmpty) {
    const auto map = makeUint8Map({1, 1, 1}, 0);
    const Map3DImpl impl{map, MapRole::Hidden, makeConfig()};
    EXPECT_EQ(impl.atVoxel({}), VoxelOccupancy::Empty);
}

// ---------------------------------------------------------------------------
// Hidden map — int8 dtype: value 3 must be Occupied, not Unmapped
// (regression: without role-based dispatch an int8 value 3 falls to default
// in the output enum switch and would wrongly return Unmapped)
// ---------------------------------------------------------------------------

TEST(Map3DImpl, Int8HiddenMapValueGt1IsOccupied) {
    for (const std::int8_t v : {std::int8_t{2}, std::int8_t{3}, std::int8_t{10}}) {
        const auto map = makeInt8Map({1, 1, 1}, v);
        const Map3DImpl impl{map, MapRole::Hidden, makeConfig()};
        EXPECT_EQ(impl.atVoxel({}), VoxelOccupancy::Occupied)
            << "stored int8 value=" << static_cast<int>(v);
    }
}

// ---------------------------------------------------------------------------
// Output map — int8 dtype: negative enum values must round-trip
// (regression: if read via the hidden path, -1 → Occupied instead of Unmapped)
// ---------------------------------------------------------------------------

TEST(Map3DImpl, Int8OutputMapUnmappedStaysUnmapped) {
    const auto map = makeInt8Map({1, 1, 1},
                                 static_cast<std::int8_t>(VoxelOccupancy::Unmapped));
    const Map3DImpl impl{map, MapRole::Output, makeConfig()};
    EXPECT_EQ(impl.atVoxel({}), VoxelOccupancy::Unmapped);
}

TEST(Map3DImpl, Int8OutputMapNegativeEnumsRoundTrip) {
    for (const VoxelOccupancy v :
         {VoxelOccupancy::PotentiallyOccupied, VoxelOccupancy::Unmapped,
          VoxelOccupancy::Empty, VoxelOccupancy::Occupied}) {
        const auto map = makeInt8Map({1, 1, 1}, static_cast<std::int8_t>(v));
        const Map3DImpl impl{map, MapRole::Output, makeConfig()};
        EXPECT_EQ(impl.atVoxel({}), v);
    }
}

// ---------------------------------------------------------------------------
// Bounds and index arithmetic
// ---------------------------------------------------------------------------

TEST(Map3DImpl, InBoundsVoxelIsDetectedCorrectly) {
    // 5x5x5 at 10 cm/cell — valid positions are 0..40 on each axis
    const auto map = makeInt8Map({5, 5, 5}, 0);
    Map3DImpl impl{map, MapRole::Hidden, makeConfig(10.0, 40.0)};

    EXPECT_TRUE(impl.isInBounds(Position3D{0.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]}));
    EXPECT_TRUE(impl.isInBounds(Position3D{20.0 * x_extent[cm], 20.0 * y_extent[cm], 20.0 * z_extent[cm]}));
    EXPECT_FALSE(impl.isInBounds(Position3D{50.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]}));
}

TEST(Map3DImpl, AtVoxelOutsideBoundsReturnsOutOfBounds) {
    const auto map = makeInt8Map({3, 3, 3}, 1);
    const Map3DImpl impl{map, MapRole::Hidden, makeConfig(10.0, 20.0)};

    EXPECT_EQ(impl.atVoxel(Position3D{50.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]}),
              VoxelOccupancy::OutOfBounds);
}

// ---------------------------------------------------------------------------
// Mutable: set / save / load round-trip
// ---------------------------------------------------------------------------

TEST(Map3DImpl, SetUpdatesMutatesVoxel) {
    const auto map = makeInt8Map({3, 3, 3},
                                 static_cast<std::int8_t>(VoxelOccupancy::Unmapped));
    Map3DImpl impl{map, MapRole::Output, makeConfig()};

    const Position3D target{10.0 * x_extent[cm], 10.0 * y_extent[cm], 10.0 * z_extent[cm]};
    impl.set(target, VoxelOccupancy::Occupied);
    EXPECT_EQ(impl.atVoxel(target), VoxelOccupancy::Occupied);
}

TEST(Map3DImpl, SetOutOfBoundsIsNoOp) {
    const auto map = makeInt8Map({3, 3, 3},
                                 static_cast<std::int8_t>(VoxelOccupancy::Unmapped));
    Map3DImpl impl{map, MapRole::Output, makeConfig()};

    const Position3D outside{100.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]};
    impl.set(outside, VoxelOccupancy::Occupied);
    EXPECT_EQ(impl.atVoxel(outside), VoxelOccupancy::OutOfBounds);
}

TEST(Map3DImpl, SaveLoadRoundTripPreservesOccupied) {
    const auto map = makeInt8Map({3, 3, 3},
                                 static_cast<std::int8_t>(VoxelOccupancy::Unmapped));
    Map3DImpl impl{map, MapRole::Output, makeConfig()};

    const Position3D target{10.0 * x_extent[cm], 10.0 * y_extent[cm], 10.0 * z_extent[cm]};
    impl.set(target, VoxelOccupancy::Occupied);

    const auto temp = std::filesystem::temp_directory_path() / "map3d_impl_roundtrip.npy";
    impl.save(temp);

    auto reloaded = std::make_shared<NpyArray>();
    const LPCSTR err = reloaded->LoadNPY(temp.string());
    ASSERT_EQ(err, nullptr);

    const Map3DImpl reloaded_impl{reloaded, MapRole::Output, makeConfig()};
    EXPECT_EQ(reloaded_impl.atVoxel(target), VoxelOccupancy::Occupied);

    std::error_code ec;
    std::filesystem::remove(temp, ec);
}

// ---------------------------------------------------------------------------
// Derive bounds from shape when config has zero boundaries
// ---------------------------------------------------------------------------

TEST(Map3DImpl, DerivesBoundsFromShapeWhenUnset) {
    // 5x5x5 at 10 cm/cell, no explicit boundaries
    const auto map = makeInt8Map({5, 5, 5}, 0);
    MapConfig cfg{};
    cfg.resolution = 10.0 * cm;  // boundaries left at zero

    const Map3DImpl impl{map, MapRole::Hidden, cfg};
    const MapConfig derived = impl.getMapConfig();

    EXPECT_DOUBLE_EQ(derived.boundaries.max_x.numerical_value_in(cm), 40.0);
    EXPECT_DOUBLE_EQ(derived.boundaries.max_y.numerical_value_in(cm), 40.0);
    EXPECT_DOUBLE_EQ(derived.boundaries.max_height.numerical_value_in(cm), 40.0);
}
