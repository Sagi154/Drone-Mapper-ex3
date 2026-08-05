// FakeMap3D.h — minimal in-memory IMutableMap3D for Algorithm tests.
// No TinyNPY dependency; no file I/O. Stores voxels in a hash map.
//
// Pass explicit grid dimensions (matching the NpyArray shape used in ex2 tests)
// so that isInBounds() only covers that sub-region, matching Map3DImpl semantics.
// Voxels inside dims but not set() return the default value (usually Unmapped).
// Positions that map to indices outside [0, dims) return OutOfBounds.

#pragma once

#include <Common/IMutableMap3D.h>
#include <Common/Units.h>

#include <array>
#include <cmath>
#include <filesystem>
#include <unordered_map>

namespace AlgorithmTest {

struct GridKey {
    int x{};
    int y{};
    int z{};
    [[nodiscard]] bool operator==(const GridKey& o) const noexcept {
        return x == o.x && y == o.y && z == o.z;
    }
};

struct GridKeyHash {
    [[nodiscard]] std::size_t operator()(const GridKey& k) const noexcept {
        const auto hx = static_cast<std::size_t>(k.x + 10000);
        const auto hy = static_cast<std::size_t>(k.y + 10000);
        const auto hz = static_cast<std::size_t>(k.z + 10000);
        return (hx * 73856093U) ^ (hy * 19349663U) ^ (hz * 83492791U);
    }
};

/// Minimal in-memory IMap3D/IMutableMap3D for tests.
///
/// `dims` mirrors the NpyArray shape from ex2 tests. A position whose grid index
/// falls outside [0, dims[axis]) reports OutOfBounds, exactly as Map3DImpl did.
class FakeMap3D final : public common::IMutableMap3D {
public:
    explicit FakeMap3D(
        std::array<int, 3> dims,
        common::types::MapConfig config,
        common::types::VoxelOccupancy default_val = common::types::VoxelOccupancy::Unmapped)
        : dims_(dims), config_(config), default_(default_val) {}

    [[nodiscard]] common::types::MapConfig getMapConfig() const override { return config_; }

    [[nodiscard]] bool isInBounds(const common::Position3D& pos) const override {
        const GridKey k = toKey(pos);
        return k.x >= 0 && k.x < dims_[0] &&
               k.y >= 0 && k.y < dims_[1] &&
               k.z >= 0 && k.z < dims_[2];
    }

    [[nodiscard]] common::types::VoxelOccupancy atVoxel(
        const common::Position3D& pos) const override {
        const GridKey k = toKey(pos);
        if (k.x < 0 || k.x >= dims_[0] || k.y < 0 || k.y >= dims_[1] ||
            k.z < 0 || k.z >= dims_[2]) {
            return common::types::VoxelOccupancy::OutOfBounds;
        }
        const auto it = cells_.find(k);
        return (it != cells_.end()) ? it->second : default_;
    }

    void set(const common::Position3D& pos,
             common::types::VoxelOccupancy value) override {
        cells_[toKey(pos)] = value;
    }

    void save(const std::filesystem::path& /*path*/) const override {
        // no-op: tests do not write maps to disk
    }

private:
    [[nodiscard]] GridKey toKey(const common::Position3D& pos) const {
        const double step = config_.resolution.force_numerical_value_in(common::cm);
        const double ox   = config_.offset.x.force_numerical_value_in(common::cm);
        const double oy   = config_.offset.y.force_numerical_value_in(common::cm);
        const double oz   = config_.offset.z.force_numerical_value_in(common::cm);
        return {
            static_cast<int>(
                std::lround((pos.x.force_numerical_value_in(common::cm) - ox) / step)),
            static_cast<int>(
                std::lround((pos.y.force_numerical_value_in(common::cm) - oy) / step)),
            static_cast<int>(
                std::lround((pos.z.force_numerical_value_in(common::cm) - oz) / step)),
        };
    }

    std::array<int, 3> dims_;
    common::types::MapConfig config_;
    common::types::VoxelOccupancy default_;
    std::unordered_map<GridKey, common::types::VoxelOccupancy, GridKeyHash> cells_;
};

} // namespace AlgorithmTest
