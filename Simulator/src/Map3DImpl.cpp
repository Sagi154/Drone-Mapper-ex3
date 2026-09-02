// Map3DImpl.cpp
// IMap3D / IMutableMap3D backed by a TinyNPY NpyArray.
//
// dtype dispatch is role-keyed, not dtype-keyed:
//   Hidden maps (instructor .npy):  any stored byte >= 1 → Occupied; 0 → Empty.
//   Output maps (our int8 arrays):  full VoxelOccupancy enum stored verbatim.
//
// Both hidden and output maps can be either int8 or uint8; the role, not the
// dtype, decides the interpretation (see docs/map3d-contract.md).

#include <Simulator/Map3DImpl.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <utility>

namespace simulator {

namespace {

using namespace common;
using namespace common::types;

[[nodiscard]] bool isZeroResolution(PhysicalLength resolution) {
    return resolution <= 0.0 * cm;
}

[[nodiscard]] bool isUnsetBoundaries(const MappingBounds& bounds) {
    return bounds.min_x == 0.0 * x_extent[cm] && bounds.max_x == 0.0 * x_extent[cm] &&
           bounds.min_y == 0.0 * y_extent[cm] && bounds.max_y == 0.0 * y_extent[cm] &&
           bounds.min_height == 0.0 * z_extent[cm] && bounds.max_height == 0.0 * z_extent[cm];
}

[[nodiscard]] MappingBounds deriveBoundsFromShape(const NpyArray& map,
                                                   const Position3D& offset,
                                                   PhysicalLength resolution) {
    MappingBounds bounds{};
    if (map.IsEmpty() || map.Shape().size() != 3 || isZeroResolution(resolution)) {
        return bounds;
    }

    const auto span = [&](std::size_t voxel_count) -> double {
        if (voxel_count == 0) {
            return 0.0;
        }
        return static_cast<double>(voxel_count - 1) * resolution.force_numerical_value_in(cm);
    };

    const double ox   = offset.x.force_numerical_value_in(cm);
    const double oy   = offset.y.force_numerical_value_in(cm);
    const double oz   = offset.z.force_numerical_value_in(cm);

    bounds.min_x      = offset.x;
    bounds.max_x      = (ox + span(map.Shape()[0])) * x_extent[cm];
    bounds.min_y      = offset.y;
    bounds.max_y      = (oy + span(map.Shape()[1])) * y_extent[cm];
    bounds.min_height = offset.z;
    bounds.max_height = (oz + span(map.Shape()[2])) * z_extent[cm];
    return bounds;
}

[[nodiscard]] MapConfig finalizeConfig(const NpyArray& map, MapConfig config) {
    if (isUnsetBoundaries(config.boundaries)) {
        config.boundaries = deriveBoundsFromShape(map, config.offset, config.resolution);
    }
    return config;
}

[[nodiscard]] bool isWithinMappingBounds(const Position3D& pos, const MappingBounds& bounds) {
    return pos.x >= bounds.min_x && pos.x <= bounds.max_x &&
           pos.y >= bounds.min_y && pos.y <= bounds.max_y &&
           pos.z >= bounds.min_height && pos.z <= bounds.max_height;
}

[[nodiscard]] std::optional<std::array<std::size_t, 3>>
toVoxelIndex(const Position3D& pos, const MapConfig& config) {
    if (isZeroResolution(config.resolution)) {
        return std::nullopt;
    }

    const double resolution_cm = config.resolution.force_numerical_value_in(cm);
    const auto toIndex = [&](auto coordinate, auto offset) -> std::optional<std::size_t> {
        const double relative_cm = (coordinate - offset).force_numerical_value_in(cm);
        if (relative_cm < 0.0) {
            return std::nullopt;
        }
        const auto index = static_cast<std::size_t>(std::floor(relative_cm / resolution_cm + 1e-9));
        return index;
    };

    const auto ix = toIndex(pos.x, config.offset.x);
    const auto iy = toIndex(pos.y, config.offset.y);
    const auto iz = toIndex(pos.z, config.offset.z);
    if (!ix || !iy || !iz) {
        return std::nullopt;
    }
    return std::array<std::size_t, 3>{*ix, *iy, *iz};
}

[[nodiscard]] bool isIndexWithinShape(const NpyArray& map, const std::array<std::size_t, 3>& idx) {
    if (map.IsEmpty() || map.Shape().size() != 3) {
        return false;
    }
    return idx[0] < map.Shape()[0] && idx[1] < map.Shape()[1] && idx[2] < map.Shape()[2];
}

[[nodiscard]] std::size_t linearIndex(const NpyArray& map, const std::array<std::size_t, 3>& idx) {
    return idx[0] * (map.Shape()[1] * map.Shape()[2]) + idx[1] * map.Shape()[2] + idx[2];
}

// ----- Role-based occupancy readers ----------------------------------------

/// Hidden maps: any value >= 1 is Occupied (regardless of int8/uint8 dtype).
[[nodiscard]] VoxelOccupancy occupancyHidden(std::int64_t raw) {
    return raw >= 1 ? VoxelOccupancy::Occupied : VoxelOccupancy::Empty;
}

/// Output maps: full VoxelOccupancy enum, unknown positive values → Unmapped.
[[nodiscard]] VoxelOccupancy occupancyOutput(std::int64_t raw) {
    switch (raw) {
    case static_cast<std::int64_t>(VoxelOccupancy::PotentiallyOccupied):
    case static_cast<std::int64_t>(VoxelOccupancy::OutOfBounds):
    case static_cast<std::int64_t>(VoxelOccupancy::Unmapped):
    case static_cast<std::int64_t>(VoxelOccupancy::Empty):
    case static_cast<std::int64_t>(VoxelOccupancy::Occupied):
        return static_cast<VoxelOccupancy>(raw);
    default:
        return VoxelOccupancy::Unmapped;
    }
}

[[nodiscard]] VoxelOccupancy readStoredValue(const NpyArray& map,
                                              std::size_t linear_idx,
                                              MapRole role) {
    if (map.SizeValueBytes() != 1) {
        return VoxelOccupancy::Unmapped;
    }

    const std::int64_t raw = (map.Type() == 'u')
        ? static_cast<std::int64_t>(map.Data<std::uint8_t>()[linear_idx])
        : static_cast<std::int64_t>(map.Data<std::int8_t>()[linear_idx]);

    return (role == MapRole::Hidden) ? occupancyHidden(raw) : occupancyOutput(raw);
}

void writeStoredValue(NpyArray& map, std::size_t linear_idx, VoxelOccupancy value) {
    const auto stored = static_cast<std::int8_t>(value);
    if (map.Type() == 'u') {
        map.Data<std::uint8_t>()[linear_idx] = static_cast<std::uint8_t>(stored);
    } else {
        map.Data<std::int8_t>()[linear_idx] = stored;
    }
}

} // namespace

// ---------------------------------------------------------------------------
// Map3DImpl constructors
// ---------------------------------------------------------------------------

Map3DImpl::Map3DImpl(std::shared_ptr<NpyArray> map_ptr, MapRole role)
    : Map3DImpl(std::move(map_ptr), role, common::types::MapConfig{}) {}

Map3DImpl::Map3DImpl(std::shared_ptr<NpyArray> map_ptr,
                     MapRole role,
                     const common::types::MapConfig& map_config)
    : map_(std::move(map_ptr)),
      role_(role),
      config_(map_ ? finalizeConfig(*map_, map_config) : map_config) {
    if (!map_) {
        throw std::invalid_argument("Map3DImpl requires a valid map pointer.");
    }
}

// ---------------------------------------------------------------------------
// IMap3D implementation
// ---------------------------------------------------------------------------

common::types::VoxelOccupancy Map3DImpl::atVoxel(const common::Position3D& pos) const {
    if (!isInBounds(pos)) {
        return common::types::VoxelOccupancy::OutOfBounds;
    }
    const auto index = toVoxelIndex(pos, config_);
    if (!index || !isIndexWithinShape(*map_, *index)) {
        return common::types::VoxelOccupancy::OutOfBounds;
    }
    return readStoredValue(*map_, linearIndex(*map_, *index), role_);
}

common::types::MapConfig Map3DImpl::getMapConfig() const {
    return config_;
}

bool Map3DImpl::isInBounds(const common::Position3D& pos) const {
    if (!isWithinMappingBounds(pos, config_.boundaries)) {
        return false;
    }
    const auto index = toVoxelIndex(pos, config_);
    if (!index) {
        return false;
    }
    return isIndexWithinShape(*map_, *index);
}

// ---------------------------------------------------------------------------
// IMutableMap3D implementation
// ---------------------------------------------------------------------------

void Map3DImpl::set(const common::Position3D& pos, common::types::VoxelOccupancy value) {
    if (!isInBounds(pos)) {
        return;
    }
    const auto index = *toVoxelIndex(pos, config_);
    writeStoredValue(*map_, linearIndex(*map_, index), value);
}

void Map3DImpl::save(const std::filesystem::path& path) const {
    if (map_->IsEmpty()) {
        throw std::runtime_error("Cannot save an empty map.");
    }
    const LPCSTR error = map_->SaveNPY(path.string());
    if (error != nullptr) {
        throw std::runtime_error(error);
    }
}

} // namespace simulator
