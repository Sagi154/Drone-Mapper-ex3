#pragma once

#include <Common/IMap3D.h>
#include <Common/Units.h>

#include <cmath>

namespace user_common_207190406_209543255 {

/// Converts mission-local initial pose to hidden-map world coordinates.
/// Only the Z axis shifts by map_offset_z; X and Y are unchanged.
[[nodiscard]] inline common::Position3D worldInitialDronePosition(
    common::Position3D local_spawn, common::ZLength map_offset_z) {
    local_spawn.z = local_spawn.z + map_offset_z;
    return local_spawn;
}

/// Calls `fn(sample_position)` for every resolution-grid sample inside the sphere.
/// `fn` returns `false` to stop early. Does not interpret occupancy.
template <typename Fn>
void forEachSphereSample(const common::IMap3D& map, const common::Position3D& center,
                         common::PhysicalLength radius, Fn&& fn) {
    const common::types::MapConfig cfg = map.getMapConfig();
    if (cfg.resolution <= 0.0 * common::cm || radius < 0.0 * common::cm) {
        return;
    }
    const int steps = static_cast<int>(
        std::ceil((radius / cfg.resolution).numerical_value_in(mp_units::one)));
    using common::cm;
    using common::x_extent;
    using common::y_extent;
    using common::z_extent;
    for (int dx = -steps; dx <= steps; ++dx) {
        for (int dy = -steps; dy <= steps; ++dy) {
            for (int dz = -steps; dz <= steps; ++dz) {
                const auto ox = static_cast<double>(dx) * cfg.resolution;
                const auto oy = static_cast<double>(dy) * cfg.resolution;
                const auto oz = static_cast<double>(dz) * cfg.resolution;
                if (ox * ox + oy * oy + oz * oz > radius * radius) {
                    continue;
                }
                const common::Position3D sample{
                    center.x + mp_units::quantity_cast<x_extent>(ox),
                    center.y + mp_units::quantity_cast<y_extent>(oy),
                    center.z + mp_units::quantity_cast<z_extent>(oz),
                };
                if (!fn(sample)) {
                    return;
                }
            }
        }
    }
}

[[nodiscard]] bool sphereHitsOccupiedOrOutOfBounds(const common::IMap3D& map,
                                                   const common::Position3D& center,
                                                   common::PhysicalLength radius);

[[nodiscard]] bool isDroneSpawnPassable(const common::IMap3D& hidden_map,
                                        common::PhysicalLength radius,
                                        const common::Position3D& center);

} // namespace user_common_207190406_209543255
