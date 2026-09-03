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
    using common::cm;
    using common::x_extent;
    using common::y_extent;
    using common::z_extent;
    const double res_cm = cfg.resolution.numerical_value_in(cm);
    const double r_cm = radius.numerical_value_in(cm);
    const double cx = center.x.numerical_value_in(cm);
    const double cy = center.y.numerical_value_in(cm);
    const double cz = center.z.numerical_value_in(cm);
    const int steps = static_cast<int>(std::ceil(r_cm / res_cm));
    const double r2 = r_cm * r_cm;
    for (int dx = -steps; dx <= steps; ++dx) {
        for (int dy = -steps; dy <= steps; ++dy) {
            for (int dz = -steps; dz <= steps; ++dz) {
                const double ox = dx * res_cm;
                const double oy = dy * res_cm;
                const double oz = dz * res_cm;
                if (ox * ox + oy * oy + oz * oz > r2) {
                    continue;
                }
                const common::Position3D sample{
                    (cx + ox) * x_extent[cm],
                    (cy + oy) * y_extent[cm],
                    (cz + oz) * z_extent[cm],
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
