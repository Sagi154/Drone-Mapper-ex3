#include <UserCommon_207190406_209543255/SimulationCoordUtil.h>

#include <cmath>

namespace UserCommon_207190406_209543255 {

namespace {

bool sphereCollides(const common::IMap3D& map,
                    const common::Position3D& center,
                    common::PhysicalLength radius) {
    const common::types::MapConfig cfg = map.getMapConfig();
    const double res_cm = cfg.resolution.numerical_value_in(common::cm);
    if (res_cm <= 0.0) {
        return false;
    }
    const double r_cm = radius.numerical_value_in(common::cm);
    const double cx = center.x.numerical_value_in(common::cm);
    const double cy = center.y.numerical_value_in(common::cm);
    const double cz = center.z.numerical_value_in(common::cm);

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
                    (cx + ox) * common::x_extent[common::cm],
                    (cy + oy) * common::y_extent[common::cm],
                    (cz + oz) * common::z_extent[common::cm],
                };
                if (!map.isInBounds(sample)) {
                    return true;
                }
                if (map.atVoxel(sample) == common::types::VoxelOccupancy::Occupied) {
                    return true;
                }
            }
        }
    }
    return false;
}

} // namespace

common::Position3D worldInitialDronePosition(
    const simulator::types::SimulationConfigData& simulation) {
    common::Position3D world = simulation.initial_drone_position;
    world.z = world.z + simulation.map_offset.z;
    return world;
}

bool isDroneSpawnPassable(const common::IMap3D& hidden_map,
                          common::PhysicalLength radius,
                          const common::Position3D& center) {
    return !sphereCollides(hidden_map, center, radius);
}

} // namespace UserCommon_207190406_209543255
