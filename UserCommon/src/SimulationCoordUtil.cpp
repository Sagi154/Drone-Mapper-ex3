#include <user_common_207190406_209543255/SimulationCoordUtil.h>

namespace user_common_207190406_209543255 {

bool sphereHitsOccupiedOrOutOfBounds(const common::IMap3D& map,
                                     const common::Position3D& center,
                                     common::PhysicalLength radius) {
    bool hit = false;
    forEachSphereSample(map, center, radius, [&](const common::Position3D& sample) {
        if (!map.isInBounds(sample) ||
            map.atVoxel(sample) == common::types::VoxelOccupancy::Occupied) {
            hit = true;
            return false;
        }
        return true;
    });
    return hit;
}

bool isDroneSpawnPassable(const common::IMap3D& hidden_map, common::PhysicalLength radius,
                          const common::Position3D& center) {
    return !sphereHitsOccupiedOrOutOfBounds(hidden_map, center, radius);
}

} // namespace user_common_207190406_209543255
