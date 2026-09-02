#include <MissionControl/ScanResultToVoxels.h>

#include <user_common_207190406_209543255/BeamMath.h>
#include <user_common_207190406_209543255/LidarConstants.h>

namespace mission_control_207190406_209543255 {
namespace {

using common::Orientation;
using common::PhysicalLength;
using common::Position3D;
using common::cm;
using user_common_207190406_209543255::kLidarTraceResolutionFactor;
namespace bm = user_common_207190406_209543255::beam_math;

// Evidence strength for conflicting writes to the same voxel.
[[nodiscard]] int occupancyPriority(common::types::VoxelOccupancy occupancy) {
    switch (occupancy) {
    case common::types::VoxelOccupancy::Occupied:
        return 3;
    case common::types::VoxelOccupancy::Empty:
        return 2;
    case common::types::VoxelOccupancy::PotentiallyOccupied:
        return 1;
    case common::types::VoxelOccupancy::Unmapped:
    case common::types::VoxelOccupancy::OutOfBounds:
        return 0;
    }

    return 0;
}

void setIfStronger(common::IMutableMap3D& output_map,
                   const Position3D& position,
                   common::types::VoxelOccupancy value) {
    if (!output_map.isInBounds(position)) {
        return;
    }

    const common::types::VoxelOccupancy current_value = output_map.atVoxel(position);
    if (occupancyPriority(value) > occupancyPriority(current_value)) {
        output_map.set(position, value);
    }
}

void setEmptyIfNotOccupied(common::IMutableMap3D& map, const Position3D& position) {
    if (!map.isInBounds(position)) {
        return;
    }
    if (map.atVoxel(position) != common::types::VoxelOccupancy::Occupied) {
        map.set(position, common::types::VoxelOccupancy::Empty);
    }
}

void setOccupied(common::IMutableMap3D& map, const Position3D& position) {
    if (!map.isInBounds(position)) {
        return;
    }
    map.set(position, common::types::VoxelOccupancy::Occupied);
}

void markBeamSegment(common::IMutableMap3D& output_map,
                     const Position3D& scan_origin,
                     const Orientation& beam_orientation,
                     PhysicalLength start_distance,
                     PhysicalLength end_distance,
                     PhysicalLength step,
                     common::types::VoxelOccupancy value) {
    for (PhysicalLength distance = start_distance; distance <= end_distance; distance += step) {
        const Position3D current_point = bm::pointAlongBeam(scan_origin, beam_orientation, distance);
        if (!output_map.isInBounds(current_point)) {
            break;
        }
        setIfStronger(output_map, current_point, value);
    }
}

// Grid-centre extra so voxel centres along each beam are marked even when the
// sub-voxel trace does not land exactly on a centre.
void supplementGridAlignedFusion(common::IMutableMap3D& map,
                                 const Position3D& scan_origin,
                                 const Orientation& drone_heading,
                                 const common::types::LidarScanResult& scan,
                                 PhysicalLength fusion_max) {
    const PhysicalLength step = map.getMapConfig().resolution;
    if (step <= 0.0 * cm) {
        return;
    }

    for (const common::types::LidarHit& hit : scan) {
        if (bm::isZeroDistance(hit.distance)) {
            continue;
        }

        const Orientation beam_orientation = bm::absoluteBeamOrientation(drone_heading, hit.angle);

        if (bm::isMissDistance(hit.distance)) {
            for (PhysicalLength t = step; t <= fusion_max; t += step) {
                setEmptyIfNotOccupied(map, bm::pointAlongBeam(scan_origin, beam_orientation, t));
            }
            continue;
        }

        if (hit.distance > fusion_max) {
            continue;
        }
        for (PhysicalLength t = step; t < hit.distance; t += step) {
            setEmptyIfNotOccupied(map, bm::pointAlongBeam(scan_origin, beam_orientation, t));
        }
        setOccupied(map, bm::pointAlongBeam(scan_origin, beam_orientation, hit.distance));
    }
}

} // namespace

void applyScanToMap(common::IMutableMap3D& output_map,
                    const Position3D& scan_origin,
                    const Orientation& drone_heading,
                    const common::types::LidarScanResult& scan,
                    const common::types::LidarConfigData& lidar_config) {
    if (!output_map.isInBounds(scan_origin)) {
        return;
    }

    const PhysicalLength step = kLidarTraceResolutionFactor * output_map.getMapConfig().resolution;
    if (step <= 0.0 * cm) {
        return;
    }

    for (const common::types::LidarHit& hit : scan) {
        const Orientation beam_orientation = bm::absoluteBeamOrientation(drone_heading, hit.angle);

        if (bm::isZeroDistance(hit.distance)) {
            markBeamSegment(output_map,
                            scan_origin,
                            beam_orientation,
                            0.0 * cm,
                            lidar_config.z_min,
                            step,
                            common::types::VoxelOccupancy::PotentiallyOccupied);
            continue;
        }

        if (bm::isMissDistance(hit.distance)) {
            markBeamSegment(output_map,
                            scan_origin,
                            beam_orientation,
                            0.0 * cm,
                            lidar_config.z_max,
                            step,
                            common::types::VoxelOccupancy::Empty);
            continue;
        }

        if (hit.distance > 0.0 * cm) {
            markBeamSegment(output_map,
                            scan_origin,
                            beam_orientation,
                            0.0 * cm,
                            hit.distance,
                            step,
                            common::types::VoxelOccupancy::Empty);
            setIfStronger(output_map,
                          bm::pointAlongBeam(scan_origin, beam_orientation, hit.distance),
                          common::types::VoxelOccupancy::Occupied);
        }
    }

    supplementGridAlignedFusion(output_map, scan_origin, drone_heading, scan, lidar_config.z_max);
}

} // namespace mission_control_207190406_209543255
