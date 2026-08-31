#pragma once

#include "MappingAlgorithmFrontier.h"

#include <user_common_207190406_209543255/ConeTemplate.h>

#include <Common/IMap3D.h>
#include <Common/types/DroneTypes.h>
#include <Common/types/LidarTypes.h>

#include <optional>
#include <vector>

namespace algorithm_207190406_209543255::detail {

[[nodiscard]] bool isGainMasked(const GridKey& key, const FrontierCells& frontier);

/// Outdoor cubes (small_out / large_out): height and both XY spans >= 200 cm.
[[nodiscard]] bool isOpenVolumeMission(const common::types::MapConfig& config);

/// small_out (200 cm cube), not large_out (300).
[[nodiscard]] bool isSmallOutdoorMission(const common::types::MapConfig& config);

/// Wide low box (house_full): XY >= 250 cm and height in [100, 200).
[[nodiscard]] bool isHouseVolumeMission(const common::types::MapConfig& config);

[[nodiscard]] std::vector<common::Orientation> buildSweepDirections(
    const common::IMap3D& map,
    const common::Position3D& origin,
    const common::types::LidarConfigData& lidar,
    const FrontierCells& frontier,
    const std::vector<user_common_207190406_209543255::cone_template::ConeTemplate>&
        templates,
    user_common_207190406_209543255::cone_template::VoxelStamp& stamp);

/// World-frame travel-scan orientation. The executor converts to the drone frame.
[[nodiscard]] std::optional<common::Orientation> bestTravelScan(
    const common::IMap3D& map,
    const common::types::DroneState& predicted,
    const common::Position3D& next_waypoint,
    const common::types::LidarConfigData& lidar,
    const FrontierCells& frontier,
    const std::vector<user_common_207190406_209543255::cone_template::ConeTemplate>&
        templates,
    user_common_207190406_209543255::cone_template::VoxelStamp& stamp);

[[nodiscard]] bool clusterStillFrontier(const common::IMap3D& map,
                                        const std::vector<GridKey>& keys);

} // namespace
