#pragma once

#include <Common/IMutableMap3D.h>
#include <Common/types/LidarTypes.h>

namespace mission_control_207190406_209543255 {

void applyScanToMap(common::IMutableMap3D& output_map,
                    const common::Position3D& scan_origin,
                    const common::Orientation& drone_heading,
                    const common::types::LidarScanResult& scan,
                    const common::types::LidarConfigData& lidar_config);

} // namespace mission_control_207190406_209543255
