#pragma once

#include <MissionControl/IDroneControl.h>

#include <Common/IDroneMovement.h>
#include <Common/IGPS.h>
#include <Common/ILidar.h>
#include <Common/IMappingAlgorithm.h>
#include <Common/IMutableMap3D.h>

namespace mission_control_207190406_209543255 {

/// Executes one drone step: algorithm command, movement, scan, and voxel fusion.
class DroneControlImpl final : public mission_control::IDroneControl {
public:
    DroneControlImpl(const common::types::DroneConfigData& drone,
                     const common::types::MissionConfigData& mission,
                     const common::types::LidarConfigData& lidar,
                     common::ILidar& lidar_sensor,
                     common::IGPS& gps,
                     common::IDroneMovement& movement,
                     common::IMutableMap3D& output_map,
                     common::IMappingAlgorithm& mapping_algorithm);

    [[nodiscard]] common::types::DroneStepResult step() override;
    [[nodiscard]] common::types::DroneState state() const override;

private:
    [[nodiscard]] common::types::DroneStepResult applyMovement(
        const common::types::MappingStepCommand& command);
    void applyScanIfRequested(const common::types::MappingStepCommand& command);

    common::types::DroneConfigData drone_;
    common::types::MissionConfigData mission_;
    common::types::LidarConfigData lidar_;
    common::ILidar& lidar_sensor_;
    common::IGPS& gps_;
    common::IDroneMovement& movement_;
    common::IMutableMap3D& output_map_;
    common::IMappingAlgorithm& mapping_algorithm_;
    common::types::LidarScanResult latest_scan_{};
    bool has_latest_scan_ = false;
    std::size_t step_index_ = 0;
};

} // namespace mission_control_207190406_209543255
