#pragma once

#include <MissionControl/IDroneControl.h>

namespace common {
class ILidar;
class IGPS;
class IDroneMovement;
class IMutableMap3D;
class IMappingAlgorithm;
} // namespace common

namespace mission_control_207190406_209543255 {

/// Executes one drone step: algorithm command, movement, scan, and voxel fusion.
class DroneControlImpl final : public mission_control::IDroneControl {
public:
    DroneControlImpl(const common::types::DroneConfigData& drone,
                     const common::types::LidarConfigData& lidar,
                     const common::ILidar& lidar_sensor,
                     const common::IGPS& gps,
                     common::IDroneMovement& movement,
                     common::IMutableMap3D& output_map,
                     common::IMappingAlgorithm& mapping_algorithm,
                     common::types::MappingBounds mission_bounds = {});

    [[nodiscard]] common::types::DroneStepResult step() override;
    [[nodiscard]] common::types::DroneState state() const override;

private:
    [[nodiscard]] common::types::DroneStepResult applyMovement(
        const common::types::MappingStepCommand& command);
    void applyScanIfRequested(const common::types::MappingStepCommand& command);

    common::types::DroneConfigData drone_;
    common::types::LidarConfigData lidar_;
    const common::ILidar& lidar_sensor_;
    const common::IGPS& gps_;
    common::IDroneMovement& movement_;
    common::IMutableMap3D& output_map_;
    common::IMappingAlgorithm& mapping_algorithm_;
    common::types::MappingBounds mission_bounds_{};
    common::types::LidarScanResult latest_scan_{};
    bool has_latest_scan_ = false;
    std::size_t step_index_ = 0;
};

} // namespace mission_control_207190406_209543255
