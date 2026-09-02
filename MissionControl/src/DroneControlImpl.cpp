#include <MissionControl/DroneControlImpl.h>

#include <MissionControl/ScanResultToVoxels.h>

#include <user_common_207190406_209543255/SimulationCoordUtil.h>

#include <mp-units/math.h>

#include <exception>

namespace mission_control_207190406_209543255 {

namespace {

using common::PhysicalLength;
using common::Position3D;
using user_common_207190406_209543255::forEachSphereSample;

void markDroneFootprintEmpty(common::IMutableMap3D& map, const Position3D& centre,
                             PhysicalLength radius) {
    forEachSphereSample(map, centre, radius, [&](const Position3D& sample) {
        if (map.atVoxel(sample) != common::types::VoxelOccupancy::Occupied) {
            map.set(sample, common::types::VoxelOccupancy::Empty);
        }
        return true;
    });
}

[[nodiscard]] bool isSupportedMovementType(common::types::MovementCommandType type) {
    switch (type) {
    case common::types::MovementCommandType::Hover:
    case common::types::MovementCommandType::Rotate:
    case common::types::MovementCommandType::Advance:
    case common::types::MovementCommandType::Elevate:
        return true;
    }
    return false;
}

[[nodiscard]] bool movementWithinLimits(const common::types::MovementCommand& command,
                                        const common::types::DroneConfigData& drone) {
    switch (command.type) {
    case common::types::MovementCommandType::Hover:
        return true;
    case common::types::MovementCommandType::Rotate:
        return command.angle <= drone.max_rotate;
    case common::types::MovementCommandType::Advance:
        return command.distance <= drone.max_advance;
    case common::types::MovementCommandType::Elevate:
        return mp_units::abs(command.distance) <= drone.max_elevate;
    }
    return false;
}

[[nodiscard]] common::types::MovementResult executeMovement(
    common::IDroneMovement& movement, const common::types::MovementCommand& command) {
    switch (command.type) {
    case common::types::MovementCommandType::Hover:
        return common::types::MovementResult{true, {}};
    case common::types::MovementCommandType::Rotate:
        return movement.rotate(command.rotation, command.angle);
    case common::types::MovementCommandType::Advance:
        return movement.advance(command.distance);
    case common::types::MovementCommandType::Elevate:
        return movement.elevate(command.distance);
    }
    return common::types::MovementResult{false, "Unsupported movement command."};
}

} // namespace

DroneControlImpl::DroneControlImpl(const common::types::DroneConfigData& drone,
                                   const common::types::MissionConfigData& mission,
                                   const common::types::LidarConfigData& lidar,
                                   common::ILidar& lidar_sensor,
                                   common::IGPS& gps,
                                   common::IDroneMovement& movement,
                                   common::IMutableMap3D& output_map,
                                   common::IMappingAlgorithm& mapping_algorithm)
    : drone_(drone),
      mission_(mission),
      lidar_(lidar),
      lidar_sensor_(lidar_sensor),
      gps_(gps),
      movement_(movement),
      output_map_(output_map),
      mapping_algorithm_(mapping_algorithm) {}

common::types::DroneStepResult DroneControlImpl::applyMovement(
    const common::types::MappingStepCommand& command) {
    if (!command.movement.has_value()) {
        return {common::types::DroneStepStatus::Continue, {}};
    }
    if (!isSupportedMovementType(command.movement->type)) {
        return {common::types::DroneStepStatus::Error, "Unsupported movement command."};
    }
    if (!movementWithinLimits(*command.movement, drone_)) {
        return {common::types::DroneStepStatus::Error, "Movement command exceeds drone limits."};
    }
    try {
        const common::types::MovementResult movement_result =
            executeMovement(movement_, *command.movement);
        if (!movement_result.success) {
            return {common::types::DroneStepStatus::Continue, {}};
        }
    } catch (const std::exception&) {
        return {common::types::DroneStepStatus::Continue, {}};
    }
    return {common::types::DroneStepStatus::Continue, {}};
}

void DroneControlImpl::applyScanIfRequested(const common::types::MappingStepCommand& command) {
    if (!command.scan_orientation.has_value()) {
        return;
    }
    latest_scan_ = lidar_sensor_.scan(*command.scan_orientation);
    has_latest_scan_ = true;
    applyScanToMap(output_map_, gps_.position(), gps_.heading(), latest_scan_, lidar_);
}

common::types::DroneStepResult DroneControlImpl::step() {
    const common::types::DroneState current_state = state();
    markDroneFootprintEmpty(output_map_, current_state.position, drone_.radius);

    const common::types::LidarScanResult* latest_scan_ptr =
        has_latest_scan_ ? &latest_scan_ : nullptr;
    const common::types::MappingStepCommand command =
        mapping_algorithm_.nextStep(current_state, latest_scan_ptr);

    if (command.status == common::types::AlgorithmStatus::Finished ||
        command.status == common::types::AlgorithmStatus::FinishedWithUnmappableVoxels) {
        return {common::types::DroneStepStatus::Completed, {}};
    }

    const auto move_result = applyMovement(command);
    if (move_result.status == common::types::DroneStepStatus::Error) {
        return move_result;
    }

    const bool pose_changed = gps_.position().x != current_state.position.x ||
                              gps_.position().y != current_state.position.y ||
                              gps_.position().z != current_state.position.z;
    applyScanIfRequested(command);
    if (command.scan_orientation.has_value() && pose_changed) {
        markDroneFootprintEmpty(output_map_, gps_.position(), drone_.radius);
    }

    ++step_index_;
    return {common::types::DroneStepStatus::Continue, {}};
}

common::types::DroneState DroneControlImpl::state() const {
    return {gps_.position(), gps_.heading(), step_index_};
}

} // namespace mission_control_207190406_209543255
