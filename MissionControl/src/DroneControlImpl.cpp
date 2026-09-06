#include <MissionControl/DroneControlImpl.h>

#include <MissionControl/ScanResultToVoxels.h>

#include <Common/IDroneMovement.h>
#include <Common/IGPS.h>
#include <Common/ILidar.h>
#include <Common/IMappingAlgorithm.h>
#include <Common/IMutableMap3D.h>

#include <user_common_207190406_209543255/SimulationCoordUtil.h>

#include <mp-units/math.h>

#include <cmath>
#include <exception>
#include <numbers>

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

[[nodiscard]] bool isUnsetMissionBounds(const common::types::MappingBounds& bounds) {
    using common::cm;
    using common::x_extent;
    using common::y_extent;
    using common::z_extent;
    return bounds.min_x == 0.0 * x_extent[cm] && bounds.max_x == 0.0 * x_extent[cm] &&
           bounds.min_y == 0.0 * y_extent[cm] && bounds.max_y == 0.0 * y_extent[cm] &&
           bounds.min_height == 0.0 * z_extent[cm] &&
           bounds.max_height == 0.0 * z_extent[cm];
}

[[nodiscard]] Position3D predictedDestination(const Position3D& pos,
                                              const common::Orientation& heading,
                                              const common::types::MovementCommand& command) {
    using common::cm;
    using common::deg;
    using common::x_extent;
    using common::y_extent;
    using common::z_extent;
    if (command.type == common::types::MovementCommandType::Elevate) {
        const double dist_cm = command.distance.numerical_value_in(cm);
        return Position3D{pos.x, pos.y, pos.z + dist_cm * z_extent[cm]};
    }
    if (command.type != common::types::MovementCommandType::Advance) {
        return pos;
    }
    const double dist_cm = command.distance.numerical_value_in(cm);
    const double angle_rad =
        heading.horizontal.numerical_value_in(deg) * (std::numbers::pi / 180.0);
    const double dx = std::cos(angle_rad);
    const double dy = std::sin(angle_rad);
    return Position3D{
        pos.x + (dist_cm * dx) * x_extent[cm],
        pos.y + (dist_cm * dy) * y_extent[cm],
        pos.z,
    };
}

[[nodiscard]] common::types::MovementCommand clampMovementToMissionBounds(
    common::types::MovementCommand command, const Position3D& pos,
    const common::Orientation& heading, const common::types::MappingBounds& bounds) {
    using common::cm;
    using common::deg;
    if (command.type == common::types::MovementCommandType::Elevate) {
        const double z0 = pos.z.numerical_value_in(cm);
        const double dz = command.distance.numerical_value_in(cm);
        const double z1 = z0 + dz;
        const double zmin = bounds.min_height.numerical_value_in(cm);
        const double zmax = bounds.max_height.numerical_value_in(cm);
        double clamped_z = z1;
        if (clamped_z > zmax) {
            clamped_z = zmax;
        }
        if (clamped_z < zmin) {
            clamped_z = zmin;
        }
        command.distance = (clamped_z - z0) * cm;
        return command;
    }
    if (command.type != common::types::MovementCommandType::Advance) {
        return command;
    }
    const double dist = command.distance.numerical_value_in(cm);
    if (dist <= 0.0) {
        return command;
    }
    const double rad =
        heading.horizontal.numerical_value_in(deg) * (std::numbers::pi / 180.0);
    const double dirx = std::cos(rad);
    const double diry = std::sin(rad);
    const double px = pos.x.numerical_value_in(cm);
    const double py = pos.y.numerical_value_in(cm);
    double t = dist;
    constexpr double kEps = 1e-9;
    const double xmin = bounds.min_x.numerical_value_in(cm);
    const double xmax = bounds.max_x.numerical_value_in(cm);
    const double ymin = bounds.min_y.numerical_value_in(cm);
    const double ymax = bounds.max_y.numerical_value_in(cm);
    if (dirx > kEps) {
        t = std::min(t, (xmax - px) / dirx);
    } else if (dirx < -kEps) {
        t = std::min(t, (xmin - px) / dirx);
    }
    if (diry > kEps) {
        t = std::min(t, (ymax - py) / diry);
    } else if (diry < -kEps) {
        t = std::min(t, (ymin - py) / diry);
    }
    if (t < 0.0) {
        t = 0.0;
    }
    command.distance = t * cm;
    return command;
}

[[nodiscard]] bool isZeroLengthMove(const common::types::MovementCommand& command) {
    using common::cm;
    using common::deg;
    switch (command.type) {
    case common::types::MovementCommandType::Advance:
    case common::types::MovementCommandType::Elevate:
        return mp_units::abs(command.distance) <= 0.0 * cm;
    case common::types::MovementCommandType::Rotate:
        return command.angle <= 0.0 * common::horizontal_angle[deg];
    case common::types::MovementCommandType::Hover:
        return false;
    }
    return false;
}

} // namespace

DroneControlImpl::DroneControlImpl(const common::types::DroneConfigData& drone,
                                   const common::types::LidarConfigData& lidar,
                                   const common::ILidar& lidar_sensor,
                                   const common::IGPS& gps,
                                   common::IDroneMovement& movement,
                                   common::IMutableMap3D& output_map,
                                   common::IMappingAlgorithm& mapping_algorithm,
                                   common::types::MappingBounds mission_bounds)
    : drone_(drone),
      lidar_(lidar),
      lidar_sensor_(lidar_sensor),
      gps_(gps),
      movement_(movement),
      output_map_(output_map),
      mapping_algorithm_(mapping_algorithm),
      mission_bounds_(mission_bounds) {}

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
    common::types::MovementCommand move = *command.movement;
    const Position3D here = gps_.position();
    const common::Orientation heading = gps_.heading();
    if (!isUnsetMissionBounds(mission_bounds_)) {
        move = clampMovementToMissionBounds(move, here, heading, mission_bounds_);
    }
    if (isZeroLengthMove(move)) {
        return {common::types::DroneStepStatus::Continue, {}};
    }
    if (move.type == common::types::MovementCommandType::Advance ||
        move.type == common::types::MovementCommandType::Elevate) {
        const Position3D dest = predictedDestination(here, heading, move);
        if (!output_map_.isInBounds(dest)) {
            return {common::types::DroneStepStatus::Continue, {}};
        }
    }
    try {
        const common::types::MovementResult movement_result = executeMovement(movement_, move);
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
