#include <MissionControl/DroneControlImpl.h>

#include <MissionControl/ScanResultToVoxels.h>

#include "BeamMath.hpp"

#include <cmath>
#include <exception>
#include <utility>

namespace mission_control_207190406_209543255 {

namespace {

using common::Orientation;
using common::PhysicalLength;
using common::Position3D;
using common::cm;
using common::x_extent;
using common::y_extent;
using common::z_extent;
namespace bm = beam_math;

void markDroneFootprintEmpty(common::IMutableMap3D& map,
                             const Position3D& centre,
                             PhysicalLength radius) {
    const common::types::MapConfig config = map.getMapConfig();
    const double resolution_cm = config.resolution.force_numerical_value_in(cm);
    if (resolution_cm <= 0.0) {
        return;
    }

    const double radius_cm = radius.force_numerical_value_in(cm);
    const double cx = centre.x.force_numerical_value_in(cm);
    const double cy = centre.y.force_numerical_value_in(cm);
    const double cz = centre.z.force_numerical_value_in(cm);
    const int steps = static_cast<int>(std::ceil(radius_cm / resolution_cm));
    const double radius_sq = radius_cm * radius_cm;

    for (int dx = -steps; dx <= steps; ++dx) {
        for (int dy = -steps; dy <= steps; ++dy) {
            for (int dz = -steps; dz <= steps; ++dz) {
                const double ox = dx * resolution_cm;
                const double oy = dy * resolution_cm;
                const double oz = dz * resolution_cm;
                if (ox * ox + oy * oy + oz * oz > radius_sq) {
                    continue;
                }
                const Position3D sample{
                    (cx + ox) * x_extent[cm],
                    (cy + oy) * y_extent[cm],
                    (cz + oz) * z_extent[cm],
                };
                if (map.atVoxel(sample) != common::types::VoxelOccupancy::Occupied) {
                    map.set(sample, common::types::VoxelOccupancy::Empty);
                }
            }
        }
    }
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
        return std::abs(command.distance.force_numerical_value_in(cm)) <=
               drone.max_elevate.force_numerical_value_in(cm);
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

[[nodiscard]] bool isRecoverableMovementFailure(const std::string& message) {
    return message.find("blocked") != std::string::npos ||
           message.find("boundary") != std::string::npos;
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

// Grid-aligned fusion supplement so voxel centres inside the proximity bubble are marked.
void supplementGridAlignedFusion(common::IMutableMap3D& map,
                                 const Position3D& scan_origin,
                                 const Orientation& drone_heading,
                                 const common::types::LidarScanResult& scan,
                                 PhysicalLength fusion_max) {
    const double step_cm = map.getMapConfig().resolution.force_numerical_value_in(cm);
    if (step_cm <= 0.0) {
        return;
    }
    const double fusion_max_cm = fusion_max.force_numerical_value_in(cm);

    for (const common::types::LidarHit& hit : scan) {
        if (hit.distance == 0.0 * cm) {
            continue;
        }

        const Orientation beam_orientation =
            bm::absoluteBeamOrientation(drone_heading, hit.angle);

        if (bm::isMissDistance(hit.distance)) {
            for (double t_cm = step_cm; t_cm <= fusion_max_cm + 1e-9; t_cm += step_cm) {
                setEmptyIfNotOccupied(
                    map, bm::pointAlongBeam(scan_origin, beam_orientation, t_cm * cm));
            }
            continue;
        }

        const double hit_cm = hit.distance.force_numerical_value_in(cm);
        if (hit_cm > fusion_max_cm) {
            continue;
        }
        for (double t_cm = step_cm; t_cm < hit_cm - 1e-9; t_cm += step_cm) {
            setEmptyIfNotOccupied(
                map, bm::pointAlongBeam(scan_origin, beam_orientation, t_cm * cm));
        }
        setOccupied(map, bm::pointAlongBeam(scan_origin, beam_orientation, hit.distance));
    }
}

} // namespace

DroneControlImpl::DroneControlImpl(common::types::DroneConfigData drone,
                                   common::types::MissionConfigData mission,
                                   common::types::LidarConfigData lidar,
                                   common::ILidar& lidar_sensor,
                                   common::IGPS& gps,
                                   common::IDroneMovement& movement,
                                   common::IMutableMap3D& output_map,
                                   common::IMappingAlgorithm& mapping_algorithm)
    : drone_(std::move(drone)),
      mission_(std::move(mission)),
      lidar_(std::move(lidar)),
      lidar_sensor_(lidar_sensor),
      gps_(gps),
      movement_(movement),
      output_map_(output_map),
      mapping_algorithm_(mapping_algorithm) {}

common::types::DroneStepResult DroneControlImpl::step() {
    const common::types::DroneState current_state = state();
    markDroneFootprintEmpty(output_map_, current_state.position, drone_.radius);

    const common::types::LidarScanResult* latest_scan_ptr =
        has_latest_scan_ ? &latest_scan_ : nullptr;
    common::types::MappingStepCommand command =
        mapping_algorithm_.nextStep(current_state, latest_scan_ptr);

    if (command.status == common::types::AlgorithmStatus::Finished ||
        command.status == common::types::AlgorithmStatus::FinishedWithUnmappableVoxels) {
        return common::types::DroneStepResult{common::types::DroneStepStatus::Completed, {}};
    }

    // Batch consecutive scan commands into a single mission step.
    // Cap the batch: an algorithm that always returns Working+scan (VAR-03
    // adversarial_bad_scan) would otherwise loop forever inside one step.
    constexpr std::size_t kMaxScansPerStep = 16;
    std::size_t scans_this_step = 0;
    while (command.scan_orientation.has_value() &&
           command.status == common::types::AlgorithmStatus::Working &&
           scans_this_step < kMaxScansPerStep) {
        latest_scan_ = lidar_sensor_.scan(*command.scan_orientation);
        has_latest_scan_ = true;
        ++scans_this_step;
        // fusion_max removed in ex3 — always fuse at full lidar z_max.
        ScanResultToVoxels::applyToMap(
            output_map_, gps_.position(), gps_.heading(), latest_scan_, lidar_);
        supplementGridAlignedFusion(
            output_map_, gps_.position(), gps_.heading(), latest_scan_, lidar_.z_max);
        markDroneFootprintEmpty(output_map_, gps_.position(), drone_.radius);

        const common::types::DroneState post_scan_state = state();
        latest_scan_ptr = &latest_scan_;
        command = mapping_algorithm_.nextStep(post_scan_state, latest_scan_ptr);

        if (command.status == common::types::AlgorithmStatus::Finished ||
            command.status == common::types::AlgorithmStatus::FinishedWithUnmappableVoxels) {
            ++step_index_;
            return common::types::DroneStepResult{common::types::DroneStepStatus::Completed, {}};
        }
    }

    if (command.movement.has_value()) {
        if (!movementWithinLimits(*command.movement, drone_)) {
            return common::types::DroneStepResult{
                common::types::DroneStepStatus::Error,
                "Movement command exceeds drone limits.",
            };
        }

        try {
            const common::types::MovementResult movement_result =
                executeMovement(movement_, *command.movement);
            if (!movement_result) {
                if (isRecoverableMovementFailure(movement_result.message)) {
                    ++step_index_;
                    return common::types::DroneStepResult{
                        common::types::DroneStepStatus::Continue, {}};
                }
                return common::types::DroneStepResult{
                    common::types::DroneStepStatus::Error,
                    movement_result.message.empty() ? "Movement failed."
                                                    : movement_result.message,
                };
            }
        } catch (const std::exception& ex) {
            // MockMovement throws on wall/boundary; recover like a false MovementResult.
            // Non-recoverable exceptions rethrow for SimulationRunImpl.
            if (isRecoverableMovementFailure(ex.what())) {
                ++step_index_;
                return common::types::DroneStepResult{
                    common::types::DroneStepStatus::Continue, {}};
            }
            throw;
        }
    }

    ++step_index_;
    return common::types::DroneStepResult{common::types::DroneStepStatus::Continue, {}};
}

common::types::DroneState DroneControlImpl::state() const {
    return common::types::DroneState{gps_.position(), gps_.heading(), step_index_};
}

} // namespace mission_control_207190406_209543255
