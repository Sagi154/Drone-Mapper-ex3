// MockMovement.cpp
// Simulated drone movement driver. Each command is validated against the
// per-command limits stored in DroneConfigData; oversized commands are
// rejected immediately with success=false.
//
// For advance and elevate, sphere collision detection is performed after
// computing the candidate new position: every grid cell within drone_radius
// of the new centre is checked against the hidden truth map.
// Out-of-bounds positions count as blocked (the drone cannot leave the map).
// If a collision is found, std::runtime_error is thrown — this is the
// mandatory collision scenario in docs/error-handling-matrix.md.
// If no collision is found, the GPS is updated (snapping happens inside
// MockGPS::setPosition).
//
// Heading convention: horizontal angle 0° = +X, 90° = +Y.

#include <Simulator/MockMovement.h>
#include <Simulator/MockGPS.h>

#include <user_common_207190406_209543255/BeamMath.h>
#include <user_common_207190406_209543255/SimulationCoordUtil.h>

#include <mp-units/math.h>

#include <stdexcept>

namespace simulator {

MockMovement::MockMovement(MockGPS& gps,
                           const common::IMap3D& hidden_map,
                           const common::types::DroneConfigData& drone_config)
    : gps_(gps), hidden_map_(hidden_map), drone_(drone_config) {}

common::types::MovementResult MockMovement::rotate(common::types::RotationDirection direction,
                                                    common::HorizontalAngle angle) {
    if (mp_units::abs(angle) > drone_.max_rotate) {
        return {false, "rotate: angle exceeds max_rotate"};
    }
    const common::Orientation current = gps_.heading();
    const common::HorizontalAngle delta =
        (direction == common::types::RotationDirection::Left) ? angle : -angle;
    gps_.setHeading(common::Orientation{current.horizontal + delta, current.altitude});
    return {true, {}};
}

common::types::MovementResult MockMovement::advance(common::PhysicalLength distance) {
    if (mp_units::abs(distance) > drone_.max_advance) {
        return {false, "advance: distance exceeds max_advance"};
    }
    const common::Position3D pos = gps_.position();
    const common::Orientation heading = gps_.heading();
    const common::Position3D new_pos = user_common_207190406_209543255::beam_math::pointAlongBeam(
        pos, common::Orientation{heading.horizontal, 0.0 * common::deg}, distance);
    if (user_common_207190406_209543255::sphereHitsOccupiedOrOutOfBounds(
            hidden_map_, new_pos, drone_.radius)) {
        throw std::runtime_error("advance: destination blocked by obstacle or map boundary");
    }
    gps_.setPosition(new_pos);
    return {true, {}};
}

common::types::MovementResult MockMovement::elevate(common::PhysicalLength distance) {
    if (mp_units::abs(distance) > drone_.max_elevate) {
        return {false, "elevate: distance exceeds max_elevate"};
    }
    const common::Position3D pos = gps_.position();
    const common::Position3D new_pos{
        pos.x, pos.y, pos.z + mp_units::quantity_cast<common::z_extent>(distance)};
    if (user_common_207190406_209543255::sphereHitsOccupiedOrOutOfBounds(
            hidden_map_, new_pos, drone_.radius)) {
        throw std::runtime_error("elevate: destination blocked by obstacle or map boundary");
    }
    gps_.setPosition(new_pos);
    return {true, {}};
}

} // namespace simulator
