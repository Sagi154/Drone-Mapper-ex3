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

#include <cmath>
#include <numbers>
#include <stdexcept>

namespace simulator {

namespace {

namespace mp = common::mp;
namespace si = common::si;
using common::cm;
using common::x_extent;
using common::y_extent;
using common::z_extent;
using common::PhysicalLength;
using common::Position3D;

/// Returns true when any voxel within `radius` of `center` is Occupied in
/// `map`, or when any such voxel is out-of-bounds.
bool sphereCollides(const common::IMap3D& map,
                    const Position3D& center,
                    PhysicalLength radius) {
    const common::types::MapConfig cfg = map.getMapConfig();
    const double res_cm = cfg.resolution.numerical_value_in(cm);
    if (res_cm <= 0.0) {
        return false;
    }
    const double r_cm = radius.numerical_value_in(cm);
    const double cx   = center.x.numerical_value_in(cm);
    const double cy   = center.y.numerical_value_in(cm);
    const double cz   = center.z.numerical_value_in(cm);

    const int steps = static_cast<int>(std::ceil(r_cm / res_cm));
    const double r2 = r_cm * r_cm;

    for (int dx = -steps; dx <= steps; ++dx) {
        for (int dy = -steps; dy <= steps; ++dy) {
            for (int dz = -steps; dz <= steps; ++dz) {
                const double ox = dx * res_cm;
                const double oy = dy * res_cm;
                const double oz = dz * res_cm;
                if (ox * ox + oy * oy + oz * oz > r2) {
                    continue;
                }
                const Position3D sample{
                    (cx + ox) * x_extent[cm],
                    (cy + oy) * y_extent[cm],
                    (cz + oz) * z_extent[cm],
                };
                if (!map.isInBounds(sample)) {
                    return true;
                }
                if (map.atVoxel(sample) == common::types::VoxelOccupancy::Occupied) {
                    return true;
                }
            }
        }
    }
    return false;
}

} // namespace

MockMovement::MockMovement(MockGPS& gps,
                           const common::IMap3D& hidden_map,
                           common::types::DroneConfigData drone_config)
    : gps_(gps), hidden_map_(hidden_map), drone_(std::move(drone_config)) {}

common::types::MovementResult MockMovement::rotate(common::types::RotationDirection direction,
                                                    common::HorizontalAngle angle) {
    using common::deg;
    const double deg_val = angle.numerical_value_in(deg);
    const double max_deg = drone_.max_rotate.numerical_value_in(deg);
    if (std::abs(deg_val) > max_deg) {
        return {false, "rotate: angle exceeds max_rotate"};
    }
    const common::Orientation current   = gps_.heading();
    const common::HorizontalAngle delta =
        (direction == common::types::RotationDirection::Left) ? angle : -angle;
    gps_.setHeading(common::Orientation{current.horizontal + delta, current.altitude});
    return {true, {}};
}

common::types::MovementResult MockMovement::advance(common::PhysicalLength distance) {
    using common::cm;
    using common::deg;
    using common::x_extent;
    using common::y_extent;

    const double dist_cm  = distance.numerical_value_in(cm);
    const double limit_cm = drone_.max_advance.numerical_value_in(cm);
    if (std::abs(dist_cm) > limit_cm) {
        return {false, "advance: distance exceeds max_advance"};
    }

    const common::Position3D  pos     = gps_.position();
    const common::Orientation heading  = gps_.heading();
    const double angle_rad =
        heading.horizontal.numerical_value_in(deg) * (std::numbers::pi / 180.0);

    const double dx = std::cos(angle_rad);
    const double dy = std::sin(angle_rad);

    const common::Position3D new_pos{
        pos.x + (dist_cm * dx) * x_extent[cm],
        pos.y + (dist_cm * dy) * y_extent[cm],
        pos.z,
    };

    if (sphereCollides(hidden_map_, new_pos, drone_.radius)) {
        throw std::runtime_error("advance: destination blocked by obstacle or map boundary");
    }

    gps_.setPosition(new_pos);
    return {true, {}};
}

common::types::MovementResult MockMovement::elevate(common::PhysicalLength distance) {
    using common::cm;
    using common::z_extent;

    const double dist_cm  = distance.numerical_value_in(cm);
    const double limit_cm = drone_.max_elevate.numerical_value_in(cm);
    if (std::abs(dist_cm) > limit_cm) {
        return {false, "elevate: distance exceeds max_elevate"};
    }

    const common::Position3D pos = gps_.position();
    const common::Position3D new_pos{
        pos.x,
        pos.y,
        pos.z + dist_cm * z_extent[cm],
    };

    if (sphereCollides(hidden_map_, new_pos, drone_.radius)) {
        throw std::runtime_error("elevate: destination blocked by obstacle or map boundary");
    }

    gps_.setPosition(new_pos);
    return {true, {}};
}

} // namespace simulator
