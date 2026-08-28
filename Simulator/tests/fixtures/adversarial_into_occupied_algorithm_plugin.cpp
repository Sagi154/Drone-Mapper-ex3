// TEST-ONLY adversarial MappingAlgorithm: Advance toward Occupied, else max_advance.
// See ASSUMPTIONS.md in this directory.

#include <Common/IMappingAlgorithm.h>
#include <Common/MappingAlgorithmFactory.h>
#include <Common/MappingAlgorithmRegistration.h>
#include <Common/Types.h>
#include <Common/Units.h>

#include <cmath>
#include <optional>
#include <utility>

namespace {

[[nodiscard]] double cm_of(const common::XLength& q) {
    return q.numerical_value_in(common::cm);
}
[[nodiscard]] double cm_of(const common::YLength& q) {
    return q.numerical_value_in(common::cm);
}
[[nodiscard]] double cm_of(const common::ZLength& q) {
    return q.numerical_value_in(common::cm);
}
[[nodiscard]] double cm_of(const common::PhysicalLength& q) {
    return q.numerical_value_in(common::cm);
}
[[nodiscard]] double deg_of(const common::HorizontalAngle& q) {
    return q.numerical_value_in(common::deg);
}

[[nodiscard]] constexpr double rad_to_deg(double rad) {
    return rad * 180.0 / 3.14159265358979323846;
}

// Wrap to (-180, 180].
[[nodiscard]] double wrap180(double degrees) {
    degrees = std::fmod(degrees, 360.0);
    if (degrees > 180.0) {
        degrees -= 360.0;
    } else if (degrees <= -180.0) {
        degrees += 360.0;
    }
    return degrees;
}

[[nodiscard]] common::Position3D pos_cm(double x, double y, double z) {
    return {common::XLength{x * common::cm}, common::YLength{y * common::cm},
            common::ZLength{z * common::cm}};
}

constexpr double kHeadingAlignDeg = 1.0;

}  // namespace

class AdversarialIntoOccupiedAlgorithm final : public common::IMappingAlgorithm {
public:
    explicit AdversarialIntoOccupiedAlgorithm(common::MappingAlgorithmDependencies dependencies)
        : common::IMappingAlgorithm(std::move(dependencies)) {}

    [[nodiscard]] common::types::MappingStepCommand nextStep(
        const common::types::DroneState& state,
        const common::types::LidarScanResult*) override {
        common::types::MappingStepCommand cmd{};
        cmd.status = common::types::AlgorithmStatus::Working;
        cmd.scan_orientation = std::nullopt;

        const auto occupied = nearestOccupied(state.position);
        if (!occupied.has_value()) {
            cmd.movement = makeAdvance(drone_config_.max_advance);
            return cmd;
        }

        const double dx = cm_of(occupied->x) - cm_of(state.position.x);
        const double dy = cm_of(occupied->y) - cm_of(state.position.y);
        const double dist_xy = std::hypot(dx, dy);

        if (dist_xy > 0.0) {
            const double desired = rad_to_deg(std::atan2(dy, dx));
            const double delta = wrap180(desired - deg_of(state.heading.horizontal));
            if (std::abs(delta) > kHeadingAlignDeg) {
                cmd.movement = makeRotate(delta);
                return cmd;
            }
        }

        cmd.movement = makeAdvance(drone_config_.max_advance);
        return cmd;
    }

private:
    [[nodiscard]] std::optional<common::Position3D> nearestOccupied(
        const common::Position3D& from) const {
        const common::types::MapConfig cfg = output_map_.getMapConfig();
        const double res = cm_of(cfg.resolution);
        if (!(res > 0.0)) {
            return std::nullopt;
        }

        const double min_x = cm_of(cfg.boundaries.min_x);
        const double max_x = cm_of(cfg.boundaries.max_x);
        const double min_y = cm_of(cfg.boundaries.min_y);
        const double max_y = cm_of(cfg.boundaries.max_y);
        const double min_z = cm_of(cfg.boundaries.min_height);
        const double max_z = cm_of(cfg.boundaries.max_height);

        const auto count = [&](double lo, double hi) -> std::size_t {
            if (!(hi > lo)) {
                return 0;
            }
            const double n = std::floor((hi - lo) / res);
            if (n <= 0.0) {
                return 0;
            }
            return static_cast<std::size_t>(n);
        };

        const std::size_t nx = count(min_x, max_x);
        const std::size_t ny = count(min_y, max_y);
        const std::size_t nz = count(min_z, max_z);
        if (nx == 0 || ny == 0 || nz == 0) {
            return std::nullopt;
        }

        const double fx = cm_of(from.x);
        const double fy = cm_of(from.y);
        const double fz = cm_of(from.z);

        std::optional<common::Position3D> best;
        double best_d2 = 0.0;

        for (std::size_t ix = 0; ix < nx; ++ix) {
            const double x = min_x + (static_cast<double>(ix) + 0.5) * res;
            for (std::size_t iy = 0; iy < ny; ++iy) {
                const double y = min_y + (static_cast<double>(iy) + 0.5) * res;
                for (std::size_t iz = 0; iz < nz; ++iz) {
                    const double z = min_z + (static_cast<double>(iz) + 0.5) * res;
                    const common::Position3D p = pos_cm(x, y, z);
                    if (!output_map_.isInBounds(p)) {
                        continue;
                    }
                    if (output_map_.atVoxel(p) != common::types::VoxelOccupancy::Occupied) {
                        continue;
                    }
                    const double ddx = x - fx;
                    const double ddy = y - fy;
                    const double ddz = z - fz;
                    const double d2 = ddx * ddx + ddy * ddy + ddz * ddz;
                    if (!best.has_value() || d2 < best_d2) {
                        best = p;
                        best_d2 = d2;
                    }
                }
            }
        }
        return best;
    }

    [[nodiscard]] static common::types::MovementCommand makeAdvance(
        common::PhysicalLength distance) {
        common::types::MovementCommand move{};
        move.type = common::types::MovementCommandType::Advance;
        move.distance = distance;
        return move;
    }

    [[nodiscard]] static common::types::MovementCommand makeRotate(double delta_deg) {
        common::types::MovementCommand move{};
        move.type = common::types::MovementCommandType::Rotate;
        if (delta_deg > 0.0) {
            move.rotation = common::types::RotationDirection::Right;
            move.angle = common::HorizontalAngle{delta_deg * common::deg};
        } else {
            move.rotation = common::types::RotationDirection::Left;
            move.angle = common::HorizontalAngle{(-delta_deg) * common::deg};
        }
        return move;
    }
};

REGISTER_MAPPING_ALGORITHM(AdversarialIntoOccupiedAlgorithm);
