// PathShaping.cpp — string-pulling and the command-level step-cost model.

#include "PathShaping.h"

#include "MappingAlgorithmFrontier.h"

#include <mp-units/math.h>
#include <mp-units/systems/si/math.h>

#include <cmath>
#include <numbers>

namespace algorithm_207190406_209543255::detail {

namespace {

using common::Orientation;
using common::Position3D;
using common::cm;
using common::deg;
using common::x_extent;
using common::y_extent;
using common::z_extent;

constexpr common::ZLength kSameAltitudeEpsilon = 1e-6 * z_extent[cm];
constexpr common::PhysicalLength kSameAxisEpsilon = 1e-6 * cm;

[[nodiscard]] bool sameAltitude(const Position3D& a, const Position3D& b) {
    return mp_units::abs(a.z - b.z) <= kSameAltitudeEpsilon;
}

[[nodiscard]] std::size_t ceilDiv(double amount, double per_step) {
    if (!(per_step > 0.0) || amount <= 0.0) {
        return 0;
    }
    return static_cast<std::size_t>(std::ceil(amount / per_step - 1e-9));
}

[[nodiscard]] double wrapDeg(double degrees) {
    double x = std::fmod(degrees, 360.0);
    if (x <= -180.0) {
        x += 360.0;
    } else if (x > 180.0) {
        x -= 360.0;
    }
    return x;
}

} // namespace

std::vector<Position3D> stringPullConstantAltitude(const common::IMap3D& map,
                                                   const std::vector<Position3D>& path,
                                                   common::PhysicalLength drone_radius) {
    if (path.size() <= 2) {
        return path;
    }

    std::vector<Position3D> out;
    out.reserve(path.size());
    out.push_back(path.front());

    std::size_t anchor = 0;
    while (anchor + 1 < path.size()) {
        std::size_t best = anchor + 1;
        for (std::size_t probe = anchor + 2; probe < path.size(); ++probe) {
            if (!sameAltitude(path[anchor], path[probe])) {
                break;
            }
            if (!hasClearLineOfSight(map, path[anchor], path[probe], drone_radius)) {
                break;
            }
            best = probe;
        }
        out.push_back(path[best]);
        anchor = best;
    }

    return out;
}

std::size_t stepCostForPath(const std::vector<Position3D>& waypoints,
                            const Position3D& start_position,
                            const Orientation& start_heading,
                            const MovementLimits& limits) {
    const double advance_cm = limits.max_advance.numerical_value_in(cm);
    const double elevate_cm = limits.max_elevate.numerical_value_in(cm);
    const double rotate_deg = limits.max_rotate.numerical_value_in(deg);

    std::size_t steps = 0;
    Position3D from = start_position;
    double heading_deg = start_heading.horizontal.numerical_value_in(deg);

    for (const Position3D& to : waypoints) {
        const Position3D d = to - from;
        const auto dz = mp_units::quantity_cast<common::isq::length>(d.z);
        if (mp_units::abs(dz) > kSameAxisEpsilon) {
            steps += ceilDiv(mp_units::abs(dz).numerical_value_in(cm), elevate_cm);
        }

        const auto dx = mp_units::quantity_cast<common::isq::length>(d.x);
        const auto dy = mp_units::quantity_cast<common::isq::length>(d.y);
        const auto planar = mp_units::sqrt(dx * dx + dy * dy);
        if (planar > kSameAxisEpsilon) {
            const double target_deg =
                std::atan2(dy.numerical_value_in(cm), dx.numerical_value_in(cm)) *
                (180.0 / std::numbers::pi);
            const double turn = std::abs(wrapDeg(target_deg - heading_deg));
            if (turn > 1e-9) {
                steps += ceilDiv(turn, rotate_deg);
                heading_deg = target_deg;
            }
            steps += ceilDiv(planar.numerical_value_in(cm), advance_cm);
        }

        from = to;
    }

    return steps;
}

} // namespace algorithm_207190406_209543255::detail
