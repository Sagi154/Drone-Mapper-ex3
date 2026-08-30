// PathShaping.cpp — string-pulling and the command-level step-cost model.

#include "PathShaping.h"

#include "MappingAlgorithmFrontier.h"

#include <cmath>
#include <numbers>

namespace algorithm_207190406_209543255::detail {

namespace {

using common::Orientation;
using common::Position3D;
using common::cm;
using common::deg;

constexpr double kSameAltitudeEpsilonCm = 1e-6;
constexpr double kSameAxisEpsilonCm = 1e-6;

[[nodiscard]] double xCm(const Position3D& p) { return p.x.force_numerical_value_in(cm); }
[[nodiscard]] double yCm(const Position3D& p) { return p.y.force_numerical_value_in(cm); }
[[nodiscard]] double zCm(const Position3D& p) { return p.z.force_numerical_value_in(cm); }

[[nodiscard]] bool sameAltitude(const Position3D& a, const Position3D& b) {
    return std::abs(zCm(a) - zCm(b)) <= kSameAltitudeEpsilonCm;
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
                break;  // an altitude change ends the mergeable run
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
    const double advance_cm = limits.max_advance.force_numerical_value_in(cm);
    const double elevate_cm = limits.max_elevate.force_numerical_value_in(cm);
    const double rotate_deg = limits.max_rotate.force_numerical_value_in(deg);

    std::size_t steps = 0;
    Position3D from = start_position;
    double heading_deg = start_heading.horizontal.force_numerical_value_in(deg);

    for (const Position3D& to : waypoints) {
        const double dz = zCm(to) - zCm(from);
        if (std::abs(dz) > kSameAxisEpsilonCm) {
            steps += ceilDiv(std::abs(dz), elevate_cm);
        }

        const double dx = xCm(to) - xCm(from);
        const double dy = yCm(to) - yCm(from);
        const double planar = std::sqrt(dx * dx + dy * dy);
        if (planar > kSameAxisEpsilonCm) {
            const double target_deg = std::atan2(dy, dx) * (180.0 / std::numbers::pi);
            const double turn = std::abs(wrapDeg(target_deg - heading_deg));
            if (turn > 1e-9) {
                steps += ceilDiv(turn, rotate_deg);
                heading_deg = target_deg;
            }
            steps += ceilDiv(planar, advance_cm);
        }

        from = to;
    }

    return steps;
}

} // namespace algorithm_207190406_209543255::detail
