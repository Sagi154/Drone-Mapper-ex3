#pragma once

// Lidar cone FOV helpers for Algorithm scan planning (and future NBV scoring).
// Half-angle matches MockLidar / HostLidar: atan2((fov_circles-1)*d, z_min).

#include <user_common_207190406_209543255/BeamMath.h>

#include <Common/IMap3D.h>
#include <Common/types/LidarTypes.h>

#include <cmath>
#include <cstddef>
#include <numbers>
#include <vector>

namespace user_common_207190406_209543255::lidar_cone {

namespace bm = user_common_207190406_209543255::beam_math;
using common::Orientation;
using common::Position3D;
using common::cm;
using common::deg;

[[nodiscard]] inline double coneHalfAngleRad(const common::types::LidarConfigData& cfg) {
    if (cfg.fov_circles == 0) {
        return 0.0;
    }
    const double z_min_cm = cfg.z_min.force_numerical_value_in(cm);
    if (!(z_min_cm > 1e-9)) {
        return 0.0;
    }
    const double d_cm = cfg.d.force_numerical_value_in(cm);
    // Outer ring index is fov_circles-1 when ≥2 rings exist. A single-circle lidar
    // (centre beam only) has α=0 for the physical cone; for sphere tiling use one
    // ring width atan2(d, z_min) so planning still produces scan directions.
    const std::size_t outer_ring =
        (cfg.fov_circles <= 1) ? 1 : (cfg.fov_circles - 1);
    const double outer_radius_cm = static_cast<double>(outer_ring) * d_cm;
    return std::atan2(outer_radius_cm, z_min_cm);
}

[[nodiscard]] inline std::size_t directionCountForHalfAngle(double half_angle_rad,
                                                            double overlap = 0.85) {
    if (!(half_angle_rad > 1e-9)) {
        return 0;
    }
    const double spacing = 2.0 * half_angle_rad * overlap;
    if (!(spacing > 1e-9)) {
        return 0;
    }
    const double n =
        std::ceil((4.0 * std::numbers::pi) / (spacing * spacing));
    const auto count = static_cast<std::size_t>(n);
    // Floor at 6 so the mandatory axis-aligned set always fits.
    if (count < 6) {
        return 6;
    }
    if (count > 64) {
        return 64;
    }
    return count;
}

[[nodiscard]] inline std::vector<Orientation> fibonacciSphereOrientations(std::size_t count) {
    std::vector<Orientation> out;
    // Always include the six axis-aligned directions first so thin corridors and
    // legacy scan-pass expectations still get ±X/±Y/±Z coverage.
    const Orientation axes[] = {
        Orientation{0.0 * deg, 0.0 * deg},
        Orientation{180.0 * deg, 0.0 * deg},
        Orientation{90.0 * deg, 0.0 * deg},
        Orientation{270.0 * deg, 0.0 * deg},
        Orientation{0.0 * deg, 90.0 * deg},
        Orientation{0.0 * deg, -90.0 * deg},
    };
    for (const Orientation& axis : axes) {
        out.push_back(axis);
        if (out.size() >= count) {
            return out;
        }
    }
    if (count <= 6) {
        return out;
    }

    constexpr double kGoldenAngle = std::numbers::pi * (3.0 - std::sqrt(5.0));
    const std::size_t fib_count = count - 6;
    for (std::size_t i = 0; i < fib_count; ++i) {
        const double y = 1.0 - (2.0 * static_cast<double>(i) + 1.0) / static_cast<double>(fib_count);
        const double radius = std::sqrt(std::max(0.0, 1.0 - y * y));
        const double theta = kGoldenAngle * static_cast<double>(i);
        const double x = std::cos(theta) * radius;
        const double z = std::sin(theta) * radius;
        const double az_deg = std::atan2(y, x) * (180.0 / std::numbers::pi);
        const double el_deg = std::atan2(z, std::hypot(x, y)) * (180.0 / std::numbers::pi);
        double az_norm = az_deg;
        while (az_norm < 0.0) {
            az_norm += 360.0;
        }
        while (az_norm >= 360.0) {
            az_norm -= 360.0;
        }
        out.push_back(Orientation{az_norm * deg, el_deg * deg});
    }
    return out;
}

namespace detail {

[[nodiscard]] inline std::size_t beamsOnCircle(std::size_t circle_index) {
    std::size_t count = 1;
    for (std::size_t i = 0; i < circle_index; ++i) {
        count *= 4;
    }
    return count;
}

[[nodiscard]] inline bool beamHitsUnresolved(const common::IMap3D& map,
                                             const Position3D& origin,
                                             const Orientation& absolute_beam,
                                             double z_max_cm,
                                             double step_cm) {
    if (!(z_max_cm > 0.0) || !(step_cm > 0.0)) {
        return false;
    }
    for (double dist = step_cm; dist <= z_max_cm + 1e-9; dist += step_cm) {
        const Position3D p = bm::pointAlongBeam(origin, absolute_beam, dist * cm);
        const auto occ = map.atVoxel(p);
        if (occ == common::types::VoxelOccupancy::Unmapped) {
            return true;
        }
        if (occ == common::types::VoxelOccupancy::Occupied ||
            occ == common::types::VoxelOccupancy::OutOfBounds) {
            return false;
        }
    }
    return false;
}

} // namespace detail

/// True if any MockLidar-equivalent beam in the cone still sees Unmapped voxels.
[[nodiscard]] inline bool coneCoversUnresolved(const common::IMap3D& map,
                                               const Position3D& origin,
                                               const Orientation& drone_heading,
                                               const Orientation& relative_scan,
                                               const common::types::LidarConfigData& cfg) {
    if (cfg.fov_circles == 0) {
        return false;
    }
    const double z_min_cm = cfg.z_min.force_numerical_value_in(cm);
    const double z_max_cm = cfg.z_max.force_numerical_value_in(cm);
    const double d_cm = cfg.d.force_numerical_value_in(cm);
    const double step_cm = 0.5 * map.getMapConfig().resolution.force_numerical_value_in(cm);
    if (!(z_max_cm > 0.0) || !(step_cm > 0.0)) {
        return false;
    }

    const Orientation center_abs =
        bm::normalizeOrientation(bm::absoluteBeamOrientation(drone_heading, relative_scan));
    if (detail::beamHitsUnresolved(map, origin, center_abs, z_max_cm, step_cm)) {
        return true;
    }

    const double z_min_safe = (z_min_cm > 1e-9) ? z_min_cm : 1.0;
    for (std::size_t circle = 1; circle < cfg.fov_circles; ++circle) {
        const std::size_t beam_count = detail::beamsOnCircle(circle);
        const double polar = std::atan2(static_cast<double>(circle) * d_cm, z_min_safe);
        const double cp = std::cos(polar);
        const double sp = std::sin(polar);

        // Orthonormal basis around center_abs (same construction as HostLidar).
        const double ch = center_abs.horizontal.numerical_value_in(deg) * (std::numbers::pi / 180.0);
        const double ca = center_abs.altitude.numerical_value_in(deg) * (std::numbers::pi / 180.0);
        const double fx = std::cos(ca) * std::cos(ch);
        const double fy = std::cos(ca) * std::sin(ch);
        const double fz = std::sin(ca);
        // right ≈ normalize(cross(forward, world_up)) with fallback
        double rx = -fy;
        double ry = fx;
        double rz = 0.0;
        const double rlen = std::hypot(rx, ry);
        if (rlen < 1e-9) {
            rx = 1.0;
            ry = 0.0;
        } else {
            rx /= rlen;
            ry /= rlen;
        }
        // up = cross(right, forward)
        const double ux = ry * fz - rz * fy;
        const double uy = rz * fx - rx * fz;
        const double uz = rx * fy - ry * fx;

        for (std::size_t j = 0; j < beam_count; ++j) {
            const double phi =
                (beam_count == 1)
                    ? 0.0
                    : (2.0 * std::numbers::pi * static_cast<double>(j) /
                       static_cast<double>(beam_count));
            const double ax = ux * std::cos(phi) + rx * std::sin(phi);
            const double ay = uy * std::cos(phi) + ry * std::sin(phi);
            const double az = uz * std::cos(phi) + rz * std::sin(phi);
            double dx = fx * cp + ax * sp;
            double dy = fy * cp + ay * sp;
            double dz = fz * cp + az * sp;
            const double len = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (len < 1e-12) {
                continue;
            }
            dx /= len;
            dy /= len;
            dz /= len;
            const double az_deg = std::atan2(dy, dx) * (180.0 / std::numbers::pi);
            const double el_deg = std::atan2(dz, std::hypot(dx, dy)) * (180.0 / std::numbers::pi);
            const Orientation beam_abs =
                bm::normalizeOrientation(Orientation{az_deg * deg, el_deg * deg});
            if (detail::beamHitsUnresolved(map, origin, beam_abs, z_max_cm, step_cm)) {
                return true;
            }
        }
    }
    return false;
}

} // namespace user_common_207190406_209543255::lidar_cone
