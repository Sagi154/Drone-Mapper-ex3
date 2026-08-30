#pragma once

// Lidar cone FOV helpers for Algorithm scan planning (and future NBV scoring).
// Half-angle matches MockLidar / HostLidar: atan2((fov_circles-1)*d, z_min).

#include <user_common_207190406_209543255/BeamMath.h>

#include <Common/IMap3D.h>
#include <Common/types/LidarTypes.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <unordered_set>
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

} // namespace detail

[[nodiscard]] inline std::int64_t voxelKey(const common::types::MapConfig& config,
                                           const Position3D& p) {
    const double step = config.resolution.force_numerical_value_in(cm);
    if (!(step > 0.0)) {
        return 0;
    }
    const auto quant = [step](double value, double origin) {
        return static_cast<std::int64_t>(std::llround((value - origin) / step));
    };
    const std::int64_t qx = quant(p.x.force_numerical_value_in(cm),
                                  config.offset.x.force_numerical_value_in(cm));
    const std::int64_t qy = quant(p.y.force_numerical_value_in(cm),
                                  config.offset.y.force_numerical_value_in(cm));
    const std::int64_t qz = quant(p.z.force_numerical_value_in(cm),
                                  config.offset.z.force_numerical_value_in(cm));
    constexpr std::int64_t kBias = 1 << 20;
    constexpr std::int64_t kSpan = 1 << 21;
    return ((qx + kBias) * kSpan + (qy + kBias)) * kSpan + (qz + kBias);
}

/// Invokes on_beam with each absolute beam orientation in the cone (centre first).
/// on_beam returns false to stop the walk early.
template <typename Fn>
inline void forEachConeBeam(const common::types::LidarConfigData& cfg,
                            const Orientation& center_abs,
                            Fn&& on_beam) {
    if (cfg.fov_circles == 0) {
        return;
    }
    if (!on_beam(center_abs)) {
        return;
    }

    const double z_min_cm = cfg.z_min.force_numerical_value_in(cm);
    const double d_cm = cfg.d.force_numerical_value_in(cm);
    const double z_min_safe = (z_min_cm > 1e-9) ? z_min_cm : 1.0;

    // Orthonormal basis around center_abs, same construction as HostLidar.
    const double ch = center_abs.horizontal.numerical_value_in(deg) * (std::numbers::pi / 180.0);
    const double ca = center_abs.altitude.numerical_value_in(deg) * (std::numbers::pi / 180.0);
    const double fx = std::cos(ca) * std::cos(ch);
    const double fy = std::cos(ca) * std::sin(ch);
    const double fz = std::sin(ca);
    double rx = -fy;
    double ry = fx;
    const double rz = 0.0;
    const double rlen = std::hypot(rx, ry);
    if (rlen < 1e-9) {
        rx = 1.0;
        ry = 0.0;
    } else {
        rx /= rlen;
        ry /= rlen;
    }
    const double ux = ry * fz - rz * fy;
    const double uy = rz * fx - rx * fz;
    const double uz = rx * fy - ry * fx;

    for (std::size_t circle = 1; circle < cfg.fov_circles; ++circle) {
        const std::size_t beam_count = detail::beamsOnCircle(circle);
        const double polar = std::atan2(static_cast<double>(circle) * d_cm, z_min_safe);
        const double cp = std::cos(polar);
        const double sp = std::sin(polar);

        for (std::size_t j = 0; j < beam_count; ++j) {
            const double phi = (beam_count == 1)
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
            if (!on_beam(bm::normalizeOrientation(Orientation{az_deg * deg, el_deg * deg}))) {
                return;
            }
        }
    }
}

namespace detail {

/// Walks one beam from the first sample out to z_max. Calls on_unresolved for each
/// Unmapped sample; stops the beam at Occupied / OutOfBounds. Carving starts at 0 in
/// ScanResultToVoxels, so the walk deliberately does NOT skip the sub-z_min region.
/// PotentiallyOccupied is a resolved state and is skipped, not counted.
template <typename Fn>
inline bool walkBeam(const common::IMap3D& map,
                     const Position3D& origin,
                     const Orientation& absolute_beam,
                     double z_max_cm,
                     double step_cm,
                     Fn&& on_unresolved) {
    if (!(z_max_cm > 0.0) || !(step_cm > 0.0)) {
        return true;
    }
    for (double dist = step_cm; dist <= z_max_cm + 1e-9; dist += step_cm) {
        const Position3D p = bm::pointAlongBeam(origin, absolute_beam, dist * cm);
        const auto occ = map.atVoxel(p);
        if (occ == common::types::VoxelOccupancy::Occupied ||
            occ == common::types::VoxelOccupancy::OutOfBounds) {
            return true;
        }
        if (occ == common::types::VoxelOccupancy::Unmapped) {
            if (!on_unresolved(p)) {
                return false;
            }
        }
    }
    return true;
}

} // namespace detail

[[nodiscard]] inline std::size_t countUnresolvedVoxels(
    const common::IMap3D& map,
    const Position3D& origin,
    const Orientation& drone_heading,
    const Orientation& relative_scan,
    const common::types::LidarConfigData& cfg,
    std::unordered_set<std::int64_t>& seen) {
    const double z_max_cm = cfg.z_max.force_numerical_value_in(cm);
    const double step_cm = 0.5 * map.getMapConfig().resolution.force_numerical_value_in(cm);
    const common::types::MapConfig config = map.getMapConfig();
    const Orientation center_abs =
        bm::normalizeOrientation(bm::absoluteBeamOrientation(drone_heading, relative_scan));

    std::size_t added = 0;
    forEachConeBeam(cfg, center_abs, [&](const Orientation& beam) {
        detail::walkBeam(map, origin, beam, z_max_cm, step_cm, [&](const Position3D& p) {
            if (seen.insert(voxelKey(config, p)).second) {
                ++added;
            }
            return true;
        });
        return true;
    });
    return added;
}

/// True if any beam in the cone still sees Unmapped voxels.
[[nodiscard]] inline bool coneCoversUnresolved(const common::IMap3D& map,
                                               const Position3D& origin,
                                               const Orientation& drone_heading,
                                               const Orientation& relative_scan,
                                               const common::types::LidarConfigData& cfg) {
    const double z_max_cm = cfg.z_max.force_numerical_value_in(cm);
    const double step_cm = 0.5 * map.getMapConfig().resolution.force_numerical_value_in(cm);
    const Orientation center_abs =
        bm::normalizeOrientation(bm::absoluteBeamOrientation(drone_heading, relative_scan));

    bool found = false;
    forEachConeBeam(cfg, center_abs, [&](const Orientation& beam) {
        detail::walkBeam(map, origin, beam, z_max_cm, step_cm, [&](const Position3D&) {
            found = true;
            return false;  // stop this beam
        });
        return !found;     // stop the cone once anything unresolved is seen
    });
    return found;
}

} // namespace user_common_207190406_209543255::lidar_cone
