#pragma once

// Lidar cone FOV helpers for Algorithm scan planning (and future NBV scoring).
// Half-angle matches MockLidar / HostLidar: atan2((fov_circles-1)*d, z_min).

#include <user_common_207190406_209543255/BeamMath.h>
#include <user_common_207190406_209543255/LidarConstants.h>

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
using common::PhysicalLength;
using common::Position3D;
using common::cm;
using common::deg;

// ── Non-template declarations (bodies in LidarCone.cpp) ──────────────────────

[[nodiscard]] double coneHalfAngleRad(const common::types::LidarConfigData& cfg);

[[nodiscard]] std::size_t directionCountForHalfAngle(
    double half_angle_rad, double overlap = kConeDirectionOverlap);

[[nodiscard]] std::vector<Orientation> fibonacciSphereOrientations(std::size_t count);

[[nodiscard]] std::int64_t voxelKey(const common::types::MapConfig& config,
                                    const Position3D& p);

[[nodiscard]] std::size_t countUnresolvedVoxels(
    const common::IMap3D& map,
    const Position3D& origin,
    const Orientation& drone_heading,
    const Orientation& relative_scan,
    const common::types::LidarConfigData& cfg,
    std::unordered_set<std::int64_t>& seen);

[[nodiscard]] bool coneCoversUnresolved(const common::IMap3D& map,
                                        const Position3D& origin,
                                        const Orientation& drone_heading,
                                        const Orientation& relative_scan,
                                        const common::types::LidarConfigData& cfg);

// ── detail helpers (inline; needed by forEachConeBeam template below) ─────────

namespace detail {

[[nodiscard]] inline std::size_t beamsOnCircle(std::size_t circle_index) {
    std::size_t count = 1;
    for (std::size_t i = 0; i < circle_index; ++i) {
        count *= 4;
    }
    return count;
}

} // namespace detail

// ── forEachConeBeam template (stays in header) ────────────────────────────────

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
            double bx = fx * cp + ax * sp;
            double by = fy * cp + ay * sp;
            double bz = fz * cp + az * sp;
            const double len = std::sqrt(bx * bx + by * by + bz * bz);
            if (len < 1e-12) {
                continue;
            }
            bx /= len;
            by /= len;
            bz /= len;
            const double az_deg = std::atan2(by, bx) * (180.0 / std::numbers::pi);
            const double el_deg = std::atan2(bz, std::hypot(bx, by)) * (180.0 / std::numbers::pi);
            if (!on_beam(bm::normalizeOrientation(Orientation{az_deg * deg, el_deg * deg}))) {
                return;
            }
        }
    }
}

// ── detail::walkBeam template (stays in header) ───────────────────────────────

namespace detail {

/// Walks one beam from the first sample out to z_max. Calls on_unresolved for each
/// Unmapped sample; stops the beam at Occupied / OutOfBounds. Carving starts at 0 in
/// ScanResultToVoxels, so the walk deliberately does NOT skip the sub-z_min region.
/// PotentiallyOccupied is a resolved state and is skipped, not counted.
template <typename Fn>
inline bool walkBeam(const common::IMap3D& map,
                     const Position3D& origin,
                     const Orientation& absolute_beam,
                     PhysicalLength z_max,
                     PhysicalLength step,
                     Fn&& on_unresolved) {
    if (z_max <= 0.0 * cm || step <= 0.0 * cm) {
        return true;
    }
    for (PhysicalLength dist = step; dist <= z_max + 1e-9 * cm; dist += step) {
        const Position3D p = bm::pointAlongBeam(origin, absolute_beam, dist);
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

} // namespace user_common_207190406_209543255::lidar_cone
