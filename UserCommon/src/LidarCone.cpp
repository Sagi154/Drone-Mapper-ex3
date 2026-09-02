#include <user_common_207190406_209543255/LidarCone.h>

#include <algorithm>
#include <cmath>
#include <numbers>

namespace user_common_207190406_209543255::lidar_cone {

double coneHalfAngleRad(const common::types::LidarConfigData& cfg) {
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

std::size_t directionCountForHalfAngle(double half_angle_rad, double overlap) {
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
    if (count < kMinSphereDirections) {
        return kMinSphereDirections;
    }
    if (count > kMaxSphereDirections) {
        return kMaxSphereDirections;
    }
    return count;
}

std::vector<Orientation> fibonacciSphereOrientations(std::size_t count) {
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
    if (count <= kMinSphereDirections) {
        return out;
    }

    constexpr double kGoldenAngle = std::numbers::pi * (3.0 - std::sqrt(5.0));
    const std::size_t fib_count = count - kMinSphereDirections;
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

std::int64_t voxelKey(const common::types::MapConfig& config, const Position3D& p) {
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

std::size_t countUnresolvedVoxels(
    const common::IMap3D& map,
    const Position3D& origin,
    const Orientation& drone_heading,
    const Orientation& relative_scan,
    const common::types::LidarConfigData& cfg,
    std::unordered_set<std::int64_t>& seen) {
    const PhysicalLength z_max = cfg.z_max;
    const common::types::MapConfig& config = map.getMapConfig();
    const PhysicalLength step = kConeWalkResolutionFactor * config.resolution;
    const Orientation center_abs =
        bm::normalizeOrientation(bm::absoluteBeamOrientation(drone_heading, relative_scan));

    std::size_t added = 0;
    forEachConeBeam(cfg, center_abs, [&](const Orientation& beam) {
        detail::walkBeam(map, origin, beam, z_max, step, [&](const Position3D& p) {
            if (seen.insert(voxelKey(config, p)).second) {
                ++added;
            }
            return true;
        });
        return true;
    });
    return added;
}

bool coneCoversUnresolved(const common::IMap3D& map,
                          const Position3D& origin,
                          const Orientation& drone_heading,
                          const Orientation& relative_scan,
                          const common::types::LidarConfigData& cfg) {
    const PhysicalLength z_max = cfg.z_max;
    const common::types::MapConfig& config = map.getMapConfig();
    const PhysicalLength step = kConeWalkResolutionFactor * config.resolution;
    const Orientation center_abs =
        bm::normalizeOrientation(bm::absoluteBeamOrientation(drone_heading, relative_scan));

    bool found = false;
    forEachConeBeam(cfg, center_abs, [&](const Orientation& beam) {
        detail::walkBeam(map, origin, beam, z_max, step, [&](const Position3D&) {
            found = true;
            return false;
        });
        return !found;
    });
    return found;
}

} // namespace user_common_207190406_209543255::lidar_cone
