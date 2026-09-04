// MockLidar.cpp
// Simulated LiDAR. Fires a cone of beams arranged in concentric circles
// (fov_circles). The centre beam is at scan_orientation; ring i has
// 4^i beams spaced evenly, offset by i*d around a cone of half-angle
// atan2(i*d, z_min). Each beam is traced through the hidden map by
// stepping in small increments until it hits an Occupied voxel or z_max.

#include <Simulator/MockLidar.h>

#include <user_common_207190406_209543255/BeamMath.h>

#include <mp-units/systems/si/math.h>

#include <cmath>

namespace simulator {

namespace {

namespace si = common::si;
using common::cm;
using common::deg;
using common::PhysicalLength;
using common::HorizontalAngle;
using common::AltitudeAngle;
using common::Orientation;

// Adversarial plugins may pass absurd angles (e.g. 1e12 deg). mp-units / libm
// sin/cos on those values can hang; reduce to (-180, 180] in plain double first.
[[nodiscard]] double wrap_deg(double degrees) {
    double x = std::fmod(degrees, 360.0);
    if (x <= -180.0) {
        x += 360.0;
    } else if (x > 180.0) {
        x -= 360.0;
    }
    return x;
}

[[nodiscard]] Orientation normalize_orientation(Orientation orientation) {
    return Orientation{
        HorizontalAngle{wrap_deg(orientation.horizontal.numerical_value_in(deg)) * deg},
        AltitudeAngle{wrap_deg(orientation.altitude.numerical_value_in(deg)) * deg},
    };
}

[[nodiscard]] std::size_t beams_on_circle(std::size_t circle_index) {
    std::size_t count = 1;
    for (std::size_t i = 0; i < circle_index; ++i) {
        count *= 4;
    }
    return count;
}

[[nodiscard]] HorizontalAngle horizontal_delta(PhysicalLength offset, PhysicalLength distance) {
    return HorizontalAngle{si::atan2(offset, distance)};
}

[[nodiscard]] AltitudeAngle altitude_delta(PhysicalLength offset, PhysicalLength distance) {
    return AltitudeAngle{si::atan2(offset, distance)};
}

} // namespace

MockLidar::MockLidar(const common::types::LidarConfigData& config,
                     const common::IMap3D& map,
                     const common::IGPS& gps)
    : config_(config), map_(map), gps_(gps) {}

common::types::LidarConfigData MockLidar::config() const {
    return config_;
}

common::types::LidarScanResult MockLidar::scan(common::Orientation scan_orientation) const {
    common::types::LidarScanResult results;
    if (config_.fov_circles == 0) {
        return results;
    }

    // Normalize before any trig and before publishing hit angles (fusion also uses them).
    const Orientation scan = normalize_orientation(scan_orientation);
    const Orientation sensor_heading = normalize_orientation(gps_.heading());
    const Orientation center_beam_abs{
        scan.horizontal + sensor_heading.horizontal,
        scan.altitude   + sensor_heading.altitude,
    };

    const PhysicalLength center_distance = traceBeam(center_beam_abs);
    results.push_back(common::types::LidarHit{center_distance, scan});

    for (std::size_t circle = 1; circle < config_.fov_circles; ++circle) {
        const std::size_t beam_count = beams_on_circle(circle);
        const PhysicalLength radius = static_cast<double>(circle) * config_.d;

        for (std::size_t i = 0; i < beam_count; ++i) {
            const auto theta =
                (360.0 * static_cast<double>(i) / static_cast<double>(beam_count)) * deg;
            const PhysicalLength horizontal_offset = radius * si::cos(theta);
            const PhysicalLength altitude_offset   = radius * si::sin(theta);

            const Orientation offset{
                horizontal_delta(horizontal_offset, config_.z_min),
                altitude_delta(altitude_offset, config_.z_min),
            };
            const Orientation relative_beam = normalize_orientation(Orientation{
                scan.horizontal + offset.horizontal,
                scan.altitude   + offset.altitude,
            });
            const Orientation absolute_beam{
                relative_beam.horizontal + sensor_heading.horizontal,
                relative_beam.altitude   + sensor_heading.altitude,
            };
            const PhysicalLength distance = traceBeam(absolute_beam);
            results.push_back(common::types::LidarHit{distance, relative_beam});
        }
    }

    return results;
}

common::PhysicalLength MockLidar::traceBeam(const common::Orientation& beam) const {
    using user_common_207190406_209543255::kLidarMissDistance;
    using user_common_207190406_209543255::kLidarTraceResolutionFactor;
    namespace bm = user_common_207190406_209543255::beam_math;
    using common::cm;
    using common::PhysicalLength;
    using common::Position3D;

    const Position3D origin = gps_.position();
    const PhysicalLength step = kLidarTraceResolutionFactor * map_.getMapConfig().resolution;
    for (PhysicalLength distance = 0.0 * cm; distance <= config_.z_max; distance += step) {
        const Position3D sample = bm::pointAlongBeam(origin, beam, distance);
        if (map_.atVoxel(sample) == common::types::VoxelOccupancy::Occupied) {
            if (distance < config_.z_min) {
                return 0.0 * cm;
            }
            return distance;
        }
    }
    return kLidarMissDistance;
}

} // namespace simulator
