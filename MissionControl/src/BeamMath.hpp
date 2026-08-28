#pragma once

// Private header — used only within MissionControl/src/.
// Shared beam-geometry helpers used by both ScanResultToVoxels and DroneControlImpl.

#include <Common/Types.h>

#include <mp-units/systems/si/math.h>

#include <cmath>
#include <limits>

namespace mission_control_207190406_209543255::beam_math {

namespace mp = common::mp;
namespace si = common::si;
using common::cm;
using common::PhysicalLength;
using common::Orientation;
using common::Position3D;

[[nodiscard]] inline bool isZeroDistance(PhysicalLength distance) {
    return distance == 0.0 * cm;
}

[[nodiscard]] inline bool isMissDistance(PhysicalLength distance) {
    return distance.force_numerical_value_in(cm) == std::numeric_limits<double>::max();
}

[[nodiscard]] inline Orientation absoluteBeamOrientation(const Orientation& drone_heading,
                                                          const Orientation& relative_beam) {
    return Orientation{
        relative_beam.horizontal + drone_heading.horizontal,
        relative_beam.altitude   + drone_heading.altitude,
    };
}

[[nodiscard]] inline double wrapDeg(double degrees) {
    double x = std::fmod(degrees, 360.0);
    if (x <= -180.0) {
        x += 360.0;
    } else if (x > 180.0) {
        x -= 360.0;
    }
    return x;
}

[[nodiscard]] inline Orientation normalizeOrientation(Orientation orientation) {
    using common::deg;
    using common::HorizontalAngle;
    using common::AltitudeAngle;
    return Orientation{
        HorizontalAngle{wrapDeg(orientation.horizontal.numerical_value_in(deg)) * deg},
        AltitudeAngle{wrapDeg(orientation.altitude.numerical_value_in(deg)) * deg},
    };
}

[[nodiscard]] inline Position3D pointAlongBeam(const Position3D& origin,
                                                const Orientation& beam_orientation,
                                                PhysicalLength distance) {
    using common::x_extent;
    using common::y_extent;
    using common::z_extent;

    const Orientation beam = normalizeOrientation(beam_orientation);
    const auto cos_altitude = si::cos(beam.altitude);
    const auto dx = cos_altitude * si::cos(beam.horizontal);
    const auto dy = cos_altitude * si::sin(beam.horizontal);
    const auto dz = si::sin(beam.altitude);

    const double distance_cm = distance.force_numerical_value_in(cm);
    const double dir_x = dx.force_numerical_value_in(mp::one);
    const double dir_y = dy.force_numerical_value_in(mp::one);
    const double dir_z = dz.force_numerical_value_in(mp::one);

    return Position3D{
        origin.x + dir_x * distance_cm * x_extent[cm],
        origin.y + dir_y * distance_cm * y_extent[cm],
        origin.z + dir_z * distance_cm * z_extent[cm],
    };
}

} // namespace mission_control_207190406_209543255::beam_math
