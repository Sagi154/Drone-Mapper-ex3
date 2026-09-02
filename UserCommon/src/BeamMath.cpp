#include <user_common_207190406_209543255/BeamMath.h>

#include <mp-units/systems/si/math.h>

#include <cmath>

namespace user_common_207190406_209543255::beam_math {

namespace mp = common::mp;
namespace si = common::si;
using common::cm;
using common::deg;
using common::x_extent;
using common::y_extent;
using common::z_extent;

bool isZeroDistance(PhysicalLength distance) { return distance == 0.0 * cm; }

bool isMissDistance(PhysicalLength distance) {
    return distance == kLidarMissDistance;
}

Orientation absoluteBeamOrientation(const Orientation& drone_heading,
                                    const Orientation& relative_beam) {
    return Orientation{relative_beam.horizontal + drone_heading.horizontal,
                       relative_beam.altitude + drone_heading.altitude};
}

namespace {
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

Orientation normalizeOrientation(Orientation orientation) {
    using common::AltitudeAngle;
    using common::HorizontalAngle;
    return Orientation{
        HorizontalAngle{wrapDeg(orientation.horizontal.numerical_value_in(deg)) * deg},
        AltitudeAngle{wrapDeg(orientation.altitude.numerical_value_in(deg)) * deg},
    };
}

Position3D pointAlongBeam(const Position3D& origin, const Orientation& beam_orientation,
                          PhysicalLength distance) {
    const Orientation beam = normalizeOrientation(beam_orientation);
    const auto cos_altitude = si::cos(beam.altitude);
    const auto dx = cos_altitude * si::cos(beam.horizontal);
    const auto dy = cos_altitude * si::sin(beam.horizontal);
    const auto dz = si::sin(beam.altitude);
    return Position3D{
        origin.x + mp::quantity_cast<x_extent>(dx * distance),
        origin.y + mp::quantity_cast<y_extent>(dy * distance),
        origin.z + mp::quantity_cast<z_extent>(dz * distance),
    };
}

} // namespace user_common_207190406_209543255::beam_math
