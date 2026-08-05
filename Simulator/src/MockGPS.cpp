// MockGPS.cpp
// Simulated GPS: stores drone position and heading, snapping position
// coordinates to the nearest resolution-grid multiple on every write.
// Snapping ensures the algorithm and movement layers agree on a quantized
// coordinate system, matching MissionConfigData::gps_resolution.

#include <Simulator/MockGPS.h>

#include <cmath>

namespace simulator {

namespace {

double snapToCm(double value, double res_cm) {
    if (res_cm <= 0.0) {
        return value;
    }
    return std::round(value / res_cm) * res_cm;
}

} // namespace

MockGPS::MockGPS(common::Position3D position,
                 common::Orientation heading,
                 common::PhysicalLength gps_resolution)
    : position_(position), heading_(heading), resolution_(gps_resolution) {}

common::Position3D MockGPS::position() const {
    return position_;
}

common::Orientation MockGPS::heading() const {
    return heading_;
}

void MockGPS::setPosition(common::Position3D position) {
    using common::cm;
    using common::x_extent;
    using common::y_extent;
    using common::z_extent;

    const double res_cm = resolution_.numerical_value_in(cm);
    position_ = common::Position3D{
        snapToCm(position.x.numerical_value_in(cm), res_cm) * x_extent[cm],
        snapToCm(position.y.numerical_value_in(cm), res_cm) * y_extent[cm],
        snapToCm(position.z.numerical_value_in(cm), res_cm) * z_extent[cm],
    };
}

void MockGPS::setHeading(common::Orientation heading) {
    heading_ = heading;
}

} // namespace simulator
