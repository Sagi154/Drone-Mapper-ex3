// MockGPS.cpp
// Simulated GPS: stores drone position and heading, snapping position
// coordinates to the nearest resolution-grid multiple on every write.
// Snapping ensures the algorithm and movement layers agree on a quantized
// coordinate system, matching MissionConfigData::gps_resolution.

#include <Simulator/MockGPS.h>

#include <cmath>

namespace simulator {

namespace {

[[nodiscard]] common::PhysicalLength snapToRes(common::PhysicalLength value,
                                               common::PhysicalLength res) {
    if (res <= 0.0 * common::cm) {
        return value;
    }
    const double n = std::round((value / res).numerical_value_in(mp_units::one));
    return n * res;
}

} // namespace

MockGPS::MockGPS(const common::Position3D& position,
                 const common::Orientation& heading,
                 common::PhysicalLength gps_resolution)
    : heading_(heading), resolution_(gps_resolution) {
    setPosition(position);
}

common::Position3D MockGPS::position() const {
    return position_;
}

common::Orientation MockGPS::heading() const {
    return heading_;
}

void MockGPS::setPosition(const common::Position3D& position) {
    using common::x_extent;
    using common::y_extent;
    using common::z_extent;

    position_ = common::Position3D{
        mp_units::quantity_cast<x_extent>(
            snapToRes(mp_units::quantity_cast<common::isq::length>(position.x), resolution_)),
        mp_units::quantity_cast<y_extent>(
            snapToRes(mp_units::quantity_cast<common::isq::length>(position.y), resolution_)),
        mp_units::quantity_cast<z_extent>(
            snapToRes(mp_units::quantity_cast<common::isq::length>(position.z), resolution_)),
    };
}

void MockGPS::setHeading(const common::Orientation& heading) {
    heading_ = heading;
}

} // namespace simulator
