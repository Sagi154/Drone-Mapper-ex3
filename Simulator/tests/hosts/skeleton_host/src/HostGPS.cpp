#include "HostGPS.h"

namespace skeleton_host {

HostGPS::HostGPS(common::Position3D position,
                 common::Orientation heading,
                 common::PhysicalLength gps_resolution)
    : position_(position), heading_(heading), gps_resolution_(gps_resolution) {}

common::Position3D HostGPS::position() const {
    return position_;
}

common::Orientation HostGPS::heading() const {
    return heading_;
}

void HostGPS::setPosition(common::Position3D position) {
    position_ = position;
}

void HostGPS::setHeading(common::Orientation heading) {
    heading_ = heading;
}

}  // namespace skeleton_host
