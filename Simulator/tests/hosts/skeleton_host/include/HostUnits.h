#pragma once

#include <Common/Units.h>

#include <cmath>

namespace skeleton_host {

inline double cm_of(const common::XLength& q) {
    return q.numerical_value_in(common::cm);
}

inline double cm_of(const common::YLength& q) {
    return q.numerical_value_in(common::cm);
}

inline double cm_of(const common::ZLength& q) {
    return q.numerical_value_in(common::cm);
}

inline double cm_of(const common::PhysicalLength& q) {
    return q.numerical_value_in(common::cm);
}

inline double deg_of(const common::HorizontalAngle& q) {
    return q.numerical_value_in(common::deg);
}

inline double deg_of(const common::AltitudeAngle& q) {
    return q.numerical_value_in(common::deg);
}

inline common::XLength x_cm(double v) {
    return common::XLength{v * common::cm};
}

inline common::YLength y_cm(double v) {
    return common::YLength{v * common::cm};
}

inline common::ZLength z_cm(double v) {
    return common::ZLength{v * common::cm};
}

inline common::PhysicalLength length_cm(double v) {
    return common::PhysicalLength{v * common::cm};
}

inline common::HorizontalAngle horiz_deg(double v) {
    return common::HorizontalAngle{v * common::deg};
}

inline common::AltitudeAngle alt_deg(double v) {
    return common::AltitudeAngle{v * common::deg};
}

inline common::Position3D pos_cm(double x, double y, double z) {
    return {x_cm(x), y_cm(y), z_cm(z)};
}

inline double wrap_deg(double deg) {
    deg = std::fmod(deg, 360.0);
    if (deg < 0.0) {
        deg += 360.0;
    }
    return deg;
}

inline double deg_to_rad(double deg) {
    return deg * 3.14159265358979323846 / 180.0;
}

inline double rad_to_deg(double rad) {
    return rad * 180.0 / 3.14159265358979323846;
}

}  // namespace skeleton_host
