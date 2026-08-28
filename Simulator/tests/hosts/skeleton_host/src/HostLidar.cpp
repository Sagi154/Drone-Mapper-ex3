#include "HostLidar.h"

#include "HostGPS.h"
#include "HostMap3D.h"
#include "HostUnits.h"

#include <cmath>
#include <cstddef>

namespace skeleton_host {

namespace {

struct Vec3 {
    double x = 0;
    double y = 0;
    double z = 0;
};

Vec3 operator+(Vec3 a, Vec3 b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

Vec3 operator*(Vec3 a, double s) {
    return {a.x * s, a.y * s, a.z * s};
}

double dot(Vec3 a, Vec3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3 cross(Vec3 a, Vec3 b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

double norm(Vec3 a) {
    return std::sqrt(dot(a, a));
}

Vec3 normalize(Vec3 a) {
    const double n = norm(a);
    if (n < 1e-15) {
        return {1.0, 0.0, 0.0};
    }
    return a * (1.0 / n);
}

Vec3 directionFromOrientation(double horiz_deg, double alt_deg) {
    const double h = deg_to_rad(horiz_deg);
    const double a = deg_to_rad(alt_deg);
    const double c = std::cos(a);
    return {c * std::cos(h), c * std::sin(h), std::sin(a)};
}

void orthonormalBasis(Vec3 forward, Vec3& right, Vec3& up) {
    forward = normalize(forward);
    const Vec3 world_up{0.0, 0.0, 1.0};
    Vec3 r = cross(world_up, forward);
    if (norm(r) < 1e-8) {
        r = cross(Vec3{1.0, 0.0, 0.0}, forward);
        if (norm(r) < 1e-8) {
            r = {0.0, 1.0, 0.0};
        }
    }
    right = normalize(r);
    up = normalize(cross(forward, right));
}

struct RayHit {
    bool hit = false;
    double distance = 0;
};

RayHit marchRay(const HostMap3D& map, Vec3 origin, Vec3 dir, double max_dist) {
    dir = normalize(dir);
    const double step = std::min(std::max(map.resolutionCm() * 0.25, 0.25), 1.0);
    for (double t = 0.0; t <= max_dist + 1e-9; t += step) {
        const double x = origin.x + dir.x * t;
        const double y = origin.y + dir.y * t;
        const double z = origin.z + dir.z * t;
        const auto occ = map.atVoxel(pos_cm(x, y, z));
        if (occ == common::types::VoxelOccupancy::Occupied) {
            RayHit hit;
            hit.hit = true;
            hit.distance = t;
            return hit;
        }
        if (occ == common::types::VoxelOccupancy::OutOfBounds) {
            return {};
        }
    }
    return {};
}

std::size_t beamsOnCircle(std::size_t circle) {
    std::size_t n = 1;
    for (std::size_t i = 0; i < circle; ++i) {
        n *= 4;
    }
    return n;
}

}  // namespace

HostLidar::HostLidar(const HostGPS& gps,
                     const HostMap3D& hidden_map,
                     common::types::LidarConfigData config)
    : gps_(gps), hidden_map_(hidden_map), config_(std::move(config)) {}

common::types::LidarConfigData HostLidar::config() const {
    return config_;
}

common::types::LidarScanResult HostLidar::scan(common::Orientation scan_orientation) const {
    common::types::LidarScanResult hits;
    const double z_min = cm_of(config_.z_min);
    const double z_max = cm_of(config_.z_max);
    const double d = cm_of(config_.d);
    const std::size_t circles = config_.fov_circles;
    if (!(z_max > 0.0) || circles == 0) {
        return hits;
    }

    const auto origin_pos = gps_.position();
    const Vec3 origin{cm_of(origin_pos.x), cm_of(origin_pos.y), cm_of(origin_pos.z)};
    const double fov_h = deg_of(scan_orientation.horizontal);
    const double fov_a = deg_of(scan_orientation.altitude);
    const Vec3 forward = directionFromOrientation(fov_h, fov_a);
    Vec3 right{};
    Vec3 up{};
    orthonormalBasis(forward, right, up);

    const double z_min_safe = (z_min > 1e-9) ? z_min : 1.0;

    for (std::size_t circle = 0; circle < circles; ++circle) {
        const std::size_t n_beams = beamsOnCircle(circle);
        const double polar = std::atan2(static_cast<double>(circle) * d, z_min_safe);
        const double cp = std::cos(polar);
        const double sp = std::sin(polar);
        for (std::size_t j = 0; j < n_beams; ++j) {
            const double phi = (n_beams == 1)
                                   ? 0.0
                                   : (2.0 * 3.14159265358979323846 * static_cast<double>(j) /
                                      static_cast<double>(n_beams));
            const Vec3 around = up * std::cos(phi) + right * std::sin(phi);
            const Vec3 dir = normalize(forward * cp + around * sp);
            const RayHit ray = marchRay(hidden_map_, origin, dir, z_max);
            if (!ray.hit) {
                continue;
            }
            const double horiz = wrap_deg(rad_to_deg(std::atan2(dir.y, dir.x)));
            const double alt = rad_to_deg(std::atan2(dir.z, std::hypot(dir.x, dir.y)));
            common::types::LidarHit hit;
            hit.distance = length_cm(ray.distance < z_min ? 0.0 : ray.distance);
            hit.angle = {horiz_deg(horiz), alt_deg(alt)};
            hits.push_back(hit);
        }
    }
    return hits;
}

}  // namespace skeleton_host
