#include "HostMovement.h"

#include "HostGPS.h"
#include "HostMap3D.h"
#include "HostUnits.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>

namespace skeleton_host {

namespace {
bool diagEnabled() {
    static const bool on = std::getenv("HOST_DIAG_ILLEGAL") != nullptr;
    return on;
}
}  // namespace

HostMovement::HostMovement(HostGPS& gps,
                           const HostMap3D& hidden_map,
                           common::types::MappingBounds mission_bounds,
                           common::PhysicalLength radius)
    : gps_(gps),
      hidden_map_(hidden_map),
      mission_bounds_(mission_bounds),
      radius_cm_(cm_of(radius)) {}

common::types::MovementResult HostMovement::fail(const std::string& message) {
    ++illegal_move_attempts_;
    return common::types::MovementResult{false, message};
}

bool HostMovement::centerInMissionBounds(double x, double y, double z) const {
    const double min_x = cm_of(mission_bounds_.min_x);
    const double max_x = cm_of(mission_bounds_.max_x);
    const double min_y = cm_of(mission_bounds_.min_y);
    const double max_y = cm_of(mission_bounds_.max_y);
    const double min_z = cm_of(mission_bounds_.min_height);
    const double max_z = cm_of(mission_bounds_.max_height);
    return x >= min_x && x <= max_x && y >= min_y && y <= max_y && z >= min_z && z <= max_z;
}

bool HostMovement::sphereHitsWallOrLeavesMap(double x, double y, double z) const {
    const double r = radius_cm_;
    const double res = hidden_map_.resolutionCm();
    const int ix0 = static_cast<int>(std::floor((x - r - hidden_map_.minXcm()) / res)) - 1;
    const int iy0 = static_cast<int>(std::floor((y - r - hidden_map_.minYcm()) / res)) - 1;
    const int iz0 = static_cast<int>(std::floor((z - r - hidden_map_.minZcm()) / res)) - 1;
    const int ix1 = static_cast<int>(std::floor((x + r - hidden_map_.minXcm()) / res)) + 1;
    const int iy1 = static_cast<int>(std::floor((y + r - hidden_map_.minYcm()) / res)) + 1;
    const int iz1 = static_cast<int>(std::floor((z + r - hidden_map_.minZcm()) / res)) + 1;

    const double r2 = r * r;
    bool any_voxel = false;
    for (int ix = ix0; ix <= ix1; ++ix) {
        for (int iy = iy0; iy <= iy1; ++iy) {
            for (int iz = iz0; iz <= iz1; ++iz) {
                if (!hidden_map_.indexInRange(ix, iy, iz)) {
                    continue;
                }
                any_voxel = true;
                if (hidden_map_.atIndex(static_cast<std::size_t>(ix), static_cast<std::size_t>(iy),
                                        static_cast<std::size_t>(iz)) !=
                    common::types::VoxelOccupancy::Occupied) {
                    continue;
                }
                const double vx0 = hidden_map_.minXcm() + static_cast<double>(ix) * res;
                const double vy0 = hidden_map_.minYcm() + static_cast<double>(iy) * res;
                const double vz0 = hidden_map_.minZcm() + static_cast<double>(iz) * res;
                const double vx1 = vx0 + res;
                const double vy1 = vy0 + res;
                const double vz1 = vz0 + res;
                const double cx = std::clamp(x, vx0, vx1);
                const double cy = std::clamp(y, vy0, vy1);
                const double cz = std::clamp(z, vz0, vz1);
                const double dx = x - cx;
                const double dy = y - cy;
                const double dz = z - cz;
                if (dx * dx + dy * dy + dz * dz < r2) {
                    return true;
                }
            }
        }
    }
    if (!any_voxel) {
        return true;
    }
    int ix = 0;
    int iy = 0;
    int iz = 0;
    if (!hidden_map_.worldToIndex(x, y, z, ix, iy, iz)) {
        return true;
    }
    return false;
}

void HostMovement::diagCulprits(double x, double y, double z) const {
    const double r = radius_cm_;
    const double res = hidden_map_.resolutionCm();
    const auto lo = [&](double c, double m) {
        return static_cast<int>(std::floor((c - r - m) / res));
    };
    const auto hi = [&](double c, double m) {
        return static_cast<int>(std::floor((c + r - m) / res));
    };
    for (int ix = lo(x, hidden_map_.minXcm()); ix <= hi(x, hidden_map_.minXcm()); ++ix) {
        for (int iy = lo(y, hidden_map_.minYcm()); iy <= hi(y, hidden_map_.minYcm()); ++iy) {
            for (int iz = lo(z, hidden_map_.minZcm()); iz <= hi(z, hidden_map_.minZcm()); ++iz) {
                if (!hidden_map_.indexInRange(ix, iy, iz)) {
                    continue;
                }
                if (hidden_map_.atIndex(static_cast<std::size_t>(ix), static_cast<std::size_t>(iy),
                                        static_cast<std::size_t>(iz)) !=
                    common::types::VoxelOccupancy::Occupied) {
                    continue;
                }
                const double vx0 = hidden_map_.minXcm() + static_cast<double>(ix) * res;
                const double vy0 = hidden_map_.minYcm() + static_cast<double>(iy) * res;
                const double vz0 = hidden_map_.minZcm() + static_cast<double>(iz) * res;
                const double dx = x - std::clamp(x, vx0, vx0 + res);
                const double dy = y - std::clamp(y, vy0, vy0 + res);
                const double dz = z - std::clamp(z, vz0, vz0 + res);
                if (dx * dx + dy * dy + dz * dz >= r * r) {
                    continue;
                }
                std::cout << " culprit_voxel=[" << vx0 << ',' << vy0 << ',' << vz0 << "]+" << res
                          << " gap=" << std::sqrt(dx * dx + dy * dy + dz * dz) << " plugin_map=";
                if (diag_output_map_ == nullptr) {
                    std::cout << "n/a";
                    continue;
                }
                int ox = 0;
                int oy = 0;
                int oz = 0;
                const double cx = vx0 + res * 0.5;
                const double cy = vy0 + res * 0.5;
                const double cz = vz0 + res * 0.5;
                if (!diag_output_map_->worldToIndex(cx, cy, cz, ox, oy, oz)) {
                    std::cout << "outside";
                    continue;
                }
                switch (diag_output_map_->atIndex(static_cast<std::size_t>(ox),
                                                 static_cast<std::size_t>(oy),
                                                 static_cast<std::size_t>(oz))) {
                    case common::types::VoxelOccupancy::Occupied:
                        std::cout << "Occupied";
                        break;
                    case common::types::VoxelOccupancy::Empty:
                        std::cout << "Empty";
                        break;
                    case common::types::VoxelOccupancy::Unmapped:
                        std::cout << "Unmapped";
                        break;
                    default:
                        std::cout << "other";
                        break;
                }
            }
        }
    }
}

bool HostMovement::pathBlocked(double x0,
                               double y0,
                               double z0,
                               double x1,
                               double y1,
                               double z1) const {
    const double dx = x1 - x0;
    const double dy = y1 - y0;
    const double dz = z1 - z0;
    const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    const double step = std::max(0.5, std::min(hidden_map_.resolutionCm() * 0.25, 1.0));
    const int n = std::max(1, static_cast<int>(std::ceil(dist / step)));
    for (int i = 1; i <= n; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(n);
        const double x = x0 + dx * t;
        const double y = y0 + dy * t;
        const double z = z0 + dz * t;
        if (!centerInMissionBounds(x, y, z)) {
            if (diagEnabled()) {
                std::cout << "DIAG_ILLEGAL kind=MISSION_BOUNDS from=(" << x0 << ',' << y0 << ','
                          << z0 << ") to=(" << x1 << ',' << y1 << ',' << z1 << ") at=(" << x << ','
                          << y << ',' << z << ") r=" << radius_cm_ << '\n';
            }
            return true;
        }
        if (sphereHitsWallOrLeavesMap(x, y, z)) {
            if (diagEnabled()) {
                int cix = 0;
                int ciy = 0;
                int ciz = 0;
                const bool in_map = hidden_map_.worldToIndex(x, y, z, cix, ciy, ciz);
                const bool center_occ =
                    in_map && hidden_map_.atIndex(static_cast<std::size_t>(cix),
                                                  static_cast<std::size_t>(ciy),
                                                  static_cast<std::size_t>(ciz)) ==
                                  common::types::VoxelOccupancy::Occupied;
                std::cout << "DIAG_ILLEGAL kind="
                          << (!in_map ? "LEAVES_MAP" : (center_occ ? "CENTER_OCCUPIED" : "SPHERE_GRAZE"))
                          << " from=(" << x0 << ',' << y0 << ',' << z0 << ") to=(" << x1 << ',' << y1
                          << ',' << z1 << ") at=(" << x << ',' << y << ',' << z
                          << ") r=" << radius_cm_ << " t=" << t;
                diagCulprits(x, y, z);
                std::cout << '\n';
            }
            return true;
        }
    }
    return false;
}

common::types::MovementResult HostMovement::rotate(common::types::RotationDirection direction,
                                                   common::HorizontalAngle angle) {
    const double delta = deg_of(angle);
    const auto heading = gps_.heading();
    double h = deg_of(heading.horizontal);
    if (direction == common::types::RotationDirection::Left) {
        h -= delta;
    } else {
        h += delta;
    }
    gps_.setHeading({horiz_deg(wrap_deg(h)), heading.altitude});
    return common::types::MovementResult{true, {}};
}

common::types::MovementResult HostMovement::advance(common::PhysicalLength distance) {
    const double dist = cm_of(distance);
    const auto pos = gps_.position();
    const double x0 = cm_of(pos.x);
    const double y0 = cm_of(pos.y);
    const double z0 = cm_of(pos.z);
    const double h = deg_to_rad(deg_of(gps_.heading().horizontal));
    const double x1 = x0 + dist * std::cos(h);
    const double y1 = y0 + dist * std::sin(h);
    if (pathBlocked(x0, y0, z0, x1, y1, z0)) {
        return fail("blocked: wall or boundary");
    }
    gps_.setPosition(pos_cm(x1, y1, z0));
    return common::types::MovementResult{true, {}};
}

common::types::MovementResult HostMovement::elevate(common::PhysicalLength distance) {
    const double dist = cm_of(distance);
    const auto pos = gps_.position();
    const double x0 = cm_of(pos.x);
    const double y0 = cm_of(pos.y);
    const double z0 = cm_of(pos.z);
    const double z1 = z0 + dist;
    if (pathBlocked(x0, y0, z0, x0, y0, z1)) {
        return fail("blocked: wall or boundary");
    }
    gps_.setPosition(pos_cm(x0, y0, z1));
    return common::types::MovementResult{true, {}};
}

}  // namespace skeleton_host
