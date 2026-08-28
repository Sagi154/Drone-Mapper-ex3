// Baseline MappingAlgorithm: coarse 3D lawnmower that always terminates.
// See ASSUMPTIONS.md in this directory.

#include <Common/IMappingAlgorithm.h>
#include <Common/MappingAlgorithmFactory.h>
#include <Common/MappingAlgorithmRegistration.h>
#include <Common/Types.h>
#include <Common/Units.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <utility>
#include <vector>

namespace {

[[nodiscard]] double cm_of(const common::XLength& q) {
    return q.numerical_value_in(common::cm);
}
[[nodiscard]] double cm_of(const common::YLength& q) {
    return q.numerical_value_in(common::cm);
}
[[nodiscard]] double cm_of(const common::ZLength& q) {
    return q.numerical_value_in(common::cm);
}
[[nodiscard]] double cm_of(const common::PhysicalLength& q) {
    return q.numerical_value_in(common::cm);
}
[[nodiscard]] double deg_of(const common::HorizontalAngle& q) {
    return q.numerical_value_in(common::deg);
}

[[nodiscard]] constexpr double rad_to_deg(double rad) {
    return rad * 180.0 / 3.14159265358979323846;
}

// Wrap to (-180, 180].
[[nodiscard]] double wrap180(double degrees) {
    degrees = std::fmod(degrees, 360.0);
    if (degrees > 180.0) {
        degrees -= 360.0;
    } else if (degrees <= -180.0) {
        degrees += 360.0;
    }
    return degrees;
}

[[nodiscard]] common::Position3D pos_cm(double x, double y, double z) {
    return {common::XLength{x * common::cm}, common::YLength{y * common::cm},
            common::ZLength{z * common::cm}};
}

struct Look {
    double horizontal_deg;
    double altitude_deg;
};

constexpr Look kLooks[] = {
    {0.0, 0.0}, {90.0, 0.0}, {180.0, 0.0}, {270.0, 0.0}, {0.0, 45.0}, {0.0, -45.0},
};
constexpr std::size_t kLookCount = sizeof(kLooks) / sizeof(kLooks[0]);
constexpr double kHeadingAlignDeg = 1.0;
constexpr double kStuckEpsCm = 0.5;
constexpr double kDefaultResolutionCm = 10.0;

[[nodiscard]] std::size_t cellCount(double lo, double hi, double res) {
    if (!(hi > lo) || !(res > 0.0)) {
        return 0;
    }
    const double n = std::floor((hi - lo) / res);
    if (n <= 0.0) {
        return 0;
    }
    return static_cast<std::size_t>(n);
}

[[nodiscard]] std::vector<double> axisCenters(double lo,
                                              double hi,
                                              double res,
                                              std::size_t stride,
                                              double radius) {
    const std::size_t n = cellCount(lo, hi, res);
    if (n == 0) {
        return {};
    }
    const std::size_t step = std::max<std::size_t>(1, stride);
    std::vector<double> raw;
    raw.reserve(n / step + 2);
    for (std::size_t i = 0; i < n; i += step) {
        raw.push_back(lo + (static_cast<double>(i) + 0.5) * res);
    }
    if ((n - 1) % step != 0) {
        raw.push_back(lo + (static_cast<double>(n - 1) + 0.5) * res);
    }

    std::vector<double> inset;
    inset.reserve(raw.size());
    for (double v : raw) {
        if (v - lo >= radius && hi - v >= radius) {
            inset.push_back(v);
        }
    }
    return inset.empty() ? raw : inset;
}

[[nodiscard]] std::size_t strideCells(double spacing_cm, double res, double z_max_cm) {
    double spacing = std::max(res, spacing_cm);
    if (z_max_cm > 0.0) {
        spacing = std::min(spacing, z_max_cm);
    }
    spacing = std::max(spacing, res);
    const auto cells = static_cast<std::size_t>(std::llround(spacing / res));
    return std::max<std::size_t>(1, cells);
}

[[nodiscard]] common::types::MovementCommand makeHover() {
    common::types::MovementCommand move{};
    move.type = common::types::MovementCommandType::Hover;
    return move;
}

[[nodiscard]] common::types::MovementCommand makeAdvance(double distance_cm) {
    common::types::MovementCommand move{};
    move.type = common::types::MovementCommandType::Advance;
    move.distance = common::PhysicalLength{distance_cm * common::cm};
    return move;
}

[[nodiscard]] common::types::MovementCommand makeElevate(double distance_cm) {
    common::types::MovementCommand move{};
    move.type = common::types::MovementCommandType::Elevate;
    move.distance = common::PhysicalLength{distance_cm * common::cm};
    return move;
}

[[nodiscard]] common::types::MovementCommand makeRotate(double delta_deg) {
    common::types::MovementCommand move{};
    move.type = common::types::MovementCommandType::Rotate;
    if (delta_deg > 0.0) {
        move.rotation = common::types::RotationDirection::Right;
        move.angle = common::HorizontalAngle{delta_deg * common::deg};
    } else {
        move.rotation = common::types::RotationDirection::Left;
        move.angle = common::HorizontalAngle{(-delta_deg) * common::deg};
    }
    return move;
}

[[nodiscard]] common::Orientation lookFor(std::size_t step_index) {
    const Look& look = kLooks[step_index % kLookCount];
    return {common::HorizontalAngle{look.horizontal_deg * common::deg},
            common::AltitudeAngle{look.altitude_deg * common::deg}};
}

}  // namespace

class BaselineLawnmowerAlgorithm final : public common::IMappingAlgorithm {
public:
    explicit BaselineLawnmowerAlgorithm(common::MappingAlgorithmDependencies dependencies)
        : common::IMappingAlgorithm(std::move(dependencies)) {}

    [[nodiscard]] common::types::MappingStepCommand nextStep(
        const common::types::DroneState& state,
        const common::types::LidarScanResult*) override {
        handleStuck(state);

        if (!initialized_) {
            buildWaypoints();
            initialized_ = true;
        }

        const bool last_step = budgetExhausted(state);
        common::types::MappingStepCommand cmd = planWorking(state);
        if (cmd.status != common::types::AlgorithmStatus::Working) {
            return cmd;
        }
        if (last_step) {
            cmd.status = finishStatus();
        }
        return cmd;
    }

private:
    bool initialized_ = false;
    std::vector<common::Position3D> waypoints_{};
    std::size_t wp_ = 0;
    bool pending_translate_ = false;
    double last_x_cm_ = 0.0;
    double last_y_cm_ = 0.0;
    double last_z_cm_ = 0.0;
    double resolution_cm_ = kDefaultResolutionCm;

    [[nodiscard]] bool budgetExhausted(const common::types::DroneState& state) const {
        if (mission_config_.max_steps == 0) {
            return true;
        }
        return state.step_index + 1 >= mission_config_.max_steps;
    }

    [[nodiscard]] double arrivalEpsCm() const {
        return std::max(1.0, 0.25 * resolution_cm_);
    }

    void handleStuck(const common::types::DroneState& state) {
        if (!pending_translate_) {
            return;
        }
        pending_translate_ = false;
        const double dx = cm_of(state.position.x) - last_x_cm_;
        const double dy = cm_of(state.position.y) - last_y_cm_;
        const double dz = cm_of(state.position.z) - last_z_cm_;
        if (std::hypot(dx, dy, dz) < kStuckEpsCm) {
            if (wp_ < waypoints_.size()) {
                ++wp_;
            }
        }
    }

    void markTranslate(const common::types::DroneState& state) {
        pending_translate_ = true;
        last_x_cm_ = cm_of(state.position.x);
        last_y_cm_ = cm_of(state.position.y);
        last_z_cm_ = cm_of(state.position.z);
    }

    void buildWaypoints() {
        const common::types::MapConfig cfg = output_map_.getMapConfig();
        resolution_cm_ = cm_of(cfg.resolution);
        if (!(resolution_cm_ > 0.0)) {
            resolution_cm_ = cm_of(mission_config_.gps_resolution);
        }
        if (!(resolution_cm_ > 0.0)) {
            resolution_cm_ = kDefaultResolutionCm;
        }

        common::types::MappingBounds bounds = cfg.boundaries;
        auto axis_ok = [](double lo, double hi) { return hi > lo; };
        if (!axis_ok(cm_of(bounds.min_x), cm_of(bounds.max_x)) ||
            !axis_ok(cm_of(bounds.min_y), cm_of(bounds.max_y)) ||
            !axis_ok(cm_of(bounds.min_height), cm_of(bounds.max_height))) {
            bounds = mission_config_.mission_bounds;
        }

        const double min_x = cm_of(bounds.min_x);
        const double max_x = cm_of(bounds.max_x);
        const double min_y = cm_of(bounds.min_y);
        const double max_y = cm_of(bounds.max_y);
        const double min_z = cm_of(bounds.min_height);
        const double max_z = cm_of(bounds.max_height);
        const double radius = std::max(0.0, cm_of(drone_config_.radius));
        const double z_max = cm_of(lidar_config_.z_max);

        const std::size_t stride_xy =
            strideCells(cm_of(drone_config_.max_advance), resolution_cm_, z_max);
        const std::size_t stride_z =
            strideCells(cm_of(drone_config_.max_elevate), resolution_cm_, z_max);

        const std::vector<double> xs = axisCenters(min_x, max_x, resolution_cm_, stride_xy, radius);
        const std::vector<double> ys = axisCenters(min_y, max_y, resolution_cm_, stride_xy, radius);
        const std::vector<double> zs = axisCenters(min_z, max_z, resolution_cm_, stride_z, radius);
        if (xs.empty() || ys.empty() || zs.empty()) {
            waypoints_.clear();
            return;
        }

        waypoints_.clear();
        waypoints_.reserve(xs.size() * ys.size() * zs.size());
        for (std::size_t iz = 0; iz < zs.size(); ++iz) {
            for (std::size_t iy = 0; iy < ys.size(); ++iy) {
                const bool reverse_x = ((iz + iy) % 2) != 0;
                if (!reverse_x) {
                    for (double x : xs) {
                        appendWaypoint(x, ys[iy], zs[iz]);
                    }
                } else {
                    for (std::size_t k = xs.size(); k > 0; --k) {
                        appendWaypoint(xs[k - 1], ys[iy], zs[iz]);
                    }
                }
            }
        }
    }

    void appendWaypoint(double x, double y, double z) {
        const common::Position3D p = pos_cm(x, y, z);
        if (output_map_.isInBounds(p)) {
            waypoints_.push_back(p);
        }
    }

    [[nodiscard]] bool skipWaypoint(const common::Position3D& p) const {
        const auto occ = output_map_.atVoxel(p);
        using common::types::VoxelOccupancy;
        return occ == VoxelOccupancy::Occupied || occ == VoxelOccupancy::PotentiallyOccupied ||
               occ == VoxelOccupancy::Empty || occ == VoxelOccupancy::OutOfBounds;
    }

    void skipMappedAndBlocked() {
        while (wp_ < waypoints_.size() && skipWaypoint(waypoints_[wp_])) {
            ++wp_;
        }
    }

    [[nodiscard]] common::types::MappingStepCommand planWorking(
        const common::types::DroneState& state) {
        const double max_rotate = std::max(0.0, deg_of(drone_config_.max_rotate));
        const double max_advance = std::max(0.0, cm_of(drone_config_.max_advance));
        const double max_elevate = std::max(0.0, cm_of(drone_config_.max_elevate));
        const double eps = arrivalEpsCm();
        const std::size_t guard = waypoints_.size() + 2;

        for (std::size_t iter = 0; iter < guard; ++iter) {
            skipMappedAndBlocked();
            if (wp_ >= waypoints_.size()) {
                return makeFinish();
            }

            const common::Position3D& target = waypoints_[wp_];
            const double x = cm_of(state.position.x);
            const double y = cm_of(state.position.y);
            const double z = cm_of(state.position.z);
            const double tx = cm_of(target.x);
            const double ty = cm_of(target.y);
            const double tz = cm_of(target.z);
            const double dz = tz - z;
            const double dx = tx - x;
            const double dy = ty - y;
            const double dist_xy = std::hypot(dx, dy);

            if (std::abs(dz) > eps) {
                if (!(max_elevate > 0.0)) {
                    ++wp_;
                    continue;
                }
                const double step = std::clamp(dz, -max_elevate, max_elevate);
                return makeWorking(state, makeElevate(step), true);
            }

            if (dist_xy > eps) {
                const double desired = rad_to_deg(std::atan2(dy, dx));
                const double delta = wrap180(desired - deg_of(state.heading.horizontal));
                if (std::abs(delta) > kHeadingAlignDeg) {
                    if (!(max_rotate > 0.0)) {
                        ++wp_;
                        continue;
                    }
                    const double mag = std::min(std::abs(delta), max_rotate);
                    return makeWorking(state, makeRotate(delta > 0.0 ? mag : -mag), false);
                }
                if (!(max_advance > 0.0)) {
                    ++wp_;
                    continue;
                }
                const double step = std::min(dist_xy, max_advance);
                return makeWorking(state, makeAdvance(step), true);
            }

            ++wp_;
        }
        return makeFinish();
    }

    [[nodiscard]] common::types::MappingStepCommand makeWorking(
        const common::types::DroneState& state,
        common::types::MovementCommand movement,
        bool is_translate) {
        if (is_translate) {
            markTranslate(state);
        }
        common::types::MappingStepCommand cmd{};
        cmd.status = common::types::AlgorithmStatus::Working;
        cmd.movement = std::move(movement);
        cmd.scan_orientation = lookFor(state.step_index);
        return cmd;
    }

    [[nodiscard]] common::types::MappingStepCommand makeFinish() const {
        common::types::MappingStepCommand cmd{};
        cmd.status = finishStatus();
        cmd.movement = makeHover();
        cmd.scan_orientation = std::nullopt;
        return cmd;
    }

    [[nodiscard]] common::types::AlgorithmStatus finishStatus() const {
        const common::types::MapConfig cfg = output_map_.getMapConfig();
        const double res = cm_of(cfg.resolution);
        if (!(res > 0.0)) {
            return common::types::AlgorithmStatus::FinishedWithUnmappableVoxels;
        }

        const double min_x = cm_of(cfg.boundaries.min_x);
        const double max_x = cm_of(cfg.boundaries.max_x);
        const double min_y = cm_of(cfg.boundaries.min_y);
        const double max_y = cm_of(cfg.boundaries.max_y);
        const double min_z = cm_of(cfg.boundaries.min_height);
        const double max_z = cm_of(cfg.boundaries.max_height);
        const std::size_t nx = cellCount(min_x, max_x, res);
        const std::size_t ny = cellCount(min_y, max_y, res);
        const std::size_t nz = cellCount(min_z, max_z, res);
        if (nx == 0 || ny == 0 || nz == 0) {
            return common::types::AlgorithmStatus::FinishedWithUnmappableVoxels;
        }

        using common::types::VoxelOccupancy;
        for (std::size_t ix = 0; ix < nx; ++ix) {
            const double x = min_x + (static_cast<double>(ix) + 0.5) * res;
            for (std::size_t iy = 0; iy < ny; ++iy) {
                const double y = min_y + (static_cast<double>(iy) + 0.5) * res;
                for (std::size_t iz = 0; iz < nz; ++iz) {
                    const double z = min_z + (static_cast<double>(iz) + 0.5) * res;
                    const common::Position3D p = pos_cm(x, y, z);
                    if (!output_map_.isInBounds(p)) {
                        continue;
                    }
                    const VoxelOccupancy occ = output_map_.atVoxel(p);
                    if (occ == VoxelOccupancy::Unmapped ||
                        occ == VoxelOccupancy::PotentiallyOccupied) {
                        return common::types::AlgorithmStatus::FinishedWithUnmappableVoxels;
                    }
                }
            }
        }
        return common::types::AlgorithmStatus::Finished;
    }
};

REGISTER_MAPPING_ALGORITHM(BaselineLawnmowerAlgorithm);
