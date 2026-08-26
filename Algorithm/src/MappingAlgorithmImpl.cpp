// MappingAlgorithmImpl.cpp
// 26-direction scan batch + density-scored BFS frontier cleanup.

#include <Algorithm/MappingAlgorithmImpl.h>

#include "MappingAlgorithmFrontier.h"

#include <Common/MappingAlgorithmRegistration.h>

#include <cmath>
#include <limits>
#include <numbers>
#include <optional>
#include <utility>

namespace algorithm_207190406_209543255 {

namespace types = common::types;
using common::Orientation;
using common::Position3D;
using common::cm;
using common::deg;

namespace {

constexpr double kHalfStepTolerance = 0.5;
constexpr double kPositionEpsilon = 1e-6;

[[nodiscard]] double gridStepCm(const types::MapConfig& config) {
    return config.resolution.force_numerical_value_in(cm);
}

[[nodiscard]] int axisSign(double delta_cm) {
    if (delta_cm > 1e-6) {
        return 1;
    }
    if (delta_cm < -1e-6) {
        return -1;
    }
    return 0;
}

[[nodiscard]] bool areCollinearSteps(const Position3D& prev,
                                     const Position3D& mid,
                                     const Position3D& next) {
    const int d1x = axisSign(mid.x.force_numerical_value_in(cm) - prev.x.force_numerical_value_in(cm));
    const int d1y = axisSign(mid.y.force_numerical_value_in(cm) - prev.y.force_numerical_value_in(cm));
    const int d1z = axisSign(mid.z.force_numerical_value_in(cm) - prev.z.force_numerical_value_in(cm));
    const int d2x = axisSign(next.x.force_numerical_value_in(cm) - mid.x.force_numerical_value_in(cm));
    const int d2y = axisSign(next.y.force_numerical_value_in(cm) - mid.y.force_numerical_value_in(cm));
    const int d2z = axisSign(next.z.force_numerical_value_in(cm) - mid.z.force_numerical_value_in(cm));
    return d1x == d2x && d1y == d2y && d1z == d2z;
}

[[nodiscard]] std::vector<Position3D> compressPath(const std::vector<Position3D>& path) {
    if (path.size() <= 2) {
        return path;
    }

    std::vector<Position3D> out;
    out.reserve(path.size());
    out.push_back(path.front());
    for (std::size_t i = 1; i + 1 < path.size(); ++i) {
        if (!areCollinearSteps(path[i - 1], path[i], path[i + 1])) {
            out.push_back(path[i]);
        }
    }
    out.push_back(path.back());
    return out;
}

} // namespace

struct MappingAlgorithmImpl_207190406_209543255::Impl {
    Phase phase = Phase::Scanning;
    detail::MappingAlgorithmFrontier frontier{};

    std::vector<Orientation> scan_orientations{};
    std::size_t scan_index = 0;

    std::vector<Position3D> current_path{};
    std::size_t path_index = 0;
    int moving_stall_ticks = 0;
    Position3D last_position{};
    bool has_last_position = false;

    detail::BlockedCells blocked_cells{};
    detail::GridIntMap frontier_visit_counts{};
    detail::GridIntMap explore_dist_cache{};
    std::optional<detail::GridKey> pending_frontier_visit{};
    bool finished = false;
    int no_frontier_stuck_cycles = 0;
    std::size_t prev_unmapped_count = std::numeric_limits<std::size_t>::max();
    int no_progress_cycles = 0;

    bool planning_initialized = false;
    int spacing_cells = 2;
    int steps_since_scan = 0;
    bool resume_moving_after_scan = false;
    bool enable_scan_during_travel = false;
};

MappingAlgorithmImpl_207190406_209543255::~MappingAlgorithmImpl_207190406_209543255() = default;

MappingAlgorithmImpl_207190406_209543255::MappingAlgorithmImpl_207190406_209543255(
    common::MappingAlgorithmDependencies dependencies)
    : common::IMappingAlgorithm(dependencies), impl_(std::make_unique<Impl>()) {
    // Mid-path scanning only for tight step budgets (large_room); full matrix runs
    // showed it regresses large_out when enabled globally.
    impl_->enable_scan_during_travel = mission_config_.max_steps <= 500;
}

void MappingAlgorithmImpl_207190406_209543255::ensurePlanningReady() {
    if (impl_->planning_initialized) {
        return;
    }

    const types::MapConfig& config = output_map_.getMapConfig();
    const double step_cm = gridStepCm(config);
    if (step_cm > 0.0) {
        const double z_max_cm = lidar_config_.z_max.force_numerical_value_in(cm);
        const int spacing_raw = static_cast<int>(std::lround(z_max_cm / step_cm / 2.0));
        impl_->spacing_cells = std::clamp(spacing_raw, 2, 15);
    }

    impl_->planning_initialized = true;
}

void MappingAlgorithmImpl_207190406_209543255::buildScanOrientations(const Orientation& heading,
                                                 const Position3D& /*position*/) {
    impl_->scan_orientations.clear();
    impl_->scan_orientations.reserve(26);

    const auto addDirection = [&](double dx, double dy, double dz) {
        const double len = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (len < 1e-9) {
            return;
        }
        dx /= len;
        dy /= len;
        dz /= len;

        const double az_deg = std::atan2(dy, dx) * (180.0 / std::numbers::pi);
        const double horiz_len = std::sqrt(dx * dx + dy * dy);
        const double el_deg = std::atan2(dz, horiz_len) * (180.0 / std::numbers::pi);

        double az_norm = az_deg;
        while (az_norm < 0.0) {
            az_norm += 360.0;
        }
        while (az_norm >= 360.0) {
            az_norm -= 360.0;
        }

        impl_->scan_orientations.push_back(
            Orientation{az_norm * deg - heading.horizontal, el_deg * deg - heading.altitude});
    };

    addDirection(1.0, 0.0, 0.0);
    addDirection(-1.0, 0.0, 0.0);
    addDirection(0.0, 1.0, 0.0);
    addDirection(0.0, -1.0, 0.0);
    addDirection(0.0, 0.0, 1.0);
    addDirection(0.0, 0.0, -1.0);

    const double inv_sqrt2 = 1.0 / std::sqrt(2.0);
    for (const int sx : {-1, 1}) {
        for (const int sy : {-1, 1}) {
            addDirection(static_cast<double>(sx) * inv_sqrt2,
                         static_cast<double>(sy) * inv_sqrt2,
                         0.0);
        }
    }
    for (const int sx : {-1, 1}) {
        for (const int sz : {-1, 1}) {
            addDirection(static_cast<double>(sx) * inv_sqrt2,
                         0.0,
                         static_cast<double>(sz) * inv_sqrt2);
        }
    }
    for (const int sy : {-1, 1}) {
        for (const int sz : {-1, 1}) {
            addDirection(0.0,
                         static_cast<double>(sy) * inv_sqrt2,
                         static_cast<double>(sz) * inv_sqrt2);
        }
    }

    const double inv_sqrt3 = 1.0 / std::sqrt(3.0);
    for (const int sx : {-1, 1}) {
        for (const int sy : {-1, 1}) {
            for (const int sz : {-1, 1}) {
                addDirection(static_cast<double>(sx) * inv_sqrt3,
                             static_cast<double>(sy) * inv_sqrt3,
                             static_cast<double>(sz) * inv_sqrt3);
            }
        }
    }

}

bool MappingAlgorithmImpl_207190406_209543255::samePosition(const Position3D& a, const Position3D& b) const {
    const double dx = std::abs(a.x.force_numerical_value_in(cm) - b.x.force_numerical_value_in(cm));
    const double dy = std::abs(a.y.force_numerical_value_in(cm) - b.y.force_numerical_value_in(cm));
    const double dz = std::abs(a.z.force_numerical_value_in(cm) - b.z.force_numerical_value_in(cm));
    return dx <= kPositionEpsilon && dy <= kPositionEpsilon && dz <= kPositionEpsilon;
}

bool MappingAlgorithmImpl_207190406_209543255::reachedWaypoint(const types::DroneState& state,
                                           const Position3D& target) const {
    const double step = gridStepCm(output_map_.getMapConfig());
    const double dx = std::abs(state.position.x.force_numerical_value_in(cm) -
                               target.x.force_numerical_value_in(cm));
    const double dy = std::abs(state.position.y.force_numerical_value_in(cm) -
                               target.y.force_numerical_value_in(cm));
    const double dz = std::abs(state.position.z.force_numerical_value_in(cm) -
                               target.z.force_numerical_value_in(cm));
    return dx <= step * kHalfStepTolerance && dy <= step * kHalfStepTolerance &&
           dz <= step * kHalfStepTolerance;
}

std::optional<types::MovementCommand> MappingAlgorithmImpl_207190406_209543255::movementToward(
    const types::DroneState& state, const Position3D& target) const {
    const double dh =
        target.z.force_numerical_value_in(cm) - state.position.z.force_numerical_value_in(cm);
    if (std::abs(dh) > 1e-6) {
        const double limit = drone_config_.max_elevate.force_numerical_value_in(cm);
        types::MovementCommand cmd{};
        cmd.type = types::MovementCommandType::Elevate;
        cmd.distance = std::clamp(dh, -limit, limit) * cm;
        return cmd;
    }

    const double dx =
        target.x.force_numerical_value_in(cm) - state.position.x.force_numerical_value_in(cm);
    const double dy =
        target.y.force_numerical_value_in(cm) - state.position.y.force_numerical_value_in(cm);
    if (std::abs(dx) < 1e-6 && std::abs(dy) < 1e-6) {
        types::MovementCommand cmd{};
        cmd.type = types::MovementCommandType::Hover;
        return cmd;
    }

    const double target_heading = std::atan2(dy, dx) * (180.0 / std::numbers::pi);
    const double current_heading = state.heading.horizontal.force_numerical_value_in(deg);
    double delta = std::fmod(target_heading - current_heading, 360.0);
    if (delta > 180.0) {
        delta -= 360.0;
    }
    if (delta < -180.0) {
        delta += 360.0;
    }

    const double rot_limit = drone_config_.max_rotate.force_numerical_value_in(deg);
    if (std::abs(delta) > 1e-6) {
        types::MovementCommand cmd{};
        cmd.type = types::MovementCommandType::Rotate;
        cmd.rotation =
            (delta > 0.0) ? types::RotationDirection::Left : types::RotationDirection::Right;
        cmd.angle = std::min(std::abs(delta), rot_limit) * deg;
        return cmd;
    }

    const double dist_cm = std::sqrt(dx * dx + dy * dy);
    const double adv_limit = drone_config_.max_advance.force_numerical_value_in(cm);
    types::MovementCommand cmd{};
    cmd.type = types::MovementCommandType::Advance;
    cmd.distance = std::min(dist_cm, adv_limit) * cm;
    return cmd;
}

types::MappingStepCommand MappingAlgorithmImpl_207190406_209543255::handleScanningPhase(const types::DroneState& state) {
    if (impl_->scan_orientations.empty()) {
        buildScanOrientations(state.heading, state.position);
        impl_->scan_index = 0;
    }

    if (impl_->scan_index < impl_->scan_orientations.size()) {
        types::MappingStepCommand cmd{};
        cmd.scan_orientation = impl_->scan_orientations[impl_->scan_index++];
        cmd.status = types::AlgorithmStatus::Working;
        return cmd;
    }

    impl_->scan_orientations.clear();
    impl_->scan_index = 0;
    if (impl_->resume_moving_after_scan) {
        impl_->resume_moving_after_scan = false;
        impl_->steps_since_scan = 0;
        impl_->phase = Phase::Moving;
        return handleMovingPhase(state);
    }
    if (impl_->pending_frontier_visit.has_value()) {
        ++impl_->frontier_visit_counts[*impl_->pending_frontier_visit];
        impl_->pending_frontier_visit.reset();
    }
    impl_->phase = Phase::Planning;
    return handlePlanningPhase(state);
}

types::MappingStepCommand MappingAlgorithmImpl_207190406_209543255::handleFrontierCleanupPhase(
    const types::DroneState& state) {
    const std::size_t unmapped = detail::countUnmappedInBounds(output_map_);
    if (impl_->prev_unmapped_count != std::numeric_limits<std::size_t>::max()) {
        if (unmapped >= impl_->prev_unmapped_count) {
            ++impl_->no_progress_cycles;
        } else {
            impl_->no_progress_cycles = 0;
        }
    }
    impl_->prev_unmapped_count = unmapped;

    // Stop when the map stops improving even though findPath may still succeed.
    constexpr int kNoProgressLimit = 100;
    if (unmapped > 0 && impl_->no_progress_cycles >= kNoProgressLimit) {
        impl_->finished = true;
        types::MappingStepCommand cmd{};
        cmd.status = types::AlgorithmStatus::FinishedWithUnmappableVoxels;
        return cmd;
    }

    const detail::FrontierPathResult result = impl_->frontier.findPath(
        output_map_, state.position, drone_config_.radius, impl_->blocked_cells,
        impl_->frontier_visit_counts);

    if (!result.found) {
        const detail::PlanningDiagnostics diag = impl_->frontier.diagnose(
            output_map_, state.position, drone_config_.radius, impl_->blocked_cells);

        const bool mission_has_unknown = detail::hasAnyNotMappedInBounds(output_map_);

        if (!mission_has_unknown) {
            impl_->finished = true;
            types::MappingStepCommand cmd{};
            cmd.status = types::AlgorithmStatus::Finished;
            return cmd;
        }

        // No reachable frontier but unknown cells remain. If this persists for
        // enough consecutive planning cycles the remaining cells are unreachable
        // from any flyable position and will never be mapped.
        constexpr int kNoFrontierStuckLimit = 100;
        ++impl_->no_frontier_stuck_cycles;
        if (impl_->no_frontier_stuck_cycles >= kNoFrontierStuckLimit) {
            impl_->finished = true;
            types::MappingStepCommand cmd{};
            cmd.status = types::AlgorithmStatus::FinishedWithUnmappableVoxels;
            return cmd;
        }

        if (diag.explore_path_available) {
            const detail::FrontierPathResult explore = impl_->frontier.findExplorePath(
                output_map_, state.position, drone_config_.radius, impl_->blocked_cells,
                &impl_->explore_dist_cache);
            if (explore.found && !explore.path.empty()) {
                impl_->no_frontier_stuck_cycles = 0;
                impl_->pending_frontier_visit.reset();
                impl_->current_path = compressPath(explore.path);
                impl_->path_index = 0;
                impl_->moving_stall_ticks = 0;
                impl_->phase = Phase::Moving;
                return handleMovingPhase(state);
            }
        }

        if (!diag.start_passable) {
            const detail::FrontierPathResult unstick =
                impl_->frontier.findUnstickPath(output_map_, state.position, drone_config_.radius);
            if (unstick.found && !unstick.path.empty()) {
                impl_->no_frontier_stuck_cycles = 0;
                impl_->pending_frontier_visit.reset();
                impl_->current_path = compressPath(unstick.path);
                impl_->path_index = 0;
                impl_->moving_stall_ticks = 0;
                impl_->phase = Phase::Moving;
                return handleMovingPhase(state);
            }
        }

        const detail::FrontierPathResult wander = impl_->frontier.findAnyPassableNeighbor(
            output_map_, state.position, drone_config_.radius, impl_->blocked_cells);
        if (wander.found && !wander.path.empty()) {
            impl_->no_frontier_stuck_cycles = 0;
            impl_->pending_frontier_visit.reset();
            impl_->current_path = compressPath(wander.path);
            impl_->path_index = 0;
            impl_->moving_stall_ticks = 0;
            impl_->phase = Phase::Moving;
            return handleMovingPhase(state);
        }

        impl_->finished = true;
        types::MappingStepCommand cmd{};
        cmd.status = types::AlgorithmStatus::FinishedWithUnmappableVoxels;
        return cmd;
    }

    impl_->no_frontier_stuck_cycles = 0;
    impl_->pending_frontier_visit = result.frontier_key;
    impl_->explore_dist_cache.clear();  // map changed; invalidate the distance-field cache
    impl_->current_path = compressPath(result.path);
    impl_->path_index = 0;
    impl_->moving_stall_ticks = 0;
    impl_->phase = Phase::Moving;
    return handleMovingPhase(state);
}

types::MappingStepCommand MappingAlgorithmImpl_207190406_209543255::handlePlanningPhase(const types::DroneState& state) {
    ensurePlanningReady();
    return handleFrontierCleanupPhase(state);
}

types::MappingStepCommand MappingAlgorithmImpl_207190406_209543255::handleMovingPhase(const types::DroneState& state) {
    if (impl_->path_index >= impl_->current_path.size()) {
        impl_->phase = Phase::Scanning;
        return handleScanningPhase(state);
    }

    if (reachedWaypoint(state, impl_->current_path[impl_->path_index])) {
        ++impl_->path_index;
        impl_->moving_stall_ticks = 0;
        ++impl_->steps_since_scan;
        if (impl_->path_index >= impl_->current_path.size()) {
            impl_->phase = Phase::Scanning;
            return handleScanningPhase(state);
        }
        if (impl_->enable_scan_during_travel &&
            impl_->steps_since_scan >= impl_->spacing_cells) {
            impl_->resume_moving_after_scan = true;
            impl_->phase = Phase::Scanning;
            return handleScanningPhase(state);
        }
    } else if (impl_->has_last_position && samePosition(impl_->last_position, state.position)) {
        ++impl_->moving_stall_ticks;
        if (impl_->moving_stall_ticks >= kMaxMovingStallTicks) {
            const Position3D& wp = impl_->current_path[impl_->path_index];
            const auto blocked_key = detail::quantizePosition(wp, output_map_.getMapConfig());
            impl_->blocked_cells.insert(blocked_key);
            impl_->moving_stall_ticks = 0;
            impl_->phase = Phase::Planning;
            return handlePlanningPhase(state);
        }
    } else {
        impl_->moving_stall_ticks = 0;
    }

    impl_->last_position = state.position;
    impl_->has_last_position = true;

    types::MappingStepCommand cmd{};
    cmd.movement = movementToward(state, impl_->current_path[impl_->path_index]);
    cmd.status = types::AlgorithmStatus::Working;
    return cmd;
}

types::MappingStepCommand MappingAlgorithmImpl_207190406_209543255::nextStep(const types::DroneState& state,
                                                         const types::LidarScanResult* latest_scan) {
    (void)latest_scan;

    if (impl_->finished) {
        types::MappingStepCommand cmd{};
        cmd.status = types::AlgorithmStatus::Finished;
        return cmd;
    }

    switch (impl_->phase) {
    case Phase::Scanning:
        return handleScanningPhase(state);
    case Phase::Planning:
        return handlePlanningPhase(state);
    case Phase::Moving:
        return handleMovingPhase(state);
    }

    types::MappingStepCommand cmd{};
    cmd.status = types::AlgorithmStatus::Finished;
    return cmd;
}

} // namespace algorithm_207190406_209543255


using MappingAlgorithmImpl_207190406_209543255 =
    algorithm_207190406_209543255::MappingAlgorithmImpl_207190406_209543255;
REGISTER_MAPPING_ALGORITHM(MappingAlgorithmImpl_207190406_209543255);
