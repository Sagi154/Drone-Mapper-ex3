// MappingAlgorithmImpl.cpp
// Budget-aware next-best-view exploration: hold a plan, emit one command per nextStep.

#include <Algorithm/MappingAlgorithmImpl.h>

#include "NbvPlanner.h"

#include <user_common_207190406_209543255/LidarCone.h>

#include <Common/MappingAlgorithmRegistration.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <optional>
#include <unordered_set>
#include <vector>

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

[[nodiscard]] [[maybe_unused]] int axisSign(double delta_cm) {
    if (delta_cm > 1e-6) {
        return 1;
    }
    if (delta_cm < -1e-6) {
        return -1;
    }
    return 0;
}

} // namespace

struct MappingAlgorithmImpl_207190406_209543255::Impl {
    detail::NbvPlanner planner{};
    detail::ExplorationPlan plan{};
    std::size_t waypoint_index = 0;
    std::size_t terminal_scan_index = 0;
    std::size_t steps_since_replan = 0;
    bool has_plan = false;

    int moving_stall_ticks = 0;
    Position3D last_position{};
    bool has_last_position = false;

    detail::BlockedCells blocked_cells{};
    detail::GridIntMap blocked_since{};  ///< cell -> step_index of insertion (TTL)
    int recovery_attempts = 0;
    bool finished = false;
    bool planning_initialized = false;
};

MappingAlgorithmImpl_207190406_209543255::~MappingAlgorithmImpl_207190406_209543255() = default;

MappingAlgorithmImpl_207190406_209543255::MappingAlgorithmImpl_207190406_209543255(
    common::MappingAlgorithmDependencies dependencies)
    : common::IMappingAlgorithm(dependencies), impl_(std::make_unique<Impl>()) {}

void MappingAlgorithmImpl_207190406_209543255::ensurePlanningReady() {
    if (impl_->planning_initialized) {
        return;
    }
    impl_->planning_initialized = true;
}

bool MappingAlgorithmImpl_207190406_209543255::samePosition(const Position3D& a,
                                                           const Position3D& b) const {
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

std::size_t MappingAlgorithmImpl_207190406_209543255::remainingSteps(
    const types::DroneState& state) const {
    const std::size_t budget = mission_config_.max_steps;
    if (budget == 0) {
        return 0;
    }
    return (state.step_index >= budget) ? 0 : (budget - state.step_index);
}

void MappingAlgorithmImpl_207190406_209543255::pruneExpiredBlockedCells(std::size_t step_index) {
    for (auto it = impl_->blocked_since.begin(); it != impl_->blocked_since.end();) {
        const auto inserted = static_cast<std::size_t>(it->second);
        if (step_index >= inserted + kBlockedTtlSteps) {
            impl_->blocked_cells.erase(it->first);
            it = impl_->blocked_since.erase(it);
        } else {
            ++it;
        }
    }
}

bool MappingAlgorithmImpl_207190406_209543255::replan(const types::DroneState& state,
                                                      bool ignore_blocked) {
    const detail::NbvInputs inputs{
        output_map_, state, lidar_config_, drone_config_,
        remainingSteps(state), impl_->blocked_cells, ignore_blocked,
    };
    impl_->plan = impl_->planner.plan(inputs);
    impl_->waypoint_index = 0;
    impl_->terminal_scan_index = 0;
    impl_->steps_since_replan = 0;
    impl_->has_plan = impl_->plan.valid;
    return impl_->has_plan;
}

types::DroneState MappingAlgorithmImpl_207190406_209543255::predictPose(
    const types::DroneState& state, const types::MovementCommand& movement) const {
    types::DroneState next = state;
    switch (movement.type) {
    case types::MovementCommandType::Hover:
        break;
    case types::MovementCommandType::Rotate: {
        const double delta = movement.angle.force_numerical_value_in(deg) *
                             ((movement.rotation == types::RotationDirection::Left) ? 1.0 : -1.0);
        next.heading.horizontal =
            (state.heading.horizontal.force_numerical_value_in(deg) + delta) * deg;
        break;
    }
    case types::MovementCommandType::Advance: {
        const double dist = movement.distance.force_numerical_value_in(cm);
        const double heading_rad =
            state.heading.horizontal.force_numerical_value_in(deg) * (std::numbers::pi / 180.0);
        next.position = Position3D{
            (state.position.x.force_numerical_value_in(cm) + dist * std::cos(heading_rad)) *
                common::x_extent[cm],
            (state.position.y.force_numerical_value_in(cm) + dist * std::sin(heading_rad)) *
                common::y_extent[cm],
            state.position.z,
        };
        break;
    }
    case types::MovementCommandType::Elevate:
        next.position = Position3D{
            state.position.x, state.position.y,
            (state.position.z.force_numerical_value_in(cm) +
             movement.distance.force_numerical_value_in(cm)) *
                common::z_extent[cm],
        };
        break;
    }
    return next;
}

std::optional<Orientation> MappingAlgorithmImpl_207190406_209543255::bestTravelScan(
    const types::DroneState& predicted) const {
    namespace lc = user_common_207190406_209543255::lidar_cone;
    // Probe only the axis-aligned directions (always the first six emitted) so a
    // per-step scan choice stays cheap; the viewpoint's scans were scored over the
    // full direction set at plan time.
    const std::vector<Orientation> world = detail::NbvPlanner::scanDirections(lidar_config_);
    const std::size_t probes = std::min(kTravelScanProbes, world.size());

    std::optional<Orientation> best;
    std::size_t best_gain = 0;
    for (std::size_t i = 0; i < probes; ++i) {
        std::unordered_set<std::int64_t> seen;
        const std::size_t gain = lc::countUnresolvedVoxels(
            output_map_, predicted.position, Orientation{}, world[i], lidar_config_, seen);
        if (gain > best_gain) {
            best_gain = gain;
            best = world[i];
        }
    }
    if (!best.has_value()) {
        return std::nullopt;
    }
    // Emit relative to the predicted heading: MissionControl scans after moving.
    return Orientation{best->horizontal - predicted.heading.horizontal,
                       best->altitude - predicted.heading.altitude};
}

types::MappingStepCommand MappingAlgorithmImpl_207190406_209543255::nextStep(
    const types::DroneState& state, const types::LidarScanResult* latest_scan) {
    [[maybe_unused]] const types::LidarScanResult* unused_scan = latest_scan;

    if (impl_->finished) {
        types::MappingStepCommand cmd{};
        cmd.status = types::AlgorithmStatus::Finished;
        return cmd;
    }

    ensurePlanningReady();
    pruneExpiredBlockedCells(state.step_index);

    // Stall detection: the waypoint we are driving at is not reachable in practice.
    if (impl_->has_plan && impl_->waypoint_index < impl_->plan.waypoints.size() &&
        impl_->has_last_position && samePosition(impl_->last_position, state.position)) {
        if (++impl_->moving_stall_ticks >= kMaxMovingStallTicks) {
            const auto key = detail::quantizePosition(
                impl_->plan.waypoints[impl_->waypoint_index], output_map_.getMapConfig());
            impl_->blocked_cells.insert(key);
            impl_->blocked_since[key] = static_cast<int>(state.step_index);
            impl_->moving_stall_ticks = 0;
            impl_->has_plan = false;
        }
    } else {
        impl_->moving_stall_ticks = 0;
    }
    impl_->last_position = state.position;
    impl_->has_last_position = true;

    // Advance past waypoints already reached.
    while (impl_->has_plan && impl_->waypoint_index < impl_->plan.waypoints.size() &&
           reachedWaypoint(state, impl_->plan.waypoints[impl_->waypoint_index])) {
        ++impl_->waypoint_index;
    }

    const bool plan_exhausted =
        !impl_->has_plan ||
        (impl_->waypoint_index >= impl_->plan.waypoints.size() &&
         impl_->terminal_scan_index >= impl_->plan.terminal_scans.size());
    const bool interval_elapsed = impl_->steps_since_replan >= kReplanIntervalSteps;

    if (plan_exhausted || interval_elapsed) {
        if (!replan(state, false)) {
            // Nothing feasible with the blocked set honoured: try recovery, then decide.
            if (impl_->recovery_attempts < kRecoveryAttempts && replan(state, true)) {
                ++impl_->recovery_attempts;
            } else {
                impl_->finished = true;
                types::MappingStepCommand cmd{};
                cmd.status = detail::hasAnyNotMappedInBounds(output_map_)
                                 ? types::AlgorithmStatus::FinishedWithUnmappableVoxels
                                 : types::AlgorithmStatus::Finished;
                return cmd;
            }
        } else {
            impl_->recovery_attempts = 0;
        }
    }

    ++impl_->steps_since_replan;

    types::MappingStepCommand cmd{};
    cmd.status = types::AlgorithmStatus::Working;

    if (impl_->waypoint_index < impl_->plan.waypoints.size()) {
        const Position3D& target = impl_->plan.waypoints[impl_->waypoint_index];
        cmd.movement = movementToward(state, target);
        if (cmd.movement.has_value()) {
            cmd.scan_orientation = bestTravelScan(predictPose(state, *cmd.movement));
        }
        return cmd;
    }

    if (impl_->terminal_scan_index < impl_->plan.terminal_scans.size()) {
        const Orientation& world = impl_->plan.terminal_scans[impl_->terminal_scan_index++];
        cmd.scan_orientation = Orientation{world.horizontal - state.heading.horizontal,
                                           world.altitude - state.heading.altitude};
        return cmd;
    }

    cmd.movement = types::MovementCommand{};  // Hover; the next call replans.
    impl_->has_plan = false;
    return cmd;
}

} // namespace algorithm_207190406_209543255


using MappingAlgorithmImpl_207190406_209543255 =
    algorithm_207190406_209543255::MappingAlgorithmImpl_207190406_209543255;
REGISTER_MAPPING_ALGORITHM(MappingAlgorithmImpl_207190406_209543255);
