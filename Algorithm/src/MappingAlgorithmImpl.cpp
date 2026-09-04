// MappingAlgorithmImpl.cpp
// Wavefront frontier execution: hold a plan, emit one command per nextStep.

#include <Algorithm/MappingAlgorithmImpl.h>

#include "ScanPlanning.h"
#include "WavefrontPlanner.h"

#include <user_common_207190406_209543255/ConeTemplate.h>

#include <Common/MappingAlgorithmRegistration.h>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <optional>
#include <vector>

namespace algorithm_207190406_209543255 {

namespace types = common::types;
using common::Orientation;
using common::Position3D;
using common::cm;
using common::deg;
using common::x_extent;
using common::y_extent;
using common::z_extent;

namespace {

constexpr double kHalfStepTolerance = 0.5;
constexpr std::size_t kMaxArrivalScans = 4;
constexpr double kPositionEpsilon = 1e-6;

} // namespace

struct MappingAlgorithmImpl_207190406_209543255::Impl {
    detail::WavefrontPlanner planner{};
    detail::ExplorationPlan plan{};
    detail::FrontierCells last_frontier{};
    user_common_207190406_209543255::cone_template::ConeTemplateCache templates{};
    user_common_207190406_209543255::cone_template::VoxelStamp stamp{};
    std::vector<common::Orientation> arrival_scans{};
    std::size_t arrival_scan_index = 0;
    std::size_t waypoint_index = 0;
    std::size_t steps_since_replan = 0;
    bool has_plan = false;
    int moving_stall_ticks = 0;
    Position3D last_position{};
    bool has_last_position = false;
    detail::BlockedCells blocked_cells{};
    detail::GridIntMap blocked_since{};
    int recovery_attempts = 0;
    int low_rate_replans = 0;
    std::size_t unmapped_at_progress_mark = 0;
    std::size_t progress_window_steps = 0;
    int low_observed_windows = 0;
    bool has_progress_baseline = false;
    std::vector<detail::ExplorationPlan> pending_plans{};
    bool finished = false;
};

MappingAlgorithmImpl_207190406_209543255::~MappingAlgorithmImpl_207190406_209543255() = default;

MappingAlgorithmImpl_207190406_209543255::MappingAlgorithmImpl_207190406_209543255(
    common::MappingAlgorithmDependencies dependencies)
    : common::IMappingAlgorithm(dependencies), impl_(std::make_unique<Impl>()) {}

bool MappingAlgorithmImpl_207190406_209543255::samePosition(const Position3D& a,
                                                           const Position3D& b) const {
    const double dx = std::abs(a.x.force_numerical_value_in(cm) - b.x.force_numerical_value_in(cm));
    const double dy = std::abs(a.y.force_numerical_value_in(cm) - b.y.force_numerical_value_in(cm));
    const double dz = std::abs(a.z.force_numerical_value_in(cm) - b.z.force_numerical_value_in(cm));
    return dx <= kPositionEpsilon && dy <= kPositionEpsilon && dz <= kPositionEpsilon;
}

bool MappingAlgorithmImpl_207190406_209543255::reachedWaypoint(
    const types::DroneState& state,
    const Position3D& target,
    const types::MapConfig& map_config) const {
    const double step = map_config.resolution.force_numerical_value_in(cm);
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
    const bool prev_stay = impl_->has_plan && impl_->plan.waypoints.empty();
    const detail::WavefrontInputs inputs{
        output_map_, state, lidar_config_, drone_config_,
        remainingSteps(state), impl_->blocked_cells, ignore_blocked, prev_stay,
    };
    impl_->pending_plans.clear();
    adoptPlan(impl_->planner.plan(inputs, &impl_->pending_plans), state);
    return impl_->has_plan;
}

bool MappingAlgorithmImpl_207190406_209543255::popPendingPlan(const types::DroneState& state) {
    while (!impl_->pending_plans.empty()) {
        detail::ExplorationPlan candidate = std::move(impl_->pending_plans.front());
        impl_->pending_plans.erase(impl_->pending_plans.begin());
        if (!candidate.internals.target_keys.empty() &&
            !detail::clusterStillFrontier(output_map_, candidate.internals.target_keys)) {
            continue;
        }
        adoptPlan(std::move(candidate), state);
        return true;
    }
    return false;
}

void MappingAlgorithmImpl_207190406_209543255::adoptPlan(detail::ExplorationPlan plan,
                                                         const types::DroneState& state) {
    impl_->plan = std::move(plan);
    impl_->waypoint_index = 0;
    impl_->arrival_scans.clear();
    impl_->arrival_scan_index = 0;
    impl_->steps_since_replan = 0;
    impl_->has_plan = impl_->plan.valid;
    impl_->last_frontier = impl_->plan.internals.frontier_cells;
    if (impl_->has_plan && impl_->plan.waypoints.empty()) {
        buildArrivalSweep(state, output_map_.getMapConfig());
        if (impl_->arrival_scans.empty()) {
            impl_->plan.expected_rate = 0.0;
        }
    }
}

void MappingAlgorithmImpl_207190406_209543255::buildArrivalSweep(
    const types::DroneState& state, const types::MapConfig& map_config) {
    const auto& templates = impl_->templates.get(lidar_config_, map_config.resolution);
    impl_->arrival_scans = detail::buildSweepDirections(
        output_map_, state.position, lidar_config_, impl_->last_frontier, templates,
        impl_->stamp);
    if (detail::isSmallOutdoorMission(map_config) &&
        impl_->arrival_scans.size() > kMaxArrivalScans) {
        impl_->arrival_scans.resize(kMaxArrivalScans);
    }
    impl_->arrival_scan_index = 0;
}

bool MappingAlgorithmImpl_207190406_209543255::targetClusterAlive() const {
    if (impl_->plan.internals.target_keys.empty()) {
        return true;
    }
    return detail::clusterStillFrontier(output_map_, impl_->plan.internals.target_keys);
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

types::MappingStepCommand MappingAlgorithmImpl_207190406_209543255::finishIfUnmapped(
    std::optional<std::size_t> known_unmapped) const {
    types::MappingStepCommand cmd{};
    const bool any_unmapped = known_unmapped.has_value()
                                  ? *known_unmapped > 0
                                  : detail::hasAnyNotMappedInBounds(output_map_);
    cmd.status = any_unmapped ? types::AlgorithmStatus::FinishedWithUnmappableVoxels
                              : types::AlgorithmStatus::Finished;
    return cmd;
}

bool MappingAlgorithmImpl_207190406_209543255::handleReplan(const types::DroneState& state,
                                                            bool plan_exhausted,
                                                            bool interval_elapsed,
                                                            bool cluster_dead) {
    if (!(plan_exhausted || interval_elapsed || cluster_dead)) {
        return false;
    }
    const bool can_reuse_queue = plan_exhausted && !interval_elapsed && !cluster_dead;
    const bool reused_queue = can_reuse_queue && popPendingPlan(state);
    const bool have = reused_queue || replan(state, false);
    if (reused_queue) {
        impl_->low_rate_replans = 0;
        impl_->recovery_attempts = 0;
    } else {
        const bool low = !have || impl_->plan.expected_rate < kMinInformationRate;
        if (low) {
            ++impl_->low_rate_replans;
            if (!have && impl_->recovery_attempts < kRecoveryAttempts &&
                replan(state, true)) {
                ++impl_->recovery_attempts;
                if (impl_->plan.expected_rate >= kMinInformationRate) {
                    impl_->low_rate_replans = 0;
                    impl_->recovery_attempts = 0;
                }
            } else if (impl_->low_rate_replans >= kLowRateReplans) {
                impl_->finished = true;
                return true;
            }
        } else {
            impl_->low_rate_replans = 0;
            impl_->recovery_attempts = 0;
        }
    }
    impl_->arrival_scans.clear();
    impl_->arrival_scan_index = 0;
    return false;
}

bool MappingAlgorithmImpl_207190406_209543255::updateProgressWindow() {
    if (!impl_->has_progress_baseline) {
        impl_->unmapped_at_progress_mark = detail::countUnmappedInBounds(output_map_);
        impl_->has_progress_baseline = true;
    }
    ++impl_->progress_window_steps;
    if (impl_->progress_window_steps < kObservedWindowSteps) {
        return false;
    }
    const std::size_t unmapped_now = detail::countUnmappedInBounds(output_map_);
    const std::size_t prev = impl_->unmapped_at_progress_mark;
    const std::size_t gained = (prev > unmapped_now) ? (prev - unmapped_now) : 0;
    const double observed_rate = static_cast<double>(gained) /
                                 static_cast<double>(impl_->progress_window_steps);
    if (observed_rate < kMinObservedInformationRate) {
        ++impl_->low_observed_windows;
    } else {
        impl_->low_observed_windows = 0;
    }
    impl_->unmapped_at_progress_mark = unmapped_now;
    impl_->progress_window_steps = 0;
    if (impl_->low_observed_windows >= kLowObservedWindows) {
        impl_->finished = true;
        return true;
    }
    return false;
}

types::MappingStepCommand MappingAlgorithmImpl_207190406_209543255::emitMovementOrScan(
    const types::DroneState& state, const types::MapConfig& map_config) {
    types::MappingStepCommand cmd{};
    cmd.status = types::AlgorithmStatus::Working;

    if (impl_->waypoint_index < impl_->plan.waypoints.size()) {
        const Position3D& target = impl_->plan.waypoints[impl_->waypoint_index];
        cmd.movement = movementToward(state, target);
        if (cmd.movement.has_value()) {
            const types::DroneState predicted = predictPose(state, *cmd.movement);
            const auto& templates = impl_->templates.get(lidar_config_, map_config.resolution);
            const std::optional<Orientation> world = detail::bestTravelScan(
                output_map_, predicted, target, lidar_config_, impl_->last_frontier,
                templates, impl_->stamp);
            if (world.has_value()) {
                cmd.scan_orientation =
                    Orientation{world->horizontal - predicted.heading.horizontal,
                                world->altitude - predicted.heading.altitude};
            }
        }
        return cmd;
    }

    if (impl_->arrival_scans.empty()) {
        buildArrivalSweep(state, map_config);
    }
    if (impl_->arrival_scan_index < impl_->arrival_scans.size()) {
        const Orientation& world = impl_->arrival_scans[impl_->arrival_scan_index++];
        cmd.scan_orientation = Orientation{world.horizontal - state.heading.horizontal,
                                           world.altitude - state.heading.altitude};
        return cmd;
    }

    impl_->has_plan = false;
    cmd.movement = types::MovementCommand{};
    return cmd;
}

types::MappingStepCommand MappingAlgorithmImpl_207190406_209543255::nextStep(
    const types::DroneState& state, const types::LidarScanResult* latest_scan) {
    [[maybe_unused]] const types::LidarScanResult* unused_scan = latest_scan;

    if (impl_->finished) {
        types::MappingStepCommand cmd{};
        cmd.status = types::AlgorithmStatus::Finished;
        return cmd;
    }

    const types::MapConfig map_config = output_map_.getMapConfig();
    pruneExpiredBlockedCells(state.step_index);

    if (impl_->has_plan && impl_->waypoint_index < impl_->plan.waypoints.size() &&
        impl_->has_last_position && samePosition(impl_->last_position, state.position)) {
        if (++impl_->moving_stall_ticks >= kMaxMovingStallTicks) {
            const auto key = detail::quantizePosition(
                impl_->plan.waypoints[impl_->waypoint_index], map_config);
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

    while (impl_->has_plan && impl_->waypoint_index < impl_->plan.waypoints.size() &&
           reachedWaypoint(state, impl_->plan.waypoints[impl_->waypoint_index], map_config)) {
        ++impl_->waypoint_index;
    }

    const bool waypoints_done =
        impl_->has_plan && impl_->waypoint_index >= impl_->plan.waypoints.size();
    if (waypoints_done && impl_->arrival_scans.empty() && impl_->arrival_scan_index == 0) {
        buildArrivalSweep(state, map_config);
    }

    const bool scans_done = impl_->arrival_scan_index >= impl_->arrival_scans.size();
    const bool plan_exhausted = !impl_->has_plan || (waypoints_done && scans_done);
    const bool interval_elapsed = impl_->steps_since_replan >= kReplanIntervalSteps;
    const bool cluster_dead = impl_->has_plan && !targetClusterAlive();

    if (handleReplan(state, plan_exhausted, interval_elapsed, cluster_dead)) {
        return finishIfUnmapped();
    }

    ++impl_->steps_since_replan;

    if (updateProgressWindow()) {
        return finishIfUnmapped(impl_->unmapped_at_progress_mark);
    }

    return emitMovementOrScan(state, map_config);
}

} // namespace algorithm_207190406_209543255


using MappingAlgorithmImpl_207190406_209543255 =
    algorithm_207190406_209543255::MappingAlgorithmImpl_207190406_209543255;
REGISTER_MAPPING_ALGORITHM(MappingAlgorithmImpl_207190406_209543255);
