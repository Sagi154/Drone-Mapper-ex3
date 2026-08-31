// MappingAlgorithmImpl.cpp
// Wavefront frontier execution: hold a plan, emit one command per nextStep.

#include <Algorithm/MappingAlgorithmImpl.h>

#include "ScanPlanning.h"
#include "WavefrontPlanner.h"

#include <user_common_207190406_209543255/ConeTemplate.h>

#include <Common/MappingAlgorithmRegistration.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <numbers>
#include <optional>
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

// TEMP PROFILING (ALGO_PROFILE=1): remove before submit. Coarse phase timers for
// diagnosing large_out per-step wall time; printed to stderr at process exit.
struct ProfileScope {
    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    double* bucket;
    explicit ProfileScope(double* b) : bucket(b) {}
    ~ProfileScope() {
        *bucket += std::chrono::duration<double, std::milli>(
                       std::chrono::steady_clock::now() - t0)
                       .count();
    }
};
struct ProfileTotals {
    double replan_ms = 0.0;
    double target_alive_ms = 0.0;
    double travel_scan_ms = 0.0;
    double arrival_sweep_ms = 0.0;
    double progress_count_ms = 0.0;
    long steps = 0;
    long replans = 0;
    long replans_plan_exhausted = 0;
    long replans_interval_elapsed = 0;
    long replans_cluster_dead = 0;
    bool enabled = std::getenv("ALGO_PROFILE") != nullptr;
    ~ProfileTotals() {
        if (!enabled) {
            return;
        }
        std::fprintf(stderr,
                      "[ALGO_PROFILE] steps=%ld replans=%ld (exhausted=%ld interval=%ld "
                      "cluster_dead=%ld) replan_ms=%.1f target_alive_ms=%.1f "
                      "travel_scan_ms=%.1f arrival_sweep_ms=%.1f progress_count_ms=%.1f\n",
                      steps, replans, replans_plan_exhausted, replans_interval_elapsed,
                      replans_cluster_dead, replan_ms, target_alive_ms, travel_scan_ms,
                      arrival_sweep_ms, progress_count_ms);
    }
};
ProfileTotals g_profile;

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
        if (!candidate.target_keys.empty() &&
            !detail::clusterStillFrontier(output_map_, candidate.target_keys)) {
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
    impl_->last_frontier = impl_->plan.frontier_cells;
    if (impl_->has_plan && impl_->plan.waypoints.empty()) {
        buildArrivalSweep(state);
        if (impl_->arrival_scans.empty()) {
            impl_->plan.expected_rate = 0.0;
        }
    }
}

void MappingAlgorithmImpl_207190406_209543255::buildArrivalSweep(
    const types::DroneState& state) {
    const auto& templates =
        impl_->templates.get(lidar_config_, output_map_.getMapConfig().resolution);
    impl_->arrival_scans = detail::buildSweepDirections(
        output_map_, state.position, lidar_config_, impl_->last_frontier,
        templates, impl_->stamp);
    if (detail::isSmallOutdoorMission(output_map_.getMapConfig()) &&
        impl_->arrival_scans.size() > 4) {
        impl_->arrival_scans.resize(4);
    }
    impl_->arrival_scan_index = 0;
}

bool MappingAlgorithmImpl_207190406_209543255::targetClusterAlive() const {
    if (impl_->plan.target_keys.empty()) {
        return true;
    }
    return detail::clusterStillFrontier(output_map_, impl_->plan.target_keys);
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

    while (impl_->has_plan && impl_->waypoint_index < impl_->plan.waypoints.size() &&
           reachedWaypoint(state, impl_->plan.waypoints[impl_->waypoint_index])) {
        ++impl_->waypoint_index;
    }

    const bool waypoints_done =
        impl_->has_plan && impl_->waypoint_index >= impl_->plan.waypoints.size();
    if (waypoints_done && impl_->arrival_scans.empty() &&
        impl_->arrival_scan_index == 0) {
        buildArrivalSweep(state);
    }

    const bool scans_done =
        impl_->arrival_scan_index >= impl_->arrival_scans.size();
    const bool plan_exhausted =
        !impl_->has_plan || (waypoints_done && scans_done);
    const bool interval_elapsed = impl_->steps_since_replan >= kReplanIntervalSteps;
    bool cluster_dead = false;
    {
        ProfileScope prof(&g_profile.target_alive_ms);
        cluster_dead = impl_->has_plan && !targetClusterAlive();
    }

    if (plan_exhausted || interval_elapsed || cluster_dead) {
        ProfileScope prof(&g_profile.replan_ms);
        ++g_profile.replans;
        if (plan_exhausted) ++g_profile.replans_plan_exhausted;
        if (interval_elapsed) ++g_profile.replans_interval_elapsed;
        if (cluster_dead) ++g_profile.replans_cluster_dead;
        // Only a plan finishing on its own is safe to serve from the queue: an
        // elapsed interval or a dead target cluster means the map or the current
        // target's assumptions may be stale, so those always force a fresh search.
        const bool can_reuse_queue = plan_exhausted && !interval_elapsed && !cluster_dead;
        const bool have = (can_reuse_queue && popPendingPlan(state)) || replan(state, false);
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
                types::MappingStepCommand cmd{};
                cmd.status = detail::hasAnyNotMappedInBounds(output_map_)
                                 ? types::AlgorithmStatus::FinishedWithUnmappableVoxels
                                 : types::AlgorithmStatus::Finished;
                return cmd;
            }
        } else {
            impl_->low_rate_replans = 0;
            impl_->recovery_attempts = 0;
        }
        impl_->arrival_scans.clear();
        impl_->arrival_scan_index = 0;
    }

    ++impl_->steps_since_replan;
    ++g_profile.steps;

    ProfileScope progress_prof(&g_profile.progress_count_ms);
    if (!impl_->has_progress_baseline) {
        impl_->unmapped_at_progress_mark = detail::countUnmappedInBounds(output_map_);
        impl_->has_progress_baseline = true;
    }
    ++impl_->progress_window_steps;
    if (impl_->progress_window_steps >= kObservedWindowSteps) {
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
            types::MappingStepCommand cmd{};
            cmd.status = detail::hasAnyNotMappedInBounds(output_map_)
                             ? types::AlgorithmStatus::FinishedWithUnmappableVoxels
                             : types::AlgorithmStatus::Finished;
            return cmd;
        }
    }

    types::MappingStepCommand cmd{};
    cmd.status = types::AlgorithmStatus::Working;

    if (impl_->waypoint_index < impl_->plan.waypoints.size()) {
        const Position3D& target = impl_->plan.waypoints[impl_->waypoint_index];
        cmd.movement = movementToward(state, target);
        if (cmd.movement.has_value()) {
            const types::DroneState predicted = predictPose(state, *cmd.movement);
            const auto& templates = impl_->templates.get(
                lidar_config_, output_map_.getMapConfig().resolution);
            ProfileScope prof(&g_profile.travel_scan_ms);
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
        ProfileScope prof(&g_profile.arrival_sweep_ms);
        buildArrivalSweep(state);
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

} // namespace algorithm_207190406_209543255


using MappingAlgorithmImpl_207190406_209543255 =
    algorithm_207190406_209543255::MappingAlgorithmImpl_207190406_209543255;
REGISTER_MAPPING_ALGORITHM(MappingAlgorithmImpl_207190406_209543255);
