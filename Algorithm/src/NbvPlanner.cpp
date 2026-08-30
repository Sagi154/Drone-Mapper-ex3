// NbvPlanner.cpp — frontier-anchored next-best-view policy.

#include "NbvPlanner.h"

#include <user_common_207190406_209543255/LidarCone.h>

#include <algorithm>
#include <cstdint>
#include <unordered_set>

namespace algorithm_207190406_209543255::detail {

namespace lc = user_common_207190406_209543255::lidar_cone;
namespace types = common::types;

using common::Orientation;
using common::Position3D;
using common::cm;

namespace {

struct ScoredCandidate {
    const ReachableCell* cell = nullptr;
    double prefilter = 0.0;
};

[[nodiscard]] MovementLimits limitsFrom(const types::DroneConfigData& drone) {
    return MovementLimits{drone.max_advance, drone.max_elevate, drone.max_rotate};
}

/// Unique Unmapped voxels of `dirs` in listed order, with a fresh `seen` set.
/// Do not reuse per-direction `added` from `gainAt`: those were counted in
/// Fibonacci order, then the list was sorted.
[[nodiscard]] double uniqueGainOf(const common::IMap3D& map,
                                  const Position3D& origin,
                                  const types::LidarConfigData& lidar,
                                  const std::vector<Orientation>& dirs) {
    const Orientation world_heading{};
    std::unordered_set<std::int64_t> seen;
    double total = 0.0;
    for (const Orientation& dir : dirs) {
        total += static_cast<double>(
            lc::countUnresolvedVoxels(map, origin, world_heading, dir, lidar, seen));
    }
    return total;
}

} // namespace

std::vector<Orientation> NbvPlanner::scanDirections(const types::LidarConfigData& lidar) {
    const double alpha = lc::coneHalfAngleRad(lidar);
    if (!(alpha > 0.0)) {
        return {};
    }
    return lc::fibonacciSphereOrientations(lc::directionCountForHalfAngle(alpha));
}

double NbvPlanner::gainAt(const common::IMap3D& map,
                          const Position3D& origin,
                          const types::LidarConfigData& lidar,
                          std::vector<Orientation>* directions_out) {
    const std::vector<Orientation> directions = scanDirections(lidar);
    // Directions are world-frame, so pass a zero heading and treat them as relative.
    const Orientation world_heading{};

    std::vector<std::pair<double, Orientation>> per_direction;
    per_direction.reserve(directions.size());

    std::unordered_set<std::int64_t> seen;
    double total = 0.0;
    for (const Orientation& dir : directions) {
        const std::size_t added =
            lc::countUnresolvedVoxels(map, origin, world_heading, dir, lidar, seen);
        if (added > 0) {
            total += static_cast<double>(added);
            per_direction.emplace_back(static_cast<double>(added), dir);
        }
    }

    if (directions_out != nullptr) {
        // Stable sort on gain descending keeps the order reproducible for equal gains.
        std::stable_sort(per_direction.begin(), per_direction.end(),
                         [](const auto& a, const auto& b) { return a.first > b.first; });
        directions_out->clear();
        directions_out->reserve(per_direction.size());
        for (const auto& [gain, dir] : per_direction) {
            directions_out->push_back(dir);
        }
    }

    return total;
}

ExplorationPlan NbvPlanner::plan(const NbvInputs& in) const {
    const types::MapConfig config = in.map.getMapConfig();
    const MovementLimits limits = limitsFrom(in.drone);
    const BlockedCells empty_blocked;
    const BlockedCells& blocked = in.ignore_blocked ? empty_blocked : in.blocked;

    ExplorationPlan best;

    // Candidate 0: stay where we are. Costs only the scans it performs.
    {
        std::vector<Orientation> here_dirs;
        (void)gainAt(in.map, in.state.position, in.lidar, &here_dirs);
        const std::size_t scan_budget = in.remaining_steps;
        if (scan_budget > 0 && !here_dirs.empty()) {
            const std::size_t scan_steps = std::min(here_dirs.size(), scan_budget);
            std::vector<Orientation> prefix(
                here_dirs.begin(),
                here_dirs.begin() + static_cast<std::ptrdiff_t>(scan_steps));
            const double prefix_gain =
                uniqueGainOf(in.map, in.state.position, in.lidar, prefix);
            if (prefix_gain > 0.0) {
                best.valid = true;
                best.waypoints.clear();
                best.terminal_scans = std::move(prefix);
                best.expected_gain = prefix_gain;
            }
        }
    }
    double best_utility = best.valid ? best.expected_gain /
                                           static_cast<double>(best.terminal_scans.size())
                                     : 0.0;

    const ReachabilityResult reach = frontier_.exploreReachable(
        in.map, in.state.position, in.drone.radius, blocked, kCandidateStrideCells,
        maxExpansionsForMap(in.map));
    if (!reach.start_passable || reach.candidates.empty()) {
        return best;
    }

    // Cheap prefilter: unmapped neighbours per unit path cost, constant work per cell.
    std::vector<ScoredCandidate> prefiltered;
    prefiltered.reserve(reach.candidates.size());
    for (const ReachableCell& cell : reach.candidates) {
        const double cost = std::max(1, cell.cost);
        prefiltered.push_back({&cell, static_cast<double>(cell.unmapped_neighbours) / cost});
    }
    std::stable_sort(prefiltered.begin(), prefiltered.end(),
                     [](const ScoredCandidate& a, const ScoredCandidate& b) {
                         if (a.prefilter != b.prefilter) {
                             return a.prefilter > b.prefilter;
                         }
                         return a.cell->cost < b.cell->cost;  // deterministic tiebreak
                     });
    if (prefiltered.size() > kScoredCandidates) {
        prefiltered.resize(kScoredCandidates);
    }

    for (const ScoredCandidate& scored : prefiltered) {
        const ReachableCell& cell = *scored.cell;

        FrontierPathResult raw =
            reconstructPathTo(reach.parent_of, reach.start_key, cell.key, config);
        if (!raw.found || raw.path.empty()) {
            continue;
        }
        std::vector<Position3D> waypoints =
            stringPullConstantAltitude(in.map, raw.path, in.drone.radius);

        std::vector<Orientation> dirs;
        const double gain = gainAt(in.map, cell.position, in.lidar, &dirs);
        if (!(gain > 0.0) || dirs.empty()) {
            continue;
        }

        const std::size_t travel =
            stepCostForPath(waypoints, in.state.position, in.state.heading, limits);
        if (travel > in.remaining_steps) {
            continue;  // budget feasibility filter
        }
        const std::size_t scan_budget = in.remaining_steps - travel;
        if (scan_budget == 0) {
            continue;
        }

        const std::size_t scan_steps = std::min(dirs.size(), scan_budget);
        std::vector<Orientation> prefix(
            dirs.begin(), dirs.begin() + static_cast<std::ptrdiff_t>(scan_steps));
        const double prefix_gain = uniqueGainOf(in.map, cell.position, in.lidar, prefix);
        if (!(prefix_gain > 0.0)) {
            continue;
        }

        const double utility = prefix_gain / static_cast<double>(travel + prefix.size());
        if (utility > best_utility) {
            best_utility = utility;
            best.valid = true;
            best.waypoints = std::move(waypoints);
            best.terminal_scans = std::move(prefix);
            best.expected_gain = prefix_gain;
        }
    }

    return best;
}

} // namespace algorithm_207190406_209543255::detail
