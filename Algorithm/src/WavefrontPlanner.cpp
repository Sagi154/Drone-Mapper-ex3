// WavefrontPlanner.cpp — rank frontier clusters by cells per reserved step.

#include "WavefrontPlanner.h"

#include <user_common_207190406_209543255/LidarCone.h>

#include <algorithm>

namespace algorithm_207190406_209543255::detail {

namespace lc = user_common_207190406_209543255::lidar_cone;
namespace types = common::types;

namespace {

constexpr std::size_t kMaxSweepReserve = 8;

[[nodiscard]] MovementLimits limitsFrom(const types::DroneConfigData& drone) {
    return MovementLimits{drone.max_advance, drone.max_elevate, drone.max_rotate};
}

[[nodiscard]] std::size_t reserveFor(const types::LidarConfigData& lidar) {
    const double alpha = lc::coneHalfAngleRad(lidar);
    const std::size_t n = lc::directionCountForHalfAngle(alpha);
    return std::min(n, kMaxSweepReserve);
}

} // namespace

ExplorationPlan WavefrontPlanner::plan(const WavefrontInputs& in) const {
    const BlockedCells empty_blocked;
    const BlockedCells& blocked = in.ignore_blocked ? empty_blocked : in.blocked;
    const types::MapConfig config = in.map.getMapConfig();
    const MovementLimits limits = limitsFrom(in.drone);
    const std::size_t reserve = reserveFor(in.lidar);

    const ReachabilityResult reach = frontier_.exploreReachable(
        in.map, in.state.position, in.drone.radius, blocked,
        /*stride_cells=*/3, maxExpansionsForMap(in.map));
    if (!reach.start_passable || reach.clusters.empty()) {
        return {};
    }

    std::vector<const FrontierCluster*> ranked;
    ranked.reserve(reach.clusters.size());
    for (const FrontierCluster& c : reach.clusters) {
        ranked.push_back(&c);
    }
    std::stable_sort(ranked.begin(), ranked.end(),
                     [](const FrontierCluster* a, const FrontierCluster* b) {
                         if (a->cell_count != b->cell_count) {
                             return a->cell_count > b->cell_count;
                         }
                         if (a->approach_cost != b->approach_cost) {
                             return a->approach_cost < b->approach_cost;
                         }
                         if (a->approach_key.qx != b->approach_key.qx) {
                             return a->approach_key.qx < b->approach_key.qx;
                         }
                         if (a->approach_key.qy != b->approach_key.qy) {
                             return a->approach_key.qy < b->approach_key.qy;
                         }
                         return a->approach_key.qz < b->approach_key.qz;
                     });
    if (ranked.size() > kRankedClusters) {
        ranked.resize(kRankedClusters);
    }

    ExplorationPlan best;
    double best_rate = -1.0;
    for (const FrontierCluster* cluster : ranked) {
        FrontierPathResult raw = reconstructPathTo(
            reach.parent_of, reach.start_key, cluster->approach_key, config);
        std::vector<common::Position3D> waypoints;
        if (cluster->approach_key == reach.start_key) {
            waypoints.clear();
        } else {
            if (!raw.found) {
                continue;
            }
            waypoints = stringPullConstantAltitude(in.map, raw.path, in.drone.radius);
        }
        const std::size_t travel =
            waypoints.empty()
                ? 0
                : stepCostForPath(waypoints, in.state.position, in.state.heading, limits);
        if (travel + reserve > in.remaining_steps) {
            continue;
        }
        const double rate = static_cast<double>(cluster->cell_count) /
                            static_cast<double>(travel + reserve);
        if (rate > best_rate) {
            best_rate = rate;
            best.valid = true;
            best.waypoints = std::move(waypoints);
            best.terminal_scans.clear();
            best.expected_gain = 0.0;
            best.target_cluster_cells = cluster->cell_count;
            best.expected_rate = rate;
            best.target_keys = cluster->keys;
            best.frontier_cells = reach.frontier_cells;
        }
    }
    return best;
}

} // namespace algorithm_207190406_209543255::detail
