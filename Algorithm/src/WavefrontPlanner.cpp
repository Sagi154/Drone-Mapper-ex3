// WavefrontPlanner.cpp — rank frontier clusters by cells per reserved step.

#include "WavefrontPlanner.h"
#include "ScanPlanning.h"

#include <user_common_207190406_209543255/LidarCone.h>
#include <user_common_207190406_209543255/LidarConstants.h>

#include <algorithm>
#include <cstdlib>

namespace algorithm_207190406_209543255::detail {

namespace lc = user_common_207190406_209543255::lidar_cone;
namespace types = common::types;
using common::PhysicalLength;
using common::Position3D;
using common::cm;
using common::x_extent;
using common::y_extent;
using common::z_extent;
using user_common_207190406_209543255::kShortRangeLidarMax;

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

[[nodiscard]] bool unmappedInColumnBelow(const common::IMap3D& map,
                                         const types::MapConfig& config,
                                         const Position3D& start,
                                         const ParentMap& parent_of,
                                         const GridKey& start_key) {
    auto z = start.z;
    GridKey cursor = start_key;
    while (true) {
        const GridKey down{cursor.qx, cursor.qy, cursor.qz - 1};
        if (!parent_of.contains(down)) {
            return false;
        }
        z -= mp_units::quantity_cast<z_extent>(config.resolution);
        if (z < config.boundaries.min_height - 1e-6 * z_extent[cm]) {
            return false;
        }
        const Position3D below{start.x, start.y, z};
        if (map.atVoxel(below) == types::VoxelOccupancy::Unmapped) {
            return true;
        }
        cursor = down;
    }
}

[[nodiscard]] bool hasHorizontalUnmapped(const common::IMap3D& map,
                                         const Position3D& start,
                                         PhysicalLength step) {
    const Position3D offsets[4] = {
        Position3D{mp_units::quantity_cast<x_extent>(step), {}, {}},
        Position3D{mp_units::quantity_cast<x_extent>(-step), {}, {}},
        Position3D{{}, mp_units::quantity_cast<y_extent>(step), {}},
        Position3D{{}, mp_units::quantity_cast<y_extent>(-step), {}},
    };
    for (const Position3D& off : offsets) {
        if (map.atVoxel(start + off) == types::VoxelOccupancy::Unmapped) {
            return true;
        }
    }
    return false;
}

[[nodiscard]] std::size_t clusterScore(const FrontierCluster& c, bool rank_volume) {
    return rank_volume ? c.volume_count : c.cell_count;
}

[[nodiscard]] std::vector<const FrontierCluster*> rankClusters(const ReachabilityResult& reach,
                                                               bool rank_volume) {
    std::vector<const FrontierCluster*> ranked;
    ranked.reserve(reach.clusters.size());
    for (const FrontierCluster& c : reach.clusters) {
        ranked.push_back(&c);
    }
    std::stable_sort(ranked.begin(), ranked.end(),
                     [&](const FrontierCluster* a, const FrontierCluster* b) {
                         const std::size_t as = clusterScore(*a, rank_volume);
                         const std::size_t bs = clusterScore(*b, rank_volume);
                         if (as != bs) {
                             return as > bs;
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
    constexpr std::size_t kRankedClusters = 8;
    if (ranked.size() > kRankedClusters) {
        ranked.resize(kRankedClusters);
    }
    return ranked;
}

[[nodiscard]] std::vector<ExplorationPlan> buildCandidatePlans(
    const WavefrontInputs& in,
    const types::MapConfig& config,
    const ReachabilityResult& reach,
    const std::vector<const FrontierCluster*>& ranked,
    bool house_volume,
    bool open_volume,
    bool rank_volume,
    std::size_t reserve,
    const MovementLimits& limits) {
    std::vector<ExplorationPlan> candidates;
    candidates.reserve(ranked.size());
    for (const FrontierCluster* cluster : ranked) {
        GridKey approach = cluster->approach_key;
        if (open_volume && approach == reach.start_key && cluster->keys.size() > 1) {
            int best_d = -1;
            for (const GridKey& key : cluster->keys) {
                const int d = std::abs(key.qx - reach.start_key.qx) +
                              std::abs(key.qy - reach.start_key.qy) +
                              std::abs(key.qz - reach.start_key.qz);
                if (d > best_d) {
                    best_d = d;
                    approach = key;
                }
            }
        }
        FrontierPathResult raw =
            reconstructPathTo(reach.parent_of, reach.start_key, approach, config);
        std::vector<Position3D> waypoints;
        if (approach == reach.start_key) {
            waypoints.clear();
            const GridKey down{reach.start_key.qx, reach.start_key.qy, reach.start_key.qz - 1};
            const auto remaining_height =
                config.boundaries.max_height - in.state.position.z;
            const auto z_min = mp_units::quantity_cast<z_extent>(in.lidar.z_min);
            const bool column_unmapped = unmappedInColumnBelow(
                in.map, config, in.state.position, reach.parent_of, reach.start_key);
            const bool near_ceiling = remaining_height <= z_min + 1e-6 * z_extent[cm];
            const bool house_layer_done =
                house_volume &&
                !hasHorizontalUnmapped(in.map, in.state.position, config.resolution);
            if ((near_ceiling || house_layer_done) && column_unmapped &&
                reach.parent_of.contains(down)) {
                const FrontierPathResult drop =
                    reconstructPathTo(reach.parent_of, reach.start_key, down, config);
                if (drop.found) {
                    waypoints = stringPullConstantAltitude(in.map, drop.path, in.drone.radius);
                }
            }
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
        const double rate = static_cast<double>(clusterScore(*cluster, rank_volume)) /
                            static_cast<double>(travel + reserve);
        ExplorationPlan candidate;
        candidate.valid = true;
        candidate.waypoints = std::move(waypoints);
        candidate.target_cluster_cells = clusterScore(*cluster, rank_volume);
        candidate.expected_rate = rate;
        candidate.target_keys = cluster->keys;
        candidate.frontier_cells = reach.frontier_cells;
        candidates.push_back(std::move(candidate));
    }
    return candidates;
}

[[nodiscard]] ExplorationPlan selectPlan(std::vector<ExplorationPlan> candidates,
                                         std::vector<ExplorationPlan>* alternates) {
    if (candidates.empty()) {
        return {};
    }
    std::stable_sort(candidates.begin(), candidates.end(),
                     [](const ExplorationPlan& a, const ExplorationPlan& b) {
                         return a.expected_rate > b.expected_rate;
                     });
    if (alternates != nullptr) {
        alternates->assign(candidates.begin() + 1, candidates.end());
    }
    return std::move(candidates.front());
}

} // namespace

ExplorationPlan WavefrontPlanner::plan(const WavefrontInputs& in,
                                       std::vector<ExplorationPlan>* alternates) const {
    if (alternates != nullptr) {
        alternates->clear();
    }
    const BlockedCells empty_blocked;
    const BlockedCells& blocked = in.ignore_blocked ? empty_blocked : in.blocked;
    const types::MapConfig config = in.map.getMapConfig();
    const MovementLimits limits = limitsFrom(in.drone);
    const std::size_t reserve = reserveFor(in.lidar);

    const ReachabilityResult reach = frontier_.exploreReachable(
        in.map, in.state.position, in.drone.radius, blocked, maxExpansionsForMap(in.map));
    if (!reach.start_passable) {
        const FrontierPathResult unstick =
            frontier_.findUnstickPath(in.map, in.state.position, in.drone.radius);
        if (!unstick.found || unstick.path.empty()) {
            return {};
        }
        ExplorationPlan escape;
        escape.valid = true;
        escape.waypoints = unstick.path;
        escape.target_cluster_cells = 1;
        escape.expected_rate = 1.0;
        return escape;
    }
    if (reach.clusters.empty()) {
        return {};
    }

    const bool house_volume = isHouseVolumeMission(config);
    const bool open_volume = isOpenVolumeMission(config);
    const bool rank_volume = open_volume && in.lidar.z_max > kShortRangeLidarMax;

    const GridKey down_key{reach.start_key.qx, reach.start_key.qy, reach.start_key.qz - 1};
    if (house_volume && in.prefer_descend && reach.parent_of.contains(down_key) &&
        unmappedInColumnBelow(in.map, config, in.state.position, reach.parent_of,
                              reach.start_key)) {
        const FrontierPathResult drop =
            reconstructPathTo(reach.parent_of, reach.start_key, down_key, config);
        if (drop.found && !drop.path.empty()) {
            ExplorationPlan forced;
            forced.valid = true;
            forced.waypoints = stringPullConstantAltitude(in.map, drop.path, in.drone.radius);
            forced.target_cluster_cells = 1;
            forced.expected_rate = 1.0;
            forced.frontier_cells = reach.frontier_cells;
            return forced;
        }
    }

    const auto ranked = rankClusters(reach, rank_volume);
    auto candidates = buildCandidatePlans(in, config, reach, ranked, house_volume, open_volume,
                                          rank_volume, reserve, limits);
    return selectPlan(std::move(candidates), alternates);
}

} // namespace algorithm_207190406_209543255::detail
