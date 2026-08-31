// WavefrontPlanner.cpp — rank frontier clusters by cells per reserved step.

#include "WavefrontPlanner.h"
#include "ScanPlanning.h"

#include <user_common_207190406_209543255/LidarCone.h>

#include <algorithm>
#include <cstdlib>

namespace algorithm_207190406_209543255::detail {

namespace lc = user_common_207190406_209543255::lidar_cone;
namespace types = common::types;
using common::cm;
using common::x_extent;
using common::y_extent;
using common::z_extent;

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
                                         const common::Position3D& start,
                                         const ParentMap& parent_of,
                                         const GridKey& start_key) {
    const double step = config.resolution.force_numerical_value_in(cm);
    const double min_z = config.boundaries.min_height.force_numerical_value_in(cm);
    double z = start.z.force_numerical_value_in(cm);
    GridKey cursor = start_key;
    while (true) {
        const GridKey down{cursor.qx, cursor.qy, cursor.qz - 1};
        if (!parent_of.contains(down)) {
            return false;
        }
        z -= step;
        if (z < min_z - 1e-6) {
            return false;
        }
        const common::Position3D below{start.x, start.y, z * z_extent[cm]};
        if (map.atVoxel(below) == types::VoxelOccupancy::Unmapped) {
            return true;
        }
        cursor = down;
    }
}

[[nodiscard]] bool hasHorizontalUnmapped(const common::IMap3D& map,
                                         const common::Position3D& start,
                                         double step_cm) {
    const double x = start.x.force_numerical_value_in(cm);
    const double y = start.y.force_numerical_value_in(cm);
    const double z = start.z.force_numerical_value_in(cm);
    const double dx[4] = {step_cm, -step_cm, 0.0, 0.0};
    const double dy[4] = {0.0, 0.0, step_cm, -step_cm};
    for (int i = 0; i < 4; ++i) {
        const common::Position3D nb{(x + dx[i]) * x_extent[cm], (y + dy[i]) * y_extent[cm],
                                    z * z_extent[cm]};
        if (map.atVoxel(nb) == types::VoxelOccupancy::Unmapped) {
            return true;
        }
    }
    return false;
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
        maxExpansionsForMap(in.map));
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
    const bool rank_volume =
        open_volume && in.lidar.z_max.force_numerical_value_in(cm) > 90.0;
    auto cluster_score = [rank_volume](const FrontierCluster* c) {
        return rank_volume ? c->volume_count : c->cell_count;
    };

    const GridKey down_key{reach.start_key.qx, reach.start_key.qy, reach.start_key.qz - 1};
    if (house_volume && in.prefer_descend && reach.parent_of.contains(down_key) &&
        unmappedInColumnBelow(in.map, config, in.state.position, reach.parent_of,
                              reach.start_key)) {
        const FrontierPathResult drop =
            reconstructPathTo(reach.parent_of, reach.start_key, down_key, config);
        if (drop.found && !drop.path.empty()) {
            ExplorationPlan forced;
            forced.valid = true;
            forced.waypoints =
                stringPullConstantAltitude(in.map, drop.path, in.drone.radius);
            forced.target_cluster_cells = 1;
            forced.expected_rate = 1.0;
            forced.frontier_cells = reach.frontier_cells;
            return forced;
        }
    }

    std::vector<const FrontierCluster*> ranked;
    ranked.reserve(reach.clusters.size());
    for (const FrontierCluster& c : reach.clusters) {
        ranked.push_back(&c);
    }
    std::stable_sort(ranked.begin(), ranked.end(),
                     [&](const FrontierCluster* a, const FrontierCluster* b) {
                         const std::size_t as = cluster_score(a);
                         const std::size_t bs = cluster_score(b);
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
    if (ranked.size() > kRankedClusters) {
        ranked.resize(kRankedClusters);
    }

    ExplorationPlan best;
    double best_rate = -1.0;
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
        FrontierPathResult raw = reconstructPathTo(
            reach.parent_of, reach.start_key, approach, config);
        std::vector<common::Position3D> waypoints;
        if (approach == reach.start_key) {
            waypoints.clear();
            const GridKey down{reach.start_key.qx, reach.start_key.qy,
                               reach.start_key.qz - 1};
            const double z = in.state.position.z.force_numerical_value_in(cm);
            const double max_z = config.boundaries.max_height.force_numerical_value_in(cm);
            const double z_min = in.lidar.z_min.force_numerical_value_in(cm);
            const double step = config.resolution.force_numerical_value_in(cm);
            const bool column_unmapped = unmappedInColumnBelow(
                in.map, config, in.state.position, reach.parent_of, reach.start_key);
            const bool near_ceiling = max_z - z <= z_min + 1e-6;
            const bool house_layer_done =
                house_volume && !hasHorizontalUnmapped(in.map, in.state.position, step);
            if ((near_ceiling || house_layer_done) && column_unmapped &&
                reach.parent_of.contains(down)) {
                const FrontierPathResult drop =
                    reconstructPathTo(reach.parent_of, reach.start_key, down, config);
                if (drop.found) {
                    waypoints = stringPullConstantAltitude(
                        in.map, drop.path, in.drone.radius);
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
        const double rate = static_cast<double>(cluster_score(cluster)) /
                            static_cast<double>(travel + reserve);
        if (rate > best_rate) {
            best_rate = rate;
            best.valid = true;
            best.waypoints = std::move(waypoints);
            best.target_cluster_cells = cluster_score(cluster);
            best.expected_rate = rate;
            best.target_keys = cluster->keys;
            best.frontier_cells = reach.frontier_cells;
        }
    }
    return best;
}

} // namespace algorithm_207190406_209543255::detail
