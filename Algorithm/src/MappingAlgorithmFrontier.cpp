// MappingAlgorithmFrontier.cpp
// BFS frontier search adapted from ex1 ExplorationFrontier for ex2 IMap3D.

#include "MappingAlgorithmFrontier.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>

namespace algorithm_207190406_209543255::detail {

namespace types = common::types;
using common::IMap3D;
using common::PhysicalLength;
using common::Position3D;
using common::cm;
using common::x_extent;
using common::y_extent;
using common::z_extent;

namespace {

constexpr int kInf = std::numeric_limits<int>::max();

struct Offset {
    int dx;
    int dy;
    int dz;
};

constexpr Offset kOffsets[6] = {
    {1, 0, 0},  {-1, 0, 0}, {0, 1, 0},
    {0, -1, 0}, {0, 0, 1},  {0, 0, -1},
};

[[nodiscard]] double gridStepCm(const types::MapConfig& config) {
    return config.resolution.force_numerical_value_in(cm);
}

[[nodiscard]] Position3D keyToPoint(const GridKey& key, const types::MapConfig& config) {
    const double step = gridStepCm(config);
    const double ox = config.offset.x.force_numerical_value_in(cm);
    const double oy = config.offset.y.force_numerical_value_in(cm);
    const double oz = config.offset.z.force_numerical_value_in(cm);
    return Position3D{
        (ox + static_cast<double>(key.qx) * step) * x_extent[cm],
        (oy + static_cast<double>(key.qy) * step) * y_extent[cm],
        (oz + static_cast<double>(key.qz) * step) * z_extent[cm],
    };
}

[[nodiscard]] types::VoxelOccupancy occupancyAt(const IMap3D& map, const Position3D& pos) {
    if (!map.isInBounds(pos)) {
        return types::VoxelOccupancy::OutOfBounds;
    }
    return map.atVoxel(pos);
}

[[nodiscard]] bool isBlockedCell(const BlockedCells& blocked,
                                 const GridKey& key) {
    return blocked.find(key) != blocked.end();
}

[[nodiscard]] bool isSpherePassable(const IMap3D& map,
                                    const Position3D& centre,
                                    double radius_cm,
                                    double step_cm,
                                    const BlockedCells& blocked) {
    // Treat Unmapped as passable: only confirmed Occupied or OutOfBounds blocks navigation.
    // This allows the planner to route through unexplored territory; stall detection handles
    // the case where an Unmapped cell turns out to be solid at execution time.
    const types::VoxelOccupancy centre_occ = occupancyAt(map, centre);
    if (centre_occ == types::VoxelOccupancy::Occupied ||
        centre_occ == types::VoxelOccupancy::OutOfBounds) {
        return false;
    }

    const types::MapConfig config = map.getMapConfig();
    const GridKey centre_key = quantizePosition(centre, config);
    if (isBlockedCell(blocked, centre_key)) {
        return false;
    }

    const double cx = centre.x.force_numerical_value_in(cm);
    const double cy = centre.y.force_numerical_value_in(cm);
    const double cz = centre.z.force_numerical_value_in(cm);

    const int rx = static_cast<int>(std::ceil(radius_cm / step_cm));
    const int rh = static_cast<int>(std::ceil(radius_cm / step_cm));

    for (int dx = -rx; dx <= rx; ++dx) {
        for (int dy = -rx; dy <= rx; ++dy) {
            for (int dz = -rh; dz <= rh; ++dz) {
                if (dx == 0 && dy == 0 && dz == 0) {
                    continue;
                }
                const double ox = dx * step_cm;
                const double oy = dy * step_cm;
                const double oz = dz * step_cm;
                if (ox * ox + oy * oy + oz * oz > radius_cm * radius_cm) {
                    continue;
                }

                const Position3D probe{
                    (cx + ox) * x_extent[cm],
                    (cy + oy) * y_extent[cm],
                    (cz + oz) * z_extent[cm],
                };
                const types::VoxelOccupancy probe_occ = occupancyAt(map, probe);
                if (probe_occ == types::VoxelOccupancy::Occupied ||
                    probe_occ == types::VoxelOccupancy::OutOfBounds) {
                    return false;
                }
            }
        }
    }
    return true;
}

[[nodiscard]] bool sphereContainsNotMapped(const IMap3D& map,
                                           const Position3D& centre,
                                           double radius_cm,
                                           double step_cm) {
    const double cx = centre.x.force_numerical_value_in(cm);
    const double cy = centre.y.force_numerical_value_in(cm);
    const double cz = centre.z.force_numerical_value_in(cm);

    const int rx = static_cast<int>(std::ceil(radius_cm / step_cm));
    const int rh = static_cast<int>(std::ceil(radius_cm / step_cm));
    const double r2 = radius_cm * radius_cm;

    for (int dx = -rx; dx <= rx; ++dx) {
        for (int dy = -rx; dy <= rx; ++dy) {
            for (int dz = -rh; dz <= rh; ++dz) {
                const double ox = dx * step_cm;
                const double oy = dy * step_cm;
                const double oz = dz * step_cm;
                if (ox * ox + oy * oy + oz * oz > r2) {
                    continue;
                }
                const Position3D probe{
                    (cx + ox) * x_extent[cm],
                    (cy + oy) * y_extent[cm],
                    (cz + oz) * z_extent[cm],
                };
                if (occupancyAt(map, probe) == types::VoxelOccupancy::Unmapped) {
                    return true;
                }
            }
        }
    }
    return false;
}

[[nodiscard]] bool isFrontier(const IMap3D& map,
                              const Position3D& cell,
                              double radius_cm,
                              double step_cm) {
    const double cx = cell.x.force_numerical_value_in(cm);
    const double cy = cell.y.force_numerical_value_in(cm);
    const double cz = cell.z.force_numerical_value_in(cm);

    const Position3D neighbours[6] = {
        {(cx + step_cm) * x_extent[cm], cy * y_extent[cm], cz * z_extent[cm]},
        {(cx - step_cm) * x_extent[cm], cy * y_extent[cm], cz * z_extent[cm]},
        {cx * x_extent[cm], (cy + step_cm) * y_extent[cm], cz * z_extent[cm]},
        {cx * x_extent[cm], (cy - step_cm) * y_extent[cm], cz * z_extent[cm]},
        {cx * x_extent[cm], cy * y_extent[cm], (cz + step_cm) * z_extent[cm]},
        {cx * x_extent[cm], cy * y_extent[cm], (cz - step_cm) * z_extent[cm]},
    };
    for (const Position3D& neighbour : neighbours) {
        const types::VoxelOccupancy occ = occupancyAt(map, neighbour);
        if (occ == types::VoxelOccupancy::Unmapped) {
            return true;
        }
        if (occ == types::VoxelOccupancy::Empty &&
            sphereContainsNotMapped(map, neighbour, radius_cm, step_cm)) {
            return true;
        }
    }
    return false;
}

[[nodiscard]] bool canTraverseForUnknownDistance(types::VoxelOccupancy value) {
    return value == types::VoxelOccupancy::Empty || value == types::VoxelOccupancy::Unmapped;
}

[[nodiscard]] GridIntMap
buildUnknownDistanceField(const IMap3D& map, double step_cm) {
    const types::MapConfig config = map.getMapConfig();
    const types::MappingBounds& bounds = config.boundaries;

    const double min_x = bounds.min_x.force_numerical_value_in(cm);
    const double max_x = bounds.max_x.force_numerical_value_in(cm);
    const double min_y = bounds.min_y.force_numerical_value_in(cm);
    const double max_y = bounds.max_y.force_numerical_value_in(cm);
    const double min_z = bounds.min_height.force_numerical_value_in(cm);
    const double max_z = bounds.max_height.force_numerical_value_in(cm);

    GridIntMap dist;
    std::queue<GridKey> queue;

    for (double x = min_x; x <= max_x + 1e-9; x += step_cm) {
        for (double y = min_y; y <= max_y + 1e-9; y += step_cm) {
            for (double z = min_z; z <= max_z + 1e-9; z += step_cm) {
                const Position3D pos{x * x_extent[cm], y * y_extent[cm], z * z_extent[cm]};
                if (occupancyAt(map, pos) != types::VoxelOccupancy::Unmapped) {
                    continue;
                }
                const GridKey key = quantizePosition(pos, config);
                if (dist.contains(key)) {
                    continue;
                }
                dist[key] = 0;
                queue.push(key);
            }
        }
    }

    while (!queue.empty()) {
        const GridKey current = queue.front();
        queue.pop();
        const int current_dist = dist.at(current);

        for (const Offset& off : kOffsets) {
            const GridKey neighbour{current.qx + off.dx, current.qy + off.dy, current.qz + off.dz};
            if (dist.contains(neighbour)) {
                continue;
            }
            const Position3D neighbour_pos = keyToPoint(neighbour, config);
            if (!canTraverseForUnknownDistance(occupancyAt(map, neighbour_pos))) {
                continue;
            }
            dist[neighbour] = current_dist + 1;
            queue.push(neighbour);
        }
    }

    return dist;
}

[[nodiscard]] FrontierPathResult reconstructPath(
    const GridKey& start_key,
    const GridKey& goal_key,
    const ParentMap& parent_of,
    const types::MapConfig& config) {
    FrontierPathResult result;
    result.found = true;
    GridKey step = goal_key;
    while (!(step == start_key)) {
        result.path.push_back(keyToPoint(step, config));
        step = parent_of.at(step);
    }
    std::reverse(result.path.begin(), result.path.end());
    return result;
}

} // namespace

GridKey quantizePosition(const Position3D& pos, const types::MapConfig& config) {
    const double step = gridStepCm(config);
    const double ox = config.offset.x.force_numerical_value_in(cm);
    const double oy = config.offset.y.force_numerical_value_in(cm);
    const double oz = config.offset.z.force_numerical_value_in(cm);
    const double px = pos.x.force_numerical_value_in(cm);
    const double py = pos.y.force_numerical_value_in(cm);
    const double pz = pos.z.force_numerical_value_in(cm);

    return GridKey{
        static_cast<int>(std::lround((px - ox) / step)),
        static_cast<int>(std::lround((py - oy) / step)),
        static_cast<int>(std::lround((pz - oz) / step)),
    };
}

bool hasNotMappedInSphere(const IMap3D& map, const Position3D& centre, PhysicalLength radius) {
    const double step = gridStepCm(map.getMapConfig());
    if (step <= 0.0) {
        return false;
    }
    const double radius_cm = radius.force_numerical_value_in(cm);
    return sphereContainsNotMapped(map, centre, radius_cm, step);
}

std::size_t countUnmappedInBounds(const IMap3D& map) {
    const types::MapConfig config = map.getMapConfig();
    const double step = gridStepCm(config);
    if (step <= 0.0) {
        return 0;
    }
    const types::MappingBounds& bounds = config.boundaries;

    const double min_x = bounds.min_x.force_numerical_value_in(cm);
    const double max_x = bounds.max_x.force_numerical_value_in(cm);
    const double min_y = bounds.min_y.force_numerical_value_in(cm);
    const double max_y = bounds.max_y.force_numerical_value_in(cm);
    const double min_z = bounds.min_height.force_numerical_value_in(cm);
    const double max_z = bounds.max_height.force_numerical_value_in(cm);

    std::size_t count = 0;
    for (double x = min_x; x <= max_x + 1e-9; x += step) {
        for (double y = min_y; y <= max_y + 1e-9; y += step) {
            for (double z = min_z; z <= max_z + 1e-9; z += step) {
                const Position3D pos{x * x_extent[cm], y * y_extent[cm], z * z_extent[cm]};
                if (occupancyAt(map, pos) == types::VoxelOccupancy::Unmapped) {
                    ++count;
                }
            }
        }
    }
    return count;
}

bool hasAnyNotMappedInBounds(const IMap3D& map) {
    return countUnmappedInBounds(map) > 0;
}

FrontierPathResult MappingAlgorithmFrontier::findPath(
    const IMap3D& map,
    const Position3D& start,
    PhysicalLength drone_radius,
    const BlockedCells& blocked_cells,
    const GridIntMap& frontier_visits) const {
    const types::MapConfig config = map.getMapConfig();
    const double step = gridStepCm(config);
    const double radius_cm = drone_radius.force_numerical_value_in(cm);

    const GridKey start_key = quantizePosition(start, config);
    const Position3D start_pt = keyToPoint(start_key, config);
    if (!isSpherePassable(map, start_pt, radius_cm, step, blocked_cells)) {
        return {};
    }

    ParentMap parent_of;
    GridIntMap depth_of;
    std::queue<GridKey> queue;
    parent_of[start_key] = start_key;
    depth_of[start_key] = 0;
    queue.push(start_key);

    // Pick the reachable frontier with best value density: unmapped_neighbours / path_steps.
    GridKey best_key = start_key;
    double best_score = -1.0;
    int best_depth = kInf;

    while (!queue.empty()) {
        const GridKey current = queue.front();
        queue.pop();
        const int current_depth = depth_of.at(current);
        const Position3D current_pt = keyToPoint(current, config);

        if (!(current == start_key) && isFrontier(map, current_pt, radius_cm, step)) {
            constexpr int kMaxFrontierVisits = 3;
            constexpr int kMaxNearFloorFrontierVisits = 5;
            const double min_z = config.boundaries.min_height.force_numerical_value_in(cm);
            const double frontier_z = current_pt.z.force_numerical_value_in(cm);
            const int max_visits =
                (frontier_z - min_z <= 2.0 * step + 1e-9) ? kMaxNearFloorFrontierVisits
                                                           : kMaxFrontierVisits;
            const int visits =
                frontier_visits.contains(current) ? frontier_visits.at(current) : 0;
            if (visits < max_visits) {
                int value = 0;
                constexpr int kFrontierValueRadius = 2;
                const int radius_sq = kFrontierValueRadius * kFrontierValueRadius;
                for (int dx = -kFrontierValueRadius; dx <= kFrontierValueRadius; ++dx) {
                    for (int dy = -kFrontierValueRadius; dy <= kFrontierValueRadius; ++dy) {
                        for (int dz = -kFrontierValueRadius; dz <= kFrontierValueRadius; ++dz) {
                            if (dx * dx + dy * dy + dz * dz > radius_sq) {
                                continue;
                            }
                            const Position3D nb_pt = keyToPoint(
                                GridKey{current.qx + dx, current.qy + dy, current.qz + dz},
                                config);
                            if (occupancyAt(map, nb_pt) == types::VoxelOccupancy::Unmapped) {
                                ++value;
                            }
                        }
                    }
                }
                if (value > 0) {
                    const double score =
                        static_cast<double>(value) / static_cast<double>(current_depth);
                    if (score > best_score ||
                        (score == best_score && current_depth < best_depth)) {
                        best_score = score;
                        best_depth = current_depth;
                        best_key = current;
                    }
                }
            }
        }

        for (const Offset& off : kOffsets) {
            const GridKey neighbour{current.qx + off.dx, current.qy + off.dy, current.qz + off.dz};
            if (parent_of.contains(neighbour)) {
                continue;
            }
            const Position3D neighbour_pt = keyToPoint(neighbour, config);
            if (!isSpherePassable(map, neighbour_pt, radius_cm, step, blocked_cells)) {
                continue;
            }
            parent_of[neighbour] = current;
            depth_of[neighbour] = current_depth + 1;
            queue.push(neighbour);
        }
    }

    if (best_score < 0.0) {
        return {};
    }

    FrontierPathResult result = reconstructPath(start_key, best_key, parent_of, config);
    result.frontier_key = best_key;
    return result;
}

FrontierPathResult MappingAlgorithmFrontier::findPathTo(
    const IMap3D& map,
    const Position3D& start,
    const Position3D& goal,
    PhysicalLength drone_radius,
    const BlockedCells& blocked_cells) const {
    const types::MapConfig config = map.getMapConfig();
    const double step = gridStepCm(config);
    const double radius_cm = drone_radius.force_numerical_value_in(cm);

    const GridKey start_key = quantizePosition(start, config);
    const GridKey goal_key = quantizePosition(goal, config);
    const Position3D start_pt = keyToPoint(start_key, config);
    const Position3D goal_pt = keyToPoint(goal_key, config);

    if (!isSpherePassable(map, start_pt, radius_cm, step, blocked_cells)) {
        return {};
    }
    if (!isSpherePassable(map, goal_pt, radius_cm, step, blocked_cells)) {
        return {};
    }
    if (start_key == goal_key) {
        return {};
    }

    ParentMap parent_of;
    std::queue<GridKey> queue;
    parent_of[start_key] = start_key;
    queue.push(start_key);

    while (!queue.empty()) {
        const GridKey current = queue.front();
        queue.pop();

        if (current == goal_key) {
            return reconstructPath(start_key, goal_key, parent_of, config);
        }

        for (const Offset& off : kOffsets) {
            const GridKey neighbour{current.qx + off.dx, current.qy + off.dy, current.qz + off.dz};
            if (parent_of.contains(neighbour)) {
                continue;
            }
            const Position3D neighbour_pt = keyToPoint(neighbour, config);
            if (!isSpherePassable(map, neighbour_pt, radius_cm, step, blocked_cells)) {
                continue;
            }
            parent_of[neighbour] = current;
            queue.push(neighbour);
        }
    }

    return {};
}

FrontierPathResult MappingAlgorithmFrontier::findFarthestPath(
    const IMap3D& map,
    const Position3D& start,
    PhysicalLength drone_radius,
    const BlockedCells& blocked_cells) const {
    const types::MapConfig config = map.getMapConfig();
    const double step = gridStepCm(config);
    const double radius_cm = drone_radius.force_numerical_value_in(cm);

    const GridKey start_key = quantizePosition(start, config);
    const Position3D start_pt = keyToPoint(start_key, config);
    if (!isSpherePassable(map, start_pt, radius_cm, step, blocked_cells)) {
        return {};
    }

    ParentMap parent_of;
    GridIntMap depth;
    std::queue<GridKey> queue;
    parent_of[start_key] = start_key;
    depth[start_key] = 0;
    queue.push(start_key);

    GridKey best_goal = start_key;
    int best_depth = -1;

    while (!queue.empty()) {
        const GridKey current = queue.front();
        queue.pop();
        const int current_depth = depth.at(current);
        const Position3D current_pt = keyToPoint(current, config);

        if (!(current == start_key) && isFrontier(map, current_pt, radius_cm, step)) {
            if (current_depth > best_depth) {
                best_depth = current_depth;
                best_goal = current;
            }
        }

        for (const Offset& off : kOffsets) {
            const GridKey neighbour{current.qx + off.dx, current.qy + off.dy, current.qz + off.dz};
            if (parent_of.contains(neighbour)) {
                continue;
            }
            const Position3D neighbour_pt = keyToPoint(neighbour, config);
            if (!isSpherePassable(map, neighbour_pt, radius_cm, step, blocked_cells)) {
                continue;
            }
            parent_of[neighbour] = current;
            depth[neighbour] = current_depth + 1;
            queue.push(neighbour);
        }
    }

    if (best_depth >= 0) {
        return reconstructPath(start_key, best_goal, parent_of, config);
    }

    return {};
}

FrontierPathResult MappingAlgorithmFrontier::findExplorePath(
    const IMap3D& map,
    const Position3D& start,
    PhysicalLength drone_radius,
    const BlockedCells& blocked_cells,
    GridIntMap* dist_cache) const {
    const types::MapConfig config = map.getMapConfig();
    const double step = gridStepCm(config);
    const double radius_cm = drone_radius.force_numerical_value_in(cm);

    const GridKey start_key = quantizePosition(start, config);
    const Position3D start_pt = keyToPoint(start_key, config);
    if (!isSpherePassable(map, start_pt, radius_cm, step, blocked_cells)) {
        return {};
    }

    GridIntMap local_dist;
    if (dist_cache != nullptr) {
        if (dist_cache->empty()) {
            *dist_cache = buildUnknownDistanceField(map, step);
        }
    } else {
        local_dist = buildUnknownDistanceField(map, step);
        dist_cache = &local_dist;
    }
    if (dist_cache->empty()) {
        return {};
    }
    const GridIntMap& unknown_dist = *dist_cache;

    ParentMap parent_of;
    GridIntMap depth;
    std::queue<GridKey> queue;
    parent_of[start_key] = start_key;
    depth[start_key] = 0;
    queue.push(start_key);

    GridKey best_key = start_key;
    int best_unknown_steps = kInf;
    int best_path_steps = kInf;

    while (!queue.empty()) {
        const GridKey current = queue.front();
        queue.pop();
        const int path_steps = depth.at(current);

        const auto dist_it = unknown_dist.find(current);
        const int unknown_steps = (dist_it == unknown_dist.end()) ? kInf : dist_it->second;

        if (!(current == start_key) && unknown_steps < best_unknown_steps) {
            best_key = current;
            best_unknown_steps = unknown_steps;
            best_path_steps = path_steps;
        } else if (!(current == start_key) && unknown_steps == best_unknown_steps &&
                   path_steps < best_path_steps) {
            best_key = current;
            best_path_steps = path_steps;
        }

        for (const Offset& off : kOffsets) {
            const GridKey neighbour{current.qx + off.dx, current.qy + off.dy, current.qz + off.dz};
            if (parent_of.contains(neighbour)) {
                continue;
            }
            const Position3D neighbour_pt = keyToPoint(neighbour, config);
            if (!isSpherePassable(map, neighbour_pt, radius_cm, step, blocked_cells)) {
                continue;
            }
            parent_of[neighbour] = current;
            depth[neighbour] = path_steps + 1;
            queue.push(neighbour);
        }
    }

    if (best_key == start_key || best_unknown_steps == kInf) {
        return {};
    }

    return reconstructPath(start_key, best_key, parent_of, config);
}

FrontierPathResult MappingAlgorithmFrontier::findUnstickPath(const IMap3D& map,
                                                             const Position3D& start,
                                                             PhysicalLength drone_radius) const {
    const types::MapConfig config = map.getMapConfig();
    const double step = gridStepCm(config);
    const double radius_cm = drone_radius.force_numerical_value_in(cm);

    const GridKey start_key = quantizePosition(start, config);
    const Position3D start_pt = keyToPoint(start_key, config);
    if (isSpherePassable(map, start_pt, radius_cm, step, {})) {
        return {};
    }

    const auto unknown_dist = buildUnknownDistanceField(map, step);

    GridKey best_key = start_key;
    int best_unknown_steps = kInf;

    for (const Offset& off : kOffsets) {
        const GridKey neighbour{start_key.qx + off.dx, start_key.qy + off.dy, start_key.qz + off.dz};
        const Position3D neighbour_pt = keyToPoint(neighbour, config);
        if (!isSpherePassable(map, neighbour_pt, radius_cm, step, {})) {
            continue;
        }
        const auto dist_it = unknown_dist.find(neighbour);
        const int unknown_steps = (dist_it == unknown_dist.end()) ? kInf : dist_it->second;
        if (unknown_steps < best_unknown_steps) {
            best_key = neighbour;
            best_unknown_steps = unknown_steps;
        }
    }

    if (best_key == start_key || best_unknown_steps == kInf) {
        return {};
    }

    FrontierPathResult result;
    result.found = true;
    result.path.push_back(keyToPoint(best_key, config));
    return result;
}

FrontierPathResult MappingAlgorithmFrontier::findGreedyUnknownStep(
    const IMap3D& map,
    const Position3D& start,
    PhysicalLength drone_radius,
    const BlockedCells& blocked_cells) const {
    const types::MapConfig config = map.getMapConfig();
    const double step = gridStepCm(config);
    const double radius_cm = drone_radius.force_numerical_value_in(cm);

    const GridKey start_key = quantizePosition(start, config);
    const Position3D start_pt = keyToPoint(start_key, config);
    if (!isSpherePassable(map, start_pt, radius_cm, step, blocked_cells)) {
        return {};
    }

    const auto unknown_dist = buildUnknownDistanceField(map, step);
    const auto dist_at = [&](const GridKey& key) -> int {
        const auto it = unknown_dist.find(key);
        return (it == unknown_dist.end()) ? kInf : it->second;
    };

    const int start_unknown = dist_at(start_key);

    GridKey best_key = start_key;
    int best_unknown = start_unknown;

    for (const Offset& off : kOffsets) {
        const GridKey neighbour{start_key.qx + off.dx, start_key.qy + off.dy, start_key.qz + off.dz};
        const Position3D neighbour_pt = keyToPoint(neighbour, config);
        if (!isSpherePassable(map, neighbour_pt, radius_cm, step, blocked_cells)) {
            continue;
        }
        const int unknown_steps = dist_at(neighbour);
        if (unknown_steps < best_unknown) {
            best_key = neighbour;
            best_unknown = unknown_steps;
        }
    }

    if (best_key == start_key && start_unknown != kInf) {
        for (const Offset& off : kOffsets) {
            const GridKey neighbour{start_key.qx + off.dx, start_key.qy + off.dy, start_key.qz + off.dz};
            if (neighbour == start_key) {
                continue;
            }
            const Position3D neighbour_pt = keyToPoint(neighbour, config);
            if (!isSpherePassable(map, neighbour_pt, radius_cm, step, blocked_cells)) {
                continue;
            }
            const int unknown_steps = dist_at(neighbour);
            if (unknown_steps <= start_unknown) {
                best_key = neighbour;
                best_unknown = unknown_steps;
                break;
            }
        }
    }

    if (best_key == start_key) {
        return {};
    }

    FrontierPathResult result;
    result.found = true;
    result.path.push_back(keyToPoint(best_key, config));
    return result;
}

FrontierPathResult MappingAlgorithmFrontier::findAnyPassableNeighbor(
    const IMap3D& map,
    const Position3D& start,
    PhysicalLength drone_radius,
    const BlockedCells& blocked_cells) const {
    const types::MapConfig config = map.getMapConfig();
    const double step = gridStepCm(config);
    const double radius_cm = drone_radius.force_numerical_value_in(cm);

    const GridKey start_key = quantizePosition(start, config);

    for (const Offset& off : kOffsets) {
        const GridKey neighbour{start_key.qx + off.dx, start_key.qy + off.dy, start_key.qz + off.dz};
        const Position3D neighbour_pt = keyToPoint(neighbour, config);
        if (!isSpherePassable(map, neighbour_pt, radius_cm, step, blocked_cells)) {
            continue;
        }
        FrontierPathResult result;
        result.found = true;
        result.path.push_back(neighbour_pt);
        return result;
    }

    return {};
}

PlanningDiagnostics MappingAlgorithmFrontier::diagnose(
    const IMap3D& map,
    const Position3D& start,
    PhysicalLength drone_radius,
    const BlockedCells& blocked_cells) const {
    PlanningDiagnostics diag{};
    const types::MapConfig config = map.getMapConfig();
    const double step = gridStepCm(config);
    const double radius_cm = drone_radius.force_numerical_value_in(cm);

    const GridKey start_key = quantizePosition(start, config);
    const Position3D start_pt = keyToPoint(start_key, config);
    diag.start_passable = isSpherePassable(map, start_pt, radius_cm, step, blocked_cells);
    if (!diag.start_passable) {
        return diag;
    }

    ParentMap parent_of;
    std::queue<GridKey> queue;
    parent_of[start_key] = start_key;
    queue.push(start_key);

    const auto unknown_dist = buildUnknownDistanceField(map, step);
    int nearest_unknown = kInf;
    int best_unknown_steps = kInf;

    while (!queue.empty()) {
        const GridKey current = queue.front();
        queue.pop();
        ++diag.passable_reached;

        const Position3D current_pt = keyToPoint(current, config);
        if (!(current == start_key) && isFrontier(map, current_pt, radius_cm, step)) {
            ++diag.reachable_frontiers;
        }

        const auto dist_it = unknown_dist.find(current);
        if (dist_it != unknown_dist.end()) {
            nearest_unknown = std::min(nearest_unknown, dist_it->second);
            if (!(current == start_key)) {
                best_unknown_steps = std::min(best_unknown_steps, dist_it->second);
            }
        }

        for (const Offset& off : kOffsets) {
            const GridKey neighbour{current.qx + off.dx, current.qy + off.dy, current.qz + off.dz};
            if (parent_of.contains(neighbour)) {
                continue;
            }
            const Position3D neighbour_pt = keyToPoint(neighbour, config);
            if (!isSpherePassable(map, neighbour_pt, radius_cm, step, blocked_cells)) {
                continue;
            }
            parent_of[neighbour] = current;
            queue.push(neighbour);
        }
    }

    diag.nearest_unknown_steps = (nearest_unknown == kInf) ? -1 : nearest_unknown;
    diag.explore_path_available = (best_unknown_steps != kInf);
    return diag;
}

} // namespace algorithm_207190406_209543255::detail
