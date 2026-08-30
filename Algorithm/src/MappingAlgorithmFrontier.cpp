// MappingAlgorithmFrontier.cpp
// BFS frontier search adapted from ex1 ExplorationFrontier for ex2 IMap3D.

#include "MappingAlgorithmFrontier.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <queue>
#include <unordered_map>
#include <utility>

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

constexpr int kEmptyTraversalCost = 1;
constexpr int kUnmappedTraversalCost = 4;

struct CostKeyGreater {
    [[nodiscard]] bool operator()(const std::pair<int, GridKey>& a,
                                  const std::pair<int, GridKey>& b) const noexcept {
        return a.first > b.first;
    }
};

using CostQueue = std::priority_queue<std::pair<int, GridKey>,
                                      std::vector<std::pair<int, GridKey>>,
                                      CostKeyGreater>;

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

[[nodiscard]] int traversalCost(const IMap3D& map, const Position3D& cell) {
    if (occupancyAt(map, cell) == types::VoxelOccupancy::Unmapped) {
        return kUnmappedTraversalCost;
    }
    return kEmptyTraversalCost;
}

// True iff the axis-aligned voxel box centered at (dx,dy,dz)*step_cm with half-extent
// step_cm/2 intersects the closed sphere of radius radius_cm at the origin. Using cell
// centres for the distance gate (the old ox²+oy²+oz² > r² test) is a no-op when
// radius_cm < step_cm: every non-zero offset is ≥ step_cm and is skipped. Nearest-point
// in the box restores footprint checks for e.g. radius 7.5 cm on a 10 cm grid.
[[nodiscard]] bool sphereIntersectsCellBox(int dx, int dy, int dz,
                                           double step_cm,
                                           double radius_cm) {
    if (dx == 0 && dy == 0 && dz == 0) {
        return true;
    }
    const double half = step_cm * 0.5;
    const double ox = static_cast<double>(dx) * step_cm;
    const double oy = static_cast<double>(dy) * step_cm;
    const double oz = static_cast<double>(dz) * step_cm;
    const auto nearest1d = [half](double o) {
        if (0.0 < o - half) {
            return o - half;
        }
        if (0.0 > o + half) {
            return o + half;
        }
        return 0.0;
    };
    const double nx = nearest1d(ox);
    const double ny = nearest1d(oy);
    const double nz = nearest1d(oz);
    return (nx * nx + ny * ny + nz * nz) <= (radius_cm * radius_cm);
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
                if (!sphereIntersectsCellBox(dx, dy, dz, step_cm, radius_cm)) {
                    continue;
                }

                const double ox = dx * step_cm;
                const double oy = dy * step_cm;
                const double oz = dz * step_cm;
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

    for (int dx = -rx; dx <= rx; ++dx) {
        for (int dy = -rx; dy <= rx; ++dy) {
            for (int dz = -rh; dz <= rh; ++dz) {
                if (!sphereIntersectsCellBox(dx, dy, dz, step_cm, radius_cm)) {
                    continue;
                }
                const double ox = dx * step_cm;
                const double oy = dy * step_cm;
                const double oz = dz * step_cm;
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
    GridIntMap cost_of;
    CostQueue queue;
    parent_of[start_key] = start_key;
    cost_of[start_key] = 0;
    queue.push({0, start_key});

    std::size_t expansions = 0;
    const std::size_t max_expansions = maxExpansionsForMap(map);
    while (!queue.empty()) {
        if (++expansions > max_expansions) {
            return {};
        }
        const auto [current_cost, current] = queue.top();
        queue.pop();
        if (current_cost > cost_of.at(current)) {
            continue;
        }

        if (current == goal_key) {
            return reconstructPath(start_key, goal_key, parent_of, config);
        }

        for (const Offset& off : kOffsets) {
            const GridKey neighbour{current.qx + off.dx, current.qy + off.dy, current.qz + off.dz};
            const Position3D neighbour_pt = keyToPoint(neighbour, config);
            if (!isSpherePassable(map, neighbour_pt, radius_cm, step, blocked_cells)) {
                continue;
            }
            const int new_cost = current_cost + traversalCost(map, neighbour_pt);
            if (cost_of.contains(neighbour) && new_cost >= cost_of.at(neighbour)) {
                continue;
            }
            parent_of[neighbour] = current;
            cost_of[neighbour] = new_cost;
            queue.push({new_cost, neighbour});
        }
    }

    return {};
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

    // Adjacent escape only: Occupied/non-passable neighbours are not walked.
    std::size_t expansions = 0;
    const std::size_t max_expansions = maxExpansionsForMap(map);
    for (const Offset& off : kOffsets) {
        if (++expansions > max_expansions) {
            return {};
        }
        const GridKey neighbour{start_key.qx + off.dx, start_key.qy + off.dy,
                                start_key.qz + off.dz};
        const Position3D neighbour_pt = keyToPoint(neighbour, config);
        if (isSpherePassable(map, neighbour_pt, radius_cm, step, {})) {
            FrontierPathResult result;
            result.found = true;
            result.path.push_back(neighbour_pt);
            return result;
        }
    }

    return {};
}

std::size_t maxExpansionsForMap(const IMap3D& map) {
    const types::MapConfig config = map.getMapConfig();
    const double step = gridStepCm(config);
    if (!(step > 0.0)) {
        return 1;
    }
    const types::MappingBounds& b = config.boundaries;
    const auto span = [step](double lo, double hi) {
        return static_cast<std::size_t>(std::max(0.0, std::floor((hi - lo) / step))) + 1U;
    };
    const std::size_t nx = span(b.min_x.force_numerical_value_in(cm),
                                b.max_x.force_numerical_value_in(cm));
    const std::size_t ny = span(b.min_y.force_numerical_value_in(cm),
                                b.max_y.force_numerical_value_in(cm));
    const std::size_t nz = span(b.min_height.force_numerical_value_in(cm),
                                b.max_height.force_numerical_value_in(cm));
    return nx * ny * nz;
}

bool hasClearLineOfSight(const IMap3D& map,
                         const Position3D& from,
                         const Position3D& to,
                         PhysicalLength drone_radius) {
    const types::MapConfig config = map.getMapConfig();
    const double step = gridStepCm(config);
    if (!(step > 0.0)) {
        return false;
    }
    const double radius_cm = drone_radius.force_numerical_value_in(cm);
    const double dx = to.x.force_numerical_value_in(cm) - from.x.force_numerical_value_in(cm);
    const double dy = to.y.force_numerical_value_in(cm) - from.y.force_numerical_value_in(cm);
    const double dz = to.z.force_numerical_value_in(cm) - from.z.force_numerical_value_in(cm);
    const double length = std::sqrt(dx * dx + dy * dy + dz * dz);
    const double sample = step * 0.5;
    const int samples = static_cast<int>(std::ceil(length / sample));

    for (int i = 0; i <= samples; ++i) {
        const double t = (samples == 0) ? 0.0 : static_cast<double>(i) / samples;
        const Position3D probe{
            (from.x.force_numerical_value_in(cm) + dx * t) * x_extent[cm],
            (from.y.force_numerical_value_in(cm) + dy * t) * y_extent[cm],
            (from.z.force_numerical_value_in(cm) + dz * t) * z_extent[cm],
        };
        if (!isSpherePassable(map, probe, radius_cm, step, {})) {
            return false;
        }
    }
    return true;
}

FrontierPathResult reconstructPathTo(const ParentMap& parent_of,
                                     const GridKey& start_key,
                                     const GridKey& goal_key,
                                     const types::MapConfig& config) {
    if (!parent_of.contains(goal_key)) {
        return {};
    }
    FrontierPathResult result = reconstructPath(start_key, goal_key, parent_of, config);
    result.frontier_key = goal_key;
    return result;
}

ReachabilityResult MappingAlgorithmFrontier::exploreReachable(
    const IMap3D& map,
    const Position3D& start,
    PhysicalLength drone_radius,
    const BlockedCells& blocked_cells,
    int stride_cells,
    std::size_t max_expansions) const {
    const types::MapConfig config = map.getMapConfig();
    const double step = gridStepCm(config);
    const double radius_cm = drone_radius.force_numerical_value_in(cm);
    const int stride = std::max(1, stride_cells);

    ReachabilityResult out;
    out.start_key = quantizePosition(start, config);
    const Position3D start_pt = keyToPoint(out.start_key, config);
    if (!isSpherePassable(map, start_pt, radius_cm, step, blocked_cells)) {
        return out;
    }
    out.start_passable = true;

    std::unordered_map<GridKey, bool, GridKeyHash> passable_memo;
    auto passable = [&](const GridKey& key, const Position3D& pt) {
        const auto it = passable_memo.find(key);
        if (it != passable_memo.end()) {
            return it->second;
        }
        const bool ok = isSpherePassable(map, pt, radius_cm, step, blocked_cells);
        passable_memo.emplace(key, ok);
        return ok;
    };

    GridIntMap cost_of;
    CostQueue queue;
    out.parent_of[out.start_key] = out.start_key;
    cost_of[out.start_key] = 0;
    queue.push({0, out.start_key});

    // bucket key -> index into out.candidates, lowest cost per bucket wins.
    std::unordered_map<GridKey, std::size_t, GridKeyHash> bucket_of;
    std::vector<GridKey> frontier_list;
    std::size_t expansions = 0;

    while (!queue.empty()) {
        if (++expansions > max_expansions) {
            out.truncated = true;
            break;
        }
        const auto [current_cost, current] = queue.top();
        queue.pop();
        if (current_cost > cost_of.at(current)) {
            continue;
        }
        const Position3D current_pt = keyToPoint(current, config);
        const types::VoxelOccupancy current_occ = occupancyAt(map, current_pt);

        int unmapped = 0;
        for (const Offset& off : kOffsets) {
            const Position3D nb = keyToPoint(
                GridKey{current.qx + off.dx, current.qy + off.dy, current.qz + off.dz}, config);
            if (occupancyAt(map, nb) == types::VoxelOccupancy::Unmapped) {
                ++unmapped;
            }
        }
        // Isolated Unmapped voxels have no Unmapped face neighbour, but they are the
        // unknown volume. Including them keeps face-connected clustering as one
        // cluster per pocket (the Empty shell alone is six disconnected cells).
        const bool self_unmapped = current_occ == types::VoxelOccupancy::Unmapped;
        if (unmapped > 0 || self_unmapped) {
            if (out.frontier_cells.insert(current).second) {
                frontier_list.push_back(current);
            }
            if (unmapped > 0 && !(current == out.start_key)) {
                const auto floor_div = [stride](int v) {
                    return (v >= 0) ? (v / stride) : -(((-v) + stride - 1) / stride);
                };
                const GridKey bucket{floor_div(current.qx), floor_div(current.qy),
                                     floor_div(current.qz)};
                const ReachableCell cell{current, current_pt, current_cost, unmapped};
                const auto it = bucket_of.find(bucket);
                if (it == bucket_of.end()) {
                    bucket_of[bucket] = out.candidates.size();
                    out.candidates.push_back(cell);
                } else if (current_cost < out.candidates[it->second].cost) {
                    out.candidates[it->second] = cell;
                }
            }
        }

        for (const Offset& off : kOffsets) {
            const GridKey neighbour{current.qx + off.dx, current.qy + off.dy, current.qz + off.dz};
            const Position3D neighbour_pt = keyToPoint(neighbour, config);
            if (!passable(neighbour, neighbour_pt)) {
                continue;
            }
            const int new_cost = current_cost + traversalCost(map, neighbour_pt);
            if (cost_of.contains(neighbour) && new_cost >= cost_of.at(neighbour)) {
                continue;
            }
            out.parent_of[neighbour] = current;
            cost_of[neighbour] = new_cost;
            queue.push({new_cost, neighbour});
        }
    }

    std::unordered_set<GridKey, GridKeyHash> clustered;
    for (const GridKey& seed : frontier_list) {
        if (clustered.contains(seed)) {
            continue;
        }
        FrontierCluster cluster;
        std::queue<GridKey> bfs;
        bfs.push(seed);
        clustered.insert(seed);
        int best_cost = cost_of.at(seed);
        GridKey best_key = seed;
        while (!bfs.empty()) {
            const GridKey cell = bfs.front();
            bfs.pop();
            cluster.keys.push_back(cell);
            const int cell_cost = cost_of.at(cell);
            if (cell_cost < best_cost) {
                best_cost = cell_cost;
                best_key = cell;
            }
            for (const Offset& off : kOffsets) {
                const GridKey neighbour{cell.qx + off.dx, cell.qy + off.dy, cell.qz + off.dz};
                if (!out.frontier_cells.contains(neighbour) || clustered.contains(neighbour)) {
                    continue;
                }
                clustered.insert(neighbour);
                bfs.push(neighbour);
            }
        }
        cluster.cell_count = cluster.keys.size();
        cluster.approach_key = best_key;
        cluster.approach_position = keyToPoint(best_key, config);
        cluster.approach_cost = best_cost;
        out.clusters.push_back(std::move(cluster));
    }

    std::sort(out.clusters.begin(), out.clusters.end(),
              [](const FrontierCluster& a, const FrontierCluster& b) {
                  if (a.approach_key.qx != b.approach_key.qx) {
                      return a.approach_key.qx < b.approach_key.qx;
                  }
                  if (a.approach_key.qy != b.approach_key.qy) {
                      return a.approach_key.qy < b.approach_key.qy;
                  }
                  return a.approach_key.qz < b.approach_key.qz;
              });

    return out;
}

} // namespace algorithm_207190406_209543255::detail
