// MappingAlgorithmFrontier.cpp
// BFS frontier search adapted from ex1 ExplorationFrontier for ex2 IMap3D.

#include "MappingAlgorithmFrontier.h"

#include <mp-units/math.h>
#include <mp-units/systems/si/math.h>

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

[[nodiscard]] Position3D keyToPoint(const GridKey& key, const types::MapConfig& config) {
    const auto step = config.resolution;
    return Position3D{
        config.offset.x + mp_units::quantity_cast<x_extent>(static_cast<double>(key.qx) * step),
        config.offset.y + mp_units::quantity_cast<y_extent>(static_cast<double>(key.qy) * step),
        config.offset.z + mp_units::quantity_cast<z_extent>(static_cast<double>(key.qz) * step),
    };
}

[[nodiscard]] types::VoxelOccupancy occupancyAt(const IMap3D& map, const Position3D& pos) {
    if (!map.isInBounds(pos)) {
        return types::VoxelOccupancy::OutOfBounds;
    }
    return map.atVoxel(pos);
}

[[nodiscard]] bool isBlockedCell(const BlockedCells& blocked, const GridKey& key) {
    return blocked.find(key) != blocked.end();
}

[[nodiscard]] int traversalCost(const IMap3D& map, const Position3D& cell) {
    if (occupancyAt(map, cell) == types::VoxelOccupancy::Unmapped) {
        return kUnmappedTraversalCost;
    }
    return kEmptyTraversalCost;
}

// True iff the axis-aligned voxel box centered at (dx,dy,dz)*step with half-extent
// step/2 intersects the closed sphere of radius at the origin. Centre-distance
// (forEachSphereSample) skips face neighbours when radius < step; box nearest-point
// restores footprint checks for e.g. radius 7.5 cm on a 10 cm grid.
[[nodiscard]] bool sphereIntersectsCellBox(int dx, int dy, int dz, PhysicalLength step,
                                           PhysicalLength radius) {
    if (dx == 0 && dy == 0 && dz == 0) {
        return true;
    }
    const auto half = step * 0.5;
    const auto ox = static_cast<double>(dx) * step;
    const auto oy = static_cast<double>(dy) * step;
    const auto oz = static_cast<double>(dz) * step;
    const auto nearest1d = [half](PhysicalLength o) -> PhysicalLength {
        if (0.0 * cm < o - half) {
            return o - half;
        }
        if (0.0 * cm > o + half) {
            return o + half;
        }
        return PhysicalLength{};
    };
    const auto nx = nearest1d(ox);
    const auto ny = nearest1d(oy);
    const auto nz = nearest1d(oz);
    return (nx * nx + ny * ny + nz * nz) <= (radius * radius);
}

[[nodiscard]] bool isSpherePassable(const IMap3D& map,
                                    const Position3D& centre,
                                    PhysicalLength radius,
                                    PhysicalLength step,
                                    const BlockedCells& blocked) {
    // Treat Unmapped as passable: only confirmed Occupied or OutOfBounds blocks navigation.
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

    const int rx = static_cast<int>(std::ceil((radius / step).numerical_value_in(mp_units::one)));
    const int rh = rx;

    for (int dx = -rx; dx <= rx; ++dx) {
        for (int dy = -rx; dy <= rx; ++dy) {
            for (int dz = -rh; dz <= rh; ++dz) {
                if (dx == 0 && dy == 0 && dz == 0) {
                    continue;
                }
                if (!sphereIntersectsCellBox(dx, dy, dz, step, radius)) {
                    continue;
                }

                const Position3D probe{
                    centre.x + mp_units::quantity_cast<x_extent>(static_cast<double>(dx) * step),
                    centre.y + mp_units::quantity_cast<y_extent>(static_cast<double>(dy) * step),
                    centre.z + mp_units::quantity_cast<z_extent>(static_cast<double>(dz) * step),
                };
                const types::VoxelOccupancy probe_occ = occupancyAt(map, probe);
                if (probe_occ == types::VoxelOccupancy::Occupied) {
                    return false;
                }
            }
        }
    }
    return true;
}

[[nodiscard]] bool sphereContainsNotMapped(const IMap3D& map,
                                           const Position3D& centre,
                                           PhysicalLength radius,
                                           PhysicalLength step) {
    const int rx = static_cast<int>(std::ceil((radius / step).numerical_value_in(mp_units::one)));
    const int rh = rx;

    for (int dx = -rx; dx <= rx; ++dx) {
        for (int dy = -rx; dy <= rx; ++dy) {
            for (int dz = -rh; dz <= rh; ++dz) {
                if (!sphereIntersectsCellBox(dx, dy, dz, step, radius)) {
                    continue;
                }
                const Position3D probe{
                    centre.x + mp_units::quantity_cast<x_extent>(static_cast<double>(dx) * step),
                    centre.y + mp_units::quantity_cast<y_extent>(static_cast<double>(dy) * step),
                    centre.z + mp_units::quantity_cast<z_extent>(static_cast<double>(dz) * step),
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

struct BoundedSearchResult {
    ParentMap parent_of{};
    GridIntMap cost_of{};
    std::vector<GridKey> frontier_list{};
    FrontierCells frontier_cells{};
    bool truncated = false;
};

[[nodiscard]] BoundedSearchResult runBoundedSearch(
    const IMap3D& map,
    const types::MapConfig& config,
    const GridKey& start_key,
    PhysicalLength radius,
    PhysicalLength step,
    const BlockedCells& blocked_cells,
    std::size_t max_expansions) {
    BoundedSearchResult out;
    std::unordered_map<GridKey, bool, GridKeyHash> passable_memo;
    auto passable = [&](const GridKey& key, const Position3D& pt) {
        const auto it = passable_memo.find(key);
        if (it != passable_memo.end()) {
            return it->second;
        }
        const bool ok = isSpherePassable(map, pt, radius, step, blocked_cells);
        passable_memo.emplace(key, ok);
        return ok;
    };

    CostQueue queue;
    out.parent_of[start_key] = start_key;
    out.cost_of[start_key] = 0;
    queue.push({0, start_key});

    std::size_t expansions = 0;
    while (!queue.empty()) {
        if (++expansions > max_expansions) {
            out.truncated = true;
            break;
        }
        const auto [current_cost, current] = queue.top();
        queue.pop();
        if (current_cost > out.cost_of.at(current)) {
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
        const bool self_unmapped = current_occ == types::VoxelOccupancy::Unmapped;
        if (unmapped > 0 || self_unmapped) {
            if (out.frontier_cells.insert(current).second) {
                out.frontier_list.push_back(current);
            }
        }

        for (const Offset& off : kOffsets) {
            const GridKey neighbour{current.qx + off.dx, current.qy + off.dy, current.qz + off.dz};
            const Position3D neighbour_pt = keyToPoint(neighbour, config);
            if (!passable(neighbour, neighbour_pt)) {
                continue;
            }
            const int new_cost = current_cost + traversalCost(map, neighbour_pt);
            if (out.cost_of.contains(neighbour) && new_cost >= out.cost_of.at(neighbour)) {
                continue;
            }
            out.parent_of[neighbour] = current;
            out.cost_of[neighbour] = new_cost;
            queue.push({new_cost, neighbour});
        }
    }
    return out;
}

[[nodiscard]] std::vector<FrontierCluster> clusterFrontierCells(
    const IMap3D& map,
    const types::MapConfig& config,
    const std::vector<GridKey>& frontier_list,
    const FrontierCells& frontier_cells,
    const GridIntMap& cost_of) {
    std::vector<FrontierCluster> clusters;
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
                if (!frontier_cells.contains(neighbour) || clustered.contains(neighbour)) {
                    continue;
                }
                clustered.insert(neighbour);
                bfs.push(neighbour);
            }
        }
        cluster.volume_count = cluster.keys.size();
        std::vector<GridKey> surface;
        surface.reserve(cluster.keys.size());
        bool have_surface = false;
        int surface_best_cost = 0;
        GridKey surface_best = cluster.keys.front();
        for (const GridKey& key : cluster.keys) {
            const types::VoxelOccupancy occ = occupancyAt(map, keyToPoint(key, config));
            if (occ != types::VoxelOccupancy::Empty) {
                continue;
            }
            surface.push_back(key);
            const int cell_cost = cost_of.at(key);
            if (!have_surface || cell_cost < surface_best_cost) {
                have_surface = true;
                surface_best_cost = cell_cost;
                surface_best = key;
            }
        }
        if (!have_surface) {
            surface.push_back(best_key);
            surface_best = best_key;
            surface_best_cost = best_cost;
        }
        cluster.keys = std::move(surface);
        cluster.cell_count = cluster.keys.size();
        cluster.approach_key = surface_best;
        cluster.approach_position = keyToPoint(surface_best, config);
        cluster.approach_cost = surface_best_cost;
        clusters.push_back(std::move(cluster));
    }

    std::sort(clusters.begin(), clusters.end(),
              [](const FrontierCluster& a, const FrontierCluster& b) {
                  if (a.approach_key.qx != b.approach_key.qx) {
                      return a.approach_key.qx < b.approach_key.qx;
                  }
                  if (a.approach_key.qy != b.approach_key.qy) {
                      return a.approach_key.qy < b.approach_key.qy;
                  }
                  return a.approach_key.qz < b.approach_key.qz;
              });
    return clusters;
}

template <typename Fn>
void forEachInBoundsVoxel(const IMap3D& map, Fn&& fn) {
    const types::MapConfig config = map.getMapConfig();
    const auto step = config.resolution;
    if (!(step > 0.0 * cm)) {
        return;
    }
    const types::MappingBounds& bounds = config.boundaries;
    const auto x_eps = 1e-9 * x_extent[cm];
    const auto y_eps = 1e-9 * y_extent[cm];
    const auto z_eps = 1e-9 * z_extent[cm];
    for (auto x = bounds.min_x; x <= bounds.max_x + x_eps;
         x += mp_units::quantity_cast<x_extent>(step)) {
        for (auto y = bounds.min_y; y <= bounds.max_y + y_eps;
             y += mp_units::quantity_cast<y_extent>(step)) {
            for (auto z = bounds.min_height; z <= bounds.max_height + z_eps;
                 z += mp_units::quantity_cast<z_extent>(step)) {
                if (!fn(Position3D{x, y, z})) {
                    return;
                }
            }
        }
    }
}

} // namespace

GridKey quantizePosition(const Position3D& pos, const types::MapConfig& config) {
    const auto qx = ((pos.x - config.offset.x) /
                     mp_units::quantity_cast<x_extent>(config.resolution))
                        .numerical_value_in(mp_units::one);
    const auto qy = ((pos.y - config.offset.y) /
                     mp_units::quantity_cast<y_extent>(config.resolution))
                        .numerical_value_in(mp_units::one);
    const auto qz = ((pos.z - config.offset.z) /
                     mp_units::quantity_cast<z_extent>(config.resolution))
                        .numerical_value_in(mp_units::one);
    return GridKey{
        static_cast<int>(std::lround(qx)),
        static_cast<int>(std::lround(qy)),
        static_cast<int>(std::lround(qz)),
    };
}

bool hasNotMappedInSphere(const IMap3D& map, const Position3D& centre, PhysicalLength radius) {
    const auto step = map.getMapConfig().resolution;
    if (!(step > 0.0 * cm)) {
        return false;
    }
    return sphereContainsNotMapped(map, centre, radius, step);
}

std::size_t countUnmappedInBounds(const IMap3D& map) {
    std::size_t count = 0;
    forEachInBoundsVoxel(map, [&](const Position3D& pos) {
        if (occupancyAt(map, pos) == types::VoxelOccupancy::Unmapped) {
            ++count;
        }
        return true;
    });
    return count;
}

bool hasAnyNotMappedInBounds(const IMap3D& map) {
    bool found = false;
    forEachInBoundsVoxel(map, [&](const Position3D& pos) {
        if (occupancyAt(map, pos) == types::VoxelOccupancy::Unmapped) {
            found = true;
            return false;
        }
        return true;
    });
    return found;
}

FrontierPathResult MappingAlgorithmFrontier::findPathTo(
    const IMap3D& map,
    const Position3D& start,
    const Position3D& goal,
    PhysicalLength drone_radius,
    const BlockedCells& blocked_cells) const {
    const types::MapConfig config = map.getMapConfig();
    const auto step = config.resolution;

    const GridKey start_key = quantizePosition(start, config);
    const GridKey goal_key = quantizePosition(goal, config);
    const Position3D start_pt = keyToPoint(start_key, config);
    const Position3D goal_pt = keyToPoint(goal_key, config);

    if (!isSpherePassable(map, start_pt, drone_radius, step, blocked_cells)) {
        return {};
    }
    if (!isSpherePassable(map, goal_pt, drone_radius, step, blocked_cells)) {
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
            if (!isSpherePassable(map, neighbour_pt, drone_radius, step, blocked_cells)) {
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
    const auto step = config.resolution;

    const GridKey start_key = quantizePosition(start, config);
    const Position3D start_pt = keyToPoint(start_key, config);
    if (isSpherePassable(map, start_pt, drone_radius, step, {})) {
        return {};
    }

    std::size_t expansions = 0;
    const std::size_t max_expansions = maxExpansionsForMap(map);
    for (const Offset& off : kOffsets) {
        if (++expansions > max_expansions) {
            return {};
        }
        const GridKey neighbour{start_key.qx + off.dx, start_key.qy + off.dy,
                                start_key.qz + off.dz};
        const Position3D neighbour_pt = keyToPoint(neighbour, config);
        if (isSpherePassable(map, neighbour_pt, drone_radius, step, {})) {
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
    const auto step = config.resolution;
    if (!(step > 0.0 * cm)) {
        return 1;
    }
    const types::MappingBounds& b = config.boundaries;
    const auto span = [step](auto lo, auto hi) {
        const auto delta = mp_units::quantity_cast<common::isq::length>(hi - lo);
        return static_cast<std::size_t>(
                   std::max(0.0, std::floor((delta / step).numerical_value_in(mp_units::one)))) +
               1U;
    };
    return span(b.min_x, b.max_x) * span(b.min_y, b.max_y) *
           span(b.min_height, b.max_height);
}

bool hasClearLineOfSight(const IMap3D& map,
                         const Position3D& from,
                         const Position3D& to,
                         PhysicalLength drone_radius) {
    const types::MapConfig config = map.getMapConfig();
    const auto step = config.resolution;
    if (!(step > 0.0 * cm)) {
        return false;
    }
    const Position3D delta = to - from;
    const auto dx = mp_units::quantity_cast<common::isq::length>(delta.x);
    const auto dy = mp_units::quantity_cast<common::isq::length>(delta.y);
    const auto dz = mp_units::quantity_cast<common::isq::length>(delta.z);
    const auto length = mp_units::sqrt(dx * dx + dy * dy + dz * dz);
    const auto sample = step * 0.5;
    const int samples =
        static_cast<int>(std::ceil((length / sample).numerical_value_in(mp_units::one)));

    for (int i = 0; i <= samples; ++i) {
        const double t = (samples == 0) ? 0.0 : static_cast<double>(i) / samples;
        const Position3D probe{
            from.x + mp_units::quantity_cast<x_extent>(dx * t),
            from.y + mp_units::quantity_cast<y_extent>(dy * t),
            from.z + mp_units::quantity_cast<z_extent>(dz * t),
        };
        if (!isSpherePassable(map, probe, drone_radius, step, {})) {
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
    std::size_t max_expansions) const {
    const types::MapConfig config = map.getMapConfig();
    const auto step = config.resolution;

    ReachabilityResult out;
    out.start_key = quantizePosition(start, config);
    const Position3D start_pt = keyToPoint(out.start_key, config);
    if (!isSpherePassable(map, start_pt, drone_radius, step, blocked_cells)) {
        return out;
    }
    out.start_passable = true;

    const BoundedSearchResult search = runBoundedSearch(
        map, config, out.start_key, drone_radius, step, blocked_cells, max_expansions);
    out.parent_of = std::move(search.parent_of);
    out.frontier_cells = std::move(search.frontier_cells);
    out.truncated = search.truncated;
    out.clusters = clusterFrontierCells(map, config, search.frontier_list, out.frontier_cells,
                                        search.cost_of);
    return out;
}

} // namespace algorithm_207190406_209543255::detail
