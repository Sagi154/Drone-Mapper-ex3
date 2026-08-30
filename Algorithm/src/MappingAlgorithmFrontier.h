#pragma once

// Internal BFS frontier planner ported from ex1 ExplorationFrontier.
// Used only by MappingAlgorithmImpl — not part of the public API.

#include <Common/IMap3D.h>
#include <Common/Units.h>

#include <cstddef>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace algorithm_207190406_209543255::detail {

struct GridKey {
    int qx = 0;
    int qy = 0;
    int qz = 0;

    [[nodiscard]] bool operator==(const GridKey& other) const noexcept {
        return qx == other.qx && qy == other.qy && qz == other.qz;
    }
};

struct GridKeyHash {
    [[nodiscard]] std::size_t operator()(const GridKey& key) const noexcept {
        const std::size_t hx = static_cast<std::size_t>(key.qx);
        const std::size_t hy = static_cast<std::size_t>(key.qy);
        const std::size_t hz = static_cast<std::size_t>(key.qz);
        return (hx * 73856093U) ^ (hy * 19349663U) ^ (hz * 83492791U);
    }
};

using BlockedCells = std::unordered_set<GridKey, GridKeyHash>;
using GridIntMap = std::unordered_map<GridKey, int, GridKeyHash>;
using ParentMap = std::unordered_map<GridKey, GridKey, GridKeyHash>;

struct FrontierPathResult {
    bool found = false;
    std::vector<common::Position3D> path{};
    GridKey frontier_key{};
};

/// A reachable cell that borders unresolved space — an NBV viewpoint candidate.
struct ReachableCell {
    GridKey key{};
    common::Position3D position{};
    int cost = 0;                 ///< Dijkstra traversal cost from start.
    int unmapped_neighbours = 0;  ///< Unmapped cells in the 26-neighbourhood (cheap prefilter).
};

struct ReachabilityResult {
    bool start_passable = false;
    bool truncated = false;  ///< Expansion cap hit; treat the result as partial.
    GridKey start_key{};
    ParentMap parent_of{};
    std::vector<ReachableCell> candidates{};
};

struct PlanningDiagnostics {
    bool start_passable = false;
    std::size_t passable_reached = 0;
    std::size_t reachable_frontiers = 0;
    bool explore_path_available = false;
    int nearest_unknown_steps = -1;
};

/// BFS frontier search through confirmed-empty cells on a read-only output map.
class MappingAlgorithmFrontier {
public:
    [[nodiscard]] FrontierPathResult findPath(
        const common::IMap3D& map,
        const common::Position3D& start,
        common::PhysicalLength drone_radius,
        const BlockedCells& blocked_cells = {},
        const GridIntMap& frontier_visits = {}) const;

    /// BFS path from start to a specific goal cell (same passability rules as findPath).
    [[nodiscard]] FrontierPathResult findPathTo(
        const common::IMap3D& map,
        const common::Position3D& start,
        const common::Position3D& goal,
        common::PhysicalLength drone_radius,
        const BlockedCells& blocked_cells = {}) const;

    /// Dijkstra over the passable component with a FIXED edge set, collecting
    /// frontier-adjacent cells deduplicated onto a stride lattice (lowest cost per
    /// bucket wins). Bounded by max_expansions so a broken passability check yields
    /// a truncated result instead of an unbounded walk (ALG28).
    [[nodiscard]] ReachabilityResult exploreReachable(
        const common::IMap3D& map,
        const common::Position3D& start,
        common::PhysicalLength drone_radius,
        const BlockedCells& blocked_cells,
        int stride_cells,
        std::size_t max_expansions) const;

    /// Like findPath but targets the deepest frontier in the passable component.
    [[nodiscard]] FrontierPathResult findFarthestPath(
        const common::IMap3D& map,
        const common::Position3D& start,
        common::PhysicalLength drone_radius,
        const BlockedCells& blocked_cells = {}) const;

    /// When dist_cache is non-null the pre-built unknown-distance field is reused if
    /// non-empty; otherwise it is built and stored into *dist_cache for future calls.
    [[nodiscard]] FrontierPathResult findExplorePath(
        const common::IMap3D& map,
        const common::Position3D& start,
        common::PhysicalLength drone_radius,
        const BlockedCells& blocked_cells = {},
        GridIntMap* dist_cache = nullptr) const;

    [[nodiscard]] FrontierPathResult findUnstickPath(
        const common::IMap3D& map,
        const common::Position3D& start,
        common::PhysicalLength drone_radius) const;

    /// One grid step toward a passable neighbor with lower (or equal) unknown distance.
    [[nodiscard]] FrontierPathResult findGreedyUnknownStep(
        const common::IMap3D& map,
        const common::Position3D& start,
        common::PhysicalLength drone_radius,
        const BlockedCells& blocked_cells = {}) const;

    /// Any single passable grid step — breaks deadlocks when unknown remains.
    [[nodiscard]] FrontierPathResult findAnyPassableNeighbor(
        const common::IMap3D& map,
        const common::Position3D& start,
        common::PhysicalLength drone_radius,
        const BlockedCells& blocked_cells = {}) const;

    [[nodiscard]] PlanningDiagnostics diagnose(
        const common::IMap3D& map,
        const common::Position3D& start,
        common::PhysicalLength drone_radius,
        const BlockedCells& blocked_cells = {}) const;
};

[[nodiscard]] GridKey quantizePosition(const common::Position3D& pos, const common::types::MapConfig& config);

[[nodiscard]] bool hasNotMappedInSphere(const common::IMap3D& map,
                                        const common::Position3D& centre,
                                        common::PhysicalLength radius);

[[nodiscard]] bool hasAnyNotMappedInBounds(const common::IMap3D& map);

[[nodiscard]] std::size_t countUnmappedInBounds(const common::IMap3D& map);

/// Walks parent links from goal back to start. Empty result when goal is unreachable.
[[nodiscard]] FrontierPathResult reconstructPathTo(const ParentMap& parent_of,
                                                   const GridKey& start_key,
                                                   const GridKey& goal_key,
                                                   const common::types::MapConfig& config);

/// True when the straight segment from..to is sphere-passable at every half-resolution sample.
[[nodiscard]] bool hasClearLineOfSight(const common::IMap3D& map,
                                       const common::Position3D& from,
                                       const common::Position3D& to,
                                       common::PhysicalLength drone_radius);

/// Voxel count of the mission bounds — the expansion cap for every search.
[[nodiscard]] std::size_t maxExpansionsForMap(const common::IMap3D& map);

} // namespace algorithm_207190406_209543255::detail
