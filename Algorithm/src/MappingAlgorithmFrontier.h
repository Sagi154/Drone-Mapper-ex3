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

using FrontierCells = std::unordered_set<GridKey, GridKeyHash>;

struct FrontierCluster {
    std::size_t cell_count = 0;
    std::size_t volume_count = 0;
    GridKey approach_key{};
    common::Position3D approach_position{};
    int approach_cost = 0;
    std::vector<GridKey> keys{};
};

struct ReachabilityResult {
    bool start_passable = false;
    bool truncated = false;  ///< Expansion cap hit; treat the result as partial.
    GridKey start_key{};
    ParentMap parent_of{};
    FrontierCells frontier_cells{};
    std::vector<FrontierCluster> clusters{};
};

/// BFS / Dijkstra reachability through confirmed-empty cells on a read-only output map.
class MappingAlgorithmFrontier {
public:
    /// Bounded Dijkstra from start to a specific goal cell.
    [[nodiscard]] FrontierPathResult findPathTo(
        const common::IMap3D& map,
        const common::Position3D& start,
        const common::Position3D& goal,
        common::PhysicalLength drone_radius,
        const BlockedCells& blocked_cells = {}) const;

    /// Dijkstra over the passable component with a FIXED edge set, collecting
    /// frontier-adjacent cells and clustering them. Bounded by max_expansions so a
    /// broken passability check yields a truncated result instead of an unbounded
    /// walk (ALG28).
    [[nodiscard]] ReachabilityResult exploreReachable(
        const common::IMap3D& map,
        const common::Position3D& start,
        common::PhysicalLength drone_radius,
        const BlockedCells& blocked_cells,
        std::size_t max_expansions) const;

    /// One-step face-neighbour escape when the start sphere is blocked.
    /// Returns a path of length 1 to the first sphere-passable 6-neighbour, or empty.
    [[nodiscard]] FrontierPathResult findUnstickPath(
        const common::IMap3D& map,
        const common::Position3D& start,
        common::PhysicalLength drone_radius) const;
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
