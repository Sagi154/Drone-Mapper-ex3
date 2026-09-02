// MapsComparison.cpp — 0-100 BFS/reachability scoring, ported from
// ../Drone-Mapper-ex2/src/MapsComparison.cpp. Frozen signature scores exactly
// one target map (ex3 has no multi-target caller); -1 stays the documented
// failure sentinel for callers, though the ported algorithm itself only ever
// returns [0, 100] (an empty comparison universe scores 100, matching ex2).

#include <Simulator/MapsComparison.h>

#include <cmath>
#include <cstddef>
#include <queue>
#include <unordered_set>

namespace simulator {

namespace {

using common::cm;
using common::Position3D;
using common::x_extent;
using common::y_extent;
using common::z_extent;
using common::types::MapConfig;
using common::types::VoxelOccupancy;

struct GridKey {
    int x = 0;
    int y = 0;
    int z = 0;

    [[nodiscard]] bool operator==(const GridKey& other) const = default;
};

struct GridKeyHash {
    [[nodiscard]] std::size_t operator()(const GridKey& key) const noexcept {
        const auto hx = static_cast<std::size_t>(key.x);
        const auto hy = static_cast<std::size_t>(key.y);
        const auto hz = static_cast<std::size_t>(key.z);
        return (hx * 73856093U) ^ (hy * 19349663U) ^ (hz * 83492791U);
    }
};

struct GridExtent {
    int nx = 0;
    int ny = 0;
    int nz = 0;
    MapConfig config{};
};

[[nodiscard]] bool isKnownOccupancy(VoxelOccupancy value) {
    return value == VoxelOccupancy::Empty || value == VoxelOccupancy::Occupied ||
           value == VoxelOccupancy::PotentiallyOccupied;
}

[[nodiscard]] GridExtent gridExtent(const common::IMap3D& map) {
    GridExtent g;
    g.config = map.getMapConfig();
    if (g.config.resolution <= 0.0 * cm) {
        return g;
    }
    const auto size_x = g.config.boundaries.max_x - g.config.boundaries.min_x;
    const auto size_y = g.config.boundaries.max_y - g.config.boundaries.min_y;
    const auto size_z = g.config.boundaries.max_height - g.config.boundaries.min_height;
    g.nx = static_cast<int>(
        std::lround((size_x / g.config.resolution).numerical_value_in(mp_units::one)));
    g.ny = static_cast<int>(
        std::lround((size_y / g.config.resolution).numerical_value_in(mp_units::one)));
    g.nz = static_cast<int>(
        std::lround((size_z / g.config.resolution).numerical_value_in(mp_units::one)));
    return g;
}

[[nodiscard]] Position3D voxelCenter(const GridExtent& g, int ix, int iy, int iz) {
    const auto& b = g.config.boundaries;
    const auto res = g.config.resolution;
    const auto ox = static_cast<double>(ix) * res;
    const auto oy = static_cast<double>(iy) * res;
    const auto oz = static_cast<double>(iz) * res;
    return Position3D{
        b.min_x + mp_units::quantity_cast<x_extent>(ox),
        b.min_y + mp_units::quantity_cast<y_extent>(oy),
        b.min_height + mp_units::quantity_cast<z_extent>(oz),
    };
}

[[nodiscard]] GridKey quantizePosition(const Position3D& pos, const GridExtent& g) {
    const auto& b = g.config.boundaries;
    const double step = g.config.resolution.numerical_value_in(cm);
    return GridKey{
        static_cast<int>(
            std::lround((pos.x.numerical_value_in(cm) - b.min_x.numerical_value_in(cm)) / step)),
        static_cast<int>(
            std::lround((pos.y.numerical_value_in(cm) - b.min_y.numerical_value_in(cm)) / step)),
        static_cast<int>(
            std::lround((pos.z.numerical_value_in(cm) - b.min_height.numerical_value_in(cm)) / step)),
    };
}

template <typename Visitor>
void forEachVoxelIndex(const GridExtent& g, const Visitor& visitor) {
    if (g.config.resolution <= 0.0 * cm) {
        return;
    }
    for (int ix = 0; ix <= g.nx; ++ix) {
        for (int iy = 0; iy <= g.ny; ++iy) {
            for (int iz = 0; iz <= g.nz; ++iz) {
                visitor(ix, iy, iz);
            }
        }
    }
}

// BFS through Empty cells in `reference`, seeded at `spawn`, bounded by
// `g` (the produced/output map's region). Occupied cells adjacent
// to a visited Empty cell are added as "visible walls" (counted, not traversed).
[[nodiscard]] std::unordered_set<GridKey, GridKeyHash> computeReachable(
    const common::IMap3D& reference, const Position3D& spawn, const GridExtent& g) {
    if (g.config.resolution <= 0.0 * cm) {
        return {};
    }

    if (!reference.isInBounds(spawn) || reference.atVoxel(spawn) != VoxelOccupancy::Empty) {
        return {};
    }

    std::unordered_set<GridKey, GridKeyHash> reachable;
    reachable.reserve(512);

    const GridKey start = quantizePosition(spawn, g);
    reachable.insert(start);

    static const int kDx[6] = {1, -1, 0, 0, 0, 0};
    static const int kDy[6] = {0, 0, 1, -1, 0, 0};
    static const int kDz[6] = {0, 0, 0, 0, 1, -1};

    std::queue<GridKey> queue;
    queue.push(start);

    while (!queue.empty()) {
        const GridKey cur = queue.front();
        queue.pop();

        for (int d = 0; d < 6; ++d) {
            const GridKey nb{cur.x + kDx[d], cur.y + kDy[d], cur.z + kDz[d]};
            if (reachable.contains(nb)) {
                continue;
            }

            const Position3D nb_pos = voxelCenter(g, nb.x, nb.y, nb.z);

            if (!reference.isInBounds(nb_pos)) {
                continue;
            }

            const VoxelOccupancy occ = reference.atVoxel(nb_pos);
            if (occ == VoxelOccupancy::Empty) {
                reachable.insert(nb);
                queue.push(nb);
            } else if (occ == VoxelOccupancy::Occupied ||
                       occ == VoxelOccupancy::PotentiallyOccupied) {
                reachable.insert(nb); // visible wall — counted, not traversed
            }
        }
    }

    return reachable;
}

} // namespace

double compareMaps(const common::IMap3D& origin, const common::IMap3D& target,
                   std::optional<Position3D> spawn) {
    const GridExtent g = gridExtent(target);

    std::optional<std::unordered_set<GridKey, GridKeyHash>> reachable_set;
    if (spawn.has_value()) {
        auto rs = computeReachable(origin, *spawn, g);
        if (!rs.empty()) {
            reachable_set = std::move(rs);
        }
    }

    std::unordered_set<GridKey, GridKeyHash> ref_keys;
    ref_keys.reserve(256);

    std::size_t correct = 0;
    std::size_t total = 0;

    // Pass 1: reference cells within the target map's region.
    forEachVoxelIndex(g, [&](int ix, int iy, int iz) {
        const Position3D pos = voxelCenter(g, ix, iy, iz);
        if (!target.isInBounds(pos)) {
            return;
        }

        const VoxelOccupancy ref_value = origin.atVoxel(pos);
        if (ref_value == VoxelOccupancy::OutOfBounds || !isKnownOccupancy(ref_value)) {
            return;
        }

        const GridKey key{ix, iy, iz};
        if (reachable_set.has_value() && !reachable_set->contains(key)) {
            return;
        }

        ref_keys.insert(key);

        const VoxelOccupancy target_value = target.atVoxel(pos);
        if (target_value == VoxelOccupancy::OutOfBounds) {
            return;
        }

        ++total;
        if (target_value == ref_value) {
            ++correct;
        }
    });

    // Pass 2: target cells with known occupancy not already covered above
    // (extra mapped area outside the reference's known region).
    forEachVoxelIndex(g, [&](int ix, int iy, int iz) {
        const Position3D pos = voxelCenter(g, ix, iy, iz);
        if (!target.isInBounds(pos)) {
            return;
        }

        const VoxelOccupancy target_value = target.atVoxel(pos);
        if (target_value == VoxelOccupancy::OutOfBounds || !isKnownOccupancy(target_value)) {
            return;
        }

        const GridKey key{ix, iy, iz};
        if (ref_keys.contains(key)) {
            return;
        }

        if (origin.atVoxel(pos) == VoxelOccupancy::OutOfBounds) {
            return;
        }

        ++total;
        if (target_value == VoxelOccupancy::Empty) {
            ++correct;
        }
    });

    if (total == 0) {
        return 100.0;
    }

    return 100.0 * static_cast<double>(correct) / static_cast<double>(total);
}

} // namespace simulator
