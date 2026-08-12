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

[[nodiscard]] bool isKnownOccupancy(VoxelOccupancy value) {
    return value == VoxelOccupancy::Empty || value == VoxelOccupancy::Occupied ||
           value == VoxelOccupancy::PotentiallyOccupied;
}

[[nodiscard]] GridKey quantizePosition(const Position3D& pos, const MapConfig& config) {
    const double step = config.resolution.numerical_value_in(cm);
    const double ox = config.offset.x.numerical_value_in(cm);
    const double oy = config.offset.y.numerical_value_in(cm);
    const double oz = config.offset.z.numerical_value_in(cm);
    const double px = pos.x.numerical_value_in(cm);
    const double py = pos.y.numerical_value_in(cm);
    const double pz = pos.z.numerical_value_in(cm);
    return GridKey{
        static_cast<int>(std::lround((px - ox) / step)),
        static_cast<int>(std::lround((py - oy) / step)),
        static_cast<int>(std::lround((pz - oz) / step)),
    };
}

template <typename Visitor>
void forEachGridCenter(const MapConfig& config, const Visitor& visitor) {
    const double step = config.resolution.numerical_value_in(cm);
    if (step <= 0.0) {
        return;
    }

    const auto& bounds = config.boundaries;
    const double min_x = bounds.min_x.numerical_value_in(cm);
    const double max_x = bounds.max_x.numerical_value_in(cm);
    const double min_y = bounds.min_y.numerical_value_in(cm);
    const double max_y = bounds.max_y.numerical_value_in(cm);
    const double min_z = bounds.min_height.numerical_value_in(cm);
    const double max_z = bounds.max_height.numerical_value_in(cm);

    for (double x = min_x; x <= max_x + 1e-9; x += step) {
        for (double y = min_y; y <= max_y + 1e-9; y += step) {
            for (double z = min_z; z <= max_z + 1e-9; z += step) {
                visitor(Position3D{x * x_extent[cm], y * y_extent[cm], z * z_extent[cm]});
            }
        }
    }
}

// BFS through Empty cells in `reference`, seeded at `spawn`, bounded by
// `scoring_config` (the produced/output map's region). Occupied cells adjacent
// to a visited Empty cell are added as "visible walls" (counted, not traversed).
[[nodiscard]] std::unordered_set<GridKey, GridKeyHash> computeReachable(
    const common::IMap3D& reference, const Position3D& spawn, const MapConfig& scoring_config) {
    const double step = scoring_config.resolution.numerical_value_in(cm);
    if (step <= 0.0) {
        return {};
    }

    if (!reference.isInBounds(spawn) || reference.atVoxel(spawn) != VoxelOccupancy::Empty) {
        return {};
    }

    std::unordered_set<GridKey, GridKeyHash> reachable;
    reachable.reserve(512);

    const GridKey start = quantizePosition(spawn, scoring_config);
    reachable.insert(start);

    static const int kDx[6] = {1, -1, 0, 0, 0, 0};
    static const int kDy[6] = {0, 0, 1, -1, 0, 0};
    static const int kDz[6] = {0, 0, 0, 0, 1, -1};

    std::queue<GridKey> queue;
    queue.push(start);

    const double ox = scoring_config.offset.x.numerical_value_in(cm);
    const double oy = scoring_config.offset.y.numerical_value_in(cm);
    const double oz = scoring_config.offset.z.numerical_value_in(cm);

    while (!queue.empty()) {
        const GridKey cur = queue.front();
        queue.pop();

        for (int d = 0; d < 6; ++d) {
            const GridKey nb{cur.x + kDx[d], cur.y + kDy[d], cur.z + kDz[d]};
            if (reachable.contains(nb)) {
                continue;
            }

            const Position3D nb_pos{
                (ox + static_cast<double>(nb.x) * step) * x_extent[cm],
                (oy + static_cast<double>(nb.y) * step) * y_extent[cm],
                (oz + static_cast<double>(nb.z) * step) * z_extent[cm],
            };

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

double MapsComparison::compare(const common::IMap3D& origin, const common::IMap3D& target,
                               std::optional<Position3D> spawn) {
    const MapConfig target_config = target.getMapConfig();

    std::optional<std::unordered_set<GridKey, GridKeyHash>> reachable_set;
    if (spawn.has_value()) {
        auto rs = computeReachable(origin, *spawn, target_config);
        if (!rs.empty()) {
            reachable_set = std::move(rs);
        }
    }

    std::unordered_set<GridKey, GridKeyHash> ref_keys;
    ref_keys.reserve(256);

    std::size_t correct = 0;
    std::size_t total = 0;

    // Pass 1: reference cells within the target map's region.
    forEachGridCenter(target_config, [&](const Position3D& pos) {
        if (!target.isInBounds(pos)) {
            return;
        }

        const VoxelOccupancy ref_value = origin.atVoxel(pos);
        if (ref_value == VoxelOccupancy::OutOfBounds || !isKnownOccupancy(ref_value)) {
            return;
        }

        const GridKey key = quantizePosition(pos, target_config);
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
    forEachGridCenter(target_config, [&](const Position3D& pos) {
        if (!target.isInBounds(pos)) {
            return;
        }

        const VoxelOccupancy target_value = target.atVoxel(pos);
        if (target_value == VoxelOccupancy::OutOfBounds || !isKnownOccupancy(target_value)) {
            return;
        }

        if (ref_keys.contains(quantizePosition(pos, target_config))) {
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
