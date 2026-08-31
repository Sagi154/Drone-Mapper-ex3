// MapsComparison.h — per-run map scoring API for SimulationRunImpl.
// 0–100 BFS/reachability scorer (ported from ex2). Callers use -1 as the
// failure sentinel for Error runs; this function itself returns [0, 100]
// (an empty comparison universe scores 100).
//
// Contract:
//   - Returns a score in [0, 100].
//   - `spawn` is the drone start in **world** coordinates (local spawn + map_axes_offset).
//     When set, only cells reachable from spawn through Empty voxels in `origin` count.
//     When nullopt, every known cell in `origin` counts (raw / utility mode).

#pragma once

#include <Common/IMap3D.h>
#include <Common/Units.h>

#include <optional>

namespace simulator {

class MapsComparison {
public:
    /// Score one output map against a hidden reference map.
    [[nodiscard]] static double compare(
        const common::IMap3D& origin,
        const common::IMap3D& target,
        std::optional<common::Position3D> spawn = std::nullopt);
};

} // namespace simulator
