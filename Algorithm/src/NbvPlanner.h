#pragma once

// Frontier-anchored next-best-view policy. Internal to Algorithm.
//
// Objective: utility(v) = unresolved voxels observable from v / mission steps to get
// there and scan. Budget-awareness is a feasibility filter (a candidate costing more
// than the remaining budget is discarded), which makes the policy anytime without any
// tuned regime switch. No RNG: candidates are enumerated, so plans are reproducible.

#include "MappingAlgorithmFrontier.h"
#include "PathShaping.h"

#include <Common/IMap3D.h>
#include <Common/Units.h>
#include <Common/types/DroneTypes.h>
#include <Common/types/LidarTypes.h>

#include <cstddef>
#include <vector>

namespace algorithm_207190406_209543255::detail {

struct ExplorationPlan {
    /// Smoothed waypoints; empty means "the best viewpoint is the current pose".
    std::vector<common::Position3D> waypoints{};
    /// World-frame directions worth scanning at the viewpoint, best gain first.
    /// Converted to the drone frame at emission time.
    std::vector<common::Orientation> terminal_scans{};
    double expected_gain = 0.0;
    bool valid = false;
};

struct NbvInputs {
    const common::IMap3D& map;
    const common::types::DroneState& state;
    const common::types::LidarConfigData& lidar;
    const common::types::DroneConfigData& drone;
    std::size_t remaining_steps = 0;
    const BlockedCells& blocked;
    /// Recovery mode: plan as if the blocked set were empty.
    bool ignore_blocked = false;
};

class NbvPlanner {
public:
    [[nodiscard]] ExplorationPlan plan(const NbvInputs& in) const;

    /// World-frame scan directions for this lidar — the set the drone can actually scan.
    [[nodiscard]] static std::vector<common::Orientation> scanDirections(
        const common::types::LidarConfigData& lidar);

    /// Unresolved voxels observable from `origin` across all scan directions, plus the
    /// per-direction breakdown (positive-gain directions only, best first).
    [[nodiscard]] static double gainAt(const common::IMap3D& map,
                                       const common::Position3D& origin,
                                       const common::types::LidarConfigData& lidar,
                                       std::vector<common::Orientation>* directions_out);

private:
    static constexpr int kCandidateStrideCells = 3;
    static constexpr std::size_t kScoredCandidates = 16;

    MappingAlgorithmFrontier frontier_{};
};

} // namespace algorithm_207190406_209543255::detail
