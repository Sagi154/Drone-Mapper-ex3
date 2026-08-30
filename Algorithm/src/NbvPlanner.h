#pragma once

// Frontier-anchored next-best-view policy. Internal to Algorithm.
//
// Objective: utility(v) = unresolved voxels observable from v / mission steps to get
// there and scan. Budget-awareness is a feasibility filter (a candidate costing more
// than the remaining budget is discarded), which makes the policy anytime without any
// tuned regime switch. No RNG: candidates are enumerated, so plans are reproducible.

#include "ExplorationPlan.h"
#include "MappingAlgorithmFrontier.h"
#include "PathShaping.h"

namespace algorithm_207190406_209543255::detail {

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
