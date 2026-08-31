#pragma once

#include "ExplorationPlan.h"
#include "MappingAlgorithmFrontier.h"
#include "PathShaping.h"

#include <vector>

namespace algorithm_207190406_209543255::detail {

class WavefrontPlanner {
public:
    /// Ranks candidate frontier clusters and returns the best. When `alternates` is
    /// non-null, it is filled with the remaining ranked, budget-affordable candidates
    /// (best-to-worst, excluding the returned plan) so a caller can reuse the same
    /// expensive reachability search for several replans instead of one.
    [[nodiscard]] ExplorationPlan plan(const WavefrontInputs& in,
                                       std::vector<ExplorationPlan>* alternates = nullptr) const;

private:
    static constexpr std::size_t kRankedClusters = 8;

    MappingAlgorithmFrontier frontier_{};
};

} // namespace algorithm_207190406_209543255::detail
