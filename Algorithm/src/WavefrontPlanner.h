#pragma once

#include "ExplorationPlan.h"
#include "MappingAlgorithmFrontier.h"
#include "PathShaping.h"

namespace algorithm_207190406_209543255::detail {

class WavefrontPlanner {
public:
    [[nodiscard]] ExplorationPlan plan(const WavefrontInputs& in) const;

private:
    static constexpr std::size_t kRankedClusters = 8;

    MappingAlgorithmFrontier frontier_{};
};

} // namespace algorithm_207190406_209543255::detail
