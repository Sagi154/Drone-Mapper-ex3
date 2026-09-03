#pragma once

#include "MappingAlgorithmFrontier.h"

#include <Common/IMap3D.h>
#include <Common/Units.h>
#include <Common/types/DroneTypes.h>
#include <Common/types/LidarTypes.h>

#include <cstddef>
#include <vector>

namespace algorithm_207190406_209543255::detail {

using InformationRate = double; // voxels gained per step; dimensionless by design

namespace plan_detail {
struct Internals {
    std::vector<GridKey> target_keys{};
    FrontierCells frontier_cells{};
};
} // namespace plan_detail

struct ExplorationPlan {
    std::vector<common::Position3D> waypoints{};
    std::size_t target_cluster_cells = 0;
    InformationRate expected_rate = 0.0;
    bool valid = false;
    plan_detail::Internals internals{};
};

struct WavefrontInputs {
    const common::IMap3D& map;
    const common::types::DroneState& state;
    const common::types::LidarConfigData& lidar;
    const common::types::DroneConfigData& drone;
    std::size_t remaining_steps = 0;
    const BlockedCells& blocked;
    bool ignore_blocked = false;
    bool prefer_descend = false;
};

} // namespace algorithm_207190406_209543255::detail
