#pragma once

#include <Common/IMap3D.h>
#include <Simulator/SimulationTypes.h>

namespace user_common_207190406_209543255 {

/// Mission-local initial pose from YAML is shifted by `map_offset.z` to hidden-map world Z.
[[nodiscard]] common::Position3D worldInitialDronePosition(
    const simulator::types::SimulationConfigData& simulation);

/// True when the drone sphere at `center` is in-bounds and no voxel within `radius` is Occupied.
[[nodiscard]] bool isDroneSpawnPassable(const common::IMap3D& hidden_map,
                                        common::PhysicalLength radius,
                                        const common::Position3D& center);

} // namespace user_common_207190406_209543255
