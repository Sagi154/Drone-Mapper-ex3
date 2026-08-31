#pragma once

// Path shaping for the NBV policy: string-pulling and the command-level step cost.
// Internal to Algorithm — not part of the public API.

#include <Common/IMap3D.h>
#include <Common/Units.h>

#include <cstddef>
#include <vector>

namespace algorithm_207190406_209543255::detail {

/// Per-command movement limits from DroneConfigData.
struct MovementLimits {
    common::PhysicalLength max_advance{};
    common::PhysicalLength max_elevate{};
    common::HorizontalAngle max_rotate{};
};

/// Removes intermediate waypoints whose straight-line shortcut is clear, merging only
/// across runs at constant altitude. Elevation changes stay as their own waypoints
/// because movement emits Elevate before horizontal motion, so a mixed segment is
/// flown as an L and a 3D diagonal would not describe the real trajectory.
[[nodiscard]] std::vector<common::Position3D> stringPullConstantAltitude(
    const common::IMap3D& map,
    const std::vector<common::Position3D>& path,
    common::PhysicalLength drone_radius);

/// Mission steps to fly `waypoints` from (start_position, start_heading), charged
/// against the real command set: ceil(dz/max_elevate) elevates, ceil(turn/max_rotate)
/// rotations per heading change, ceil(run/max_advance) advances.
[[nodiscard]] std::size_t stepCostForPath(const std::vector<common::Position3D>& waypoints,
                                          const common::Position3D& start_position,
                                          const common::Orientation& start_heading,
                                          const MovementLimits& limits);

} // namespace algorithm_207190406_209543255::detail
