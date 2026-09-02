#pragma once

// Shared beam-geometry helpers (Algorithm gain-gating + MissionControl fusion).
// Compiled into each consumer — no cross-.so symbol dependency.

#include <user_common_207190406_209543255/LidarConstants.h>

#include <Common/Types.h>

namespace user_common_207190406_209543255::beam_math {

using common::Orientation;
using common::PhysicalLength;
using common::Position3D;

[[nodiscard]] bool isZeroDistance(PhysicalLength distance);
[[nodiscard]] bool isMissDistance(PhysicalLength distance);
[[nodiscard]] Orientation absoluteBeamOrientation(const Orientation& drone_heading,
                                                  const Orientation& relative_beam);
[[nodiscard]] Orientation normalizeOrientation(Orientation orientation);
[[nodiscard]] Position3D pointAlongBeam(const Position3D& origin,
                                        const Orientation& beam_orientation,
                                        PhysicalLength distance);

} // namespace user_common_207190406_209543255::beam_math
