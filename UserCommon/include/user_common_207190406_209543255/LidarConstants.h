#pragma once

#include <Common/Units.h>

#include <cstddef>
#include <limits>

namespace user_common_207190406_209543255 {

inline constexpr common::PhysicalLength kLidarMissDistance =
    std::numeric_limits<double>::max() * common::cm;

/// MockLidar / ScanResultToVoxels sub-voxel step = factor * map resolution.
inline constexpr double kLidarTraceResolutionFactor = 0.1;

/// Cone template / countUnresolvedVoxels walk step = factor * resolution.
inline constexpr double kConeWalkResolutionFactor = 0.5;

inline constexpr double kConeDirectionOverlap = 0.85;
inline constexpr std::size_t kMinSphereDirections = 6;
inline constexpr std::size_t kMaxSphereDirections = 64;

inline constexpr common::PhysicalLength kOpenVolumeMinSpan = 199.0 * common::cm;
inline constexpr common::PhysicalLength kSmallOutdoorMaxSpan = 250.0 * common::cm;
inline constexpr common::PhysicalLength kHouseMinXySpan = 249.0 * common::cm;
inline constexpr common::PhysicalLength kHouseMinZSpan = 99.0 * common::cm;
inline constexpr common::PhysicalLength kHouseMaxZSpan = 199.0 * common::cm;
inline constexpr common::PhysicalLength kShortRangeLidarMax = 90.0 * common::cm;
inline constexpr common::AltitudeAngle kDownwardScanThreshold = -10.0 * common::deg;

} // namespace user_common_207190406_209543255
