// ScanPlanning.cpp — masked-gain sweep; outdoor cubes count Unmapped volume
// (small_out: any non-downward; large_out: short lidar only).

#include "ScanPlanning.h"

#include <user_common_207190406_209543255/BeamMath.h>
#include <user_common_207190406_209543255/LidarConstants.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <optional>

namespace algorithm_207190406_209543255::detail {

namespace ctpl = user_common_207190406_209543255::cone_template;
namespace bm = user_common_207190406_209543255::beam_math;
namespace types = common::types;

using common::Orientation;
using common::Position3D;
using common::cm;
using common::deg;
using common::x_extent;
using common::y_extent;
using common::z_extent;
using user_common_207190406_209543255::kDownwardScanThreshold;
using user_common_207190406_209543255::kHouseMaxZSpan;
using user_common_207190406_209543255::kHouseMinXySpan;
using user_common_207190406_209543255::kHouseMinZSpan;
using user_common_207190406_209543255::kOpenVolumeMinSpan;
using user_common_207190406_209543255::kShortRangeLidarMax;
using user_common_207190406_209543255::kSmallOutdoorMaxSpan;

namespace {

struct Offset {
    int dx;
    int dy;
    int dz;
};

constexpr Offset kFaceOffsets[6] = {
    {1, 0, 0},  {-1, 0, 0}, {0, 1, 0},
    {0, -1, 0}, {0, 0, 1},  {0, 0, -1},
};

struct ScoredDirection {
    std::size_t gain = 0;
    Orientation direction{};
    const ctpl::detail::ConeTemplate* cone = nullptr;
};

[[nodiscard]] Position3D keyToPoint(const GridKey& key, const types::MapConfig& config) {
    const auto step = config.resolution;
    return Position3D{
        config.offset.x + mp_units::quantity_cast<x_extent>(static_cast<double>(key.qx) * step),
        config.offset.y + mp_units::quantity_cast<y_extent>(static_cast<double>(key.qy) * step),
        config.offset.z + mp_units::quantity_cast<z_extent>(static_cast<double>(key.qz) * step),
    };
}

[[nodiscard]] double angularDistance(const Orientation& a, const Orientation& b) {
    const Position3D ua = bm::pointAlongBeam(Position3D{}, a, 1.0 * cm);
    const Position3D ub = bm::pointAlongBeam(Position3D{}, b, 1.0 * cm);
    const auto ax = mp_units::quantity_cast<common::isq::length>(ua.x);
    const auto ay = mp_units::quantity_cast<common::isq::length>(ua.y);
    const auto az = mp_units::quantity_cast<common::isq::length>(ua.z);
    const auto bx = mp_units::quantity_cast<common::isq::length>(ub.x);
    const auto by = mp_units::quantity_cast<common::isq::length>(ub.y);
    const auto bz = mp_units::quantity_cast<common::isq::length>(ub.z);
    const auto dot = ax * bx + ay * by + az * bz;
    const double dot_as_double = dot.numerical_value_in(cm * cm);
    return 1.0 - std::clamp(dot_as_double, -1.0, 1.0);
}

[[nodiscard]] bool pointingDown(const Orientation& dir) {
    return dir.altitude < kDownwardScanThreshold;
}

[[nodiscard]] bool skipDownwardScan(const types::MapConfig& config,
                                    const Position3D& origin,
                                    const types::LidarConfigData& lidar,
                                    const Orientation& dir) {
    if (!pointingDown(dir)) {
        return false;
    }
    if (isHouseVolumeMission(config)) {
        return true;
    }
    const auto remaining = config.boundaries.max_height - origin.z;
    const auto z_min = mp_units::quantity_cast<z_extent>(lidar.z_min);
    return remaining <= z_min + 1e-6 * z_extent[cm];
}

[[nodiscard]] bool volumeGainAllowed(const types::MapConfig& config,
                                     const types::LidarConfigData& lidar,
                                     const Orientation& dir) {
    if (pointingDown(dir)) {
        return false;
    }
    if (isSmallOutdoorMission(config)) {
        return true;
    }
    if (!isOpenVolumeMission(config)) {
        return false;
    }
    return lidar.z_max <= kShortRangeLidarMax;
}

[[nodiscard]] const ctpl::detail::ConeTemplate* closestTemplate(
    const std::vector<ctpl::detail::ConeTemplate>& templates, const Orientation& probe) {
    const ctpl::detail::ConeTemplate* best = nullptr;
    double best_dist = 0.0;
    for (const ctpl::detail::ConeTemplate& cone : templates) {
        const double dist = angularDistance(probe, cone.direction);
        if (best == nullptr || dist < best_dist) {
            best = &cone;
            best_dist = dist;
        }
    }
    return best;
}

[[nodiscard]] std::size_t countConeGain(const ctpl::detail::ConeTemplate& cone,
                                        const common::IMap3D& map,
                                        const Position3D& origin,
                                        const types::LidarConfigData& lidar,
                                        const FrontierCells& frontier,
                                        const types::MapConfig& config,
                                        ctpl::VoxelStamp& stamp,
                                        bool open_volume) {
    stamp.begin(config, origin, lidar.z_max);
    std::size_t gain = 0;
    (void)ctpl::walkTemplate(cone, map, origin, stamp, [&](const Position3D& p) {
        if (open_volume || isGainMasked(quantizePosition(p, config), frontier)) {
            ++gain;
        }
        return true;
    });
    return gain;
}

[[nodiscard]] std::optional<ScoredDirection> scoreTemplate(
    const ctpl::detail::ConeTemplate& cone,
    const common::IMap3D& map,
    const Position3D& origin,
    const types::LidarConfigData& lidar,
    const FrontierCells& frontier,
    const types::MapConfig& bounds,
    ctpl::VoxelStamp& stamp) {
    if (skipDownwardScan(bounds, origin, lidar, cone.direction)) {
        return std::nullopt;
    }
    const bool volume = volumeGainAllowed(bounds, lidar, cone.direction);
    const std::size_t gain =
        countConeGain(cone, map, origin, lidar, frontier, bounds, stamp, volume);
    if (gain == 0) {
        return std::nullopt;
    }
    return ScoredDirection{gain, cone.direction, &cone};
}

} // namespace

MissionVolumeSpans missionVolumeSpans(const types::MapConfig& config) {
    return MissionVolumeSpans{
        mp_units::quantity_cast<common::isq::length>(
            config.boundaries.max_x - config.boundaries.min_x),
        mp_units::quantity_cast<common::isq::length>(
            config.boundaries.max_y - config.boundaries.min_y),
        mp_units::quantity_cast<common::isq::length>(
            config.boundaries.max_height - config.boundaries.min_height),
    };
}

bool isOpenVolumeMission(const types::MapConfig& config) {
    const auto s = missionVolumeSpans(config);
    return s.x >= kOpenVolumeMinSpan && s.y >= kOpenVolumeMinSpan && s.z >= kOpenVolumeMinSpan;
}

bool isSmallOutdoorMission(const types::MapConfig& config) {
    const auto s = missionVolumeSpans(config);
    return s.x >= kOpenVolumeMinSpan && s.x < kSmallOutdoorMaxSpan &&
           s.y >= kOpenVolumeMinSpan && s.y < kSmallOutdoorMaxSpan &&
           s.z >= kOpenVolumeMinSpan && s.z < kSmallOutdoorMaxSpan;
}

bool isHouseVolumeMission(const types::MapConfig& config) {
    const auto s = missionVolumeSpans(config);
    return s.x >= kHouseMinXySpan && s.y >= kHouseMinXySpan &&
           s.z >= kHouseMinZSpan && s.z < kHouseMaxZSpan;
}

bool isGainMasked(const GridKey& key, const FrontierCells& frontier) {
    if (frontier.contains(key)) {
        return true;
    }
    for (const Offset& off : kFaceOffsets) {
        if (frontier.contains(GridKey{key.qx + off.dx, key.qy + off.dy, key.qz + off.dz})) {
            return true;
        }
    }
    return false;
}

std::vector<Orientation> buildSweepDirections(
    const common::IMap3D& map,
    const Position3D& origin,
    const types::LidarConfigData& lidar,
    const FrontierCells& frontier,
    const std::vector<ctpl::detail::ConeTemplate>& templates,
    ctpl::VoxelStamp& stamp) {
    std::vector<ScoredDirection> scored;
    scored.reserve(templates.size());
    const types::MapConfig bounds = map.getMapConfig();
    for (const ctpl::detail::ConeTemplate& cone : templates) {
        if (const auto entry = scoreTemplate(cone, map, origin, lidar, frontier, bounds, stamp)) {
            scored.push_back(*entry);
        }
    }
    std::stable_sort(scored.begin(), scored.end(),
                     [](const ScoredDirection& a, const ScoredDirection& b) {
                         if (a.gain != b.gain) {
                             return a.gain > b.gain;
                         }
                         if (a.direction.horizontal != b.direction.horizontal) {
                             return a.direction.horizontal < b.direction.horizontal;
                         }
                         return a.direction.altitude < b.direction.altitude;
                     });

    stamp.begin(bounds, origin, lidar.z_max);
    std::vector<Orientation> kept;
    kept.reserve(scored.size());
    for (const ScoredDirection& entry : scored) {
        std::size_t added = 0;
        (void)ctpl::walkTemplate(*entry.cone, map, origin, stamp, [&](const Position3D& p) {
            if (volumeGainAllowed(bounds, lidar, entry.direction) ||
                isGainMasked(quantizePosition(p, bounds), frontier)) {
                ++added;
            }
            return true;
        });
        if (added > 0) {
            kept.push_back(entry.direction);
        }
    }
    return kept;
}

std::optional<Orientation> bestTravelScan(
    const common::IMap3D& map,
    const types::DroneState& predicted,
    const Position3D& next_waypoint,
    const types::LidarConfigData& lidar,
    const FrontierCells& frontier,
    const std::vector<ctpl::detail::ConeTemplate>& templates,
    ctpl::VoxelStamp& stamp) {
    if (templates.empty()) {
        return std::nullopt;
    }

    const Position3D delta = next_waypoint - predicted.position;
    const auto dx = mp_units::quantity_cast<common::isq::length>(delta.x);
    const auto dy = mp_units::quantity_cast<common::isq::length>(delta.y);

    std::vector<Orientation> probes;
    probes.reserve(3);
    if (!(dx == 0.0 * cm && dy == 0.0 * cm)) {
        const double heading_deg =
            std::atan2(dy.numerical_value_in(cm), dx.numerical_value_in(cm)) *
            (180.0 / std::numbers::pi);
        probes.emplace_back(heading_deg * deg, 0.0 * deg);
    } else {
        probes.emplace_back(predicted.heading.horizontal, 0.0 * deg);
    }
    probes.emplace_back(0.0 * deg, 90.0 * deg);
    probes.emplace_back(0.0 * deg, -90.0 * deg);

    const types::MapConfig config = map.getMapConfig();
    for (const Orientation& probe : probes) {
        const ctpl::detail::ConeTemplate* cone = closestTemplate(templates, probe);
        if (cone == nullptr) {
            continue;
        }
        if (skipDownwardScan(config, predicted.position, lidar, probe)) {
            continue;
        }
        if (ctpl::nearFieldContainsSolid(*cone, map, predicted.position)) {
            continue;
        }
        stamp.begin(config, predicted.position, lidar.z_max);
        bool hit = false;
        const bool volume = volumeGainAllowed(config, lidar, probe);
        (void)ctpl::walkTemplate(*cone, map, predicted.position, stamp,
                                 [&](const Position3D& p) {
                                     if (volume ||
                                         isGainMasked(quantizePosition(p, config), frontier)) {
                                         hit = true;
                                         return false;
                                     }
                                     return true;
                                 });
        if (hit) {
            return probe;
        }
    }
    return std::nullopt;
}

bool clusterStillFrontier(const common::IMap3D& map, const std::vector<GridKey>& keys) {
    const types::MapConfig config = map.getMapConfig();
    for (const GridKey& key : keys) {
        const Position3D cell = keyToPoint(key, config);
        if (map.atVoxel(cell) != types::VoxelOccupancy::Empty) {
            continue;
        }
        for (const Offset& off : kFaceOffsets) {
            const GridKey neighbour{key.qx + off.dx, key.qy + off.dy, key.qz + off.dz};
            const Position3D nb = keyToPoint(neighbour, config);
            if (map.atVoxel(nb) == types::VoxelOccupancy::Unmapped) {
                return true;
            }
        }
    }
    return false;
}

} // namespace algorithm_207190406_209543255::detail
