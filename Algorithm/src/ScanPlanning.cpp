// ScanPlanning.cpp — masked-gain sweep; outdoor cubes count Unmapped volume
// (small_out: any non-downward; large_out: short lidar only).

#include "ScanPlanning.h"

#include <user_common_207190406_209543255/BeamMath.h>
#include <user_common_207190406_209543255/LidarConstants.h>

#include <algorithm>
#include <array>
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
    const double step = config.resolution.force_numerical_value_in(cm);
    const double ox = config.offset.x.force_numerical_value_in(cm);
    const double oy = config.offset.y.force_numerical_value_in(cm);
    const double oz = config.offset.z.force_numerical_value_in(cm);
    return Position3D{
        (ox + static_cast<double>(key.qx) * step) * x_extent[cm],
        (oy + static_cast<double>(key.qy) * step) * y_extent[cm],
        (oz + static_cast<double>(key.qz) * step) * z_extent[cm],
    };
}

[[nodiscard]] std::array<double, 3> unitVector(const Orientation& dir) {
    const Position3D tip = bm::pointAlongBeam(Position3D{}, dir, 1.0 * cm);
    return {tip.x.force_numerical_value_in(cm), tip.y.force_numerical_value_in(cm),
            tip.z.force_numerical_value_in(cm)};
}

[[nodiscard]] double angularDistance(const Orientation& a, const Orientation& b) {
    const auto ua = unitVector(a);
    const auto ub = unitVector(b);
    const double dot = ua[0] * ub[0] + ua[1] * ub[1] + ua[2] * ub[2];
    return 1.0 - std::clamp(dot, -1.0, 1.0);
}

[[nodiscard]] bool pointingDown(const Orientation& dir) {
    return dir.altitude.force_numerical_value_in(deg) <
           kDownwardScanThreshold.force_numerical_value_in(deg);
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
    const double z = origin.z.force_numerical_value_in(cm);
    const double max_z = config.boundaries.max_height.force_numerical_value_in(cm);
    const double z_min = lidar.z_min.force_numerical_value_in(cm);
    return max_z - z <= z_min + 1e-6;
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
    return lidar.z_max.force_numerical_value_in(cm) <=
           kShortRangeLidarMax.force_numerical_value_in(cm);
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
    const double x = config.boundaries.max_x.force_numerical_value_in(cm) -
                     config.boundaries.min_x.force_numerical_value_in(cm);
    const double y = config.boundaries.max_y.force_numerical_value_in(cm) -
                     config.boundaries.min_y.force_numerical_value_in(cm);
    const double z = config.boundaries.max_height.force_numerical_value_in(cm) -
                     config.boundaries.min_height.force_numerical_value_in(cm);
    return MissionVolumeSpans{x * cm, y * cm, z * cm};
}

bool isOpenVolumeMission(const types::MapConfig& config) {
    const auto s = missionVolumeSpans(config);
    return s.x.force_numerical_value_in(cm) >= kOpenVolumeMinSpan.force_numerical_value_in(cm) &&
           s.y.force_numerical_value_in(cm) >= kOpenVolumeMinSpan.force_numerical_value_in(cm) &&
           s.z.force_numerical_value_in(cm) >= kOpenVolumeMinSpan.force_numerical_value_in(cm);
}

bool isSmallOutdoorMission(const types::MapConfig& config) {
    const auto s = missionVolumeSpans(config);
    const double x = s.x.force_numerical_value_in(cm);
    const double y = s.y.force_numerical_value_in(cm);
    const double z = s.z.force_numerical_value_in(cm);
    const double open_min = kOpenVolumeMinSpan.force_numerical_value_in(cm);
    const double small_max = kSmallOutdoorMaxSpan.force_numerical_value_in(cm);
    return x >= open_min && x < small_max && y >= open_min && y < small_max && z >= open_min &&
           z < small_max;
}

bool isHouseVolumeMission(const types::MapConfig& config) {
    const auto s = missionVolumeSpans(config);
    const double x = s.x.force_numerical_value_in(cm);
    const double y = s.y.force_numerical_value_in(cm);
    const double z = s.z.force_numerical_value_in(cm);
    return x >= kHouseMinXySpan.force_numerical_value_in(cm) &&
           y >= kHouseMinXySpan.force_numerical_value_in(cm) &&
           z >= kHouseMinZSpan.force_numerical_value_in(cm) &&
           z < kHouseMaxZSpan.force_numerical_value_in(cm);
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
                         const double ah = a.direction.horizontal.force_numerical_value_in(deg);
                         const double bh = b.direction.horizontal.force_numerical_value_in(deg);
                         if (ah != bh) {
                             return ah < bh;
                         }
                         return a.direction.altitude.force_numerical_value_in(deg) <
                                b.direction.altitude.force_numerical_value_in(deg);
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

    const double dx = next_waypoint.x.force_numerical_value_in(cm) -
                      predicted.position.x.force_numerical_value_in(cm);
    const double dy = next_waypoint.y.force_numerical_value_in(cm) -
                      predicted.position.y.force_numerical_value_in(cm);

    std::vector<Orientation> probes;
    probes.reserve(3);
    if (!(dx == 0.0 && dy == 0.0)) {
        const double heading_deg = std::atan2(dy, dx) * (180.0 / std::numbers::pi);
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
