#pragma once

// Precomputed lidar-cone geometry. A walk is origin + k*step*u, no trig, no hashing.
// Voxel set equals lidar_cone::countUnresolvedVoxels, verified by test_cone_template.

#include <user_common_207190406_209543255/LidarCone.h>

#include <Common/IMap3D.h>
#include <Common/Units.h>
#include <Common/types/LidarTypes.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace user_common_207190406_209543255::cone_template {

namespace lc = user_common_207190406_209543255::lidar_cone;
namespace bm = user_common_207190406_209543255::beam_math;
using common::Orientation;
using common::PhysicalLength;
using common::Position3D;
using common::cm;
using common::x_extent;
using common::y_extent;
using common::z_extent;

struct BeamRun {
    double ux = 0.0;
    double uy = 0.0;
    double uz = 0.0;
    std::size_t sample_count = 0;
};

struct ConeTemplate {
    Orientation direction{};
    std::vector<BeamRun> beams{};
    double step_cm = 0.0;
    std::size_t near_field_samples = 0;
};

class VoxelStamp {
public:
    void begin(const common::types::MapConfig& config,
               const Position3D& origin,
               PhysicalLength z_max) {
        const double step = config.resolution.force_numerical_value_in(cm);
        const double range = z_max.force_numerical_value_in(cm);
        const int radius =
            (step > 0.0) ? static_cast<int>(std::ceil(range / step) + 2.0) : 1;
        const int span = 2 * radius + 1;
        const std::size_t needed =
            static_cast<std::size_t>(span) * static_cast<std::size_t>(span) *
            static_cast<std::size_t>(span);
        if (cells_.size() < needed) {
            cells_.assign(needed, 0);
        }
        radius_ = radius;
        span_ = span;
        origin_qx_ = quant(origin.x.force_numerical_value_in(cm),
                           config.offset.x.force_numerical_value_in(cm), step);
        origin_qy_ = quant(origin.y.force_numerical_value_in(cm),
                           config.offset.y.force_numerical_value_in(cm), step);
        origin_qz_ = quant(origin.z.force_numerical_value_in(cm),
                           config.offset.z.force_numerical_value_in(cm), step);
        if (++generation_ == 0) {
            std::fill(cells_.begin(), cells_.end(), 0);
            generation_ = 1;
        }
    }

    [[nodiscard]] bool mark(const common::types::MapConfig& config,
                            const Position3D& p) {
        const double step = config.resolution.force_numerical_value_in(cm);
        const int qx = quant(p.x.force_numerical_value_in(cm),
                             config.offset.x.force_numerical_value_in(cm), step);
        const int qy = quant(p.y.force_numerical_value_in(cm),
                             config.offset.y.force_numerical_value_in(cm), step);
        const int qz = quant(p.z.force_numerical_value_in(cm),
                             config.offset.z.force_numerical_value_in(cm), step);
        const int dx = qx - origin_qx_ + radius_;
        const int dy = qy - origin_qy_ + radius_;
        const int dz = qz - origin_qz_ + radius_;
        if (dx < 0 || dy < 0 || dz < 0 || dx >= span_ || dy >= span_ ||
            dz >= span_) {
            return true;  // outside the cone box: treat as unseen, still count
        }
        const std::size_t idx =
            (static_cast<std::size_t>(dx) * static_cast<std::size_t>(span_) +
             static_cast<std::size_t>(dy)) *
                static_cast<std::size_t>(span_) +
            static_cast<std::size_t>(dz);
        if (cells_[idx] == generation_) {
            return false;
        }
        cells_[idx] = generation_;
        return true;
    }

private:
    [[nodiscard]] static int quant(double value, double origin, double step) {
        if (!(step > 0.0)) {
            return 0;
        }
        return static_cast<int>(std::llround((value - origin) / step));
    }

    std::vector<std::uint32_t> cells_{};
    std::uint32_t generation_ = 0;
    int origin_qx_ = 0;
    int origin_qy_ = 0;
    int origin_qz_ = 0;
    int radius_ = 0;
    int span_ = 0;
};

class ConeTemplateCache {
public:
    [[nodiscard]] const std::vector<ConeTemplate>& get(
        const common::types::LidarConfigData& lidar,
        PhysicalLength resolution) {
        const double res_cm = resolution.force_numerical_value_in(cm);
        const double z_min = lidar.z_min.force_numerical_value_in(cm);
        const double z_max = lidar.z_max.force_numerical_value_in(cm);
        const double d = lidar.d.force_numerical_value_in(cm);
        if (built_ && res_cm == res_cm_ && z_min == z_min_ && z_max == z_max_ &&
            d == d_ && lidar.fov_circles == fov_circles_) {
            return templates_;
        }
        templates_ = build(lidar, res_cm);
        res_cm_ = res_cm;
        z_min_ = z_min;
        z_max_ = z_max;
        d_ = d;
        fov_circles_ = lidar.fov_circles;
        built_ = true;
        return templates_;
    }

private:
    [[nodiscard]] static std::vector<ConeTemplate> build(
        const common::types::LidarConfigData& lidar, double res_cm) {
        std::vector<ConeTemplate> out;
        if (!(res_cm > 0.0)) {
            return out;
        }
        const double alpha = lc::coneHalfAngleRad(lidar);
        const auto dirs =
            lc::fibonacciSphereOrientations(lc::directionCountForHalfAngle(alpha));
        const double step_cm = 0.5 * res_cm;
        const double z_max = lidar.z_max.force_numerical_value_in(cm);
        const double z_min = lidar.z_min.force_numerical_value_in(cm);
        std::size_t near_field = 0;
        if (step_cm > 0.0) {
            for (double dist = step_cm; dist < z_min - 1e-9; dist += step_cm) {
                ++near_field;
            }
        }
        out.reserve(dirs.size());
        for (const Orientation& dir : dirs) {
            ConeTemplate cone;
            cone.direction = dir;
            cone.step_cm = step_cm;
            cone.near_field_samples = near_field;
            lc::forEachConeBeam(lidar, dir, [&](const Orientation& beam) {
                const Position3D unit = bm::pointAlongBeam(
                    Position3D{}, bm::normalizeOrientation(beam), 1.0 * cm);
                BeamRun run;
                run.ux = unit.x.force_numerical_value_in(cm);
                run.uy = unit.y.force_numerical_value_in(cm);
                run.uz = unit.z.force_numerical_value_in(cm);
                run.sample_count = 0;
                if (step_cm > 0.0) {
                    for (double dist = step_cm; dist <= z_max + 1e-9;
                         dist += step_cm) {
                        ++run.sample_count;
                    }
                }
                cone.beams.push_back(run);
                return true;
            });
            out.push_back(std::move(cone));
        }
        return out;
    }

    std::vector<ConeTemplate> templates_{};
    bool built_ = false;
    double res_cm_ = 0.0;
    double z_min_ = 0.0;
    double z_max_ = 0.0;
    double d_ = 0.0;
    std::size_t fov_circles_ = 0;
};

template <typename Fn>
inline std::size_t walkTemplate(const ConeTemplate& cone,
                                const common::IMap3D& map,
                                const Position3D& origin,
                                VoxelStamp& stamp,
                                Fn&& on_unresolved) {
    const auto config = map.getMapConfig();
    const double ox = origin.x.force_numerical_value_in(cm);
    const double oy = origin.y.force_numerical_value_in(cm);
    const double oz = origin.z.force_numerical_value_in(cm);
    std::size_t added = 0;
    for (const BeamRun& beam : cone.beams) {
        for (std::size_t i = 1; i <= beam.sample_count; ++i) {
            const double dist = static_cast<double>(i) * cone.step_cm;
            const Position3D p{
                (ox + dist * beam.ux) * x_extent[cm],
                (oy + dist * beam.uy) * y_extent[cm],
                (oz + dist * beam.uz) * z_extent[cm],
            };
            const auto occ = map.atVoxel(p);
            if (occ == common::types::VoxelOccupancy::Occupied ||
                occ == common::types::VoxelOccupancy::OutOfBounds) {
                break;
            }
            if (occ != common::types::VoxelOccupancy::Unmapped) {
                continue;
            }
            if (!stamp.mark(config, p)) {
                continue;
            }
            ++added;
            if (!on_unresolved(p)) {
                return added;
            }
        }
    }
    return added;
}

[[nodiscard]] inline bool nearFieldContainsSolid(const ConeTemplate& cone,
                                                 const common::IMap3D& map,
                                                 const Position3D& origin) {
    const double ox = origin.x.force_numerical_value_in(cm);
    const double oy = origin.y.force_numerical_value_in(cm);
    const double oz = origin.z.force_numerical_value_in(cm);
    const std::size_t n = cone.near_field_samples;
    if (n == 0) {
        return false;
    }
    for (const BeamRun& beam : cone.beams) {
        const std::size_t limit = std::min(n, beam.sample_count);
        for (std::size_t i = 1; i <= limit; ++i) {
            const double dist = static_cast<double>(i) * cone.step_cm;
            const Position3D p{
                (ox + dist * beam.ux) * x_extent[cm],
                (oy + dist * beam.uy) * y_extent[cm],
                (oz + dist * beam.uz) * z_extent[cm],
            };
            const auto occ = map.atVoxel(p);
            if (occ == common::types::VoxelOccupancy::Occupied) {
                return true;
            }
        }
    }
    return false;
}

} // namespace user_common_207190406_209543255::cone_template
