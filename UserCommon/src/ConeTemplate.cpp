#include <user_common_207190406_209543255/ConeTemplate.h>

#include <user_common_207190406_209543255/LidarCone.h>
#include <user_common_207190406_209543255/LidarConstants.h>

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace user_common_207190406_209543255::cone_template {

namespace lc = user_common_207190406_209543255::lidar_cone;
namespace bm = user_common_207190406_209543255::beam_math;

// ── VoxelStamp ────────────────────────────────────────────────────────────────

void VoxelStamp::begin(const common::types::MapConfig& config,
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

bool VoxelStamp::mark(const common::types::MapConfig& config,
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
        return true;
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

int VoxelStamp::quant(double value, double origin, double step) {
    if (!(step > 0.0)) {
        return 0;
    }
    return static_cast<int>(std::llround((value - origin) / step));
}

// ── ConeTemplateCache ─────────────────────────────────────────────────────────

const std::vector<detail::ConeTemplate>& ConeTemplateCache::get(
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
    templates_ = build(lidar, resolution);
    res_cm_ = res_cm;
    z_min_ = z_min;
    z_max_ = z_max;
    d_ = d;
    fov_circles_ = lidar.fov_circles;
    built_ = true;
    return templates_;
}

std::vector<detail::ConeTemplate> ConeTemplateCache::build(
    const common::types::LidarConfigData& lidar, PhysicalLength resolution) {
    std::vector<detail::ConeTemplate> out;
    if (resolution <= 0.0 * cm) {
        return out;
    }
    const double alpha = lc::coneHalfAngleRad(lidar);
    const auto dirs =
        lc::fibonacciSphereOrientations(lc::directionCountForHalfAngle(alpha));
    const PhysicalLength step = kConeWalkResolutionFactor * resolution;
    const PhysicalLength z_max = lidar.z_max;
    const PhysicalLength z_min = lidar.z_min;
    std::size_t near_field = 0;
    if (step > 0.0 * cm) {
        for (PhysicalLength dist = step; dist < z_min - 1e-9 * cm; dist += step) {
            ++near_field;
        }
    }
    out.reserve(dirs.size());
    for (const Orientation& dir : dirs) {
        detail::ConeTemplate cone;
        cone.direction = dir;
        cone.step = step;
        cone.near_field_samples = near_field;
        lc::forEachConeBeam(lidar, dir, [&](const Orientation& beam) {
            // Compute unit direction vector by taking a 1-cm step along the beam.
            const Position3D unit = bm::pointAlongBeam(
                Position3D{}, bm::normalizeOrientation(beam), 1.0 * cm);
            detail::BeamRun run;
            run.ux = unit.x.numerical_value_in(cm);
            run.uy = unit.y.numerical_value_in(cm);
            run.uz = unit.z.numerical_value_in(cm);
            run.sample_count = 0;
            if (step > 0.0 * cm) {
                for (PhysicalLength dist = step; dist <= z_max + 1e-9 * cm; dist += step) {
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

} // namespace user_common_207190406_209543255::cone_template
