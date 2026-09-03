#pragma once

// Precomputed lidar-cone geometry. A walk is origin + k*step*u, no trig, no hashing.
// Voxel set equals lidar_cone::countUnresolvedVoxels, verified by test_cone_template.

#include <Common/IMap3D.h>
#include <Common/Units.h>
#include <Common/types/LidarTypes.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace user_common_207190406_209543255::cone_template {

using common::Orientation;
using common::PhysicalLength;
using common::Position3D;
using common::cm;
using common::x_extent;
using common::y_extent;
using common::z_extent;

// ── detail namespace: precomputed structures ───────────────────────────────────

namespace detail {

struct BeamRun {
    double ux = 0.0;
    double uy = 0.0;
    double uz = 0.0;
    std::size_t sample_count = 0;
};

struct ConeTemplate {
    Orientation direction{};
    std::vector<BeamRun> beams{};
    PhysicalLength step{};
    std::size_t near_field_samples = 0;
};

} // namespace detail

// ── VoxelStamp: per-beam deduplication helper ─────────────────────────────────

class VoxelStamp {
public:
    void begin(const common::types::MapConfig& config,
               const Position3D& origin,
               PhysicalLength z_max);

    [[nodiscard]] bool mark(const common::types::MapConfig& config,
                            const Position3D& p);

private:
    [[nodiscard]] static int quant(double value, double origin, double step);

    std::vector<std::uint32_t> cells_{};
    std::uint32_t generation_ = 0;
    int origin_qx_ = 0;
    int origin_qy_ = 0;
    int origin_qz_ = 0;
    int radius_ = 0;
    int span_ = 0;
};

// ── ConeTemplateCache: builds and caches templates per lidar+resolution ────────

class ConeTemplateCache {
public:
    [[nodiscard]] const std::vector<detail::ConeTemplate>& get(
        const common::types::LidarConfigData& lidar,
        PhysicalLength resolution);

private:
    [[nodiscard]] static std::vector<detail::ConeTemplate> build(
        const common::types::LidarConfigData& lidar, PhysicalLength resolution);

    std::vector<detail::ConeTemplate> templates_{};
    bool built_ = false;
    double res_cm_ = 0.0;
    double z_min_ = 0.0;
    double z_max_ = 0.0;
    double d_ = 0.0;
    std::size_t fov_circles_ = 0;
};

// ── walkTemplate: stamp-deduped walk over precomputed beam runs ───────────────

template <typename Fn>
inline std::size_t walkTemplate(const detail::ConeTemplate& cone,
                                const common::IMap3D& map,
                                const Position3D& origin,
                                VoxelStamp& stamp,
                                Fn&& on_unresolved) {
    const auto config = map.getMapConfig();
    std::size_t added = 0;
    for (const detail::BeamRun& beam : cone.beams) {
        for (std::size_t i = 1; i <= beam.sample_count; ++i) {
            const auto dist = static_cast<double>(i) * cone.step;
            const Position3D p{
                origin.x + mp_units::quantity_cast<x_extent>(beam.ux * dist),
                origin.y + mp_units::quantity_cast<y_extent>(beam.uy * dist),
                origin.z + mp_units::quantity_cast<z_extent>(beam.uz * dist),
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

// ── nearFieldContainsSolid: checks near-field region for obstacles ─────────────

[[nodiscard]] inline bool nearFieldContainsSolid(const detail::ConeTemplate& cone,
                                                 const common::IMap3D& map,
                                                 const Position3D& origin) {
    const std::size_t n = cone.near_field_samples;
    if (n == 0) {
        return false;
    }
    for (const detail::BeamRun& beam : cone.beams) {
        const std::size_t limit = std::min(n, beam.sample_count);
        for (std::size_t i = 1; i <= limit; ++i) {
            const auto dist = static_cast<double>(i) * cone.step;
            const Position3D p{
                origin.x + mp_units::quantity_cast<x_extent>(beam.ux * dist),
                origin.y + mp_units::quantity_cast<y_extent>(beam.uy * dist),
                origin.z + mp_units::quantity_cast<z_extent>(beam.uz * dist),
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
