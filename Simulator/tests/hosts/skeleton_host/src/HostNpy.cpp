#include "HostNpy.h"

#include "HostUnits.h"

#include <TinyNPY.h>

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace skeleton_host {

namespace {

std::size_t axisCount(double min_cm, double max_cm, double res_cm) {
    if (!(res_cm > 0.0)) {
        throw std::runtime_error("map resolution must be positive");
    }
    const double extent = max_cm - min_cm;
    if (!(extent > 0.0)) {
        throw std::runtime_error("map axis extent must be positive");
    }
    const auto n = static_cast<long long>(std::llround(extent / res_cm));
    if (n <= 0) {
        throw std::runtime_error("map axis produced a non-positive voxel count");
    }
    return static_cast<std::size_t>(n);
}

}  // namespace

HostMap3D loadHiddenMapFromNpy(const std::filesystem::path& npy_path,
                               const simulator::types::SimulationConfigData& simulation) {
    NpyArray arr;
    const LPCSTR err = arr.LoadNPY(npy_path.string());
    if (err != nullptr) {
        throw std::runtime_error(std::string("TinyNPY LoadNPY failed: ") + err + " (" +
                                 npy_path.string() + ")");
    }
    const auto& shape = arr.Shape();
    if (shape.size() != 3) {
        throw std::runtime_error("map npy must be 3-dimensional");
    }
    if (arr.SizeValueBytes() != 1) {
        throw std::runtime_error("map npy voxels must be int8 or uint8 (1 byte)");
    }
    const std::size_t nx = shape[0];
    const std::size_t ny = shape[1];
    const std::size_t nz = shape[2];
    const std::uint8_t* data = arr.Data<std::uint8_t>();
    if (data == nullptr) {
        throw std::runtime_error("map npy has no payload");
    }
    std::vector<std::uint8_t> bytes(data, data + arr.NumValue());

    const double res = cm_of(simulation.map_resolution);
    const double ox = cm_of(simulation.map_offset.x);
    const double oy = cm_of(simulation.map_offset.y);
    const double oz = cm_of(simulation.map_offset.z);

    common::types::MapConfig cfg;
    cfg.resolution = simulation.map_resolution;
    cfg.offset = simulation.map_offset;
    // npy (0,0,0) lives at global 0; API/mission frame is global - offset.
    cfg.boundaries.min_x = x_cm(0.0 - ox);
    cfg.boundaries.max_x = x_cm(static_cast<double>(nx) * res - ox);
    cfg.boundaries.min_y = y_cm(0.0 - oy);
    cfg.boundaries.max_y = y_cm(static_cast<double>(ny) * res - oy);
    cfg.boundaries.min_height = z_cm(0.0 - oz);
    cfg.boundaries.max_height = z_cm(static_cast<double>(nz) * res - oz);

    return HostMap3D::fromNpyVoxels(std::move(cfg), nx, ny, nz, bytes);
}

HostMap3D makeEmptyOutputMap(const common::types::MissionConfigData& mission,
                             const simulator::types::SimulationConfigData& simulation) {
    const double res = cm_of(simulation.map_resolution);
    const double min_x = cm_of(mission.mission_bounds.min_x);
    const double max_x = cm_of(mission.mission_bounds.max_x);
    const double min_y = cm_of(mission.mission_bounds.min_y);
    const double max_y = cm_of(mission.mission_bounds.max_y);
    const double min_z = cm_of(mission.mission_bounds.min_height);
    const double max_z = cm_of(mission.mission_bounds.max_height);

    const std::size_t nx = axisCount(min_x, max_x, res);
    const std::size_t ny = axisCount(min_y, max_y, res);
    const std::size_t nz = axisCount(min_z, max_z, res);

    common::types::MapConfig cfg;
    cfg.boundaries = mission.mission_bounds;
    cfg.offset = simulation.map_offset;
    cfg.resolution = simulation.map_resolution;
    return HostMap3D(std::move(cfg), nx, ny, nz, common::types::VoxelOccupancy::Unmapped);
}

}  // namespace skeleton_host
