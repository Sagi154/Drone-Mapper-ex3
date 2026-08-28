#include "HostMap3D.h"

#include "HostUnits.h"

#include <TinyNPY.h>

#include <cmath>
#include <stdexcept>
#include <string>

namespace skeleton_host {

namespace {

int floor_div(double value, double res) {
    return static_cast<int>(std::floor(value / res));
}

}  // namespace

HostMap3D::HostMap3D(common::types::MapConfig config,
                     std::size_t nx,
                     std::size_t ny,
                     std::size_t nz,
                     common::types::VoxelOccupancy fill)
    : config_(std::move(config)),
      nx_(nx),
      ny_(ny),
      nz_(nz),
      min_x_(cm_of(config_.boundaries.min_x)),
      min_y_(cm_of(config_.boundaries.min_y)),
      min_z_(cm_of(config_.boundaries.min_height)),
      max_x_(cm_of(config_.boundaries.max_x)),
      max_y_(cm_of(config_.boundaries.max_y)),
      max_z_(cm_of(config_.boundaries.max_height)),
      res_(cm_of(config_.resolution)),
      voxels_(nx * ny * nz, fill) {
    if (nx_ == 0 || ny_ == 0 || nz_ == 0) {
        throw std::runtime_error("HostMap3D: grid dimension is zero");
    }
    if (!(res_ > 0.0)) {
        throw std::runtime_error("HostMap3D: resolution must be positive");
    }
}

HostMap3D HostMap3D::fromNpyVoxels(common::types::MapConfig config,
                                   std::size_t nx,
                                   std::size_t ny,
                                   std::size_t nz,
                                   const std::vector<std::uint8_t>& bytes) {
    if (bytes.size() != nx * ny * nz) {
        throw std::runtime_error("HostMap3D: npy voxel count does not match shape");
    }
    HostMap3D map(std::move(config), nx, ny, nz, common::types::VoxelOccupancy::Empty);
    for (std::size_t i = 0; i < bytes.size(); ++i) {
        map.voxels_[i] = (bytes[i] == 0) ? common::types::VoxelOccupancy::Empty
                                         : common::types::VoxelOccupancy::Occupied;
    }
    return map;
}

std::size_t HostMap3D::flat(std::size_t ix, std::size_t iy, std::size_t iz) const {
    return (ix * ny_ + iy) * nz_ + iz;
}

bool HostMap3D::indexInRange(int ix, int iy, int iz) const {
    return ix >= 0 && iy >= 0 && iz >= 0 &&
           static_cast<std::size_t>(ix) < nx_ &&
           static_cast<std::size_t>(iy) < ny_ &&
           static_cast<std::size_t>(iz) < nz_;
}

bool HostMap3D::worldToIndex(double x_cm, double y_cm, double z_cm, int& ix, int& iy, int& iz) const {
    if (!(x_cm >= min_x_ && x_cm < max_x_ && y_cm >= min_y_ && y_cm < max_y_ && z_cm >= min_z_ &&
          z_cm < max_z_)) {
        return false;
    }
    ix = floor_div(x_cm - min_x_, res_);
    iy = floor_div(y_cm - min_y_, res_);
    iz = floor_div(z_cm - min_z_, res_);
    return indexInRange(ix, iy, iz);
}

common::types::VoxelOccupancy HostMap3D::atIndex(std::size_t ix,
                                                 std::size_t iy,
                                                 std::size_t iz) const {
    return voxels_[flat(ix, iy, iz)];
}

common::types::VoxelOccupancy HostMap3D::atVoxel(const common::Position3D& pos) const {
    int ix = 0;
    int iy = 0;
    int iz = 0;
    if (!worldToIndex(cm_of(pos.x), cm_of(pos.y), cm_of(pos.z), ix, iy, iz)) {
        return common::types::VoxelOccupancy::OutOfBounds;
    }
    return atIndex(static_cast<std::size_t>(ix), static_cast<std::size_t>(iy),
                   static_cast<std::size_t>(iz));
}

common::types::MapConfig HostMap3D::getMapConfig() const {
    return config_;
}

bool HostMap3D::isInBounds(const common::Position3D& pos) const {
    int ix = 0;
    int iy = 0;
    int iz = 0;
    return worldToIndex(cm_of(pos.x), cm_of(pos.y), cm_of(pos.z), ix, iy, iz);
}

void HostMap3D::set(const common::Position3D& pos, common::types::VoxelOccupancy value) {
    int ix = 0;
    int iy = 0;
    int iz = 0;
    if (!worldToIndex(cm_of(pos.x), cm_of(pos.y), cm_of(pos.z), ix, iy, iz)) {
        return;
    }
    voxels_[flat(static_cast<std::size_t>(ix), static_cast<std::size_t>(iy),
                 static_cast<std::size_t>(iz))] = value;
}

void HostMap3D::save(const std::filesystem::path& path) const {
    std::vector<std::int8_t> bytes;
    bytes.reserve(voxels_.size());
    for (const auto occ : voxels_) {
        bytes.push_back(static_cast<std::int8_t>(occ));
    }
    const LPCSTR err = NpyArray::SaveNPY<std::int8_t>(
        path.string(), bytes, {nx_, ny_, nz_});
    if (err != nullptr) {
        throw std::runtime_error(std::string("HostMap3D::save failed: ") + err);
    }
}

HostVoxelCounts HostMap3D::countVoxels() const {
    HostVoxelCounts counts{};
    for (const auto occ : voxels_) {
        switch (occ) {
            case common::types::VoxelOccupancy::Empty:
                ++counts.empty;
                break;
            case common::types::VoxelOccupancy::Occupied:
                ++counts.occupied;
                break;
            case common::types::VoxelOccupancy::Unmapped:
            case common::types::VoxelOccupancy::PotentiallyOccupied:
                ++counts.unmapped;
                break;
            case common::types::VoxelOccupancy::OutOfBounds:
                break;
        }
    }
    return counts;
}

}  // namespace skeleton_host
