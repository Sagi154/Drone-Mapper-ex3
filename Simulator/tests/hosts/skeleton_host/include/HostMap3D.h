#pragma once

#include <Common/IMutableMap3D.h>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <vector>

namespace skeleton_host {

struct HostVoxelCounts {
    std::size_t empty = 0;
    std::size_t occupied = 0;
    std::size_t unmapped = 0;
};

// Grid-backed IMutableMap3D. Positions at the public API are mission-frame.
// Voxel (ix,iy,iz) covers the half-open cube
//   [min + i*res, min + (i+1)*res)
// in that frame. See ASSUMPTIONS.md.
class HostMap3D : public common::IMutableMap3D {
public:
    HostMap3D(common::types::MapConfig config,
              std::size_t nx,
              std::size_t ny,
              std::size_t nz,
              common::types::VoxelOccupancy fill);

    HostMap3D(const HostMap3D&) = delete;
    HostMap3D& operator=(const HostMap3D&) = delete;
    HostMap3D(HostMap3D&&) noexcept = default;
    HostMap3D& operator=(HostMap3D&&) noexcept = default;
    ~HostMap3D() override = default;

    [[nodiscard]] common::types::VoxelOccupancy atVoxel(
        const common::Position3D& pos) const override;
    [[nodiscard]] common::types::MapConfig getMapConfig() const override;
    [[nodiscard]] bool isInBounds(const common::Position3D& pos) const override;
    void set(const common::Position3D& pos, common::types::VoxelOccupancy value) override;
    void save(const std::filesystem::path& path) const override;

    [[nodiscard]] std::size_t nx() const { return nx_; }
    [[nodiscard]] std::size_t ny() const { return ny_; }
    [[nodiscard]] std::size_t nz() const { return nz_; }
    [[nodiscard]] HostVoxelCounts countVoxels() const;

    // Direct index access used by lidar/movement (avoids reconstructing Position3D).
    [[nodiscard]] common::types::VoxelOccupancy atIndex(std::size_t ix,
                                                        std::size_t iy,
                                                        std::size_t iz) const;
    [[nodiscard]] bool indexInRange(int ix, int iy, int iz) const;
    [[nodiscard]] bool worldToIndex(double x_cm,
                                    double y_cm,
                                    double z_cm,
                                    int& ix,
                                    int& iy,
                                    int& iz) const;

    [[nodiscard]] double minXcm() const { return min_x_; }
    [[nodiscard]] double minYcm() const { return min_y_; }
    [[nodiscard]] double minZcm() const { return min_z_; }
    [[nodiscard]] double maxXcm() const { return max_x_; }
    [[nodiscard]] double maxYcm() const { return max_y_; }
    [[nodiscard]] double maxZcm() const { return max_z_; }
    [[nodiscard]] double resolutionCm() const { return res_; }

    // Occupied iff the npy value is not 0 (air). int8 and uint8 both work.
    static HostMap3D fromNpyVoxels(common::types::MapConfig config,
                                   std::size_t nx,
                                   std::size_t ny,
                                   std::size_t nz,
                                   const std::vector<std::uint8_t>& bytes);

private:
    [[nodiscard]] std::size_t flat(std::size_t ix, std::size_t iy, std::size_t iz) const;

    common::types::MapConfig config_{};
    std::size_t nx_ = 0;
    std::size_t ny_ = 0;
    std::size_t nz_ = 0;
    double min_x_ = 0;
    double min_y_ = 0;
    double min_z_ = 0;
    double max_x_ = 0;
    double max_y_ = 0;
    double max_z_ = 0;
    double res_ = 1;
    std::vector<common::types::VoxelOccupancy> voxels_{};
};

}  // namespace skeleton_host
