#pragma once

#include <TinyNPY.h>

#include <Common/IMutableMap3D.h>

#include <filesystem>
#include <memory>

namespace simulator {

/// Role of a Map3DImpl instance — decides how stored bytes are interpreted.
///
/// Hidden:  The map was loaded from an instructor .npy file; any stored value
///          >= 1 is Occupied regardless of dtype or magnitude.
/// Output:  The map is our own int8 output map; the full VoxelOccupancy enum
///          is stored verbatim.
enum class MapRole { Hidden, Output };

class Map3DImpl final : public common::IMutableMap3D {
public:
    /// Construct with an explicit MapConfig (boundaries, offset, resolution).
    /// If boundaries are all-zero they are derived from the array shape.
    Map3DImpl(std::shared_ptr<NpyArray> map_ptr,
              MapRole role,
              common::types::MapConfig map_config);

    /// Convenience overload: role only, config is derived from array shape.
    Map3DImpl(std::shared_ptr<NpyArray> map_ptr, MapRole role);

    [[nodiscard]] common::types::VoxelOccupancy atVoxel(const common::Position3D& pos) const override;
    [[nodiscard]] common::types::MapConfig getMapConfig() const override;
    [[nodiscard]] bool isInBounds(const common::Position3D& pos) const override;

    void set(const common::Position3D& pos, common::types::VoxelOccupancy value) override;
    void save(const std::filesystem::path& path) const override;

private:
    std::shared_ptr<NpyArray> map_;
    MapRole role_;
    common::types::MapConfig config_;
};

} // namespace simulator
