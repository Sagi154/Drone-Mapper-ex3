#pragma once

#include <Common/IMutableMap3D.h>

#include <filesystem>
#include <memory>

class NpyArray;

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
    [[nodiscard]] common::types::VoxelOccupancy atVoxel(const common::Position3D& pos) const override;
    [[nodiscard]] common::types::MapConfig getMapConfig() const override;
    [[nodiscard]] bool isInBounds(const common::Position3D& pos) const override;

    void set(const common::Position3D& pos, common::types::VoxelOccupancy value) override;
    void save(const std::filesystem::path& path) const override;

private:
    struct Storage;
    std::shared_ptr<Storage> storage_;

    explicit Map3DImpl(std::shared_ptr<Storage> storage);

    friend Map3DImpl makeMap3D(std::shared_ptr<NpyArray> map, MapRole role,
                               const common::types::MapConfig& config);
    friend Map3DImpl makeMap3D(std::shared_ptr<NpyArray> map, MapRole role);
};

} // namespace simulator
