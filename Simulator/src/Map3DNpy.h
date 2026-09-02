#pragma once

#include <Simulator/Map3DImpl.h>
#include <TinyNPY.h>
#include <memory>

namespace simulator {

[[nodiscard]] Map3DImpl makeMap3D(std::shared_ptr<NpyArray> map, MapRole role,
                                  const common::types::MapConfig& config);
[[nodiscard]] Map3DImpl makeMap3D(std::shared_ptr<NpyArray> map, MapRole role);

} // namespace simulator
