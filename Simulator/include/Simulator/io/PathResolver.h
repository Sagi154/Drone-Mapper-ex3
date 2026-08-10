#pragma once

#include <filesystem>

namespace simulator::io {

/// Resolve a relative config file path against a base directory.
/// Absolute paths are returned unchanged.
[[nodiscard]] std::filesystem::path resolveConfigPath(const std::filesystem::path& base_dir,
                                                       const std::filesystem::path& config_path);

/// Resolve a hidden-map .npy filename from a simulation_config YAML.
/// Search order: as-is → simulation YAML parent dir → that dir's parent → CWD.
[[nodiscard]] std::filesystem::path resolveMapFilename(
    const std::filesystem::path& map_filename,
    const std::filesystem::path& simulation_config_path = {});

} // namespace simulator::io
