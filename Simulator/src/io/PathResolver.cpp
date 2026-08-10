#include <Simulator/io/PathResolver.h>

#include <vector>

namespace simulator::io {

namespace {

[[nodiscard]] std::filesystem::path absoluteIfExists(const std::filesystem::path& candidate) {
    std::error_code ec;
    if (!std::filesystem::exists(candidate, ec) || ec) {
        return {};
    }
    const std::filesystem::path absolute = std::filesystem::absolute(candidate, ec);
    return ec ? candidate : absolute;
}

} // namespace

std::filesystem::path resolveConfigPath(const std::filesystem::path& base_dir,
                                         const std::filesystem::path& config_path) {
    if (config_path.is_absolute()) {
        return config_path;
    }
    return base_dir / config_path;
}

std::filesystem::path resolveMapFilename(const std::filesystem::path& map_filename,
                                          const std::filesystem::path& simulation_config_path) {
    if (map_filename.empty()) {
        return map_filename;
    }

    std::vector<std::filesystem::path> candidates;
    candidates.push_back(map_filename);

    if (!simulation_config_path.empty()) {
        const std::filesystem::path config_dir = simulation_config_path.parent_path();
        if (!config_dir.empty()) {
            candidates.push_back(config_dir / map_filename);
            const std::filesystem::path config_root = config_dir.parent_path();
            if (!config_root.empty()) {
                candidates.push_back(config_root / map_filename);
            }
        }
    }

    if (!map_filename.is_absolute()) {
        candidates.push_back(std::filesystem::current_path() / map_filename);
    }

    for (const std::filesystem::path& candidate : candidates) {
        if (const std::filesystem::path resolved = absoluteIfExists(candidate); !resolved.empty()) {
            return resolved;
        }
    }
    return map_filename;
}

} // namespace simulator::io
