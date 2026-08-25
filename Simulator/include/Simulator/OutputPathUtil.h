#pragma once

#include <filesystem>
#include <string>
#include <string_view>

namespace simulator {

inline constexpr std::string_view kOutputMapFilenameSuffix = "_output_map.npy";
inline constexpr std::string_view kErrorLogFilenameSuffix = "_error.log";

/// Derive `<plugin>_run_NNNN_error.log` from `<plugin>_run_NNNN_output_map.npy`.
[[nodiscard]] inline std::filesystem::path errorLogPathFromOutputMap(
    const std::filesystem::path& map_path) {
    if (map_path.empty()) {
        return {};
    }

    const std::string filename = map_path.filename().string();
    std::string new_name;
    if (filename.size() >= kOutputMapFilenameSuffix.size() &&
        filename.compare(filename.size() - kOutputMapFilenameSuffix.size(),
                         kOutputMapFilenameSuffix.size(),
                         kOutputMapFilenameSuffix.data(),
                         kOutputMapFilenameSuffix.size()) == 0) {
        new_name = filename.substr(0, filename.size() - kOutputMapFilenameSuffix.size());
        new_name.append(kErrorLogFilenameSuffix);
    } else {
        new_name = map_path.stem().string();
        new_name.append(kErrorLogFilenameSuffix);
    }
    return map_path.parent_path() / new_name;
}

} // namespace simulator
