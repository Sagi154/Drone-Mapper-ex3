#pragma once

#include <user_common_207190406_209543255/ConfigParseResult.h>
#include <user_common_207190406_209543255/IRunErrorLog.h>

#include <Common/Types.h>
#include <Common/types/MapTypes.h>

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <optional>
#include <string>

namespace simulator::io::detail {

namespace UC = user_common_207190406_209543255;

[[nodiscard]] std::optional<YAML::Node> loadYamlFile(const std::filesystem::path& path,
                                                      user_common_207190406_209543255::IRunErrorLog& log,
                                                      const std::string& context);

[[nodiscard]] YAML::Node configRoot(const YAML::Node& root, const char* wrapper_key);

void logRecoverable(user_common_207190406_209543255::IRunErrorLog& log,
                    const std::string& code,
                    const std::string& message);

[[nodiscard]] std::optional<double>                   readScalarDouble(const YAML::Node& node, const char* key);
[[nodiscard]] std::optional<common::PhysicalLength>   readLengthCm(const YAML::Node& node, const char* key);
[[nodiscard]] std::optional<common::HorizontalAngle>  readHorizontalAngleDeg(const YAML::Node& node, const char* key);
[[nodiscard]] std::optional<common::Position3D>       readPosition3D(const YAML::Node& node);
[[nodiscard]] std::optional<common::Position3D>       readMapOffset(const YAML::Node& node);
[[nodiscard]] std::optional<common::types::MappingBounds> readMissionBoundaries(const YAML::Node& node);

template <typename T, typename Fill>
UC::ConfigParseResult<T> parseWrappedConfig(const std::filesystem::path& path,
                                            UC::IRunErrorLog& log,
                                            const char* wrapper_key,
                                            const char* missing_code,
                                            Fill&& fill) {
    UC::ConfigParseResult<T> result{};
    const auto root = loadYamlFile(path, log, wrapper_key);
    if (!root.has_value()) {
        result.errors.push_back({missing_code, path.string()});
        return result;
    }
    fill(configRoot(*root, wrapper_key), result);
    return result;
}

} // namespace simulator::io::detail
