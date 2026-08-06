#include <Simulator/io/YamlConfigParsers.h>

#include "YamlParseUtil.hpp"

namespace simulator::io {

namespace UC = UserCommon_207190406_209543255;

UC::ConfigParseResult<common::types::DroneConfigData>
parseDroneConfig(const std::filesystem::path& path, UC::IRunErrorLog& log) {
    UC::ConfigParseResult<common::types::DroneConfigData> result{};

    const auto root = detail::loadYamlFile(path, log, "[drone_config]");
    if (!root.has_value()) {
        result.errors.push_back({"CONFIG_FILE_NOT_FOUND", path.string()});
        return result;
    }

    const YAML::Node node = detail::configRoot(*root, "drone_config");

    if (const auto v = detail::readLengthCm(node, "dimensions_cm")) {
        result.value.radius = *v / 2.0;
    }
    if (const auto v = detail::readHorizontalAngleDeg(node, "max_rotate_deg")) {
        result.value.max_rotate = *v;
    }
    if (const auto v = detail::readLengthCm(node, "max_advance_cm")) {
        result.value.max_advance = *v;
    }
    if (const auto v = detail::readLengthCm(node, "max_elevate_cm")) {
        result.value.max_elevate = *v;
    }

    result.ok = true;
    return result;
}

} // namespace simulator::io
