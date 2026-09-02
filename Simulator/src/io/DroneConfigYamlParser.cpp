#include <Simulator/io/YamlConfigParsers.h>

#include "YamlParseUtil.hpp"

namespace simulator::io {

namespace UC = user_common_207190406_209543255;

UC::ConfigParseResult<common::types::DroneConfigData>
parseDroneConfig(const std::filesystem::path& path, UC::IRunErrorLog& log) {
    return detail::parseWrappedConfig<common::types::DroneConfigData>(
        path, log, "drone_config", "CONFIG_FILE_NOT_FOUND",
        [](const YAML::Node& node, UC::ConfigParseResult<common::types::DroneConfigData>& result) {
            if (const auto v = detail::readLengthCm(node, "dimensions_cm")) {
                result.value.radius = *v / 2.0;
            } else {
                result.errors.push_back({"CONFIG_MISSING_FIELD",
                                          "[drone_config] mandatory field dimensions_cm is absent"});
                return;
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
        });
}

} // namespace simulator::io
