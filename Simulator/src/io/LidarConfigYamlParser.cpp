#include <Simulator/io/YamlConfigParsers.h>

#include "YamlParseUtil.hpp"

namespace simulator::io {

namespace UC = user_common_207190406_209543255;

UC::ConfigParseResult<common::types::LidarConfigData>
parseLidarConfig(const std::filesystem::path& path, UC::IRunErrorLog& log) {
    return detail::parseWrappedConfig<common::types::LidarConfigData>(
        path, log, "lidar_config", "CONFIG_FILE_NOT_FOUND",
        [&](const YAML::Node& node, UC::ConfigParseResult<common::types::LidarConfigData>& result) {
            if (const auto v = detail::readLengthCm(node, "z_min_cm")) {
                result.value.z_min = *v;
            }

            bool has_z_max = false;
            if (const auto v = detail::readLengthCm(node, "z_max_cm")) {
                result.value.z_max = *v;
                has_z_max = true;
            }
            if (!has_z_max) {
                result.errors.push_back({"CONFIG_MISSING_FIELD",
                                          "[lidar_config] mandatory field z_max_cm is absent"});
                return;
            }

            if (const auto v = detail::readLengthCm(node, "d_cm")) {
                result.value.d = *v;
            }

            if (const YAML::Node fov = node["fov_circles"]; fov && fov.IsScalar()) {
                try {
                    result.value.fov_circles = fov.as<std::size_t>();
                } catch (const YAML::Exception&) {
                    detail::logRecoverable(log, "CONFIG_BAD_VALUE",
                                           "[lidar_config] bad value for fov_circles — field keeps default");
                    result.errors.push_back(
                        {"CONFIG_BAD_VALUE", "[lidar_config] bad value for fov_circles"});
                }
            }

            result.ok = true;
        });
}

} // namespace simulator::io
