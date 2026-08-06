#include <Simulator/io/YamlConfigParsers.h>

#include "YamlParseUtil.hpp"

namespace simulator::io {

namespace UC = UserCommon_207190406_209543255;

UC::ConfigParseResult<common::types::MissionConfigData>
parseMissionConfig(const std::filesystem::path& path, UC::IRunErrorLog& log) {
    UC::ConfigParseResult<common::types::MissionConfigData> result{};
    result.value.output_mapping_resolution_factor = 1.0;

    const auto root = detail::loadYamlFile(path, log, "[mission_config]");
    if (!root.has_value()) {
        result.errors.push_back({"CONFIG_FILE_NOT_FOUND", path.string()});
        return result;
    }

    const YAML::Node node = detail::configRoot(*root, "mission_config");

    // YAML key is "boundaries" (unchanged); field in MissionConfigData is "mission_bounds"
    if (const auto v = detail::readMissionBoundaries(node)) {
        result.value.mission_bounds = *v;
    }

    if (const YAML::Node n = node["max_steps"]; n && n.IsScalar()) {
        try {
            result.value.max_steps = n.as<std::size_t>();
        } catch (const YAML::Exception&) {
            detail::logRecoverable(log, "CONFIG_BAD_VALUE",
                                   "[mission_config] bad value for max_steps — field keeps default");
            result.errors.push_back({"CONFIG_BAD_VALUE", "[mission_config] bad value for max_steps"});
        }
    }

    if (const auto v = detail::readLengthCm(node, "gps_resolution_cm")) {
        result.value.gps_resolution = *v;
    }

    if (const auto v = detail::readScalarDouble(node, "output_mapping_resolution_factor")) {
        if (*v < 1.0) {
            detail::logRecoverable(
                log, "CONFIG_BAD_VALUE",
                "[mission_config] output_mapping_resolution_factor < 1 — ignored, using default 1");
            result.errors.push_back({"CONFIG_BAD_VALUE",
                                     "[mission_config] output_mapping_resolution_factor < 1"});
        } else {
            result.value.output_mapping_resolution_factor = *v;
        }
    }

    result.ok = true;
    return result;
}

} // namespace simulator::io
