#include <Simulator/io/YamlConfigParsers.h>
#include <Simulator/io/PathResolver.h>

#include "YamlParseUtil.hpp"

namespace simulator::io {

namespace UC = UserCommon_207190406_209543255;

UC::ConfigParseResult<simulator::types::SimulationConfigData>
parseSimulationConfig(const std::filesystem::path& path, UC::IRunErrorLog& log) {
    UC::ConfigParseResult<simulator::types::SimulationConfigData> result{};

    const auto root = detail::loadYamlFile(path, log, "[simulation_config]");
    if (!root.has_value()) {
        result.errors.push_back({"CONFIG_FILE_NOT_FOUND", path.string()});
        return result;
    }

    const YAML::Node node = detail::configRoot(*root, "simulation_config");

    if (const YAML::Node n = node["map_filename"]; n && n.IsScalar()) {
        result.value.map_filename = resolveMapFilename(n.as<std::string>(), path);
    }
    if (const auto v = detail::readLengthCm(node, "map_resolution_cm")) {
        result.value.map_resolution = *v;
    }
    if (const auto v = detail::readPosition3D(node["initial_drone_position"])) {
        result.value.initial_drone_position = *v;
    }
    if (const auto v = detail::readHorizontalAngleDeg(node, "initial_angle_deg")) {
        result.value.initial_angle = *v;
    }
    // Support both key names that appear in the instructor YAML files
    if (const auto v = detail::readMapOffset(node["map_offset"])) {
        result.value.map_offset = *v;
    } else if (const auto v2 = detail::readMapOffset(node["map_axes_offset"])) {
        result.value.map_offset = *v2;
    }

    result.ok = true;
    return result;
}

} // namespace simulator::io
