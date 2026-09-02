#include <Simulator/io/SimulatorPaths.h>
#include <Simulator/io/YamlConfigParsers.h>

#include "YamlParseUtil.hpp"

namespace simulator::io {

namespace UC = user_common_207190406_209543255;

UC::ConfigParseResult<simulator::types::SimulationConfigData>
parseSimulationConfig(const std::filesystem::path& path, UC::IRunErrorLog& log) {
    return detail::parseWrappedConfig<simulator::types::SimulationConfigData>(
        path, log, "simulation_config", "CONFIG_FILE_NOT_FOUND",
        [&](const YAML::Node& node,
            UC::ConfigParseResult<simulator::types::SimulationConfigData>& result) {
            if (const YAML::Node n = node["map_filename"]; n && n.IsScalar()) {
                result.value.map_filename = resolveMapFilename(n.as<std::string>(), path);
            } else {
                result.errors.push_back(
                    {"CONFIG_MISSING_FIELD",
                     "[simulation_config] mandatory field map_filename is absent"});
                return;
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
        });
}

} // namespace simulator::io
