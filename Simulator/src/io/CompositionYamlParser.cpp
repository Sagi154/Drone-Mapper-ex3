#include <Simulator/io/YamlConfigParsers.h>
#include <Simulator/io/PathResolver.h>

#include "YamlParseUtil.hpp"

#include <vector>

namespace simulator::io {

namespace UC = UserCommon_207190406_209543255;

namespace {

[[nodiscard]] std::vector<std::filesystem::path>
readPathList(const YAML::Node& node, const char* key) {
    std::vector<std::filesystem::path> paths;
    const YAML::Node list = node[key];
    if (!list || !list.IsSequence()) {
        return paths;
    }
    for (const YAML::Node& entry : list) {
        if (entry.IsScalar()) {
            paths.emplace_back(entry.as<std::string>());
        }
    }
    return paths;
}

} // namespace

UC::ConfigParseResult<simulator::types::SimulationCompositionData>
parseCompositionFile(const std::filesystem::path& path, UC::IRunErrorLog& log) {
    UC::ConfigParseResult<simulator::types::SimulationCompositionData> result{};

    const auto root = detail::loadYamlFile(path, log, "[simulation_compositions]");
    if (!root.has_value()) {
        result.errors.push_back({"COMPOSITION_FILE_UNREADABLE",
                                  "Could not read composition file: " + path.string()});
        return result;
    }

    const YAML::Node compositions = detail::configRoot(*root, "simulation_compositions");
    const std::filesystem::path base_dir =
        path.has_parent_path() ? path.parent_path() : std::filesystem::path{"."};

    const YAML::Node simulations_yaml = compositions["simulations"];
    if (!simulations_yaml || !simulations_yaml.IsSequence() || simulations_yaml.size() == 0) {
        result.errors.push_back({"COMPOSITION_INVALID",
                                  "simulation_compositions.simulations must be a non-empty sequence"});
        return result;
    }

    simulator::types::SimulationCompositionData composition{};
    composition.composition_file = path;

    // Build simulation_mission_groups: each entry is (SimulationConfig, [MissionConfig...])
    for (const YAML::Node& entry : simulations_yaml) {
        if (!entry["simulation_config"]) {
            detail::logRecoverable(log, "COMPOSITION_INVALID",
                                   "[simulation_compositions] entry missing simulation_config — skipped");
            continue;
        }

        const auto sim_path =
            resolveConfigPath(base_dir, entry["simulation_config"].as<std::string>());
        const auto sim_result = parseSimulationConfig(sim_path, log);
        if (!sim_result.ok) {
            detail::logRecoverable(log, "COMPOSITION_INVALID",
                                   "[simulation_compositions] failed to parse simulation_config \"" +
                                       sim_path.string() + "\" — group skipped");
            continue;
        }
        const simulator::types::SimulationConfigData sim_cfg = sim_result.value;

        const YAML::Node mission_list = entry["mission_configs"];
        if (!mission_list || !mission_list.IsSequence() || mission_list.size() == 0) {
            detail::logRecoverable(log, "COMPOSITION_INVALID",
                                   "[simulation_compositions] entry missing mission_configs — skipped");
            continue;
        }

        std::vector<common::types::MissionConfigData> missions;
        for (const YAML::Node& mission_entry : mission_list) {
            if (!mission_entry.IsScalar()) {
                continue;
            }
            const auto m_path = resolveConfigPath(base_dir, mission_entry.as<std::string>());
            const auto m_result = parseMissionConfig(m_path, log);
            if (!m_result.ok) {
                detail::logRecoverable(log, "COMPOSITION_INVALID",
                                       "[simulation_compositions] failed to parse mission_config \"" +
                                           m_path.string() + "\" — entry skipped");
                continue;
            }
            missions.push_back(m_result.value);
        }

        if (missions.empty()) {
            continue;
        }
        composition.simulation_mission_groups.emplace_back(sim_cfg, std::move(missions));
    }

    if (composition.simulation_mission_groups.empty()) {
        result.errors.push_back({"COMPOSITION_INVALID",
                                  "No valid simulation/mission groups found in composition"});
        return result;
    }

    const auto drone_paths = readPathList(compositions, "drone_configs");
    const auto lidar_paths = readPathList(compositions, "lidar_configs");

    if (drone_paths.empty() || lidar_paths.empty()) {
        result.errors.push_back({"COMPOSITION_INVALID",
                                  "simulation_compositions requires non-empty drone_configs and lidar_configs"});
        return result;
    }

    for (const auto& p : drone_paths) {
        composition.drone_configs.push_back(
            parseDroneConfig(resolveConfigPath(base_dir, p), log).value);
    }
    for (const auto& p : lidar_paths) {
        composition.lidar_configs.push_back(
            parseLidarConfig(resolveConfigPath(base_dir, p), log).value);
    }

    result.ok    = true;
    result.value = std::move(composition);
    return result;
}

} // namespace simulator::io
