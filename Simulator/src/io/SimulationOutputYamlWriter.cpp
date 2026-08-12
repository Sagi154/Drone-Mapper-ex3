#include <Simulator/io/SimulationOutputYamlWriter.h>

#include <Common/Units.h>

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <string>
#include <system_error>

namespace simulator::io {

namespace {

[[nodiscard]] std::string pathForYaml(const std::filesystem::path& output_dir,
                                      const std::filesystem::path& path) {
    if (path.empty()) {
        return {};
    }
    std::error_code ec;
    const std::filesystem::path relative = std::filesystem::relative(path, output_dir, ec);
    return !ec ? relative.generic_string() : path.generic_string();
}

[[nodiscard]] std::string resolutionRequestStatusLabel(types::ResolutionRequestStatus status) {
    switch (status) {
    case types::ResolutionRequestStatus::Accepted:
        return "ACCEPTED";
    case types::ResolutionRequestStatus::IgnoredTooSmall:
        return "IGNORED TOO SMALL";
    case types::ResolutionRequestStatus::Ignored:
        return "IGNORED";
    }
    return "IGNORED";
}

[[nodiscard]] std::string missionRunStatusLabel(common::types::MissionRunStatus status) {
    switch (status) {
    case common::types::MissionRunStatus::Completed:
        return "COMPLETED";
    case common::types::MissionRunStatus::MaxSteps:
        return "MAX_STEPS";
    case common::types::MissionRunStatus::Error:
        return "ERROR";
    }
    return "ERROR";
}

[[nodiscard]] YAML::Node positionNode(const common::Position3D& position) {
    YAML::Node node;
    node["x_cm"] = position.x.numerical_value_in(common::cm);
    node["y_cm"] = position.y.numerical_value_in(common::cm);
    node["z_cm"] = position.z.numerical_value_in(common::cm);
    return node;
}

[[nodiscard]] YAML::Node simulationNode(const types::SimulationConfigData& simulation) {
    YAML::Node node;
    node["map_filename"] = simulation.map_filename.generic_string();
    node["map_resolution_cm"] = simulation.map_resolution.numerical_value_in(common::cm);
    node["map_offset"] = positionNode(simulation.map_offset);
    node["initial_drone_position"] = positionNode(simulation.initial_drone_position);
    return node;
}

[[nodiscard]] YAML::Node missionNode(const common::types::MissionConfigData& mission) {
    YAML::Node node;
    node["max_steps"] = mission.max_steps;
    node["gps_resolution_cm"] = mission.gps_resolution.numerical_value_in(common::cm);
    node["output_mapping_resolution_factor"] = mission.output_mapping_resolution_factor;
    return node;
}

[[nodiscard]] YAML::Node missionResultsNode(
    const std::vector<common::types::MissionRunResult>& mission_results) {
    YAML::Node results = YAML::Node(YAML::NodeType::Sequence);
    for (const auto& mission_result : mission_results) {
        YAML::Node entry;
        entry["status"] = missionRunStatusLabel(mission_result.status);
        entry["steps"] = mission_result.steps;

        YAML::Node errors = YAML::Node(YAML::NodeType::Sequence);
        for (const auto& error : mission_result.errors) {
            YAML::Node error_node;
            error_node["code"] = error.code;
            error_node["message"] = error.message;
            errors.push_back(error_node);
        }
        entry["errors"] = errors;
        results.push_back(entry);
    }
    return results;
}

[[nodiscard]] YAML::Node runEntryNode(const std::filesystem::path& output_dir,
                                      const SimulationRunYamlEntry& entry) {
    const types::SimulationResult& result = entry.result;

    YAML::Node node;
    node["run_id"] = entry.run_id;
    node["output_map_file"] = pathForYaml(output_dir, result.output_map_file);
    // No per-run error log exists yet (Y11 pending) — errors surface inline via mission_results.
    node["error_log_file"] = "";
    node["resolution_request_status"] =
        resolutionRequestStatusLabel(result.resolution_request_status);
    node["mission_score"] = result.mission_score;

    YAML::Node config_indices;
    config_indices["simulation"] = entry.simulation_index;
    config_indices["mission"] = entry.mission_index;
    config_indices["drone"] = entry.drone_index;
    config_indices["lidar"] = entry.lidar_index;
    node["config_indices"] = config_indices;

    node["simulation"] = simulationNode(result.simulation_config);
    node["mission"] = missionNode(result.mission_config);
    node["mission_results"] = missionResultsNode(result.mission_results);
    return node;
}

} // namespace

void writeSimulationOutputYaml(const std::filesystem::path& output_path,
                               const types::SimulationManagerReport& report,
                               const std::vector<SimulationRunYamlEntry>& entries) {
    YAML::Node score_report;
    score_report["composition_file"] = report.composition_file.generic_string();
    score_report["generated_at_utc"] = report.generated_at_utc;
    score_report["metric"] = report.metric;
    score_report["score_range"] = YAML::Node(YAML::NodeType::Sequence);
    score_report["score_range"].push_back(std::get<0>(report.score_range));
    score_report["score_range"].push_back(std::get<1>(report.score_range));
    score_report["error_score"] = report.error_score;

    YAML::Node runs = YAML::Node(YAML::NodeType::Sequence);
    for (const SimulationRunYamlEntry& entry : entries) {
        runs.push_back(runEntryNode(output_path.parent_path(), entry));
    }
    score_report["runs"] = runs;

    YAML::Node root;
    root["score_report"] = score_report;

    std::error_code ec;
    std::filesystem::create_directories(output_path.parent_path(), ec);
    std::ofstream output(output_path);
    output << YAML::Dump(root);
}

} // namespace simulator::io
