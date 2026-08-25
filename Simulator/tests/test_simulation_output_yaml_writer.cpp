// test_simulation_output_yaml_writer.cpp

#include <Simulator/io/SimulationOutputYamlWriter.h>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>

namespace {

[[nodiscard]] simulator::types::SimulationResult makeResult(
    double score, std::size_t steps, const std::filesystem::path& output_map_file) {
    simulator::types::SimulationResult result;
    result.mission_score = score;
    result.resolution_request_status = simulator::types::ResolutionRequestStatus::Accepted;
    result.output_map_file = output_map_file;
    result.mission_results.push_back(
        common::types::MissionRunResult{common::types::MissionRunStatus::Completed, steps, {}});
    return result;
}

} // namespace

TEST(SimulationOutputYamlWriter, WritesExpectedSchema) {
    simulator::types::SimulationManagerReport report;
    report.composition_file = "sim_compose.yaml";
    report.generated_at_utc = "2026-08-12T10:00:00Z";
    report.metric = "maps_comparison_score_0_100";
    report.score_range = {0.0, 100.0};
    report.error_score = -1;

    std::vector<simulator::io::SimulationRunYamlEntry> entries;
    simulator::io::SimulationRunYamlEntry entry;
    entry.run_id = 0;
    entry.simulation_index = 0;
    entry.mission_index = 0;
    entry.drone_index = 1;
    entry.lidar_index = 2;
    const std::filesystem::path out =
        std::filesystem::temp_directory_path() / "test_simulation_output_yaml_writer_out.yaml";
    entry.result = makeResult(87.5, 42, out.parent_path() / "plugin.so_run_0003_output_map.npy");
    entries.push_back(entry);

    simulator::io::writeSimulationOutputYaml(out, report, entries);

    const YAML::Node root = YAML::LoadFile(out.string());
    ASSERT_TRUE(root["score_report"]);
    const YAML::Node score_report = root["score_report"];
    EXPECT_EQ(score_report["composition_file"].as<std::string>(), "sim_compose.yaml");
    EXPECT_EQ(score_report["metric"].as<std::string>(), "maps_comparison_score_0_100");
    ASSERT_TRUE(score_report["runs"].IsSequence());
    ASSERT_EQ(score_report["runs"].size(), 1U);
    const YAML::Node run = score_report["runs"][0];
    EXPECT_DOUBLE_EQ(run["mission_score"].as<double>(), 87.5);
    EXPECT_EQ(run["config_indices"]["drone"].as<std::size_t>(), 1U);
    EXPECT_EQ(run["config_indices"]["lidar"].as<std::size_t>(), 2U);
    EXPECT_EQ(run["mission_results"][0]["steps"].as<std::size_t>(), 42U);
    EXPECT_EQ(run["output_map_file"].as<std::string>(), "plugin.so_run_0003_output_map.npy");
    EXPECT_EQ(run["error_log_file"].as<std::string>(), "plugin.so_run_0003_error.log");

    std::filesystem::remove(out);
}

TEST(SimulationOutputYamlWriter, EmptyMapPathLeavesErrorLogFileEmpty) {
    simulator::types::SimulationManagerReport report;
    report.generated_at_utc = "2026-08-12T10:00:00Z";
    report.score_range = {0.0, 100.0};

    simulator::io::SimulationRunYamlEntry entry;
    entry.result.output_map_file.clear();
    const std::filesystem::path out =
        std::filesystem::temp_directory_path() / "test_simulation_output_yaml_writer_empty.yaml";
    simulator::io::writeSimulationOutputYaml(out, report, {entry});

    const YAML::Node run = YAML::LoadFile(out.string())["score_report"]["runs"][0];
    EXPECT_EQ(run["error_log_file"].as<std::string>(), "");
    EXPECT_EQ(run["output_map_file"].as<std::string>(), "");
    std::filesystem::remove(out);
}
