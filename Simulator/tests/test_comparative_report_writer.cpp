#include <Simulator/io/ComparativeReportWriter.h>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>

namespace {

[[nodiscard]] simulator::PluginMatrixResult makePlugin(const std::string& name, double score,
                                                       std::size_t steps) {
    simulator::PluginMatrixResult plugin;
    plugin.plugin_filename = name;
    simulator::types::SimulationResult result;
    result.mission_score = score;
    result.mission_results.push_back(
        common::types::MissionRunResult{common::types::MissionRunStatus::Completed, steps, {}});
    plugin.results.push_back(result);
    return plugin;
}

} // namespace

TEST(ComparativeReportWriter, GroupsTiedResultsAndListsErrors) {
    simulator::io::ComparativeReportInput input;
    input.composition_file = "sim_compose.yaml";
    input.mission_control_folder = "mission_controls";
    input.generated_at_utc = "2026-08-12T10:00:00Z";
    input.results = {
        makePlugin("manager1.so", 100.0, 20),
        makePlugin("manager2.so", 100.0, 20), // ties with manager1
        makePlugin("manager3.so", 50.0, 30),
    };
    input.failed_plugins = {"manager7.so"};

    const std::filesystem::path out =
        std::filesystem::temp_directory_path() / "test_comparative_report_writer_out.yaml";
    simulator::io::writeComparativeReport(out, input);

    const YAML::Node root = YAML::LoadFile(out.string());
    const YAML::Node report = root["comparative_report"];
    ASSERT_TRUE(report);
    EXPECT_EQ(report["mission_control_folder"].as<std::string>(), "mission_controls");
    ASSERT_TRUE(report["results_summary"].IsSequence());
    ASSERT_EQ(report["results_summary"].size(), 2U);

    // Largest group (2 tied managers) sorts first.
    const YAML::Node first_group = report["results_summary"][0];
    EXPECT_EQ(first_group["same_results"].size(), 2U);
    EXPECT_DOUBLE_EQ(first_group["total_score"].as<double>(), 100.0);

    ASSERT_EQ(report["errors"].size(), 1U);
    EXPECT_EQ(report["errors"][0].as<std::string>(), "manager7.so");

    std::filesystem::remove(out);
}
