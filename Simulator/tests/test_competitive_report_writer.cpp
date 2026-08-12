#include <Simulator/io/CompetitiveReportWriter.h>

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

TEST(CompetitiveReportWriter, SortsByScoreDescThenStepsAsc) {
    simulator::io::CompetitiveReportInput input;
    input.composition_file = "sim_compose.yaml";
    input.mission_control = "mission_control.so";
    input.generated_at_utc = "2026-08-12T10:00:00Z";
    input.results = {
        makePlugin("algo_low.so", 60.0, 10),
        makePlugin("algo_high_slow.so", 90.0, 200),
        makePlugin("algo_high_fast.so", 90.0, 50),
    };
    input.failed_plugins = {"algo2.so"};

    const std::filesystem::path out =
        std::filesystem::temp_directory_path() / "test_competitive_report_writer_out.yaml";
    simulator::io::writeCompetitiveReport(out, input);

    const YAML::Node root = YAML::LoadFile(out.string());
    const YAML::Node report = root["competitive_report"];
    ASSERT_TRUE(report);
    EXPECT_EQ(report["mission_control"].as<std::string>(), "mission_control.so");
    ASSERT_EQ(report["results_summary"].size(), 3U);
    EXPECT_EQ(report["results_summary"][0]["algorithm"].as<std::string>(), "algo_high_fast.so");
    EXPECT_EQ(report["results_summary"][1]["algorithm"].as<std::string>(), "algo_high_slow.so");
    EXPECT_EQ(report["results_summary"][2]["algorithm"].as<std::string>(), "algo_low.so");
    ASSERT_EQ(report["errors"].size(), 1U);

    std::filesystem::remove(out);
}
