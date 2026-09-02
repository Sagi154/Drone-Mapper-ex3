#include <Simulator/io/SimulatorReports.h>

#include "ReportAggregationUtil.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <fstream>
#include <system_error>

namespace simulator::io {

namespace {

struct Row {
    std::string algorithm;
    double total_score = 0.0;
    std::size_t total_steps = 0;
};

} // namespace

void writeCompetitiveReport(const std::filesystem::path& output_path,
                            const CompetitiveReportInput& input) {
    std::vector<Row> rows;
    rows.reserve(input.results.size());
    for (const auto& plugin_result : input.results) {
        const detail::PluginTotals totals = detail::computeTotals(plugin_result);
        rows.push_back({plugin_result.plugin_filename, totals.total_score, totals.total_steps});
    }

    // Sort by total_score desc, then total_steps asc.
    std::stable_sort(rows.begin(), rows.end(), [](const Row& a, const Row& b) {
        if (a.total_score != b.total_score) {
            return a.total_score > b.total_score;
        }
        return a.total_steps < b.total_steps;
    });

    YAML::Node root;
    YAML::Node report;
    report["composition_file"] = input.composition_file.generic_string();
    report["mission_control"] = input.mission_control;
    report["generated_at_utc"] = input.generated_at_utc;

    YAML::Node results_summary = YAML::Node(YAML::NodeType::Sequence);
    for (const Row& row : rows) {
        YAML::Node entry;
        entry["algorithm"] = row.algorithm;
        entry["total_score"] = row.total_score;
        entry["total_steps"] = row.total_steps;
        results_summary.push_back(entry);
    }
    report["results_summary"] = results_summary;

    YAML::Node errors = YAML::Node(YAML::NodeType::Sequence);
    for (const auto& name : input.failed_plugins) {
        errors.push_back(name);
    }
    report["errors"] = errors;

    root["competitive_report"] = report;

    std::error_code ec;
    std::filesystem::create_directories(output_path.parent_path(), ec);
    std::ofstream output(output_path);
    output << YAML::Dump(root);
}

} // namespace simulator::io
