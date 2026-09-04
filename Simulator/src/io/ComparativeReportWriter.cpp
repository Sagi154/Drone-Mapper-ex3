#include <Simulator/io/SimulatorReports.h>

#include "ReportAggregationUtil.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <fstream>
#include <map>
#include <system_error>
#include <tuple>

namespace simulator::io {

namespace {

struct Group {
    std::vector<std::string> same_results;
    double total_score = 0.0;
    std::size_t total_steps = 0;
};

} // namespace

void writeComparativeReport(const std::filesystem::path& output_path,
                            const ComparativeReportInput& input) {
    // Group plugins by the exact (total_score, total_steps) pair (open-questions.md #4).
    std::map<std::tuple<double, std::size_t>, Group> groups;
    for (const auto& plugin_result : input.results) {
        const detail::PluginTotals totals = detail::computeTotals(plugin_result);
        const auto key = std::make_tuple(totals.total_score, totals.total_steps);
        Group& group = groups[key];
        group.total_score = totals.total_score;
        group.total_steps = totals.total_steps;
        group.same_results.push_back(plugin_result.plugin_filename);
    }

    std::vector<Group> ordered;
    ordered.reserve(groups.size());
    for (auto& [key, group] : groups) {
        (void)key;
        ordered.push_back(std::move(group));
    }
    // Sort by group size descending; stable so equal-size groups keep insertion order.
    std::stable_sort(ordered.begin(), ordered.end(), [](const Group& a, const Group& b) {
        return a.same_results.size() > b.same_results.size();
    });

    YAML::Node root;
    YAML::Node report;
    report["composition_file"] = input.composition_file.generic_string();
    report["mission_control_folder"] = input.mission_control_folder.generic_string();
    report["generated_at_utc"] = input.generated_at_utc;

    YAML::Node results_summary = YAML::Node(YAML::NodeType::Sequence);
    for (const Group& group : ordered) {
        YAML::Node entry;
        YAML::Node same_results = YAML::Node(YAML::NodeType::Sequence);
        for (const auto& name : group.same_results) {
            same_results.push_back(name);
        }
        entry["same_results"] = same_results;
        entry["total_score"] = group.total_score;
        entry["total_steps"] = group.total_steps;
        results_summary.push_back(entry);
    }
    report["results_summary"] = results_summary;

    YAML::Node errors = YAML::Node(YAML::NodeType::Sequence);
    for (const auto& name : input.failed_plugins) {
        errors.push_back(name);
    }
    report["errors"] = errors;

    root["comparative_report"] = report;

    std::error_code ec;
    std::filesystem::create_directories(output_path.parent_path(), ec);
    std::ofstream output(output_path);
    output << YAML::Dump(root);
}

} // namespace simulator::io
