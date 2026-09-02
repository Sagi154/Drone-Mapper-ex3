// SimulatorReports.h — aggregate comparative/competitive reports and per-plugin YAML.

#pragma once

#include <Simulator/RunMatrixTypes.h>
#include <Simulator/SimulationTypes.h>

#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

namespace simulator::io {

struct ComparativeReportInput {
    std::filesystem::path composition_file;
    std::filesystem::path mission_control_folder;
    std::string generated_at_utc;
    std::vector<PluginMatrixResult> results; // one per loaded mission_control .so
    std::vector<std::string> failed_plugins; // .so filenames that failed to load/run
};

void writeComparativeReport(const std::filesystem::path& output_path,
                            const ComparativeReportInput& input);

struct CompetitiveReportInput {
    std::filesystem::path composition_file;
    std::string mission_control;             // fixed mission_control .so filename
    std::string generated_at_utc;
    std::vector<PluginMatrixResult> results; // one per loaded algorithm .so
    std::vector<std::string> failed_plugins; // .so filenames that failed to load/run
};

void writeCompetitiveReport(const std::filesystem::path& output_path,
                            const CompetitiveReportInput& input);

struct SimulationRunYamlEntry {
    int run_id = 0;
    std::size_t simulation_index = 0;
    std::size_t mission_index = 0;
    std::size_t drone_index = 0;
    std::size_t lidar_index = 0;
    types::SimulationResult result{};
};

/// Writes one ex2-style `simulation_output.yaml`-shaped file (root key
/// `score_report`) to `output_path` (a full file path, not a directory).
void writeSimulationOutputYaml(const std::filesystem::path& output_path,
                               const types::SimulationManagerReport& report,
                               const std::vector<SimulationRunYamlEntry>& entries);

} // namespace simulator::io
