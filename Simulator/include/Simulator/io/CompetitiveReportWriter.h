// CompetitiveReportWriter.h — aggregate "competitive_report" YAML, schema per
// .cursor/rules/simulator-cli-and-outputs.mdc.

#pragma once

#include <Simulator/RunMatrixOrchestrator.h>

#include <filesystem>
#include <string>
#include <vector>

namespace simulator::io {

struct CompetitiveReportInput {
    std::filesystem::path composition_file;
    std::string mission_control;             // fixed mission_control .so filename
    std::string generated_at_utc;
    std::vector<PluginMatrixResult> results; // one per loaded algorithm .so
    std::vector<std::string> failed_plugins; // .so filenames that failed to load/run
};

void writeCompetitiveReport(const std::filesystem::path& output_path,
                            const CompetitiveReportInput& input);

} // namespace simulator::io
