// ComparativeReportWriter.h — aggregate "comparative_report" YAML, schema per
// .cursor/rules/simulator-cli-and-outputs.mdc.

#pragma once

#include <Simulator/RunMatrixOrchestrator.h>

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

} // namespace simulator::io
