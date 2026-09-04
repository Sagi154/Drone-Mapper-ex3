// SimulatorPaths.h — CLI parse, path resolve, output-dir creation, error-log path.
// Never calls exit(); callers decide how to finish from the returned result.

#pragma once

#include <filesystem>
#include <iosfwd>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

namespace simulator::io {

inline constexpr std::string_view kOutputMapFilenameSuffix = "_output_map.npy";
inline constexpr std::string_view kErrorLogFilenameSuffix = "_error.log";

/// Derive `<plugin>_run_NNNN_error.log` from `<plugin>_run_NNNN_output_map.npy`.
[[nodiscard]] inline std::filesystem::path errorLogPathFromOutputMap(
    const std::filesystem::path& map_path) {
    if (map_path.empty()) {
        return {};
    }

    const std::string filename = map_path.filename().string();
    std::string new_name;
    if (filename.size() >= kOutputMapFilenameSuffix.size() &&
        filename.compare(filename.size() - kOutputMapFilenameSuffix.size(),
                         kOutputMapFilenameSuffix.size(),
                         kOutputMapFilenameSuffix.data(),
                         kOutputMapFilenameSuffix.size()) == 0) {
        new_name = filename.substr(0, filename.size() - kOutputMapFilenameSuffix.size());
        new_name.append(kErrorLogFilenameSuffix);
    } else {
        new_name = map_path.stem().string();
        new_name.append(kErrorLogFilenameSuffix);
    }
    return map_path.parent_path() / new_name;
}

/// Resolve a relative config file path against a base directory.
/// Absolute paths are returned unchanged.
[[nodiscard]] std::filesystem::path resolveConfigPath(const std::filesystem::path& base_dir,
                                                       const std::filesystem::path& config_path);

/// Resolve a hidden-map .npy filename from a simulation_config YAML.
/// Search order: as-is → simulation YAML parent dir → that dir's parent → CWD.
[[nodiscard]] std::filesystem::path resolveMapFilename(
    const std::filesystem::path& map_filename,
    const std::filesystem::path& simulation_config_path = {});

enum class OutputDirKind { Comparative, Competition };

/// Creates and returns a fresh directory under `base_folder`:
///   Comparative -> base_folder / "comparative_results_<UTC timestamp>[_N]"
///   Competition -> base_folder / "competition_<UTC timestamp>[_N]"
/// If a directory with that name already exists (e.g. two runs within the
/// same second), appends "_2", "_3", ... until an unused name is found.
/// On any filesystem failure, sets `ec` and returns whatever path was last
/// attempted (caller must check `ec`, not just path existence).
[[nodiscard]] std::filesystem::path createOutputDir(const std::filesystem::path& base_folder,
                                                    OutputDirKind kind,
                                                    std::error_code& ec);

enum class SimulatorMode {
    Comparative,
    Competition,
};

struct SimulationCliArgs {
    SimulatorMode mode{};
    std::filesystem::path simulation;
    std::filesystem::path mission_control_folder; // comparative
    std::filesystem::path algorithm;              // comparative (.so)
    std::filesystem::path mission_control;        // competition (.so)
    std::filesystem::path algorithms_folder;      // competition
    std::optional<unsigned> num_threads;          // nullopt → treat as 1
    bool verbose = false;
};

struct SimulationCliParseResult {
    bool ok = false;
    SimulationCliArgs args{};
    std::vector<std::string> errors;
};

[[nodiscard]] std::string simulationCliUsage(std::string_view program_name);

/// Parse argv without side effects on success. On failure, `ok` is false and `errors`
/// lists every problem found. When `diag` is non-null and parsing fails, writes usage
/// plus each error line to it. Never calls `exit()`.
[[nodiscard]] SimulationCliParseResult parseSimulationCliArgs(int argc, char** argv,
                                                              std::ostream* diag = nullptr);

} // namespace simulator::io
