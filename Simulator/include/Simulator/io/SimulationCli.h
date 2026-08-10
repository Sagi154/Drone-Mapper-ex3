// SimulationCli.h — parse and validate the two ex3 simulator modes.
// Never calls exit(); callers decide how to finish from the returned result.

#pragma once

#include <filesystem>
#include <iosfwd>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace simulator::io {

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
