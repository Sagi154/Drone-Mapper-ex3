#include <Simulator/io/SimulationCli.h>

#include <cctype>
#include <fstream>
#include <limits>
#include <sstream>
#include <unordered_map>

namespace simulator::io {
namespace {

constexpr std::string_view kModeComparative = "-comparative";
constexpr std::string_view kModeCompetition = "-competition";
constexpr std::string_view kFlagVerbose     = "-verbose";

constexpr std::string_view kKeySimulation            = "simulation";
constexpr std::string_view kKeyMissionControlFolder  = "mission_control_folder";
constexpr std::string_view kKeyAlgorithm             = "algorithm";
constexpr std::string_view kKeyMissionControl        = "mission_control";
constexpr std::string_view kKeyAlgorithmsFolder      = "algorithms_folder";
constexpr std::string_view kKeyNumThreads            = "num_threads";

[[nodiscard]] bool isKnownKey(std::string_view key) {
    return key == kKeySimulation || key == kKeyMissionControlFolder || key == kKeyAlgorithm ||
           key == kKeyMissionControl || key == kKeyAlgorithmsFolder || key == kKeyNumThreads;
}

[[nodiscard]] bool isKeyAllowedForMode(SimulatorMode mode, std::string_view key) {
    if (key == kKeySimulation || key == kKeyNumThreads) {
        return true;
    }
    if (mode == SimulatorMode::Comparative) {
        return key == kKeyMissionControlFolder || key == kKeyAlgorithm;
    }
    return key == kKeyMissionControl || key == kKeyAlgorithmsFolder;
}

[[nodiscard]] bool parseUnsigned(std::string_view text, unsigned& out) {
    if (text.empty()) {
        return false;
    }
    unsigned value = 0;
    for (char c : text) {
        if (!std::isdigit(static_cast<unsigned char>(c))) {
            return false;
        }
        const unsigned digit = static_cast<unsigned>(c - '0');
        if (value > (std::numeric_limits<unsigned>::max() - digit) / 10U) {
            return false;
        }
        value = value * 10U + digit;
    }
    out = value;
    return true;
}

[[nodiscard]] bool folderHasSoFile(const std::filesystem::path& folder) {
    std::error_code ec;
    const auto it = std::filesystem::directory_iterator(folder, ec);
    if (ec) {
        return false;
    }
    for (const auto& entry : it) {
        std::error_code entry_ec;
        if (!entry.is_regular_file(entry_ec) || entry_ec) {
            continue;
        }
        if (entry.path().extension() == ".so") {
            return true;
        }
    }
    return false;
}

[[nodiscard]] bool fileIsOpenable(const std::filesystem::path& path) {
    std::error_code ec;
    if (!std::filesystem::is_regular_file(path, ec) || ec) {
        return false;
    }
    std::ifstream in{path};
    return static_cast<bool>(in);
}

void appendUnique(std::vector<std::string>& errors, std::string message) {
    for (const auto& existing : errors) {
        if (existing == message) {
            return;
        }
    }
    errors.push_back(std::move(message));
}

} // namespace

std::string simulationCliUsage(std::string_view program_name) {
    std::ostringstream oss;
    oss << "Usage:\n"
        << "  " << program_name << " -comparative simulation=<composition_yaml> "
        << "mission_control_folder=<folder> algorithm=<algorithm_so> "
        << "[num_threads=<num>] [-verbose]\n"
        << "  " << program_name << " -competition simulation=<composition_yaml> "
        << "mission_control=<mission_control_so> algorithms_folder=<folder> "
        << "[num_threads=<num>] [-verbose]\n";
    return oss.str();
}

SimulationCliParseResult parseSimulationCliArgs(int argc, char** argv, std::ostream* diag) {
    SimulationCliParseResult result;
    const std::string program =
        (argc > 0 && argv != nullptr && argv[0] != nullptr) ? argv[0] : "simulator";

    std::optional<SimulatorMode> mode;
    std::unordered_map<std::string, std::string> values;
    std::vector<std::string> unsupported;
    bool verbose = false;
    bool mode_conflict = false;

    for (int i = 1; i < argc; ++i) {
        if (argv == nullptr || argv[i] == nullptr) {
            appendUnique(result.errors, "null argument at position " + std::to_string(i));
            continue;
        }
        const std::string_view token{argv[i]};

        if (token == kModeComparative || token == kModeCompetition) {
            const SimulatorMode parsed =
                (token == kModeComparative) ? SimulatorMode::Comparative : SimulatorMode::Competition;
            if (mode.has_value() && *mode != parsed) {
                mode_conflict = true;
            }
            mode = parsed;
            continue;
        }

        if (token == kFlagVerbose) {
            verbose = true;
            continue;
        }

        const auto eq = token.find('=');
        if (eq == std::string_view::npos) {
            unsupported.emplace_back(token);
            continue;
        }

        const std::string key{token.substr(0, eq)};
        const std::string value{token.substr(eq + 1)};

        if (!isKnownKey(key)) {
            unsupported.push_back(key);
            continue;
        }
        if (values.contains(key)) {
            appendUnique(result.errors, "duplicate argument '" + key + "'");
            continue;
        }
        values.emplace(key, value);
    }

    if (mode_conflict) {
        appendUnique(result.errors,
                     "conflicting mode flags: provide exactly one of -comparative or -competition");
    } else if (!mode.has_value()) {
        // Dash tokens that are not -verbose / known modes are unknown mode flags.
        bool saw_unknown_mode_flag = false;
        std::vector<std::string> remaining_unsupported;
        remaining_unsupported.reserve(unsupported.size());
        for (const auto& arg : unsupported) {
            if (!arg.empty() && arg[0] == '-') {
                appendUnique(result.errors, "unknown mode flag '" + arg + "'");
                saw_unknown_mode_flag = true;
            } else {
                remaining_unsupported.push_back(arg);
            }
        }
        unsupported.swap(remaining_unsupported);
        if (!saw_unknown_mode_flag) {
            appendUnique(result.errors,
                         "missing mode flag: expected -comparative or -competition");
        }
    }

    // Keys that are known but illegal for the selected mode count as unsupported.
    if (mode.has_value()) {
        for (const auto& [key, _] : values) {
            if (!isKeyAllowedForMode(*mode, key)) {
                unsupported.push_back(key);
            }
        }
    }

    if (!unsupported.empty()) {
        std::ostringstream oss;
        oss << "unsupported argument(s):";
        for (const auto& arg : unsupported) {
            oss << ' ' << arg;
        }
        appendUnique(result.errors, oss.str());
    }

    if (mode.has_value()) {
        std::vector<std::string_view> required;
        if (*mode == SimulatorMode::Comparative) {
            required = {kKeySimulation, kKeyMissionControlFolder, kKeyAlgorithm};
        } else {
            required = {kKeySimulation, kKeyMissionControl, kKeyAlgorithmsFolder};
        }

        std::vector<std::string> missing;
        for (const auto key : required) {
            const auto it = values.find(std::string{key});
            if (it == values.end() || it->second.empty()) {
                missing.emplace_back(key);
            }
        }
        if (!missing.empty()) {
            std::ostringstream oss;
            oss << "missing argument(s):";
            for (const auto& key : missing) {
                oss << ' ' << key;
            }
            appendUnique(result.errors, oss.str());
        }
    }

    if (const auto it = values.find(std::string{kKeyNumThreads}); it != values.end()) {
        unsigned threads = 0;
        if (!parseUnsigned(it->second, threads) || threads == 0) {
            appendUnique(result.errors,
                         "invalid num_threads '" + it->second + "': expected a positive integer");
        } else {
            result.args.num_threads = threads;
        }
    }

    // Structural / argument-shape errors: do not touch the filesystem yet.
    if (!result.errors.empty() || !mode.has_value()) {
        result.args.verbose = verbose;
        if (mode.has_value()) {
            result.args.mode = *mode;
        }
        if (diag != nullptr) {
            *diag << simulationCliUsage(program);
            for (const auto& err : result.errors) {
                *diag << "error: " << err << '\n';
            }
        }
        return result;
    }

    result.args.mode    = *mode;
    result.args.verbose = verbose;
    result.args.simulation = values.at(std::string{kKeySimulation});

    if (*mode == SimulatorMode::Comparative) {
        result.args.mission_control_folder = values.at(std::string{kKeyMissionControlFolder});
        result.args.algorithm              = values.at(std::string{kKeyAlgorithm});
    } else {
        result.args.mission_control   = values.at(std::string{kKeyMissionControl});
        result.args.algorithms_folder = values.at(std::string{kKeyAlgorithmsFolder});
    }

    // Filesystem validation — only after argument shape is clean.
    if (!fileIsOpenable(result.args.simulation)) {
        appendUnique(result.errors, "simulation file is missing or unopenable: " +
                                         result.args.simulation.string());
    }

    if (*mode == SimulatorMode::Comparative) {
        if (!fileIsOpenable(result.args.algorithm)) {
            appendUnique(result.errors, "algorithm file is missing or unopenable: " +
                                             result.args.algorithm.string());
        }

        std::error_code ec;
        if (!std::filesystem::is_directory(result.args.mission_control_folder, ec) || ec) {
            appendUnique(result.errors,
                         "mission_control_folder is missing or not traversable: " +
                             result.args.mission_control_folder.string());
        } else if (!folderHasSoFile(result.args.mission_control_folder)) {
            appendUnique(result.errors,
                         "mission_control_folder contains no .so files: " +
                             result.args.mission_control_folder.string());
        }
    } else {
        if (!fileIsOpenable(result.args.mission_control)) {
            appendUnique(result.errors, "mission_control file is missing or unopenable: " +
                                             result.args.mission_control.string());
        }

        std::error_code ec;
        if (!std::filesystem::is_directory(result.args.algorithms_folder, ec) || ec) {
            appendUnique(result.errors, "algorithms_folder is missing or not traversable: " +
                                             result.args.algorithms_folder.string());
        } else if (!folderHasSoFile(result.args.algorithms_folder)) {
            appendUnique(result.errors, "algorithms_folder contains no .so files: " +
                                             result.args.algorithms_folder.string());
        }
    }

    result.ok = result.errors.empty();
    if (!result.ok && diag != nullptr) {
        *diag << simulationCliUsage(program);
        for (const auto& err : result.errors) {
            *diag << "error: " << err << '\n';
        }
    }
    return result;
}

} // namespace simulator::io
