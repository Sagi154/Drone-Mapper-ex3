#include <Simulator/PluginLoader.h>
#include <Simulator/RunMatrixOrchestrator.h>
#include <Simulator/SimulationRunFactoryImpl.h>
#include <Simulator/io/SimulatorPaths.h>
#include <Simulator/io/SimulatorReports.h>
#include <Simulator/io/YamlConfigParsers.h>

#include <user_common_207190406_209543255/RunErrorLog.h>

#include <cstddef>
#include <filesystem>
#include <functional>
#include <iostream>
#include <memory>
#include <vector>

namespace {

namespace fs = std::filesystem;
namespace sim_io = simulator::io;
namespace UC = user_common_207190406_209543255;

struct PluginBootstrap {
    simulator::PluginLoader loader;
    std::vector<std::unique_ptr<simulator::SimulationRunFactoryImpl>> factories;
    std::vector<simulator::PluginMatrixBinding> bindings;
    std::vector<std::string> failed_plugins;
};

[[nodiscard]] PluginBootstrap loadPlugins(const sim_io::SimulationCliArgs& args) {
    PluginBootstrap boot;
    using simulator::SimulationRunFactoryImpl;

    if (args.mode == sim_io::SimulatorMode::Comparative) {
        const auto algo_outcome = boot.loader.loadAlgorithmSo(args.algorithm);
        if (!algo_outcome.errors.empty()) {
            std::cerr << "error: failed to load algorithm " << args.algorithm << '\n';
            simulator::appendLoadErrors(boot.failed_plugins, algo_outcome);
            return boot;
        }
        const auto mc_outcome =
            boot.loader.loadMissionControlsFromDirectory(args.mission_control_folder);
        for (const auto& name : mc_outcome.errors) {
            std::cerr << "warning: mission control failed to load: " << name << '\n';
            boot.failed_plugins.push_back(name);
        }
        const auto& algorithm_factory = boot.loader.algorithmAt(0).factory;
        for (std::size_t i = 0; i < boot.loader.missionControlCount(); ++i) {
            const auto& mc = boot.loader.missionControlAt(i);
            boot.factories.push_back(
                std::make_unique<SimulationRunFactoryImpl>(algorithm_factory, mc.factory,
                                                           args.verbose));
            boot.bindings.push_back({mc.filename, std::ref(*boot.factories.back())});
        }
        return boot;
    }

    const auto mc_outcome = boot.loader.loadMissionControlSo(args.mission_control);
    if (!mc_outcome.errors.empty()) {
        std::cerr << "error: failed to load mission control " << args.mission_control << '\n';
        simulator::appendLoadErrors(boot.failed_plugins, mc_outcome);
        return boot;
    }
    const auto algo_outcome = boot.loader.loadAlgorithmsFromDirectory(args.algorithms_folder);
    for (const auto& name : algo_outcome.errors) {
        std::cerr << "warning: algorithm failed to load: " << name << '\n';
        boot.failed_plugins.push_back(name);
    }
    const auto& mission_control_factory = boot.loader.missionControlAt(0).factory;
    for (std::size_t i = 0; i < boot.loader.algorithmCount(); ++i) {
        const auto& algo = boot.loader.algorithmAt(i);
        boot.factories.push_back(std::make_unique<SimulationRunFactoryImpl>(
            algo.factory, mission_control_factory, args.verbose));
        boot.bindings.push_back({algo.filename, std::ref(*boot.factories.back())});
    }
    return boot;
}

void writeModeReport(sim_io::SimulatorMode mode, const fs::path& output_root,
                     const sim_io::SimulationCliArgs& args,
                     const simulator::types::SimulationCompositionData& composition,
                     const std::string& generated_at_utc,
                     const std::vector<simulator::PluginMatrixResult>& results,
                     const std::vector<std::string>& failed_plugins) {
    if (mode == sim_io::SimulatorMode::Comparative) {
        sim_io::ComparativeReportInput input;
        input.composition_file = composition.composition_file;
        input.mission_control_folder = args.mission_control_folder;
        input.generated_at_utc = generated_at_utc;
        input.results = results;
        input.failed_plugins = failed_plugins;
        sim_io::writeComparativeReport(output_root / "comparative_report.yaml", input);
        return;
    }
    sim_io::CompetitiveReportInput input;
    input.composition_file = composition.composition_file;
    input.mission_control = args.mission_control.filename().string();
    input.generated_at_utc = generated_at_utc;
    input.results = results;
    input.failed_plugins = failed_plugins;
    sim_io::writeCompetitiveReport(output_root / "competitive_report.yaml", input);
}

void writePerPluginSimulationYaml(
    const fs::path& output_root,
    const simulator::types::SimulationCompositionData& composition,
    const std::string& generated_at_utc,
    const std::vector<simulator::PluginMatrixResult>& results) {
    const std::vector<simulator::MatrixCell> cells =
        simulator::expandRunMatrix(composition);

    for (const auto& plugin_result : results) {
        simulator::types::SimulationManagerReport report;
        report.composition_file = composition.composition_file;
        report.generated_at_utc = generated_at_utc;
        report.metric = "maps_comparison_score_0_100";
        report.score_range = {0.0, 100.0};
        report.error_score = -1;
        report.runs = plugin_result.results;

        std::vector<sim_io::SimulationRunYamlEntry> entries;
        entries.reserve(plugin_result.results.size());
        for (std::size_t i = 0; i < plugin_result.results.size() && i < cells.size(); ++i) {
            sim_io::SimulationRunYamlEntry entry;
            entry.run_id = static_cast<int>(i);
            entry.simulation_index = cells[i].group_index;
            entry.mission_index = cells[i].mission_index;
            entry.drone_index = cells[i].drone_index;
            entry.lidar_index = cells[i].lidar_index;
            entry.result = plugin_result.results[i];
            entries.push_back(entry);
        }

        sim_io::writeSimulationOutputYaml(
            output_root / (plugin_result.plugin_filename + "_simulation_output.yaml"), report,
            entries);
    }
}

void printRunCounts(const std::vector<simulator::PluginMatrixResult>& results) {
    for (const auto& plugin_result : results) {
        std::size_t failed = 0;
        for (const auto& r : plugin_result.results) {
            if (r.mission_score < 0.0) {
                ++failed;
            }
        }
        std::cout << plugin_result.plugin_filename << ": " << plugin_result.results.size()
                  << " runs, " << failed << " unscored (score < 0)\n";
    }
}

} // namespace

int main(int argc, char** argv) {
    const sim_io::SimulationCliParseResult cli =
        sim_io::parseSimulationCliArgs(argc, argv, &std::cerr);
    if (!cli.ok) {
        return 0; // usage + errors already written to std::cerr; never exit()
    }
    const sim_io::SimulationCliArgs& args = cli.args;

    const sim_io::OutputDirKind dir_kind = args.mode == sim_io::SimulatorMode::Comparative
                                               ? sim_io::OutputDirKind::Comparative
                                               : sim_io::OutputDirKind::Competition;
    const fs::path base_folder = args.mode == sim_io::SimulatorMode::Comparative
                                     ? args.mission_control_folder
                                     : args.algorithms_folder;

    std::error_code mkdir_ec;
    const fs::path output_root = sim_io::createOutputDir(base_folder, dir_kind, mkdir_ec);
    if (mkdir_ec) {
        std::cerr << "error: could not create output directory under " << base_folder << ": "
                  << mkdir_ec.message() << '\n';
        return 0;
    }

    UC::RunErrorLog startup_log(output_root / "startup_error.log");
    const auto composition_result = sim_io::parseCompositionFile(args.simulation, startup_log);
    if (!composition_result.ok) {
        std::cerr << "error: failed to parse composition file " << args.simulation << '\n';
        for (const auto& err : composition_result.errors) {
            std::cerr << "  " << err.code << ": " << err.message << '\n';
        }
        return 0;
    }
    const simulator::types::SimulationCompositionData& composition = composition_result.value;

    PluginBootstrap plugins = loadPlugins(args);
    const std::string generated_at_utc = UC::currentUtcTimestamp();
    std::vector<simulator::PluginMatrixResult> results;
    if (plugins.bindings.empty()) {
        std::cerr << "error: no plugins loaded successfully — nothing to run\n";
    } else {
        results = simulator::runPluginMatrix(
            plugins.bindings, composition, output_root, args.num_threads.value_or(1));
        printRunCounts(results);
        writePerPluginSimulationYaml(output_root, composition, generated_at_utc, results);
    }
    writeModeReport(args.mode, output_root, args, composition, generated_at_utc, results,
                    plugins.failed_plugins);
    std::cout << "output written under: " << output_root << '\n';

    // Destroy run-related state before dlclose, per plugin-architecture.mdc ordering.
    plugins.bindings.clear();
    plugins.factories.clear();
    plugins.loader.unloadAll();
    return 0;
}
