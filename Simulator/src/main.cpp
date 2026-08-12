#include <Simulator/PluginLoader.h>
#include <Simulator/RunMatrixOrchestrator.h>
#include <Simulator/SimulationRunFactoryImpl.h>
#include <Simulator/io/SimulationCli.h>
#include <Simulator/io/YamlConfigParsers.h>

#include <UserCommon_207190406_209543255/RunErrorLog.h>

#include <chrono>
#include <filesystem>
#include <iostream>
#include <memory>
#include <vector>

namespace {

namespace fs = std::filesystem;
namespace sim_io = simulator::io;
namespace UC = UserCommon_207190406_209543255;

[[nodiscard]] fs::path makeTempOutputRoot() {
    const auto now = std::chrono::system_clock::now();
    const auto seconds =
        std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
    return fs::temp_directory_path() / ("simulator_vertical_slice_" + std::to_string(seconds));
}

} // namespace

int main(int argc, char** argv) {
    const sim_io::SimulationCliParseResult cli =
        sim_io::parseSimulationCliArgs(argc, argv, &std::cerr);
    if (!cli.ok) {
        return 0; // usage + errors already written to std::cerr; never exit()
    }
    const sim_io::SimulationCliArgs& args = cli.args;

    const fs::path output_root = makeTempOutputRoot();
    std::error_code mkdir_ec;
    fs::create_directories(output_root, mkdir_ec);
    if (mkdir_ec) {
        std::cerr << "error: could not create output directory " << output_root << ": "
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

    simulator::PluginLoader loader;
    std::vector<simulator::PluginMatrixBinding> bindings;
    // Factories are non-owning per PluginMatrixBinding — keep them alive here.
    std::vector<std::unique_ptr<simulator::SimulationRunFactoryImpl>> factories;

    if (args.mode == sim_io::SimulatorMode::Comparative) {
        const auto algo_outcome = loader.loadAlgorithmSo(args.algorithm);
        if (!algo_outcome.errors.empty()) {
            std::cerr << "error: failed to load algorithm " << args.algorithm << '\n';
            return 0;
        }
        const auto mc_outcome =
            loader.loadMissionControlsFromDirectory(args.mission_control_folder);
        for (const auto& name : mc_outcome.errors) {
            std::cerr << "warning: mission control failed to load: " << name << '\n';
        }
        const auto& algorithm_factory = loader.algorithms().front().factory;
        for (const auto& mc : loader.missionControls()) {
            factories.push_back(std::make_unique<simulator::SimulationRunFactoryImpl>(
                algorithm_factory, mc.factory));
            bindings.push_back({mc.filename, factories.back().get()});
        }
    } else {
        const auto mc_outcome = loader.loadMissionControlSo(args.mission_control);
        if (!mc_outcome.errors.empty()) {
            std::cerr << "error: failed to load mission control " << args.mission_control << '\n';
            return 0;
        }
        const auto algo_outcome = loader.loadAlgorithmsFromDirectory(args.algorithms_folder);
        for (const auto& name : algo_outcome.errors) {
            std::cerr << "warning: algorithm failed to load: " << name << '\n';
        }
        const auto& mission_control_factory = loader.missionControls().front().factory;
        for (const auto& algo : loader.algorithms()) {
            factories.push_back(std::make_unique<simulator::SimulationRunFactoryImpl>(
                algo.factory, mission_control_factory));
            bindings.push_back({algo.filename, factories.back().get()});
        }
    }

    if (bindings.empty()) {
        std::cerr << "error: no plugins loaded successfully — nothing to run\n";
        return 0;
    }

    const unsigned num_threads = args.num_threads.value_or(1);
    const std::vector<simulator::PluginMatrixResult> results =
        simulator::RunMatrixOrchestrator::run(bindings, composition, output_root, num_threads);

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

    std::cout << "output written under: " << output_root << '\n';

    // Destroy run-related state before dlclose, per plugin-architecture.mdc ordering.
    bindings.clear();
    factories.clear();
    loader.unloadAll();

    return 0;
}
