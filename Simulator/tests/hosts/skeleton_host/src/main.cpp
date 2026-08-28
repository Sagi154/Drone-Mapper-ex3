#include "HostConfig.h"
#include "HostGPS.h"
#include "HostLidar.h"
#include "HostMap3D.h"
#include "HostMovement.h"
#include "HostNpy.h"
#include "HostRegistrar.h"
#include "HostUnits.h"

#include <Common/IMappingAlgorithm.h>
#include <Common/IMissionControl.h>
#include <Common/MissionControlFactory.h>

#include <dlfcn.h>

#include <exception>
#include <filesystem>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace {

struct CliArgs {
    std::filesystem::path algorithm;
    std::filesystem::path mission_control;
    std::filesystem::path simulation;
    std::filesystem::path mission;
    std::optional<std::filesystem::path> drone;
    std::optional<std::filesystem::path> lidar;
};

void printUsage(std::ostream& out) {
    out << "Usage: skeleton_host --algorithm=<so> --mission-control=<so> "
           "--simulation=<yaml> --mission=<yaml> [--drone=<yaml>] [--lidar=<yaml>]\n";
}

bool startsWith(std::string_view s, std::string_view prefix) {
    return s.size() >= prefix.size() && s.substr(0, prefix.size()) == prefix;
}

std::optional<std::string_view> flagValue(std::string_view arg, std::string_view name) {
    const std::string prefix = std::string("--") + std::string(name) + "=";
    if (!startsWith(arg, prefix)) {
        return std::nullopt;
    }
    return arg.substr(prefix.size());
}

int parseCli(int argc, char** argv, CliArgs& args, std::string& error) {
    std::vector<std::string> unsupported;
    bool have_algorithm = false;
    bool have_mc = false;
    bool have_sim = false;
    bool have_mission = false;

    for (int i = 1; i < argc; ++i) {
        const std::string_view arg{argv[i]};
        if (auto v = flagValue(arg, "algorithm")) {
            args.algorithm = std::filesystem::path{*v};
            have_algorithm = true;
            continue;
        }
        if (auto v = flagValue(arg, "mission-control")) {
            args.mission_control = std::filesystem::path{*v};
            have_mc = true;
            continue;
        }
        if (auto v = flagValue(arg, "simulation")) {
            args.simulation = std::filesystem::path{*v};
            have_sim = true;
            continue;
        }
        if (auto v = flagValue(arg, "mission")) {
            args.mission = std::filesystem::path{*v};
            have_mission = true;
            continue;
        }
        if (auto v = flagValue(arg, "drone")) {
            args.drone = std::filesystem::path{*v};
            continue;
        }
        if (auto v = flagValue(arg, "lidar")) {
            args.lidar = std::filesystem::path{*v};
            continue;
        }
        unsupported.emplace_back(arg);
    }

    if (!unsupported.empty()) {
        error = "unsupported argument(s):";
        for (const auto& u : unsupported) {
            error += " " + u;
        }
        return 1;
    }

    std::string missing;
    if (!have_algorithm) {
        missing += " --algorithm";
    }
    if (!have_mc) {
        missing += " --mission-control";
    }
    if (!have_sim) {
        missing += " --simulation";
    }
    if (!have_mission) {
        missing += " --mission";
    }
    if (!missing.empty()) {
        error = "missing required argument(s):" + missing;
        return 1;
    }
    return 0;
}

bool missingFile(const std::filesystem::path& path, std::string& error) {
    if (!std::filesystem::is_regular_file(path)) {
        error = "missing or unreadable file: " + path.string();
        return true;
    }
    return false;
}

struct DlHandle {
    void* handle = nullptr;

    DlHandle() = default;
    explicit DlHandle(void* h) : handle(h) {}
    DlHandle(const DlHandle&) = delete;
    DlHandle& operator=(const DlHandle&) = delete;
    DlHandle(DlHandle&& other) noexcept : handle(other.handle) { other.handle = nullptr; }
    DlHandle& operator=(DlHandle&& other) noexcept {
        if (this != &other) {
            reset();
            handle = other.handle;
            other.handle = nullptr;
        }
        return *this;
    }
    ~DlHandle() { reset(); }

    void reset() {
        if (handle != nullptr) {
            dlclose(handle);
            handle = nullptr;
        }
    }
};

DlHandle openPlugin(const std::filesystem::path& path, std::string& error) {
    const auto abs = std::filesystem::absolute(path);
    void* handle = dlopen(abs.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (handle == nullptr) {
        const char* dlerr = dlerror();
        error = "dlopen failed for " + abs.string() + ": " + (dlerr ? dlerr : "unknown");
        return {};
    }
    return DlHandle{handle};
}

const char* statusText(common::types::MissionRunStatus status) {
    switch (status) {
        case common::types::MissionRunStatus::Completed:
            return "Completed";
        case common::types::MissionRunStatus::MaxSteps:
            return "MaxSteps";
        case common::types::MissionRunStatus::Error:
            return "Error";
    }
    return "Error";
}

}  // namespace

int main(int argc, char** argv) {
    CliArgs args;
    std::string error;
    if (parseCli(argc, argv, args, error) != 0) {
        std::cerr << error << '\n';
        printUsage(std::cerr);
        return 1;
    }

    if (missingFile(args.algorithm, error) || missingFile(args.mission_control, error) ||
        missingFile(args.simulation, error) || missingFile(args.mission, error)) {
        std::cerr << error << '\n';
        printUsage(std::cerr);
        return 1;
    }

    const auto inputs_root = args.simulation.parent_path().parent_path();
    const std::filesystem::path drone_path =
        args.drone.value_or(inputs_root / "drone" / "drone_small.yaml");
    const std::filesystem::path lidar_path =
        args.lidar.value_or(inputs_root / "lidar" / "lidar_short.yaml");
    if (missingFile(drone_path, error) || missingFile(lidar_path, error)) {
        std::cerr << error << '\n';
        printUsage(std::cerr);
        return 1;
    }

    skeleton_host::LoadedConfigs configs;
    try {
        configs = skeleton_host::loadStaffConfigs(args.simulation, args.mission, drone_path,
                                                  lidar_path);
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << '\n';
        printUsage(std::cerr);
        return 1;
    }

    std::unique_ptr<skeleton_host::HostMap3D> hidden_map;
    std::unique_ptr<skeleton_host::HostMap3D> output_map;
    try {
        hidden_map = std::make_unique<skeleton_host::HostMap3D>(
            skeleton_host::loadHiddenMapFromNpy(configs.map_path, configs.simulation));
        output_map = std::make_unique<skeleton_host::HostMap3D>(
            skeleton_host::makeEmptyOutputMap(configs.mission, configs.simulation));
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << '\n';
        printUsage(std::cerr);
        return 1;
    }

    skeleton_host::HostGPS gps(configs.simulation.initial_drone_position,
                               common::Orientation{configs.simulation.initial_angle,
                                                   skeleton_host::alt_deg(0.0)},
                               configs.mission.gps_resolution);
    skeleton_host::HostLidar lidar(gps, *hidden_map, configs.lidar);
    skeleton_host::HostMovement movement(gps, *hidden_map, configs.mission.mission_bounds,
                                         configs.drone.radius);

    auto& registrar = skeleton_host::HostRegistrar::instance();
    registrar.clear();

    DlHandle algorithm_so = openPlugin(args.algorithm, error);
    if (algorithm_so.handle == nullptr) {
        std::cerr << error << '\n';
        printUsage(std::cerr);
        return 1;
    }
    auto mapping_factory = registrar.takeMappingFactory();
    if (!mapping_factory) {
        std::cerr << "plugin did not register a MappingAlgorithmFactory: "
                  << args.algorithm.string() << '\n';
        printUsage(std::cerr);
        return 1;
    }

    DlHandle mission_control_so = openPlugin(args.mission_control, error);
    if (mission_control_so.handle == nullptr) {
        std::cerr << error << '\n';
        printUsage(std::cerr);
        return 1;
    }
    auto mission_factory = registrar.takeMissionControlFactory();
    if (!mission_factory) {
        std::cerr << "plugin did not register a MissionControlFactory: "
                  << args.mission_control.string() << '\n';
        printUsage(std::cerr);
        return 1;
    }

    common::types::MissionRunResult result;
    result.status = common::types::MissionRunStatus::Error;
    {
        std::unique_ptr<common::IMappingAlgorithm> algorithm;
        std::unique_ptr<common::IMissionControl> mission_control;
        try {
            common::MappingAlgorithmDependencies mapping_deps{
                configs.mission, configs.lidar, configs.drone, *output_map};
            algorithm = mapping_factory(std::move(mapping_deps));
            if (!algorithm) {
                throw std::runtime_error("MappingAlgorithm factory returned null");
            }

            const std::filesystem::path output_map_file{"skeleton_host_output.npy"};
            common::MissionControlDependencies mc_deps{configs.mission,
                                                       configs.drone,
                                                       lidar,
                                                       gps,
                                                       movement,
                                                       *output_map,
                                                       *algorithm,
                                                       output_map_file,
                                                       false};
            mission_control = mission_factory(std::move(mc_deps));
            if (!mission_control) {
                throw std::runtime_error("MissionControl factory returned null");
            }
            result = mission_control->runMission();
        } catch (const std::exception& ex) {
            result.status = common::types::MissionRunStatus::Error;
            result.errors.push_back({"HOST_EXCEPTION", ex.what()});
        }
        mission_control.reset();
        algorithm.reset();
    }

    // Factories close over plugin code. Destroy them before dlclose.
    mapping_factory = {};
    mission_factory = {};
    registrar.clear();

    const auto counts = output_map->countVoxels();
    std::cout << "HOST_STATUS=" << statusText(result.status) << '\n';
    std::cout << "HOST_STEPS=" << result.steps << '\n';
    std::cout << "HOST_VOXELS_EMPTY=" << counts.empty << '\n';
    std::cout << "HOST_VOXELS_OCCUPIED=" << counts.occupied << '\n';
    std::cout << "HOST_VOXELS_UNMAPPED=" << counts.unmapped << '\n';
    std::cout << "HOST_ILLEGAL_MOVE_ATTEMPTS=" << movement.illegalMoveAttempts() << '\n';
    std::cout.flush();

    mission_control_so.reset();
    algorithm_so.reset();
    return 0;
}
