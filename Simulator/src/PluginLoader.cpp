#include <Simulator/PluginLoader.h>

#include <Simulator/PluginRegistrar.h>

#include <dlfcn.h>

#include <iostream>
#include <utility>

namespace simulator {
namespace {

[[nodiscard]] std::string basenameOf(const std::filesystem::path& path) {
    return path.filename().string();
}

[[nodiscard]] std::string canonicalize(const std::filesystem::path& path) {
    std::error_code ec;
    const auto canonical = std::filesystem::weakly_canonical(path, ec);
    if (!ec) {
        return canonical.string();
    }
    return std::filesystem::absolute(path, ec).lexically_normal().string();
}

// Shared dlopen → pending-factory-take → store/error-path for algorithm and mission-control.
// tryOpen is private, so callers pass a bound opener instead of PluginLoader::tryOpen.
template <typename Loaded, typename TryOpenFn, typename ClearOwnFn, typename TakeFactoryFn,
          typename BuildLoadedFn>
[[nodiscard]] PluginLoadOutcome loadOnePlugin(TryOpenFn&& try_open,
                                              const std::filesystem::path& so_path,
                                              std::vector<DlHandle>& handles,
                                              std::unordered_set<std::string>& loaded_paths,
                                              std::vector<Loaded>& storage, const char* kind_label,
                                              ClearOwnFn&& clear_own_pending,
                                              TakeFactoryFn&& take_factory,
                                              BuildLoadedFn&& build_loaded) {
    PluginLoadOutcome outcome;
    const std::string filename = basenameOf(so_path);

    auto& registrar = PluginRegistrar::instance();
    clear_own_pending(registrar);

    std::string canonical;
    std::string detail;
    DlHandle handle = try_open(so_path, canonical, detail);
    if (!handle) {
        std::cerr << "error: failed to load " << kind_label << " '" << filename << "': " << detail
                  << '\n';
        outcome.errors.push_back(filename);
        return outcome;
    }

    auto factory = take_factory(registrar);
    if (!factory.has_value() || !*factory) {
        std::cerr << "error: " << kind_label << " '" << filename
                  << "' loaded but did not register a factory\n";
        // Clear both slots — a wrong-kind .so may have filled the other pending slot.
        registrar.clearPendingAlgorithmFactory();
        registrar.clearPendingMissionControlFactory();
        // Record the path so a later retry cannot reload after this close.
        loaded_paths.insert(canonical);
        handle.reset(); // dlclose via DlHandle destructor
        outcome.errors.push_back(filename);
        return outcome;
    }

    loaded_paths.insert(canonical);
    handles.push_back(std::move(handle));
    storage.push_back(build_loaded(filename, so_path, std::move(*factory)));
    return outcome;
}

} // namespace

void DlCloser::operator()(void* handle) const noexcept {
    if (handle != nullptr) {
        ::dlclose(handle);
    }
}

PluginLoader::~PluginLoader() { unloadAll(); }

void PluginLoader::unloadAll() {
    algorithms_.clear();
    mission_controls_.clear();

    // Moved-from / never-loaded loaders must not touch the process-wide registrar
    // or dlclose handles they no longer own (DlHandle unique_ptr is already empty).
    const bool owns_session = !handles_.empty() || !loaded_canonical_paths_.empty();
    if (owns_session) {
        PluginRegistrar::instance().clearPendingAlgorithmFactory();
        PluginRegistrar::instance().clearPendingMissionControlFactory();
    }

    handles_.clear();
    loaded_canonical_paths_.clear();
}

std::vector<std::filesystem::path> PluginLoader::listSoFiles(
    const std::filesystem::path& directory) {
    std::vector<std::filesystem::path> files;
    std::error_code ec;
    for (const auto& entry : std::filesystem::directory_iterator(directory, ec)) {
        if (ec) {
            break;
        }
        std::error_code file_ec;
        if (!entry.is_regular_file(file_ec) || file_ec) {
            continue;
        }
        if (entry.path().extension() == ".so") {
            files.push_back(entry.path());
        }
    }
    return files;
}

DlHandle PluginLoader::tryOpen(const std::filesystem::path& so_path, std::string& canonical_out,
                               std::string& error_detail) const {
    canonical_out = canonicalize(so_path);
    if (loaded_canonical_paths_.contains(canonical_out)) {
        error_detail = "already loaded (reload forbidden)";
        return {};
    }

    ::dlerror(); // clear
    void* handle = ::dlopen(so_path.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (handle == nullptr) {
        const char* err = ::dlerror();
        error_detail    = (err != nullptr) ? err : "dlopen returned null";
        return {};
    }

    return DlHandle{handle};
}

PluginLoadOutcome PluginLoader::loadOneAlgorithm(const std::filesystem::path& so_path) {
    return loadOnePlugin(
        [this](const std::filesystem::path& path, std::string& canonical, std::string& detail) {
            return tryOpen(path, canonical, detail);
        },
        so_path, handles_, loaded_canonical_paths_, algorithms_, "algorithm plugin",
        [](PluginRegistrar& registrar) { registrar.clearPendingAlgorithmFactory(); },
        [](PluginRegistrar& registrar) { return registrar.takePendingAlgorithmFactory(); },
        [](std::string filename, const std::filesystem::path& path,
           common::MappingAlgorithmFactory factory) {
            return LoadedAlgorithmPlugin{std::move(filename), path, std::move(factory)};
        });
}

PluginLoadOutcome PluginLoader::loadOneMissionControl(const std::filesystem::path& so_path) {
    return loadOnePlugin(
        [this](const std::filesystem::path& path, std::string& canonical, std::string& detail) {
            return tryOpen(path, canonical, detail);
        },
        so_path, handles_, loaded_canonical_paths_, mission_controls_, "mission-control plugin",
        [](PluginRegistrar& registrar) { registrar.clearPendingMissionControlFactory(); },
        [](PluginRegistrar& registrar) { return registrar.takePendingMissionControlFactory(); },
        [](std::string filename, const std::filesystem::path& path,
           common::MissionControlFactory factory) {
            return LoadedMissionControlPlugin{std::move(filename), path, std::move(factory)};
        });
}

PluginLoadOutcome PluginLoader::loadAlgorithmSo(const std::filesystem::path& so_path) {
    return loadOneAlgorithm(so_path);
}

PluginLoadOutcome PluginLoader::loadMissionControlSo(const std::filesystem::path& so_path) {
    return loadOneMissionControl(so_path);
}

PluginLoadOutcome PluginLoader::loadAlgorithmsFromDirectory(
    const std::filesystem::path& directory) {
    PluginLoadOutcome outcome;
    for (const auto& so_path : listSoFiles(directory)) {
        auto one = loadOneAlgorithm(so_path);
        outcome.errors.insert(outcome.errors.end(), one.errors.begin(), one.errors.end());
    }
    return outcome;
}

PluginLoadOutcome PluginLoader::loadMissionControlsFromDirectory(
    const std::filesystem::path& directory) {
    PluginLoadOutcome outcome;
    for (const auto& so_path : listSoFiles(directory)) {
        auto one = loadOneMissionControl(so_path);
        outcome.errors.insert(outcome.errors.end(), one.errors.begin(), one.errors.end());
    }
    return outcome;
}

} // namespace simulator
