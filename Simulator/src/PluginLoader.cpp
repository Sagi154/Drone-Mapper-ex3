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

} // namespace

PluginLoader::~PluginLoader() { unloadAll(); }

void PluginLoader::unloadAll() {
    algorithms_.clear();
    mission_controls_.clear();
    PluginRegistrar::instance().clearPendingAlgorithmFactory();
    PluginRegistrar::instance().clearPendingMissionControlFactory();

    for (auto& entry : handles_) {
        if (entry.handle != nullptr) {
            ::dlclose(entry.handle);
            entry.handle = nullptr;
        }
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

bool PluginLoader::tryOpen(const std::filesystem::path& so_path, std::string& canonical_out,
                           void*& handle_out, std::string& error_detail) {
    canonical_out = canonicalize(so_path);
    if (loaded_canonical_paths_.contains(canonical_out)) {
        error_detail = "already loaded (reload forbidden)";
        handle_out   = nullptr;
        return false;
    }

    ::dlerror(); // clear
    void* handle = ::dlopen(so_path.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (handle == nullptr) {
        const char* err = ::dlerror();
        error_detail    = (err != nullptr) ? err : "dlopen returned null";
        handle_out      = nullptr;
        return false;
    }

    handle_out = handle;
    return true;
}

PluginLoadOutcome PluginLoader::loadOneAlgorithm(const std::filesystem::path& so_path) {
    PluginLoadOutcome outcome;
    const std::string filename = basenameOf(so_path);

    auto& registrar = PluginRegistrar::instance();
    registrar.clearPendingAlgorithmFactory();

    std::string canonical;
    void* handle = nullptr;
    std::string detail;
    if (!tryOpen(so_path, canonical, handle, detail)) {
        std::cerr << "error: failed to load algorithm plugin '" << filename << "': " << detail
                  << '\n';
        outcome.errors.push_back(filename);
        return outcome;
    }

    auto factory = registrar.takePendingAlgorithmFactory();
    if (!factory.has_value() || !*factory) {
        std::cerr << "error: algorithm plugin '" << filename
                  << "' loaded but did not register a factory\n";
        // Clear both slots — a wrong-kind .so may have filled the MC pending slot.
        registrar.clearPendingAlgorithmFactory();
        registrar.clearPendingMissionControlFactory();
        // Record the path so a later retry cannot reload after this close.
        loaded_canonical_paths_.insert(canonical);
        ::dlclose(handle);
        outcome.errors.push_back(filename);
        return outcome;
    }

    loaded_canonical_paths_.insert(canonical);
    handles_.push_back(OpenHandle{canonical, handle});
    algorithms_.push_back(LoadedAlgorithmPlugin{filename, so_path, std::move(*factory)});
    return outcome;
}

PluginLoadOutcome PluginLoader::loadOneMissionControl(const std::filesystem::path& so_path) {
    PluginLoadOutcome outcome;
    const std::string filename = basenameOf(so_path);

    auto& registrar = PluginRegistrar::instance();
    registrar.clearPendingMissionControlFactory();

    std::string canonical;
    void* handle = nullptr;
    std::string detail;
    if (!tryOpen(so_path, canonical, handle, detail)) {
        std::cerr << "error: failed to load mission-control plugin '" << filename
                  << "': " << detail << '\n';
        outcome.errors.push_back(filename);
        return outcome;
    }

    auto factory = registrar.takePendingMissionControlFactory();
    if (!factory.has_value() || !*factory) {
        std::cerr << "error: mission-control plugin '" << filename
                  << "' loaded but did not register a factory\n";
        // Clear both slots — a wrong-kind .so may have filled the algorithm pending slot.
        registrar.clearPendingAlgorithmFactory();
        registrar.clearPendingMissionControlFactory();
        // Record the path so a later retry cannot reload after this close.
        loaded_canonical_paths_.insert(canonical);
        ::dlclose(handle);
        outcome.errors.push_back(filename);
        return outcome;
    }

    loaded_canonical_paths_.insert(canonical);
    handles_.push_back(OpenHandle{canonical, handle});
    mission_controls_.push_back(
        LoadedMissionControlPlugin{filename, so_path, std::move(*factory)});
    return outcome;
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
