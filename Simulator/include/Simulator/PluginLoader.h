// PluginLoader.h — dlopen/dlclose of Algorithm / MissionControl .so plugins.
// Load on the main thread only. Never reloads a path already held open.

#pragma once

#include <Common/MappingAlgorithmFactory.h>
#include <Common/MissionControlFactory.h>

#include <filesystem>
#include <string>
#include <unordered_set>
#include <vector>

namespace simulator {

struct LoadedAlgorithmPlugin {
    std::string filename; // basename only — report identity
    std::filesystem::path path;
    common::MappingAlgorithmFactory factory;
};

struct LoadedMissionControlPlugin {
    std::string filename;
    std::filesystem::path path;
    common::MissionControlFactory factory;
};

struct PluginLoadOutcome {
    std::vector<std::string> errors; // basenames that failed this call
};

class PluginLoader {
public:
    PluginLoader() = default;
    ~PluginLoader();

    PluginLoader(const PluginLoader&)            = delete;
    PluginLoader& operator=(const PluginLoader&) = delete;

    [[nodiscard]] PluginLoadOutcome loadAlgorithmSo(const std::filesystem::path& so_path);
    [[nodiscard]] PluginLoadOutcome loadAlgorithmsFromDirectory(
        const std::filesystem::path& directory);

    [[nodiscard]] PluginLoadOutcome loadMissionControlSo(const std::filesystem::path& so_path);
    [[nodiscard]] PluginLoadOutcome loadMissionControlsFromDirectory(
        const std::filesystem::path& directory);

    [[nodiscard]] const std::vector<LoadedAlgorithmPlugin>& algorithms() const {
        return algorithms_;
    }
    [[nodiscard]] const std::vector<LoadedMissionControlPlugin>& missionControls() const {
        return mission_controls_;
    }

    /// Destroys stored factories, then dlclose every handle. Safe to call multiple times.
    void unloadAll();

private:
    struct OpenHandle {
        std::string canonical;
        void* handle = nullptr;
    };

    [[nodiscard]] PluginLoadOutcome loadOneAlgorithm(const std::filesystem::path& so_path);
    [[nodiscard]] PluginLoadOutcome loadOneMissionControl(const std::filesystem::path& so_path);
    [[nodiscard]] bool tryOpen(const std::filesystem::path& so_path, std::string& canonical_out,
                               void*& handle_out, std::string& error_detail);
    [[nodiscard]] static std::vector<std::filesystem::path> listSoFiles(
        const std::filesystem::path& directory);

    std::vector<LoadedAlgorithmPlugin> algorithms_;
    std::vector<LoadedMissionControlPlugin> mission_controls_;
    std::vector<OpenHandle> handles_;
    std::unordered_set<std::string> loaded_canonical_paths_;
};

} // namespace simulator
