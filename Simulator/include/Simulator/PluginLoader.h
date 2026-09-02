// PluginLoader.h — dlopen/dlclose of Algorithm / MissionControl .so plugins.
// Load on the main thread only. Never reloads a path already held open.

#pragma once

#include <Simulator/PluginLoadTypes.h>

#include <dlfcn.h>

#include <cstddef>
#include <filesystem>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

namespace simulator {

struct DlCloser {
    void operator()(void* handle) const noexcept {
        if (handle != nullptr) {
            ::dlclose(handle);
        }
    }
};
using DlHandle = std::unique_ptr<void, DlCloser>;

class PluginLoader {
public:
    PluginLoader() = default;
    ~PluginLoader();

    PluginLoader(const PluginLoader&)            = delete;
    PluginLoader& operator=(const PluginLoader&) = delete;
    PluginLoader(PluginLoader&&) noexcept        = default;
    PluginLoader& operator=(PluginLoader&&)      = delete;

    [[nodiscard]] PluginLoadOutcome loadAlgorithmSo(const std::filesystem::path& so_path);
    [[nodiscard]] PluginLoadOutcome loadAlgorithmsFromDirectory(
        const std::filesystem::path& directory);

    [[nodiscard]] PluginLoadOutcome loadMissionControlSo(const std::filesystem::path& so_path);
    [[nodiscard]] PluginLoadOutcome loadMissionControlsFromDirectory(
        const std::filesystem::path& directory);

    [[nodiscard]] std::size_t algorithmCount() const { return algorithms_.size(); }
    [[nodiscard]] const LoadedAlgorithmPlugin& algorithmAt(std::size_t i) const {
        return algorithms_.at(i);
    }
    [[nodiscard]] std::size_t missionControlCount() const { return mission_controls_.size(); }
    [[nodiscard]] const LoadedMissionControlPlugin& missionControlAt(std::size_t i) const {
        return mission_controls_.at(i);
    }

    /// Destroys stored factories, then dlclose every handle. Safe to call multiple times.
    void unloadAll();

private:
    [[nodiscard]] PluginLoadOutcome loadOneAlgorithm(const std::filesystem::path& so_path);
    [[nodiscard]] PluginLoadOutcome loadOneMissionControl(const std::filesystem::path& so_path);
    [[nodiscard]] DlHandle tryOpen(const std::filesystem::path& so_path, std::string& canonical_out,
                                   std::string& error_detail);
    [[nodiscard]] static std::vector<std::filesystem::path> listSoFiles(
        const std::filesystem::path& directory);

    std::vector<LoadedAlgorithmPlugin> algorithms_;
    std::vector<LoadedMissionControlPlugin> mission_controls_;
    std::vector<DlHandle> handles_;
    std::unordered_set<std::string> loaded_canonical_paths_;
};

} // namespace simulator
