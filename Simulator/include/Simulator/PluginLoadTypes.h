// PluginLoadTypes.h — loaded-plugin records and load-outcome helpers.
// Split from PluginLoader so the loader class header stays about lifetime.

#pragma once

#include <Common/MappingAlgorithmFactory.h>
#include <Common/MissionControlFactory.h>

#include <filesystem>
#include <string>
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

inline void appendLoadErrors(std::vector<std::string>& dest, const PluginLoadOutcome& outcome) {
    dest.insert(dest.end(), outcome.errors.begin(), outcome.errors.end());
}

} // namespace simulator
