// PluginRegistrar.h — Simulator-owned singleton that receives factories from
// MappingAlgorithmRegistration / MissionControlRegistration constructors.
// Pending factories are taken exactly once by the plugin loader.

#pragma once

#include <Common/MappingAlgorithmFactory.h>
#include <Common/MissionControlFactory.h>

#include <optional>

namespace simulator {

class PluginRegistrar {
public:
    [[nodiscard]] static PluginRegistrar& instance();

    PluginRegistrar(const PluginRegistrar&)            = delete;
    PluginRegistrar& operator=(const PluginRegistrar&) = delete;

    void setPendingAlgorithmFactory(const common::MappingAlgorithmFactory& factory);
    void clearPendingAlgorithmFactory();
    [[nodiscard]] std::optional<common::MappingAlgorithmFactory> takePendingAlgorithmFactory();

    void setPendingMissionControlFactory(const common::MissionControlFactory& factory);
    void clearPendingMissionControlFactory();
    [[nodiscard]] std::optional<common::MissionControlFactory> takePendingMissionControlFactory();

private:
    PluginRegistrar() = default;

    std::optional<common::MappingAlgorithmFactory> pending_algorithm_;
    std::optional<common::MissionControlFactory> pending_mission_control_;
};

} // namespace simulator
