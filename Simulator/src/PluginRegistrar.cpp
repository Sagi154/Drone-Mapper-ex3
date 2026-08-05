#include <Simulator/PluginRegistrar.h>

#include <utility>

namespace simulator {

PluginRegistrar& PluginRegistrar::instance() {
    static PluginRegistrar registrar;
    return registrar;
}

void PluginRegistrar::setPendingAlgorithmFactory(common::MappingAlgorithmFactory factory) {
    pending_algorithm_ = std::move(factory);
}

void PluginRegistrar::clearPendingAlgorithmFactory() {
    pending_algorithm_.reset();
}

std::optional<common::MappingAlgorithmFactory> PluginRegistrar::takePendingAlgorithmFactory() {
    auto taken = std::move(pending_algorithm_);
    pending_algorithm_.reset();
    return taken;
}

void PluginRegistrar::setPendingMissionControlFactory(common::MissionControlFactory factory) {
    pending_mission_control_ = std::move(factory);
}

void PluginRegistrar::clearPendingMissionControlFactory() {
    pending_mission_control_.reset();
}

std::optional<common::MissionControlFactory> PluginRegistrar::takePendingMissionControlFactory() {
    auto taken = std::move(pending_mission_control_);
    pending_mission_control_.reset();
    return taken;
}

} // namespace simulator
