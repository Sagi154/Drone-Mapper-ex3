// MissionControlRegistration.cpp — constructor body lives in the Simulator
// only. Plugins leave this symbol undefined so dlopen resolves it here.

#include <Common/MissionControlRegistration.h>

#include <Simulator/PluginRegistrar.h>

#include <utility>

namespace common {

MissionControlRegistration::MissionControlRegistration(MissionControlFactory factory) {
    simulator::PluginRegistrar::instance().setPendingMissionControlFactory(std::move(factory));
}

} // namespace common
