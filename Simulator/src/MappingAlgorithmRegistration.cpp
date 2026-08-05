// MappingAlgorithmRegistration.cpp — constructor body lives in the Simulator
// only. Plugins leave this symbol undefined so dlopen resolves it here.

#include <Common/MappingAlgorithmRegistration.h>

#include <Simulator/PluginRegistrar.h>

#include <utility>

namespace common {

MappingAlgorithmRegistration::MappingAlgorithmRegistration(MappingAlgorithmFactory factory) {
    simulator::PluginRegistrar::instance().setPendingAlgorithmFactory(std::move(factory));
}

} // namespace common
