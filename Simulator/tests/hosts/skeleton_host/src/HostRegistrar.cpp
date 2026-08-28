#include "HostRegistrar.h"

#include <utility>

namespace skeleton_host {

HostRegistrar& HostRegistrar::instance() {
    static HostRegistrar registrar;
    return registrar;
}

void HostRegistrar::storeMappingFactory(common::MappingAlgorithmFactory factory) {
    mapping_factory_ = std::move(factory);
}

void HostRegistrar::storeMissionControlFactory(common::MissionControlFactory factory) {
    mission_control_factory_ = std::move(factory);
}

common::MappingAlgorithmFactory HostRegistrar::takeMappingFactory() {
    common::MappingAlgorithmFactory factory = std::move(mapping_factory_);
    mapping_factory_ = {};
    return factory;
}

common::MissionControlFactory HostRegistrar::takeMissionControlFactory() {
    common::MissionControlFactory factory = std::move(mission_control_factory_);
    mission_control_factory_ = {};
    return factory;
}

void HostRegistrar::clear() {
    mapping_factory_ = {};
    mission_control_factory_ = {};
}

}  // namespace skeleton_host
