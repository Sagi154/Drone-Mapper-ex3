#include "HostRegistrar.h"

#include <Common/MappingAlgorithmRegistration.h>

#include <utility>

namespace common {

MappingAlgorithmRegistration::MappingAlgorithmRegistration(MappingAlgorithmFactory factory) {
    ::skeleton_host::HostRegistrar::instance().storeMappingFactory(std::move(factory));
}

}  // namespace common
