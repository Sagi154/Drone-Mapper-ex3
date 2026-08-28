#include "HostRegistrar.h"

#include <Common/MissionControlRegistration.h>

#include <utility>

namespace common {

MissionControlRegistration::MissionControlRegistration(MissionControlFactory factory) {
    ::skeleton_host::HostRegistrar::instance().storeMissionControlFactory(std::move(factory));
}

}  // namespace common
