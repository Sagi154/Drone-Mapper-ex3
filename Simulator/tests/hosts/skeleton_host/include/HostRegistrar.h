#pragma once

#include <Common/MappingAlgorithmFactory.h>
#include <Common/MissionControlFactory.h>

namespace skeleton_host {

// Singleton that receives factories from the registration constructors
// invoked as static initializers during dlopen of a plugin.
class HostRegistrar {
public:
    static HostRegistrar& instance();

    HostRegistrar(const HostRegistrar&) = delete;
    HostRegistrar& operator=(const HostRegistrar&) = delete;

    void storeMappingFactory(common::MappingAlgorithmFactory factory);
    void storeMissionControlFactory(common::MissionControlFactory factory);

    [[nodiscard]] common::MappingAlgorithmFactory takeMappingFactory();
    [[nodiscard]] common::MissionControlFactory takeMissionControlFactory();

    void clear();

private:
    HostRegistrar() = default;

    common::MappingAlgorithmFactory mapping_factory_{};
    common::MissionControlFactory mission_control_factory_{};
};

}  // namespace skeleton_host
