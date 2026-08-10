// StubPluginRegistration.cpp
// Satisfies the MappingAlgorithmRegistration linker symbol when algorithm
// sources are compiled directly into a unit-test executable (not loaded via
// dlopen). In production the Simulator provides the real constructor body.

#include <Common/MappingAlgorithmFactory.h>
#include <Common/MappingAlgorithmRegistration.h>

namespace common {
MappingAlgorithmRegistration::MappingAlgorithmRegistration(
    MappingAlgorithmFactory /*factory*/) {
    // no-op: tests exercise the algorithm class directly, not via the registrar
}
} // namespace common
