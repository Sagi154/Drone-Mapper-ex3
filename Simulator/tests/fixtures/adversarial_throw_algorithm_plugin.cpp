// TEST-ONLY adversarial MappingAlgorithm: throw from nextStep.
// See ASSUMPTIONS.md in this directory.

#include <Common/IMappingAlgorithm.h>
#include <Common/MappingAlgorithmFactory.h>
#include <Common/MappingAlgorithmRegistration.h>

#include <stdexcept>
#include <utility>

class AdversarialThrowAlgorithm final : public common::IMappingAlgorithm {
public:
    explicit AdversarialThrowAlgorithm(common::MappingAlgorithmDependencies dependencies)
        : common::IMappingAlgorithm(std::move(dependencies)) {}

    [[nodiscard]] common::types::MappingStepCommand nextStep(
        const common::types::DroneState&,
        const common::types::LidarScanResult*) override {
        throw std::runtime_error("adversarial_throw_algorithm: nextStep");
    }
};

REGISTER_MAPPING_ALGORITHM(AdversarialThrowAlgorithm);
