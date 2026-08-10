// Test-only Algorithm .so fixture: registers a trivial factory via the macro.
// Must NOT link Simulator registration .cpp — the ctor symbol stays undefined
// and is resolved from the test executable (ENABLE_EXPORTS).

#include <Common/IMappingAlgorithm.h>
#include <Common/MappingAlgorithmRegistration.h>

namespace FixtureAlgo {

class StubMappingAlgorithm final : public common::IMappingAlgorithm {
public:
    explicit StubMappingAlgorithm(common::MappingAlgorithmDependencies deps)
        : common::IMappingAlgorithm(std::move(deps)) {}

    [[nodiscard]] common::types::MappingStepCommand nextStep(
        const common::types::DroneState& /*state*/,
        const common::types::LidarScanResult* /*latest_scan*/) override {
        common::types::MappingStepCommand cmd;
        cmd.status = common::types::AlgorithmStatus::Finished;
        return cmd;
    }
};

} // namespace FixtureAlgo

using StubMappingAlgorithm = FixtureAlgo::StubMappingAlgorithm;
REGISTER_MAPPING_ALGORITHM(StubMappingAlgorithm);
