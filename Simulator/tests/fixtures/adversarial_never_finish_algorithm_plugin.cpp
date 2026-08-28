// TEST-ONLY adversarial MappingAlgorithm: never finishes (Working + Hover).
// See ASSUMPTIONS.md in this directory.

#include <Common/IMappingAlgorithm.h>
#include <Common/MappingAlgorithmFactory.h>
#include <Common/MappingAlgorithmRegistration.h>

#include <utility>

class AdversarialNeverFinishAlgorithm final : public common::IMappingAlgorithm {
public:
    explicit AdversarialNeverFinishAlgorithm(common::MappingAlgorithmDependencies dependencies)
        : common::IMappingAlgorithm(std::move(dependencies)) {}

    [[nodiscard]] common::types::MappingStepCommand nextStep(
        const common::types::DroneState&,
        const common::types::LidarScanResult*) override {
        common::types::MappingStepCommand cmd{};
        cmd.status = common::types::AlgorithmStatus::Working;
        common::types::MovementCommand hover{};
        hover.type = common::types::MovementCommandType::Hover;
        cmd.movement = hover;
        cmd.scan_orientation = std::nullopt;
        return cmd;
    }
};

REGISTER_MAPPING_ALGORITHM(AdversarialNeverFinishAlgorithm);
