// TEST-ONLY adversarial MappingAlgorithm: scan_orientation with extreme angles.
// See ASSUMPTIONS.md in this directory.

#include <Common/IMappingAlgorithm.h>
#include <Common/MappingAlgorithmFactory.h>
#include <Common/MappingAlgorithmRegistration.h>
#include <Common/Units.h>

#include <utility>

namespace {

constexpr double kExtremeDeg = 1.0e12;

}  // namespace

class AdversarialBadScanOrientationAlgorithm final : public common::IMappingAlgorithm {
public:
    explicit AdversarialBadScanOrientationAlgorithm(
        common::MappingAlgorithmDependencies dependencies)
        : common::IMappingAlgorithm(std::move(dependencies)) {}

    [[nodiscard]] common::types::MappingStepCommand nextStep(
        const common::types::DroneState&,
        const common::types::LidarScanResult*) override {
        common::types::MappingStepCommand cmd{};
        cmd.status = common::types::AlgorithmStatus::Working;
        cmd.movement = std::nullopt;
        cmd.scan_orientation = common::Orientation{
            common::HorizontalAngle{kExtremeDeg * common::deg},
            common::AltitudeAngle{-kExtremeDeg * common::deg}};
        return cmd;
    }
};

REGISTER_MAPPING_ALGORITHM(AdversarialBadScanOrientationAlgorithm);
