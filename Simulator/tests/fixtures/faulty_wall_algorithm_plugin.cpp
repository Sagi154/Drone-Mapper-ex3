// Test-only Algorithm .so: always Advance so MockMovement can throw on a real wall.
#include <Common/IMappingAlgorithm.h>
#include <Common/MappingAlgorithmRegistration.h>
#include <Common/Units.h>

namespace FixtureFaultyWall {

class FaultyWallAlgorithm final : public common::IMappingAlgorithm {
public:
    explicit FaultyWallAlgorithm(common::MappingAlgorithmDependencies deps)
        : common::IMappingAlgorithm(std::move(deps)) {}

    [[nodiscard]] common::types::MappingStepCommand nextStep(
        const common::types::DroneState& /*state*/,
        const common::types::LidarScanResult* /*latest_scan*/) override {
        using namespace mp_units::si::unit_symbols;
        common::types::MappingStepCommand cmd;
        cmd.status = common::types::AlgorithmStatus::Working;
        cmd.movement = common::types::MovementCommand{
            .type = common::types::MovementCommandType::Advance,
            .distance = 50.0 * cm,
        };
        return cmd;
    }
};

} // namespace FixtureFaultyWall

using FaultyWallAlgorithm = FixtureFaultyWall::FaultyWallAlgorithm;
REGISTER_MAPPING_ALGORITHM(FaultyWallAlgorithm);
