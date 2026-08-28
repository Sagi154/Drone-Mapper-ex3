// TEST-ONLY adversarial MissionControl: Error with implausible step count.
// See ASSUMPTIONS.md in this directory.

#include <Common/IMissionControl.h>
#include <Common/MissionControlFactory.h>
#include <Common/MissionControlRegistration.h>

namespace {

constexpr std::size_t kImplausibleSteps = 999999;

}  // namespace

class AdversarialImplausibleStepsMissionControl final : public common::IMissionControl {
public:
    explicit AdversarialImplausibleStepsMissionControl(common::MissionControlDependencies) {}

    [[nodiscard]] common::types::MissionRunResult runMission() override {
        common::types::MissionRunResult result{};
        result.status = common::types::MissionRunStatus::Error;
        result.steps = kImplausibleSteps;
        return result;
    }
};

REGISTER_MISSION_CONTROL(AdversarialImplausibleStepsMissionControl);
