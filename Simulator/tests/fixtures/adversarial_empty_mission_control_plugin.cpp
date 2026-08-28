// TEST-ONLY adversarial MissionControl: Completed, steps 0, immediately.
// See ASSUMPTIONS.md in this directory.

#include <Common/IMissionControl.h>
#include <Common/MissionControlFactory.h>
#include <Common/MissionControlRegistration.h>

class AdversarialEmptyMissionControl final : public common::IMissionControl {
public:
    explicit AdversarialEmptyMissionControl(common::MissionControlDependencies) {}

    [[nodiscard]] common::types::MissionRunResult runMission() override {
        common::types::MissionRunResult result{};
        result.status = common::types::MissionRunStatus::Completed;
        result.steps = 0;
        return result;
    }
};

REGISTER_MISSION_CONTROL(AdversarialEmptyMissionControl);
