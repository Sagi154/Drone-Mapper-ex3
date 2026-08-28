// TEST-ONLY adversarial MissionControl: throw from runMission.
// See ASSUMPTIONS.md in this directory.

#include <Common/IMissionControl.h>
#include <Common/MissionControlFactory.h>
#include <Common/MissionControlRegistration.h>

#include <stdexcept>

class AdversarialThrowMissionControl final : public common::IMissionControl {
public:
    explicit AdversarialThrowMissionControl(common::MissionControlDependencies) {}

    [[nodiscard]] common::types::MissionRunResult runMission() override {
        throw std::runtime_error("adversarial_throw_mission_control: runMission");
    }
};

REGISTER_MISSION_CONTROL(AdversarialThrowMissionControl);
