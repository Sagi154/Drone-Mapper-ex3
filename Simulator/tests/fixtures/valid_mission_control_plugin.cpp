// Test-only MissionControl .so fixture: registers a trivial factory via the macro.
// Must NOT link Simulator registration .cpp — the ctor symbol stays undefined
// and is resolved from the test executable (ENABLE_EXPORTS).

#include <Common/IMissionControl.h>
#include <Common/MissionControlRegistration.h>

namespace FixtureMc {

class StubMissionControl final : public common::IMissionControl {
public:
    explicit StubMissionControl(common::MissionControlDependencies /*deps*/) {}

    [[nodiscard]] common::types::MissionRunResult runMission() override {
        return common::types::MissionRunResult{};
    }
};

} // namespace FixtureMc

using StubMissionControl = FixtureMc::StubMissionControl;
REGISTER_MISSION_CONTROL(StubMissionControl);
