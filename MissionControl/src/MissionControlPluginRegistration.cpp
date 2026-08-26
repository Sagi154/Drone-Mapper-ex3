#include <MissionControl/MissionControlImpl.h>

#include <Common/MissionControlRegistration.h>

// Macro token-pastes the argument into an identifier, so a nested name with `::`
// cannot be passed directly. Alias at global scope, then register.
using MissionControlImpl_207190406_209543255 =
    mission_control_207190406_209543255::MissionControlImpl_207190406_209543255;

REGISTER_MISSION_CONTROL(MissionControlImpl_207190406_209543255);
