#pragma once

#include <MissionControl/IDroneControl.h>

#include <Common/Types.h>

#include <filesystem>

namespace mission_control_207190406_209543255 {

[[nodiscard]] common::types::MissionRunResult runMissionSteps(
    mission_control::IDroneControl& drone_control, std::size_t max_steps,
    const std::filesystem::path& output_map_file, bool verbose);

} // namespace mission_control_207190406_209543255
