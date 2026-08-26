#pragma once

#include <user_common_207190406_209543255/ConfigParseResult.h>
#include <user_common_207190406_209543255/IRunErrorLog.h>

#include <Common/types/DroneTypes.h>
#include <Common/types/LidarTypes.h>
#include <Common/types/MissionTypes.h>
#include <Simulator/SimulationTypes.h>

#include <filesystem>

namespace simulator::io {

/// Each function parses one config file and returns a ConfigParseResult<T>.
/// Missing files and parse errors are logged via IRunErrorLog and reflected
/// in the result — callers should inspect result.ok and result.errors.

[[nodiscard]] user_common_207190406_209543255::ConfigParseResult<common::types::DroneConfigData>
parseDroneConfig(const std::filesystem::path& path,
                 user_common_207190406_209543255::IRunErrorLog& log);

[[nodiscard]] user_common_207190406_209543255::ConfigParseResult<common::types::LidarConfigData>
parseLidarConfig(const std::filesystem::path& path,
                 user_common_207190406_209543255::IRunErrorLog& log);

[[nodiscard]] user_common_207190406_209543255::ConfigParseResult<common::types::MissionConfigData>
parseMissionConfig(const std::filesystem::path& path,
                   user_common_207190406_209543255::IRunErrorLog& log);

[[nodiscard]] user_common_207190406_209543255::ConfigParseResult<simulator::types::SimulationConfigData>
parseSimulationConfig(const std::filesystem::path& path,
                      user_common_207190406_209543255::IRunErrorLog& log);

[[nodiscard]] user_common_207190406_209543255::ConfigParseResult<simulator::types::SimulationCompositionData>
parseCompositionFile(const std::filesystem::path& path,
                     user_common_207190406_209543255::IRunErrorLog& log);

} // namespace simulator::io
