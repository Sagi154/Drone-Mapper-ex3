// SimulationRunImpl.cpp
// Drives one simulation run.
//
// Exceptions from runMission() are contained here so the output map can still
// be saved and the matrix can continue. Scoring uses MapsComparison with a
// world-space spawn from SimulationCoordUtil.

#include <Simulator/SimulationRunImpl.h>

#include <Simulator/MapsComparison.h>
#include <Simulator/OutputPathUtil.h>

#include <user_common_207190406_209543255/RunErrorLog.h>
#include <user_common_207190406_209543255/SimulationCoordUtil.h>

#include <memory>
#include <stdexcept>
#include <vector>

namespace simulator {

namespace {

[[nodiscard]] types::ResolutionRequestStatus resolutionStatus(double factor) {
    if (factor < 1.0) {
        return types::ResolutionRequestStatus::IgnoredTooSmall;
    }
    if (factor > 1.0) {
        return types::ResolutionRequestStatus::Accepted;
    }
    return types::ResolutionRequestStatus::Ignored;
}

void logErrors(user_common_207190406_209543255::RunErrorLog* log,
               const std::vector<common::types::ErrorRef>& errors) {
    if (log == nullptr) {
        return;
    }
    for (const auto& error : errors) {
        log->log(error);
    }
}

} // namespace

SimulationRunImpl::SimulationRunImpl(
    std::unique_ptr<common::IMap3D>            hidden_map,
    std::unique_ptr<common::IMutableMap3D>     output_map,
    std::unique_ptr<common::IGPS>              gps,
    std::unique_ptr<common::IDroneMovement>    movement,
    std::unique_ptr<common::ILidar>            lidar,
    std::unique_ptr<common::IMappingAlgorithm> mapping_algorithm,
    std::unique_ptr<common::IMissionControl>   mission_control,
    types::SimulationConfigData                simulation_config,
    common::types::MissionConfigData           mission_config,
    std::filesystem::path                      output_map_file,
    std::vector<common::types::ErrorRef>       startup_errors)
    : hidden_map_(std::move(hidden_map)),
      output_map_(std::move(output_map)),
      gps_(std::move(gps)),
      movement_(std::move(movement)),
      lidar_(std::move(lidar)),
      mapping_algorithm_(std::move(mapping_algorithm)),
      mission_control_(std::move(mission_control)),
      simulation_config_(std::move(simulation_config)),
      mission_config_(std::move(mission_config)),
      output_map_file_(std::move(output_map_file)),
      startup_errors_(std::move(startup_errors)) {
    if (!hidden_map_ || !output_map_ || !gps_ || !movement_ ||
        !lidar_ || !mapping_algorithm_ || !mission_control_) {
        throw std::invalid_argument("SimulationRunImpl: all dependencies must be non-null.");
    }
}

types::SimulationResult SimulationRunImpl::run() {
    types::SimulationResult result{};
    result.simulation_config = simulation_config_;
    result.mission_config    = mission_config_;
    result.output_map_file   = output_map_file_;
    result.resolution_request_status =
        resolutionStatus(mission_config_.output_mapping_resolution_factor);

    std::unique_ptr<user_common_207190406_209543255::RunErrorLog> error_log;
    if (!output_map_file_.empty()) {
        error_log = std::make_unique<user_common_207190406_209543255::RunErrorLog>(
            errorLogPathFromOutputMap(output_map_file_));
    }

    if (!startup_errors_.empty()) {
        logErrors(error_log.get(), startup_errors_);
        result.mission_score = -1.0;
        result.mission_results.push_back(common::types::MissionRunResult{
            common::types::MissionRunStatus::Error,
            0,
            startup_errors_,
        });
        return result;
    }

    // MockMovement wall collisions throw; DroneControl lets them through so this
    // boundary can contain them, still save the output map, and return score -1.
    common::types::MissionRunResult mission_result;
    try {
        mission_result = mission_control_->runMission();
    } catch (const std::exception& ex) {
        mission_result = common::types::MissionRunResult{
            common::types::MissionRunStatus::Error,
            0,
            {common::types::ErrorRef{"MISSION_EXCEPTION", ex.what()}},
        };
    } catch (...) {
        mission_result = common::types::MissionRunResult{
            common::types::MissionRunStatus::Error,
            0,
            {common::types::ErrorRef{"MISSION_EXCEPTION", "unknown exception from runMission"}},
        };
    }
    result.mission_results.push_back(mission_result);
    logErrors(error_log.get(), mission_result.errors);
    result.output_map_config = output_map_->getMapConfig();

    if (!output_map_file_.empty()) {
        std::error_code ec;
        std::filesystem::create_directories(output_map_file_.parent_path(), ec);
        try {
            output_map_->save(output_map_file_);
        } catch (const std::exception& ex) {
            const common::types::ErrorRef save_error{"MAP_SAVE_FAILED", ex.what()};
            logErrors(error_log.get(), {save_error});
            result.mission_results.push_back(common::types::MissionRunResult{
                common::types::MissionRunStatus::Error,
                0,
                {save_error},
            });
            result.mission_score = -1.0;
            return result;
        }
    }

    // Score only when the mission reached a real terminal state; Error (incl.
    // the caught runMission()/save exceptions above) stays at -1.
    if (mission_result.status == common::types::MissionRunStatus::Error) {
        result.mission_score = -1.0;
    } else {
        const common::Position3D spawn =
            user_common_207190406_209543255::worldInitialDronePosition(simulation_config_);
        result.mission_score = MapsComparison::compare(*hidden_map_, *output_map_, spawn);
    }
    return result;
}

} // namespace simulator
