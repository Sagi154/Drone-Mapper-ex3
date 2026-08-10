#include <MissionControl/MissionControlImpl.h>

#include <fstream>
#include <utility>
#include <vector>

namespace MissionControl_207190406_209543255 {

namespace {

[[nodiscard]] common::types::MissionRunResult finalizeMission(
    common::types::MissionRunStatus status,
    std::size_t steps,
    std::vector<common::types::ErrorRef> errors) {
    return common::types::MissionRunResult{
        status,
        steps,
        std::move(errors),
    };
}

void writeVerboseLog(const std::filesystem::path& output_map_file,
                     common::types::MissionRunStatus status,
                     std::size_t steps) {
    std::filesystem::path verbose_path = output_map_file;
    verbose_path += ".verbose.txt";
    if (verbose_path.has_parent_path()) {
        std::filesystem::create_directories(verbose_path.parent_path());
    }
    std::ofstream out(verbose_path);
    out << "steps=" << steps << '\n';
    out << "status=";
    switch (status) {
    case common::types::MissionRunStatus::Completed:
        out << "Completed";
        break;
    case common::types::MissionRunStatus::MaxSteps:
        out << "MaxSteps";
        break;
    case common::types::MissionRunStatus::Error:
        out << "Error";
        break;
    }
    out << '\n';
}

} // namespace

MissionControlImpl_207190406_209543255::MissionControlImpl_207190406_209543255(
    common::MissionControlDependencies dependencies)
    : mission_(dependencies.mission_config),
      output_map_file_(std::move(dependencies.output_map_file)),
      verbose_(dependencies.verbose),
      drone_control_(std::make_unique<DroneControlImpl>(
          dependencies.drone_config,
          dependencies.mission_config,
          dependencies.lidar.config(),
          dependencies.lidar,
          dependencies.gps,
          dependencies.movement,
          dependencies.output_map,
          dependencies.mapping_algorithm)) {}

common::types::MissionRunResult MissionControlImpl_207190406_209543255::runMission() {
    std::size_t steps = 0;
    common::types::MissionRunStatus status = common::types::MissionRunStatus::MaxSteps;
    std::vector<common::types::ErrorRef> errors;

    while (steps < mission_.max_steps) {
        const common::types::DroneStepResult step_result = drone_control_->step();
        ++steps;

        if (step_result.status == common::types::DroneStepStatus::Error) {
            status = common::types::MissionRunStatus::Error;
            errors.push_back(common::types::ErrorRef{
                "DRONE_STEP_FAILED",
                step_result.message.empty() ? "Drone step failed." : step_result.message,
            });
            auto result = finalizeMission(status, steps, std::move(errors));
            if (verbose_ && !output_map_file_.empty()) {
                writeVerboseLog(output_map_file_, result.status, result.steps);
            }
            return result;
        }

        if (step_result.status == common::types::DroneStepStatus::Completed) {
            status = common::types::MissionRunStatus::Completed;
            auto result = finalizeMission(status, steps, std::move(errors));
            if (verbose_ && !output_map_file_.empty()) {
                writeVerboseLog(output_map_file_, result.status, result.steps);
            }
            return result;
        }
    }

    auto result = finalizeMission(status, steps, std::move(errors));
    if (verbose_ && !output_map_file_.empty()) {
        writeVerboseLog(output_map_file_, result.status, result.steps);
    }
    return result;
}

} // namespace MissionControl_207190406_209543255
