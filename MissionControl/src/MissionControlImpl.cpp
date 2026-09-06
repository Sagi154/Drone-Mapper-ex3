#include <MissionControl/MissionControlImpl.h>

#include <MissionControl/DroneControlImpl.h>
#include <MissionControl/MissionRunLoop.h>

#include <fstream>
#include <utility>
#include <vector>

namespace mission_control_207190406_209543255 {

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
          dependencies.lidar.config(),
          dependencies.lidar,
          dependencies.gps,
          dependencies.movement,
          dependencies.output_map,
          dependencies.mapping_algorithm,
          mission_.mission_bounds)) {}

MissionControlImpl_207190406_209543255::~MissionControlImpl_207190406_209543255() = default;

common::types::MissionRunResult runMissionSteps(
    mission_control::IDroneControl& drone_control, std::size_t max_steps,
    const std::filesystem::path& output_map_file, bool verbose) {
    std::size_t steps = 0;
    common::types::MissionRunStatus status = common::types::MissionRunStatus::MaxSteps;
    std::vector<common::types::ErrorRef> errors;

    while (steps < max_steps) {
        const common::types::DroneStepResult step_result = drone_control.step();
        ++steps;

        if (step_result.status == common::types::DroneStepStatus::Error) {
            errors.push_back(
                common::types::ErrorRef{"DRONE_STEP_FAILED", "Drone step failed."});
            continue;
        }

        if (step_result.status == common::types::DroneStepStatus::Completed) {
            status = common::types::MissionRunStatus::Completed;
            auto result = finalizeMission(status, steps, std::move(errors));
            if (verbose && !output_map_file.empty()) {
                writeVerboseLog(output_map_file, result.status, result.steps);
            }
            return result;
        }
    }

    auto result = finalizeMission(status, steps, std::move(errors));
    if (verbose && !output_map_file.empty()) {
        writeVerboseLog(output_map_file, result.status, result.steps);
    }
    return result;
}

common::types::MissionRunResult MissionControlImpl_207190406_209543255::runMission() {
    return runMissionSteps(*drone_control_, mission_.max_steps, output_map_file_, verbose_);
}

} // namespace mission_control_207190406_209543255
