// TEST-ONLY foreign MissionControl: hits-only Occupied writes, no free-space carving.
// See ASSUMPTIONS.md in this directory.

#include <Common/IMissionControl.h>
#include <Common/MissionControlFactory.h>
#include <Common/MissionControlRegistration.h>
#include <Common/Types.h>
#include <Common/Units.h>

#include <cmath>
#include <cstddef>
#include <exception>
#include <memory>
#include <utility>

namespace {

constexpr std::size_t kHitStrideN = 4;  // write every Nth hit (i % N == 0)

[[nodiscard]] double cm_of(const common::XLength& q) {
    return q.numerical_value_in(common::cm);
}
[[nodiscard]] double cm_of(const common::YLength& q) {
    return q.numerical_value_in(common::cm);
}
[[nodiscard]] double cm_of(const common::ZLength& q) {
    return q.numerical_value_in(common::cm);
}
[[nodiscard]] double cm_of(const common::PhysicalLength& q) {
    return q.numerical_value_in(common::cm);
}
[[nodiscard]] double deg_of(const common::HorizontalAngle& q) {
    return q.numerical_value_in(common::deg);
}
[[nodiscard]] double deg_of(const common::AltitudeAngle& q) {
    return q.numerical_value_in(common::deg);
}

[[nodiscard]] constexpr double deg_to_rad(double degrees) {
    return degrees * 3.14159265358979323846 / 180.0;
}

[[nodiscard]] common::Position3D hit_world_position(const common::Position3D& origin,
                                                    const common::types::LidarHit& hit) {
    const double horiz = deg_to_rad(deg_of(hit.angle.horizontal));
    const double alt = deg_to_rad(deg_of(hit.angle.altitude));
    const double c = std::cos(alt);
    const double dist = cm_of(hit.distance);
    const double dx = c * std::cos(horiz) * dist;
    const double dy = c * std::sin(horiz) * dist;
    const double dz = std::sin(alt) * dist;
    return {common::XLength{(cm_of(origin.x) + dx) * common::cm},
            common::YLength{(cm_of(origin.y) + dy) * common::cm},
            common::ZLength{(cm_of(origin.z) + dz) * common::cm}};
}

}  // namespace

class ForeignHitsOnlyMissionControl final : public common::IMissionControl {
public:
    explicit ForeignHitsOnlyMissionControl(common::MissionControlDependencies dependencies)
        : deps_(std::move(dependencies)) {}

    [[nodiscard]] common::types::MissionRunResult runMission() override {
        using common::types::AlgorithmStatus;
        using common::types::MissionRunResult;
        using common::types::MissionRunStatus;
        using common::types::MovementCommandType;
        using common::types::VoxelOccupancy;

        MissionRunResult result{};
        result.status = MissionRunStatus::Completed;
        result.steps = 0;

        const std::size_t max_steps = deps_.mission_config.max_steps;

        for (;;) {
            common::types::DroneState state{};
            state.position = deps_.gps.position();
            state.heading = deps_.gps.heading();
            state.step_index = result.steps;

            // Independence probe: never feed scans to the algorithm.
            const common::types::MappingStepCommand cmd =
                deps_.mapping_algorithm.nextStep(state, nullptr);

            if (cmd.movement.has_value()) {
                const auto& movement_cmd = *cmd.movement;
                try {
                    switch (movement_cmd.type) {
                    case MovementCommandType::Hover:
                        break;
                    case MovementCommandType::Rotate:
                        (void)deps_.movement.rotate(movement_cmd.rotation, movement_cmd.angle);
                        break;
                    case MovementCommandType::Advance:
                        (void)deps_.movement.advance(movement_cmd.distance);
                        break;
                    case MovementCommandType::Elevate:
                        (void)deps_.movement.elevate(movement_cmd.distance);
                        break;
                    }
                    // MovementResult::success == false is ignored → continue.
                } catch (const std::exception&) {
                    // Host threw on movement → continue (scan still allowed).
                }
            }

            if (cmd.scan_orientation.has_value()) {
                const common::types::LidarScanResult hits =
                    deps_.lidar.scan(*cmd.scan_orientation);
                const common::Position3D origin = deps_.gps.position();  // post-move GPS
                for (std::size_t i = 0; i < hits.size(); ++i) {
                    if (i % kHitStrideN != 0) {
                        continue;
                    }
                    deps_.output_map.set(hit_world_position(origin, hits[i]),
                                        VoxelOccupancy::Occupied);
                }
            }

            ++result.steps;

            if (cmd.status == AlgorithmStatus::Finished ||
                cmd.status == AlgorithmStatus::FinishedWithUnmappableVoxels) {
                result.status = MissionRunStatus::Completed;
                return result;
            }

            if (result.steps >= max_steps) {
                result.status = MissionRunStatus::MaxSteps;
                return result;
            }
        }
    }

private:
    common::MissionControlDependencies deps_;
};

REGISTER_MISSION_CONTROL(ForeignHitsOnlyMissionControl);
