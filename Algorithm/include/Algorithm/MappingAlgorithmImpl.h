#pragma once

#include <Common/IMappingAlgorithm.h>

#include <memory>
#include <optional>
#include <vector>

namespace Algorithm_207190406_209543255 {

/// 26-direction scan batch + density-scored BFS frontier cleanup.
/// Each nextStep emits at most one scan orientation or one movement command.
class MappingAlgorithmImpl_207190406_209543255 final : public common::IMappingAlgorithm {
public:
    /// @param dependencies Mission, sensor, drone, and output-map dependencies.
    explicit MappingAlgorithmImpl_207190406_209543255(
        common::MappingAlgorithmDependencies dependencies);

    /// Returns the next scan orientation and/or movement for DroneControl to execute.
    /// @param state Current drone pose from GPS.
    /// @param latest_scan Previous scan result, or nullptr on the first step.
    /// @return Command with optional movement and/or scan_orientation.
    [[nodiscard]] common::types::MappingStepCommand nextStep(
        const common::types::DroneState& state,
        const common::types::LidarScanResult* latest_scan) override;

    ~MappingAlgorithmImpl_207190406_209543255() override;

    MappingAlgorithmImpl_207190406_209543255(
        const MappingAlgorithmImpl_207190406_209543255&) = delete;
    MappingAlgorithmImpl_207190406_209543255& operator=(
        const MappingAlgorithmImpl_207190406_209543255&) = delete;
    MappingAlgorithmImpl_207190406_209543255(
        MappingAlgorithmImpl_207190406_209543255&&) = delete;
    MappingAlgorithmImpl_207190406_209543255& operator=(
        MappingAlgorithmImpl_207190406_209543255&&) = delete;

private:
    enum class Phase { Scanning, Planning, Moving };

    struct Impl;

    [[nodiscard]] common::types::MappingStepCommand handleScanningPhase(
        const common::types::DroneState& state);
    [[nodiscard]] common::types::MappingStepCommand handlePlanningPhase(
        const common::types::DroneState& state);
    [[nodiscard]] common::types::MappingStepCommand handleMovingPhase(
        const common::types::DroneState& state);
    [[nodiscard]] common::types::MappingStepCommand handleFrontierCleanupPhase(
        const common::types::DroneState& state);

    void ensurePlanningReady();

    void buildScanOrientations(const common::Orientation& heading,
                               const common::Position3D& position);
    [[nodiscard]] std::optional<common::types::MovementCommand> movementToward(
        const common::types::DroneState& state, const common::Position3D& target) const;
    [[nodiscard]] bool reachedWaypoint(const common::types::DroneState& state,
                                       const common::Position3D& target) const;
    [[nodiscard]] bool samePosition(const common::Position3D& a, const common::Position3D& b) const;

    std::unique_ptr<Impl> impl_;

    static constexpr int kMaxMovingStallTicks = 8;
};

} // namespace Algorithm_207190406_209543255
