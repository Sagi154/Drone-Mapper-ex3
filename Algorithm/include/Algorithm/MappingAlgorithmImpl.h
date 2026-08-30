#pragma once

#include <Common/IMappingAlgorithm.h>

#include <cstddef>
#include <memory>
#include <optional>

namespace algorithm_207190406_209543255 {

/// Wavefront Frontier Detection over the reachability substrate.
/// Each nextStep emits a movement and, when the resulting pose would observe
/// something new, a scan in the same command.
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
    struct Impl;

    void ensurePlanningReady();
    [[nodiscard]] std::size_t remainingSteps(const common::types::DroneState& state) const;
    void pruneExpiredBlockedCells(std::size_t step_index);
    [[nodiscard]] bool replan(const common::types::DroneState& state, bool ignore_blocked);
    [[nodiscard]] std::optional<common::types::MovementCommand> movementToward(
        const common::types::DroneState& state, const common::Position3D& target) const;
    [[nodiscard]] common::types::DroneState predictPose(
        const common::types::DroneState& state,
        const common::types::MovementCommand& movement) const;
    void buildArrivalSweep(const common::types::DroneState& state);
    [[nodiscard]] bool targetClusterAlive() const;
    [[nodiscard]] bool reachedWaypoint(const common::types::DroneState& state,
                                       const common::Position3D& target) const;
    [[nodiscard]] bool samePosition(const common::Position3D& a,
                                    const common::Position3D& b) const;

    std::unique_ptr<Impl> impl_;

    static constexpr int kMaxMovingStallTicks = 2;
    static constexpr std::size_t kReplanIntervalSteps = 25;
    static constexpr std::size_t kBlockedTtlSteps = 50;
    static constexpr int kRecoveryAttempts = 3;
    static constexpr int kLowRateReplans = 3;
    static constexpr double kMinInformationRate = 0.25;
};

} // namespace algorithm_207190406_209543255
