#pragma once

#include <Common/IDroneMovement.h>
#include <Common/IMap3D.h>
#include <Simulator/MockGPS.h>

namespace simulator {

/// Simulated drone movement driver.
/// Validates per-command limits and performs sphere-based collision detection
/// against the hidden truth map before committing each move to the GPS.
/// Throws std::runtime_error when a move would collide with a wall — this is
/// the mandatory collision scenario from docs/error-handling-matrix.md.
class MockMovement final : public common::IDroneMovement {
public:
    MockMovement(MockGPS& gps,
                 const common::IMap3D& hidden_map,
                 common::types::DroneConfigData drone_config);

    common::types::MovementResult rotate(common::types::RotationDirection direction,
                                         common::HorizontalAngle angle) override;
    common::types::MovementResult advance(common::PhysicalLength distance) override;
    common::types::MovementResult elevate(common::PhysicalLength distance) override;

private:
    MockGPS& gps_;
    const common::IMap3D& hidden_map_;
    common::types::DroneConfigData drone_;
};

} // namespace simulator
