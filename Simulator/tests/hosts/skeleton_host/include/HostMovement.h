#pragma once

#include <Common/IDroneMovement.h>
#include <Common/types/MapTypes.h>

#include <cstddef>
#include <string>

namespace skeleton_host {

class HostGPS;
class HostMap3D;

class HostMovement : public common::IDroneMovement {
public:
    HostMovement(HostGPS& gps,
                 const HostMap3D& hidden_map,
                 common::types::MappingBounds mission_bounds,
                 common::PhysicalLength radius);

    common::types::MovementResult rotate(common::types::RotationDirection direction,
                                         common::HorizontalAngle angle) override;
    common::types::MovementResult advance(common::PhysicalLength distance) override;
    common::types::MovementResult elevate(common::PhysicalLength distance) override;

    [[nodiscard]] std::size_t illegalMoveAttempts() const { return illegal_move_attempts_; }

private:
    [[nodiscard]] common::types::MovementResult fail(const std::string& message);
    [[nodiscard]] bool centerInMissionBounds(double x, double y, double z) const;
    [[nodiscard]] bool sphereHitsWallOrLeavesMap(double x, double y, double z) const;
    [[nodiscard]] bool pathBlocked(double x0,
                                   double y0,
                                   double z0,
                                   double x1,
                                   double y1,
                                   double z1) const;

    HostGPS& gps_;
    const HostMap3D& hidden_map_;
    common::types::MappingBounds mission_bounds_{};
    double radius_cm_ = 0;
    std::size_t illegal_move_attempts_ = 0;
};

}  // namespace skeleton_host
