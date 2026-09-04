#pragma once

#include <Common/IGPS.h>

namespace simulator {

/// Simulated GPS that snaps every written position to the nearest
/// gps_resolution grid multiple, matching MissionConfigData::gps_resolution.
class MockGPS final : public common::IGPS {
public:
    MockGPS(const common::Position3D& position,
            const common::Orientation& heading,
            common::PhysicalLength gps_resolution);

    [[nodiscard]] common::Position3D position() const override;
    [[nodiscard]] common::Orientation heading() const override;

    void setPosition(const common::Position3D& position);
    void setHeading(const common::Orientation& heading);

private:
    common::Position3D position_{};
    common::Orientation heading_{};
    common::PhysicalLength resolution_{};
};

} // namespace simulator
