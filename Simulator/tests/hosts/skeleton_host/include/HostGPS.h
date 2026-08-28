#pragma once

#include <Common/IGPS.h>

namespace skeleton_host {

class HostGPS : public common::IGPS {
public:
    HostGPS(common::Position3D position,
            common::Orientation heading,
            common::PhysicalLength gps_resolution);

    [[nodiscard]] common::Position3D position() const override;
    [[nodiscard]] common::Orientation heading() const override;

    void setPosition(common::Position3D position);
    void setHeading(common::Orientation heading);
    [[nodiscard]] common::PhysicalLength gpsResolution() const { return gps_resolution_; }

private:
    common::Position3D position_{};
    common::Orientation heading_{};
    common::PhysicalLength gps_resolution_{};
};

}  // namespace skeleton_host
