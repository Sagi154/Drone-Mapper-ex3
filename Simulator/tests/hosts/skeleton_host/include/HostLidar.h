#pragma once

#include <Common/ILidar.h>

namespace skeleton_host {

class HostGPS;
class HostMap3D;

class HostLidar : public common::ILidar {
public:
    HostLidar(const HostGPS& gps,
              const HostMap3D& hidden_map,
              common::types::LidarConfigData config);

    [[nodiscard]] common::types::LidarScanResult scan(
        common::Orientation scan_orientation) const override;
    [[nodiscard]] common::types::LidarConfigData config() const override;

private:
    const HostGPS& gps_;
    const HostMap3D& hidden_map_;
    common::types::LidarConfigData config_{};
};

}  // namespace skeleton_host
