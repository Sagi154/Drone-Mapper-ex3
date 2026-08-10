#pragma once

#include <Common/IMissionControl.h>
#include <Common/MissionControlFactory.h>

#include <MissionControl/DroneControlImpl.h>

#include <memory>
#include <filesystem>

namespace MissionControl_207190406_209543255 {

class MissionControlImpl_207190406_209543255 final : public common::IMissionControl {
public:
    explicit MissionControlImpl_207190406_209543255(common::MissionControlDependencies dependencies);

    [[nodiscard]] common::types::MissionRunResult runMission() override;

private:
    common::types::MissionConfigData mission_;
    common::types::DroneConfigData drone_;
    common::IMutableMap3D& output_map_;
    std::filesystem::path output_map_file_;
    bool verbose_ = false;
    std::unique_ptr<DroneControlImpl> drone_control_;
};

} // namespace MissionControl_207190406_209543255
