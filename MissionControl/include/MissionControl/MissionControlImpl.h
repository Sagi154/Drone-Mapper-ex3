#pragma once

#include <Common/IMissionControl.h>
#include <Common/MissionControlFactory.h>

#include <MissionControl/DroneControlImpl.h>

#include <memory>
#include <filesystem>

namespace mission_control_207190406_209543255 {

class MissionControlImpl_207190406_209543255 final : public common::IMissionControl {
public:
    explicit MissionControlImpl_207190406_209543255(common::MissionControlDependencies dependencies);

    [[nodiscard]] common::types::MissionRunResult runMission() override;

private:
    common::types::MissionConfigData mission_;
    std::filesystem::path output_map_file_;
    bool verbose_ = false;
    std::unique_ptr<DroneControlImpl> drone_control_;
};

} // namespace mission_control_207190406_209543255
