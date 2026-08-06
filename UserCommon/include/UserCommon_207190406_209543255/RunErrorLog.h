#pragma once

#include <UserCommon_207190406_209543255/IRunErrorLog.h>

#include <Common/types/MissionTypes.h>
#include <filesystem>
#include <fstream>

namespace UserCommon_207190406_209543255 {

class RunErrorLog final : public IRunErrorLog {
public:
    explicit RunErrorLog(std::filesystem::path log_path);

    void log(const common::types::ErrorRef& error) override;

private:
    std::filesystem::path log_path_;
    std::ofstream stream_;
};

} // namespace UserCommon_207190406_209543255
