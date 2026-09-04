#pragma once

#include <user_common_207190406_209543255/IRunErrorLog.h>

#include <Common/types/MissionTypes.h>
#include <filesystem>
#include <memory>

namespace user_common_207190406_209543255 {

class RunErrorLog final : public IRunErrorLog {
public:
    explicit RunErrorLog(std::filesystem::path log_path);
    ~RunErrorLog();
    RunErrorLog(RunErrorLog&&) noexcept;
    RunErrorLog& operator=(RunErrorLog&&) noexcept;
    RunErrorLog(const RunErrorLog&) = delete;
    RunErrorLog& operator=(const RunErrorLog&) = delete;
    void log(const common::types::ErrorRef& error) override;

private:
    struct Stream;
    std::unique_ptr<Stream> stream_;
};

} // namespace user_common_207190406_209543255
