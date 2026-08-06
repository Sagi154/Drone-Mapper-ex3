#include <UserCommon_207190406_209543255/RunErrorLog.h>
#include <UserCommon_207190406_209543255/TimeFormat.h>

#include <utility>

namespace UserCommon_207190406_209543255 {

RunErrorLog::RunErrorLog(std::filesystem::path log_path)
    : log_path_(std::move(log_path)) {
    if (log_path_.has_parent_path()) {
        std::filesystem::create_directories(log_path_.parent_path());
    }
    stream_.open(log_path_, std::ios::app);
}

void RunErrorLog::log(const common::types::ErrorRef& error) {
    if (!stream_.is_open()) {
        stream_.open(log_path_, std::ios::app);
    }
    stream_ << currentUtcTimestamp() << ' ' << error.code << ' ' << error.message << '\n';
    stream_.flush();
}

} // namespace UserCommon_207190406_209543255
