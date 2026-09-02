#include <user_common_207190406_209543255/RunErrorLog.h>
#include <user_common_207190406_209543255/IRunErrorLog.h>

#include <utility>

namespace user_common_207190406_209543255 {

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
    if (!stream_.is_open()) {
        return;
    }
    stream_ << currentUtcTimestamp() << ' ' << error.code << ' ' << error.message << '\n';
    stream_.flush();
}

} // namespace user_common_207190406_209543255
