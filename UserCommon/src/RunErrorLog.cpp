#include <user_common_207190406_209543255/RunErrorLog.h>
#include <user_common_207190406_209543255/IRunErrorLog.h>

#include <fstream>
#include <utility>

namespace user_common_207190406_209543255 {

struct RunErrorLog::Stream {
    std::filesystem::path log_path;
    std::ofstream stream;
};

RunErrorLog::RunErrorLog(std::filesystem::path log_path)
    : stream_(std::make_unique<Stream>()) {
    stream_->log_path = std::move(log_path);
    if (stream_->log_path.has_parent_path()) {
        std::filesystem::create_directories(stream_->log_path.parent_path());
    }
    stream_->stream.open(stream_->log_path, std::ios::app);
}

RunErrorLog::~RunErrorLog() = default;
RunErrorLog::RunErrorLog(RunErrorLog&&) noexcept = default;
RunErrorLog& RunErrorLog::operator=(RunErrorLog&&) noexcept = default;

void RunErrorLog::log(const common::types::ErrorRef& error) {
    if (!stream_) {
        return;
    }
    if (!stream_->stream.is_open()) {
        stream_->stream.open(stream_->log_path, std::ios::app);
    }
    if (!stream_->stream.is_open()) {
        return;
    }
    stream_->stream << currentUtcTimestamp() << ' ' << error.code << ' ' << error.message << '\n';
    stream_->stream.flush();
}

} // namespace user_common_207190406_209543255
