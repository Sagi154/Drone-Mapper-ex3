#include <user_common_207190406_209543255/TimeFormat.h>

#include <chrono>
#include <iomanip>
#include <sstream>

namespace user_common_207190406_209543255 {

std::string formatUtcTimestamp(std::chrono::system_clock::time_point tp) {
    const auto time = std::chrono::system_clock::to_time_t(tp);
    std::tm utc{};
    gmtime_r(&time, &utc);
    std::ostringstream formatted;
    formatted << std::put_time(&utc, "%Y-%m-%dT%H:%M:%SZ");
    return formatted.str();
}

std::string currentUtcTimestamp() {
    return formatUtcTimestamp(std::chrono::system_clock::now());
}

} // namespace user_common_207190406_209543255
