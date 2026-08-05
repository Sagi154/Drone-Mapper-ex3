#pragma once

#include <chrono>
#include <string>

namespace UserCommon_207190406_209543255 {

/// Returns the current UTC time as an ISO-8601 string, e.g. "2026-08-05T17:30:00Z".
[[nodiscard]] std::string currentUtcTimestamp();

/// Formats a given time_point as an ISO-8601 UTC string.
/// Useful in tests where a fixed reference point is needed.
[[nodiscard]] std::string formatUtcTimestamp(std::chrono::system_clock::time_point tp);

} // namespace UserCommon_207190406_209543255
