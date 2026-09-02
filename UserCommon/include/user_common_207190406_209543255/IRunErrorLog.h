#pragma once

#include <chrono>
#include <string>

// Forward-declare only — consumers that create ErrorRef objects must include
// <Common/types/MissionTypes.h> themselves.
namespace common::types {
struct ErrorRef;
} // namespace common::types

namespace user_common_207190406_209543255 {

class IRunErrorLog {
public:
    virtual ~IRunErrorLog() = default;

    virtual void log(const common::types::ErrorRef& error) = 0;
};

/// Returns the current UTC time as an ISO-8601 string, e.g. "2026-08-05T17:30:00Z".
[[nodiscard]] std::string currentUtcTimestamp();

/// Formats a given time_point as an ISO-8601 UTC string.
/// Useful in tests where a fixed reference point is needed.
[[nodiscard]] std::string formatUtcTimestamp(std::chrono::system_clock::time_point tp);

} // namespace user_common_207190406_209543255
