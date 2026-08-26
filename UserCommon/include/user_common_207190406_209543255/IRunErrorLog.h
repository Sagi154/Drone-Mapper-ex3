#pragma once

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

} // namespace user_common_207190406_209543255
