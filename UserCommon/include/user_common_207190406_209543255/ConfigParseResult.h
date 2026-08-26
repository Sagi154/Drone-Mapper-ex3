#pragma once

#include <Common/types/MissionTypes.h>

#include <vector>

namespace user_common_207190406_209543255 {

template <typename T>
struct ConfigParseResult {
    bool ok = false;
    T value{};
    std::vector<common::types::ErrorRef> errors{};
};

} // namespace user_common_207190406_209543255
