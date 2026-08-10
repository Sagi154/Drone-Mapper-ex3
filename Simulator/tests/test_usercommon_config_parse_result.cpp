#include <UserCommon_207190406_209543255/ConfigParseResult.h>

#include <gtest/gtest.h>

namespace {

UserCommon_207190406_209543255::ConfigParseResult<int> parseIntStub(bool succeed) {
    UserCommon_207190406_209543255::ConfigParseResult<int> result;
    if (succeed) {
        result.ok = true;
        result.value = 42;
        return result;
    }
    result.ok = false;
    result.errors.push_back({"PARSE_FAIL", "stub parse failure"});
    return result;
}

} // namespace

TEST(ConfigParseResult, SuccessCarriesValue) {
    const auto result = parseIntStub(true);
    EXPECT_TRUE(result.ok);
    EXPECT_EQ(result.value, 42);
    EXPECT_TRUE(result.errors.empty());
}

TEST(ConfigParseResult, FailureCarriesErrors) {
    const auto result = parseIntStub(false);
    EXPECT_FALSE(result.ok);
    ASSERT_EQ(result.errors.size(), 1U);
    EXPECT_EQ(result.errors.front().code, "PARSE_FAIL");
    EXPECT_EQ(result.errors.front().message, "stub parse failure");
}
