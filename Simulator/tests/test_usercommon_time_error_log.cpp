#include <user_common_207190406_209543255/RunErrorLog.h>
#include <user_common_207190406_209543255/TimeFormat.h>

#include <gtest/gtest.h>

#include <chrono>
#include <fstream>
#include <regex>
#include <string>
#include <vector>

namespace {

[[nodiscard]] std::vector<std::string> readAllLines(const std::filesystem::path& path) {
    std::ifstream input(path);
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(input, line)) {
        lines.push_back(line);
    }
    return lines;
}

[[nodiscard]] bool matchesIso8601Utc(const std::string& s) {
    static const std::regex kPattern{R"(^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}Z$)"};
    return std::regex_match(s, kPattern);
}

} // namespace

TEST(TimeFormat, FixedTimePointProducesExpectedIso8601String) {
    // 2000-01-01 00:00:00 UTC = Unix epoch + 946684800s
    const auto tp = std::chrono::system_clock::from_time_t(946684800);
    EXPECT_EQ(user_common_207190406_209543255::formatUtcTimestamp(tp), "2000-01-01T00:00:00Z");
}

TEST(TimeFormat, CurrentTimestampMatchesIso8601Format) {
    const std::string ts = user_common_207190406_209543255::currentUtcTimestamp();
    EXPECT_TRUE(matchesIso8601Utc(ts)) << "Unexpected format: " << ts;
}

TEST(RunErrorLog, WritesStructuredLineAndFlushesImmediately) {
    const std::filesystem::path log_path =
        std::filesystem::temp_directory_path() / "uc_run_error_log_flush_test.log";

    {
        user_common_207190406_209543255::RunErrorLog logger{log_path};
        logger.log({"TEST_ERROR_CODE", "user-facing message"});
        // Readable from disk before the logger is destroyed (no explicit close/flush by caller).
        const auto lines = readAllLines(log_path);
        ASSERT_EQ(lines.size(), 1U);
        EXPECT_NE(lines.front().find("TEST_ERROR_CODE"), std::string::npos);
        EXPECT_NE(lines.front().find("user-facing message"), std::string::npos);
        // Timestamp portion should be ISO-8601.
        const std::string ts = lines.front().substr(0, 20);
        EXPECT_TRUE(matchesIso8601Utc(ts)) << "Timestamp: " << ts;
    }

    std::error_code ec;
    std::filesystem::remove(log_path, ec);
}

TEST(RunErrorLog, CreatesParentDirectoryIfMissing) {
    const std::filesystem::path log_path =
        std::filesystem::temp_directory_path() / "uc_run_error_log_nested" / "deep" / "error.log";

    user_common_207190406_209543255::RunErrorLog logger{log_path};
    logger.log({"DIR_TEST", "created nested log path"});

    EXPECT_TRUE(std::filesystem::exists(log_path));

    std::error_code ec;
    std::filesystem::remove_all(
        std::filesystem::temp_directory_path() / "uc_run_error_log_nested", ec);
}

TEST(RunErrorLog, AppendsMultipleEntries) {
    const std::filesystem::path log_path =
        std::filesystem::temp_directory_path() / "uc_run_error_log_append_test.log";

    {
        user_common_207190406_209543255::RunErrorLog logger{log_path};
        logger.log({"FIRST", "first message"});
        logger.log({"SECOND", "second message"});
    }

    const auto lines = readAllLines(log_path);
    ASSERT_EQ(lines.size(), 2U);
    EXPECT_NE(lines[0].find("FIRST"), std::string::npos);
    EXPECT_NE(lines[1].find("SECOND"), std::string::npos);

    std::error_code ec;
    std::filesystem::remove(log_path, ec);
}
