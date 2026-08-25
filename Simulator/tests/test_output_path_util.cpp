#include <Simulator/OutputPathUtil.h>

#include <gtest/gtest.h>

#include <filesystem>

TEST(OutputPathUtil, EmptyMapPathYieldsEmptyErrorLogPath) {
    EXPECT_TRUE(simulator::errorLogPathFromOutputMap({}).empty());
}

TEST(OutputPathUtil, ReplacesOutputMapSuffixWithErrorLog) {
    const std::filesystem::path map =
        std::filesystem::path("out") / "plugin.so_run_0003_output_map.npy";
    EXPECT_EQ(simulator::errorLogPathFromOutputMap(map),
              std::filesystem::path("out") / "plugin.so_run_0003_error.log");
}

TEST(OutputPathUtil, DistinctCellIndicesStayUnique) {
    const auto a = simulator::errorLogPathFromOutputMap("Algo.so_run_0000_output_map.npy");
    const auto b = simulator::errorLogPathFromOutputMap("Algo.so_run_0001_output_map.npy");
    EXPECT_EQ(a.filename().string(), "Algo.so_run_0000_error.log");
    EXPECT_EQ(b.filename().string(), "Algo.so_run_0001_error.log");
    EXPECT_NE(a, b);
}

TEST(OutputPathUtil, FallbackAppendsErrorLogToStem) {
    EXPECT_EQ(simulator::errorLogPathFromOutputMap("unexpected.map").filename().string(),
              "unexpected_error.log");
}
