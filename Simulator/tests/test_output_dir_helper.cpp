// test_output_dir_helper.cpp

#include <Simulator/io/SimulatorPaths.h>

#include <gtest/gtest.h>

#include <filesystem>
#include <system_error>

TEST(OutputDirHelper, ComparativePrefixAndNoCollisionInSameSecond) {
    const std::filesystem::path base =
        std::filesystem::temp_directory_path() / "output_dir_helper_test_comparative";
    std::filesystem::remove_all(base);
    std::filesystem::create_directories(base);

    std::error_code ec1;
    const auto first =
        simulator::io::createOutputDir(base, simulator::io::OutputDirKind::Comparative, ec1);
    ASSERT_FALSE(ec1);
    ASSERT_TRUE(std::filesystem::exists(first));
    EXPECT_EQ(first.filename().string().rfind("comparative_results_", 0), 0U);

    std::error_code ec2;
    const auto second =
        simulator::io::createOutputDir(base, simulator::io::OutputDirKind::Comparative, ec2);
    ASSERT_FALSE(ec2);
    ASSERT_TRUE(std::filesystem::exists(second));

    EXPECT_NE(first, second); // two calls within the same wall-clock second must not collide

    std::filesystem::remove_all(base);
}

TEST(OutputDirHelper, CompetitionUsesCompetitionPrefix) {
    const std::filesystem::path base =
        std::filesystem::temp_directory_path() / "output_dir_helper_test_competition";
    std::filesystem::remove_all(base);
    std::filesystem::create_directories(base);

    std::error_code ec;
    const auto dir =
        simulator::io::createOutputDir(base, simulator::io::OutputDirKind::Competition, ec);
    ASSERT_FALSE(ec);
    EXPECT_EQ(dir.filename().string().rfind("competition_", 0), 0U);

    std::filesystem::remove_all(base);
}
