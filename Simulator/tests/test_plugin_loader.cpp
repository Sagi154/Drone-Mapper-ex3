// test_plugin_loader.cpp — dlopen fixtures: valid register, unregistered, no reload.

#include <Simulator/PluginLoader.h>
#include <Simulator/PluginRegistrar.h>

#include <gtest/gtest.h>

#include <filesystem>
#include <string>

#ifndef PLUGIN_FIXTURES_DIR
#error "PLUGIN_FIXTURES_DIR must be defined by CMake"
#endif

namespace {

namespace fs = std::filesystem;

[[nodiscard]] fs::path fixturePath(const std::string& name) {
    return fs::path{PLUGIN_FIXTURES_DIR} / name;
}

} // namespace

class PluginLoaderTest : public ::testing::Test {
protected:
    void SetUp() override {
        auto& reg = simulator::PluginRegistrar::instance();
        reg.clearPendingAlgorithmFactory();
        reg.clearPendingMissionControlFactory();
    }
};

TEST_F(PluginLoaderTest, ValidFixtureLoadsAndFactoryIsRetrievable) {
    simulator::PluginLoader loader;
    const fs::path so = fixturePath("valid_algorithm_plugin.so");
    ASSERT_TRUE(fs::exists(so)) << so;

    const auto outcome = loader.loadAlgorithmSo(so);
    EXPECT_TRUE(outcome.errors.empty());
    ASSERT_EQ(loader.algorithms().size(), 1U);
    EXPECT_EQ(loader.algorithms().front().filename, "valid_algorithm_plugin.so");
    ASSERT_TRUE(static_cast<bool>(loader.algorithms().front().factory));

    loader.unloadAll();
    EXPECT_TRUE(loader.algorithms().empty());
}

TEST_F(PluginLoaderTest, UnregisteredSoGoesToErrorsList) {
    simulator::PluginLoader loader;
    const fs::path so = fixturePath("unregistered_plugin.so");
    ASSERT_TRUE(fs::exists(so)) << so;

    const auto outcome = loader.loadAlgorithmSo(so);
    ASSERT_EQ(outcome.errors.size(), 1U);
    EXPECT_EQ(outcome.errors.front(), "unregistered_plugin.so");
    EXPECT_TRUE(loader.algorithms().empty());
}

TEST_F(PluginLoaderTest, ReloadSamePathIsBlocked) {
    simulator::PluginLoader loader;
    const fs::path so = fixturePath("valid_algorithm_plugin.so");
    ASSERT_TRUE(fs::exists(so)) << so;

    ASSERT_TRUE(loader.loadAlgorithmSo(so).errors.empty());
    ASSERT_EQ(loader.algorithms().size(), 1U);

    const auto second = loader.loadAlgorithmSo(so);
    ASSERT_EQ(second.errors.size(), 1U);
    EXPECT_EQ(second.errors.front(), "valid_algorithm_plugin.so");
    EXPECT_EQ(loader.algorithms().size(), 1U); // still exactly one successful load
}

TEST_F(PluginLoaderTest, DirectoryLoadCollectsValidAndFailed) {
    simulator::PluginLoader loader;
    const fs::path dir{PLUGIN_FIXTURES_DIR};
    ASSERT_TRUE(fs::is_directory(dir));

    const auto outcome = loader.loadAlgorithmsFromDirectory(dir);
    // At least the unregistered fixture fails; valid one succeeds.
    EXPECT_FALSE(outcome.errors.empty());
    EXPECT_FALSE(loader.algorithms().empty());

    bool saw_unregistered = false;
    for (const auto& err : outcome.errors) {
        if (err == "unregistered_plugin.so") {
            saw_unregistered = true;
        }
    }
    EXPECT_TRUE(saw_unregistered);

    bool saw_valid = false;
    for (const auto& plugin : loader.algorithms()) {
        if (plugin.filename == "valid_algorithm_plugin.so") {
            saw_valid = true;
        }
    }
    EXPECT_TRUE(saw_valid);
}
