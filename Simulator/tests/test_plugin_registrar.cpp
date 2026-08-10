// test_plugin_registrar.cpp — registration ctor → take-once pending factory.

#include <Common/MappingAlgorithmRegistration.h>
#include <Common/MissionControlRegistration.h>
#include <Simulator/PluginRegistrar.h>

#include <gtest/gtest.h>

#include <memory>

namespace {

[[nodiscard]] common::MappingAlgorithmFactory makeDummyAlgorithmFactory() {
    return [](common::MappingAlgorithmDependencies /*deps*/)
               -> std::unique_ptr<common::IMappingAlgorithm> { return nullptr; };
}

[[nodiscard]] common::MissionControlFactory makeDummyMissionControlFactory() {
    return [](common::MissionControlDependencies /*deps*/)
               -> std::unique_ptr<common::IMissionControl> { return nullptr; };
}

} // namespace

class PluginRegistrarTest : public ::testing::Test {
protected:
    void SetUp() override {
        auto& reg = simulator::PluginRegistrar::instance();
        reg.clearPendingAlgorithmFactory();
        reg.clearPendingMissionControlFactory();
    }
};

TEST_F(PluginRegistrarTest, TakePendingAlgorithmFactoryExactlyOnce) {
    auto& reg = simulator::PluginRegistrar::instance();

    common::MappingAlgorithmRegistration registration{makeDummyAlgorithmFactory()};

    auto first = reg.takePendingAlgorithmFactory();
    ASSERT_TRUE(first.has_value());
    EXPECT_TRUE(static_cast<bool>(*first));

    auto second = reg.takePendingAlgorithmFactory();
    EXPECT_FALSE(second.has_value());
}

TEST_F(PluginRegistrarTest, TakePendingMissionControlFactoryExactlyOnce) {
    auto& reg = simulator::PluginRegistrar::instance();

    common::MissionControlRegistration registration{makeDummyMissionControlFactory()};

    auto first = reg.takePendingMissionControlFactory();
    ASSERT_TRUE(first.has_value());
    EXPECT_TRUE(static_cast<bool>(*first));

    auto second = reg.takePendingMissionControlFactory();
    EXPECT_FALSE(second.has_value());
}

TEST_F(PluginRegistrarTest, ClearPendingRemovesFactoryWithoutTake) {
    auto& reg = simulator::PluginRegistrar::instance();

    common::MappingAlgorithmRegistration registration{makeDummyAlgorithmFactory()};
    reg.clearPendingAlgorithmFactory();

    EXPECT_FALSE(reg.takePendingAlgorithmFactory().has_value());
}
