// test_yaml_config_parsers.cpp
// Covers all five YAML parsers.
// Key end-to-end verify: parse inputs/sim_compose.yaml and confirm
// 5 simulation groups totalling 6 (sim, mission) pairs, 2 drone_configs,
// 2 lidar_configs = 24 run cells per docs/map3d-contract.md.

#include <Simulator/io/YamlConfigParsers.h>
#include <Simulator/io/PathResolver.h>
#include <Simulator/SimulationTypes.h>

#include <UserCommon_207190406_209543255/RunErrorLog.h>

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <numeric>
#include <string>
#include <tuple>

namespace fs = std::filesystem;
using namespace common;
using namespace common::types;
using namespace simulator;
using namespace UserCommon_207190406_209543255;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Returns the path to the inputs/ directory relative to the test binary's
/// CWD, or via source-tree macro if set by CMake.
[[nodiscard]] static fs::path inputsDir() {
#ifdef SIMULATOR_INPUTS_DIR
    return fs::path{SIMULATOR_INPUTS_DIR};
#else
    // When run from build/default, navigate up to find inputs/
    fs::path p = fs::current_path();
    for (int i = 0; i < 6; ++i) {
        if (fs::exists(p / "inputs" / "sim_compose.yaml")) {
            return p / "inputs";
        }
        p = p.parent_path();
    }
    return fs::path{"inputs"};
#endif
}

/// Null error log for tests that expect clean parses.
struct NullLog : IRunErrorLog {
    void log(const ErrorRef&) override {}
};

/// Recording log — captures errors for inspection.
struct RecordingLog : IRunErrorLog {
    std::vector<ErrorRef> errors;
    void log(const ErrorRef& e) override { errors.push_back(e); }
};

// ---------------------------------------------------------------------------
// PathResolver
// ---------------------------------------------------------------------------

TEST(PathResolver, ResolveAbsoluteReturnedUnchanged) {
    const fs::path abs = fs::absolute(".");
    EXPECT_EQ(simulator::io::resolveConfigPath("/some/base", abs), abs);
}

TEST(PathResolver, ResolveRelativeJoinsBase) {
    const auto resolved = simulator::io::resolveConfigPath("/base/dir", "sub/file.yaml");
    EXPECT_EQ(resolved, fs::path("/base/dir/sub/file.yaml"));
}

// ---------------------------------------------------------------------------
// DroneConfigParser
// ---------------------------------------------------------------------------

TEST(DroneConfigParser, ParsesRealDroneSmall) {
    const fs::path drone_path = inputsDir() / "drone" / "drone_small.yaml";
    if (!fs::exists(drone_path)) {
        GTEST_SKIP() << "inputs not available";
    }
    NullLog log;
    const auto result = simulator::io::parseDroneConfig(drone_path, log);
    EXPECT_TRUE(result.ok);
    EXPECT_GT(result.value.radius.numerical_value_in(cm), 0.0);
    EXPECT_GT(result.value.max_advance.numerical_value_in(cm), 0.0);
}

TEST(DroneConfigParser, MissingFileReturnsError) {
    RecordingLog log;
    const auto result = simulator::io::parseDroneConfig("/nonexistent/drone.yaml", log);
    EXPECT_FALSE(result.ok);
    EXPECT_FALSE(result.errors.empty());
}

// ---------------------------------------------------------------------------
// LidarConfigParser
// ---------------------------------------------------------------------------

TEST(LidarConfigParser, ParsesRealLidarLong) {
    const fs::path lidar_path = inputsDir() / "lidar" / "lidar_long.yaml";
    if (!fs::exists(lidar_path)) {
        GTEST_SKIP() << "inputs not available";
    }
    NullLog log;
    const auto result = simulator::io::parseLidarConfig(lidar_path, log);
    EXPECT_TRUE(result.ok);
    EXPECT_GT(result.value.z_max.numerical_value_in(cm), 0.0);
    EXPECT_GT(result.value.fov_circles, 0u);
}

// ---------------------------------------------------------------------------
// MissionConfigParser
// ---------------------------------------------------------------------------

TEST(MissionConfigParser, ParsesRealMissionConfig) {
    const fs::path mission_path = inputsDir() / "mission" / "small_mission_room.yaml";
    if (!fs::exists(mission_path)) {
        GTEST_SKIP() << "inputs not available";
    }
    NullLog log;
    const auto result = simulator::io::parseMissionConfig(mission_path, log);
    EXPECT_TRUE(result.ok);
    EXPECT_GT(result.value.max_steps, 0u);
}

// ---------------------------------------------------------------------------
// SimulationConfigParser
// ---------------------------------------------------------------------------

TEST(SimulationConfigParser, ParsesRealSmallSimulation) {
    const fs::path sim_path = inputsDir() / "simulation" / "small_simulation_room.yaml";
    if (!fs::exists(sim_path)) {
        GTEST_SKIP() << "inputs not available";
    }
    NullLog log;
    const auto result = simulator::io::parseSimulationConfig(sim_path, log);
    EXPECT_TRUE(result.ok);
    EXPECT_FALSE(result.value.map_filename.empty());
    EXPECT_GT(result.value.map_resolution.numerical_value_in(cm), 0.0);
}

TEST(SimulationConfigParser, ParsesHouseSimulationWithOffset) {
    const fs::path sim_path = inputsDir() / "simulation" / "house_simulation.yaml";
    if (!fs::exists(sim_path)) {
        GTEST_SKIP() << "inputs not available";
    }
    NullLog log;
    const auto result = simulator::io::parseSimulationConfig(sim_path, log);
    EXPECT_TRUE(result.ok);
    // House scenario has map_axes_offset: height_offset: 150
    EXPECT_DOUBLE_EQ(result.value.map_offset.z.numerical_value_in(cm), 150.0);
}

// ---------------------------------------------------------------------------
// CompositionParser — end-to-end run-count check
// ---------------------------------------------------------------------------

TEST(CompositionParser, Sim_ComposeYamlProduces24RunCells) {
    const fs::path compose_path = inputsDir() / "sim_compose.yaml";
    if (!fs::exists(compose_path)) {
        GTEST_SKIP() << "inputs not available";
    }

    NullLog log;
    const auto result = simulator::io::parseCompositionFile(compose_path, log);
    ASSERT_TRUE(result.ok);

    const auto& comp = result.value;

    // Count total (sim, mission) pairs across all groups
    const std::size_t sim_mission_pairs = std::accumulate(
        comp.simulation_mission_groups.begin(),
        comp.simulation_mission_groups.end(),
        std::size_t{0},
        [](std::size_t acc, const auto& group) {
            return acc + std::get<1>(group).size();
        });

    const std::size_t total_runs =
        sim_mission_pairs * comp.drone_configs.size() * comp.lidar_configs.size();

    EXPECT_EQ(sim_mission_pairs, 6u)  << "(sim, mission) pairs should be 6";
    EXPECT_EQ(comp.drone_configs.size(), 2u)  << "drone_configs should be 2";
    EXPECT_EQ(comp.lidar_configs.size(), 2u)  << "lidar_configs should be 2";
    EXPECT_EQ(total_runs, 24u) << "total run cells should be 24";
}

TEST(CompositionParser, MissingFileReturnsError) {
    RecordingLog log;
    const auto result = simulator::io::parseCompositionFile("/no/such/file.yaml", log);
    EXPECT_FALSE(result.ok);
    EXPECT_FALSE(result.errors.empty());
}
