// test_simulation_cli.cpp — standalone SimulationCli parse/validation tests.
// No loader, threading, or plugins. Never expects exit().

#include <Simulator/io/SimulatorPaths.h>

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace {

namespace fs = std::filesystem;
using simulator::io::parseSimulationCliArgs;
using simulator::io::SimulatorMode;

class TempTree {
public:
    TempTree()
        : root_(fs::temp_directory_path() /
                ("simcli_" + std::to_string(
                                 std::chrono::steady_clock::now().time_since_epoch().count()))) {
        fs::create_directories(root_);
    }

    ~TempTree() {
        std::error_code ec;
        fs::remove_all(root_, ec);
    }

    TempTree(const TempTree&)            = delete;
    TempTree& operator=(const TempTree&) = delete;

    [[nodiscard]] const fs::path& root() const { return root_; }

    fs::path writeFile(const std::string& relative, const std::string& contents) const {
        const fs::path path = root_ / relative;
        fs::create_directories(path.parent_path());
        std::ofstream out{path};
        out << contents;
        return path;
    }

    [[nodiscard]] fs::path makeDir(const std::string& relative) const {
        const fs::path path = root_ / relative;
        fs::create_directories(path);
        return path;
    }

private:
    fs::path root_;
};

struct ArgvOwner {
    explicit ArgvOwner(std::vector<std::string> args) : storage_(std::move(args)) {
        pointers_.reserve(storage_.size());
        for (auto& s : storage_) {
            pointers_.push_back(s.data());
        }
    }

    [[nodiscard]] int argc() const { return static_cast<int>(pointers_.size()); }
    [[nodiscard]] char** argv() { return pointers_.data(); }

private:
    std::vector<std::string> storage_;
    std::vector<char*> pointers_;
};

} // namespace

TEST(SimulationCli, ScrambledComparativeArgumentOrderSucceeds) {
    TempTree tree;
    const fs::path sim = tree.writeFile("composition.yaml", "ok");
    const fs::path algo = tree.writeFile("Algorithm.so", "");
    const fs::path folder = tree.makeDir("mcs");
    tree.writeFile("mcs/MissionControl.so", "");

    ArgvOwner args{{"simulator",
                    "algorithm=" + algo.string(),
                    "-verbose",
                    "mission_control_folder=" + folder.string(),
                    "-comparative",
                    "num_threads=4",
                    "simulation=" + sim.string()}};

    const auto result = parseSimulationCliArgs(args.argc(), args.argv());
    ASSERT_TRUE(result.ok) << result.errors.front();
    EXPECT_EQ(result.args.mode, SimulatorMode::Comparative);
    EXPECT_EQ(result.args.simulation, sim);
    EXPECT_EQ(result.args.algorithm, algo);
    EXPECT_EQ(result.args.mission_control_folder, folder);
    ASSERT_TRUE(result.args.num_threads.has_value());
    EXPECT_EQ(*result.args.num_threads, 4U);
    EXPECT_TRUE(result.args.verbose);
}

TEST(SimulationCli, TwoMissingArgumentsReportedTogether) {
    ArgvOwner args{{"simulator", "-comparative", "simulation=does_not_matter.yaml"}};

    const auto result = parseSimulationCliArgs(args.argc(), args.argv());
    ASSERT_FALSE(result.ok);
    ASSERT_FALSE(result.errors.empty());

    const std::string& err = result.errors.front();
    EXPECT_NE(err.find("missing argument(s):"), std::string::npos);
    EXPECT_NE(err.find("mission_control_folder"), std::string::npos);
    EXPECT_NE(err.find("algorithm"), std::string::npos);
}

TEST(SimulationCli, TwoUnsupportedArgumentsReportedTogether) {
    ArgvOwner args{{"simulator", "-comparative", "simulation=x.yaml",
                    "mission_control_folder=mcs", "algorithm=a.so", "foo=1", "bar=2"}};

    const auto result = parseSimulationCliArgs(args.argc(), args.argv());
    ASSERT_FALSE(result.ok);
    ASSERT_FALSE(result.errors.empty());

    bool found_unsupported = false;
    for (const auto& err : result.errors) {
        if (err.find("unsupported argument(s):") != std::string::npos) {
            found_unsupported = true;
            EXPECT_NE(err.find("foo"), std::string::npos);
            EXPECT_NE(err.find("bar"), std::string::npos);
        }
    }
    EXPECT_TRUE(found_unsupported);
}

TEST(SimulationCli, NonexistentFilePathReported) {
    TempTree tree;
    const fs::path folder = tree.makeDir("mcs");
    tree.writeFile("mcs/MissionControl.so", "");
    const fs::path algo = tree.writeFile("Algorithm.so", "");
    const fs::path missing = tree.root() / "missing_composition.yaml";

    ArgvOwner args{{"simulator", "-comparative", "simulation=" + missing.string(),
                    "mission_control_folder=" + folder.string(), "algorithm=" + algo.string()}};

    const auto result = parseSimulationCliArgs(args.argc(), args.argv());
    ASSERT_FALSE(result.ok);
    bool found = false;
    for (const auto& err : result.errors) {
        if (err.find("simulation file is missing or unopenable") != std::string::npos) {
            found = true;
        }
    }
    EXPECT_TRUE(found);
}

TEST(SimulationCli, FolderWithZeroSoFilesReported) {
    TempTree tree;
    const fs::path sim = tree.writeFile("composition.yaml", "ok");
    const fs::path algo = tree.writeFile("Algorithm.so", "");
    const fs::path empty_folder = tree.makeDir("empty_mcs");
    tree.writeFile("empty_mcs/readme.txt", "no plugins here");

    ArgvOwner args{{"simulator", "-comparative", "simulation=" + sim.string(),
                    "mission_control_folder=" + empty_folder.string(),
                    "algorithm=" + algo.string()}};

    const auto result = parseSimulationCliArgs(args.argc(), args.argv());
    ASSERT_FALSE(result.ok);
    bool found = false;
    for (const auto& err : result.errors) {
        if (err.find("contains no .so files") != std::string::npos) {
            found = true;
        }
    }
    EXPECT_TRUE(found);
}

TEST(SimulationCli, UnknownModeFlagReported) {
    ArgvOwner args{{"simulator", "-competitive", "simulation=x.yaml"}};

    const auto result = parseSimulationCliArgs(args.argc(), args.argv());
    ASSERT_FALSE(result.ok);
    bool found = false;
    for (const auto& err : result.errors) {
        if (err.find("unknown mode flag '-competitive'") != std::string::npos) {
            found = true;
        }
    }
    EXPECT_TRUE(found);
}

TEST(SimulationCli, CompetitionModeSucceedsWithScrambledOrder) {
    TempTree tree;
    const fs::path sim = tree.writeFile("composition.yaml", "ok");
    const fs::path mc = tree.writeFile("MissionControl.so", "");
    const fs::path folder = tree.makeDir("algos");
    tree.writeFile("algos/Algorithm.so", "");

    ArgvOwner args{{"simulator", "algorithms_folder=" + folder.string(),
                    "simulation=" + sim.string(), "-competition",
                    "mission_control=" + mc.string()}};

    const auto result = parseSimulationCliArgs(args.argc(), args.argv());
    ASSERT_TRUE(result.ok) << (result.errors.empty() ? "" : result.errors.front());
    EXPECT_EQ(result.args.mode, SimulatorMode::Competition);
    EXPECT_FALSE(result.args.num_threads.has_value());
    EXPECT_FALSE(result.args.verbose);
}

TEST(SimulationCli, WritesUsageAndErrorsToDiagWithoutExit) {
    ArgvOwner args{{"simulator", "-comparative"}};
    std::ostringstream diag;

    const auto result = parseSimulationCliArgs(args.argc(), args.argv(), &diag);
    ASSERT_FALSE(result.ok);
    EXPECT_NE(diag.str().find("Usage:"), std::string::npos);
    EXPECT_NE(diag.str().find("error:"), std::string::npos);
}
