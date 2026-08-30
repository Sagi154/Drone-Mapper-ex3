# Wavefront Frontier Exploration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Keep D's plan-execution architecture and replace `NbvPlanner`'s full-cone unique-voxel scoring with Wavefront Frontier Detection over the reachability sweep, so a replan costs a bounded multiple of one O(V) pass and the objective matches `MapsComparison`.

**Architecture:** `ConeTemplate` precomputes per-lidar cone geometry once (Lever 1). `exploreReachable` grows frontier clusters and memoises sphere passability (Lever 2). A new `WavefrontPlanner` ranks those clusters by `cell_count / (travel + reserve)` and returns an `ExplorationPlan` with no cone casts. `ScanPlanning` builds the arrival-time sweep and the pass-2-gated travel scan (Lever 3). `MappingAlgorithmImpl` stays a plan executor.

**Tech Stack:** C++20, `mp-units` strong types (`common/include/Common/Units.h`), GoogleTest, CMake, the Python harness in `scripts/benchmark/`.

**Spec:** `docs/superpowers/specs/2026-08-31-wavefront-frontier-exploration-design.md`. Read it before task 1.

## Global Constraints

- Human approval before every `git commit` (`.cursor/rules/git-workflow.mdc`). Stage and propose; do not commit unprompted. Plan text is not approval.
- Never edit `common/` or any published header. `IMappingAlgorithm`, `MappingStepCommand`, the plugin class name and `REGISTER_MAPPING_ALGORITHM` stay exactly as they are.
- `Algorithm_207190406_209543255.so` must stay independently loadable: no link-time dependency on MissionControl symbols. `UserCommon/` is included as headers per plugin.
- New code uses `mp-units` strong types. `force_numerical_value_in(cm)` is allowed only at the boundary with the existing double-based frontier substrate; no new raw-double geometry APIs. Never write a per-function unit alias (e17s) — use `common`'s `cm` / `deg`.
- No RNG anywhere. Determinism is a hard requirement of the benchmark harness.
- `Unmapped` stays traversable at soft cost 4 vs 1 for `Empty`. The safety invariant is: never command a move whose drone-sphere footprint contains `Occupied` or `OutOfBounds`.
- Constants, verbatim from the spec: `kRankedClusters = 8`, `kSweepStepsReserve = min(directions.size(), 8)`, `kMinInformationRate = 0.25`, `kLowRateReplans = 3`, `kReplanIntervalSteps = 25`, `kTravelScanProbes = 3`, `kMaxMovingStallTicks = 2` (unchanged), `kBlockedTtlSteps = 50` (unchanged), `kRecoveryAttempts = 3` (unchanged).
- The cone template walk must visit exactly the same voxel set as today's trig `countUnresolvedVoxels`. Speed comes from removing redundant work, not from a coarser cone.
- Do not change `MissionControl/`, `Simulator/`, or the harness scoring. Pass-2 is adapted to, not modified.
- Build and test from the repo root. Debug iteration: `cmake -S . -B build && cmake --build build -j` then `./build/Algorithm/algorithm_test --gtest_filter='...'`. Measurement builds: `cmake -S . -B build/opt -DCMAKE_BUILD_TYPE=Release && cmake --build build/opt -j`. In Docker, `apt-get install -y python3-venv python3-pip` may be needed once per container for the harness.
- After each task the tree must compile. `NbvPlanner` stays until Task 5; Task 2 therefore *adds* clusters alongside the existing `candidates` / `stride_cells` API.

### Spec lock-ins the implementer must not "simplify away"

1. **Start cell is a frontier cell when it has an `Unmapped` face neighbour.** Today's `exploreReachable` skips `start_key` when collecting candidates. Clustering must not. Otherwise spawn (all-`Unmapped`) has no cluster whose `approach_key` is the start, and in-place sweep dies.
2. **`ExplorationPlan` stores `target_keys`**, not just `target_cluster_cells`. The spec's local invalidation check ("any cell of the target cluster is still a frontier") needs the keys.
3. **A replan casts zero cones.** Ranking uses `cell_count` from the sweep. Cone walks happen at arrival (sweep list) and as a cheap per-step travel probe.
4. **Cluster `cell_count` is the frontier-cell count**, including reachable `Unmapped` cells that themselves have `Unmapped` neighbours. Because `Unmapped` is traversable, that count approximates the connected unknown volume, not just the Empty shell. That is what makes a 400-voxel room beat a 3-cell crumb.
5. **`kMinInformationRate` is the only tuned constant.** Do not change it in Tasks 1–5. Task 6 may sweep it; if a different value wins, record it and change it in one place.

---

## File Structure

| File | Responsibility | Task |
|------|----------------|------|
| `UserCommon/include/user_common_207190406_209543255/ConeTemplate.h` | Precomputed cone geometry, `VoxelStamp`, template walk | 1 |
| `UserCommon/include/user_common_207190406_209543255/LidarCone.h` | Unchanged trig oracle; Task 1 does not rewrite it | 1 |
| `Algorithm/tests/test_cone_template.cpp` | Equivalence, occlusion, stamp reuse, `near_field_samples` | 1 |
| `Algorithm/src/MappingAlgorithmFrontier.h/.cpp` | Clusters + memoised passability; keep `candidates` until Task 5 | 2, 5 |
| `Algorithm/tests/test_mapping_algorithm_frontier.cpp` | Cluster geometry, 6-face flag, expansion bound | 2, 5 |
| `Algorithm/src/WavefrontPlanner.h/.cpp` | Rank clusters; emit `ExplorationPlan`; no cone casts | 3 |
| `Algorithm/tests/test_wavefront_planner.cpp` | Distant-room-vs-crumb, budget, spawn, determinism | 3 |
| `Algorithm/src/ScanPlanning.h/.cpp` | Arrival-time two-pass sweep; masked gain; pass-2 travel gate | 4 |
| `Algorithm/tests/test_scan_planning.cpp` | Mask, independent-then-marginal order, near-field reject | 4 |
| `Algorithm/src/MappingAlgorithmImpl.cpp`, `Algorithm/include/Algorithm/MappingAlgorithmImpl.h` | Sweep-on-arrival; rate termination; cluster invalidation | 4 |
| `Algorithm/tests/test_mapping_algorithm.cpp` | Co-emission, keep-working, low-rate finish, cluster abandon | 4 |
| `Algorithm/src/NbvPlanner.h/.cpp`, `Algorithm/tests/test_nbv_planner.cpp` | Deleted | 5 |
| `Algorithm/CMakeLists.txt` | New sources in both targets; drop NBV in Task 5 | 1, 3, 4, 5 |
| `docs/*` + `docs/benchmarks/2026-08-31-post_f_honest.{csv,md}` | Measurement and documentation | 6 |

`PathShaping.{h,cpp}` is unchanged.

---

### Task 1: Cone templates

Header-only, like `LidarCone.h`. The trig walk stays the equivalence oracle — do not delete or rewrite `countUnresolvedVoxels`.

**Files:**
- Create: `UserCommon/include/user_common_207190406_209543255/ConeTemplate.h`
- Create: `Algorithm/tests/test_cone_template.cpp`
- Modify: `Algorithm/CMakeLists.txt` (add the test file to `algorithm_test` only)

**Interfaces:**
- Consumes: `lidar_cone::{forEachConeBeam, fibonacciSphereOrientations, directionCountForHalfAngle, coneHalfAngleRad, voxelKey, countUnresolvedVoxels}`; `beam_math::{pointAlongBeam, normalizeOrientation}`.
- Produces, all in `namespace user_common_207190406_209543255::cone_template`:
  - `struct BeamRun { double ux; double uy; double uz; std::size_t sample_count; };`
  - `struct ConeTemplate { common::Orientation direction; std::vector<BeamRun> beams; double step_cm; std::size_t near_field_samples; };`
  - `class VoxelStamp` with `void begin(const common::types::MapConfig&, const common::Position3D& origin, common::PhysicalLength z_max);` and `bool mark(const common::types::MapConfig&, const common::Position3D&)` — `true` if this generation has not seen the voxel.
  - `class ConeTemplateCache` with `const std::vector<ConeTemplate>& get(const common::types::LidarConfigData&, common::PhysicalLength resolution);`
  - `std::size_t walkTemplate(const ConeTemplate&, const common::IMap3D&, const common::Position3D& origin, VoxelStamp&, Fn&& on_unresolved)` — `on_unresolved(const Position3D&) -> bool`; `false` stops the whole cone. Occupied / OutOfBounds terminate the current beam only.
  - `bool nearFieldContainsSolid(const ConeTemplate&, const common::IMap3D&, const common::Position3D& origin);`

- [ ] **Step 1: Write the failing tests**

Create `Algorithm/tests/test_cone_template.cpp`:

```cpp
// test_cone_template.cpp — ConeTemplate walk equals the trig oracle.

#include "FakeMap3D.h"

#include <user_common_207190406_209543255/ConeTemplate.h>
#include <user_common_207190406_209543255/LidarCone.h>

#include <gtest/gtest.h>

#include <cstdint>
#include <unordered_set>
#include <vector>

namespace lc = user_common_207190406_209543255::lidar_cone;
namespace ctpl = user_common_207190406_209543255::cone_template;
using Map = AlgorithmTest::FakeMap3D;

namespace {

using common::Orientation;
using common::Position3D;
using common::cm;
using common::deg;
using common::x_extent;
using common::y_extent;
using common::z_extent;
namespace ct = common::types;

[[nodiscard]] ct::LidarConfigData makeShortLidar() {
    ct::LidarConfigData cfg{};
    cfg.z_min = 20.0 * cm;
    cfg.z_max = 80.0 * cm;
    cfg.d = 2.5 * cm;
    cfg.fov_circles = 4;
    return cfg;
}

[[nodiscard]] ct::MapConfig makeSmallMapConfig() {
    ct::MapConfig config{};
    config.resolution = 10.0 * cm;
    config.offset = Position3D{};
    config.boundaries.min_x = 0.0 * x_extent[cm];
    config.boundaries.max_x = 100.0 * x_extent[cm];
    config.boundaries.min_y = 0.0 * y_extent[cm];
    config.boundaries.max_y = 100.0 * y_extent[cm];
    config.boundaries.min_height = 0.0 * z_extent[cm];
    config.boundaries.max_height = 100.0 * z_extent[cm];
    return config;
}

[[nodiscard]] std::unordered_set<std::int64_t> trigKeys(
    const Map& map, const Position3D& origin, const Orientation& dir,
    const ct::LidarConfigData& lidar) {
    std::unordered_set<std::int64_t> seen;
    (void)lc::countUnresolvedVoxels(map, origin, Orientation{}, dir, lidar, seen);
    return seen;
}

[[nodiscard]] std::unordered_set<std::int64_t> templateKeys(
    const ctpl::ConeTemplate& cone, const Map& map, const Position3D& origin) {
    ctpl::VoxelStamp stamp;
    stamp.begin(map.getMapConfig(), origin, 80.0 * cm);
    std::unordered_set<std::int64_t> keys;
    const auto config = map.getMapConfig();
    (void)ctpl::walkTemplate(cone, map, origin, stamp, [&](const Position3D& p) {
        keys.insert(lc::voxelKey(config, p));
        return true;
    });
    return keys;
}

} // namespace

TEST(ConeTemplate, WalkMatchesTrigVoxelSetOnAlignedOrigin) {
    const ct::MapConfig config = makeSmallMapConfig();
    Map map{{11, 11, 11}, config, ct::VoxelOccupancy::Unmapped};
    const Position3D origin{50.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]};
    const ct::LidarConfigData lidar = makeShortLidar();

    ctpl::ConeTemplateCache cache;
    const auto& templates = cache.get(lidar, config.resolution);
    ASSERT_FALSE(templates.empty());

    const auto trig = trigKeys(map, origin, templates.front().direction, lidar);
    const auto tmpl = templateKeys(templates.front(), map, origin);
    EXPECT_EQ(trig, tmpl);
}

TEST(ConeTemplate, WalkMatchesTrigVoxelSetOnNonAlignedOrigin) {
    const ct::MapConfig config = makeSmallMapConfig();
    Map map{{11, 11, 11}, config, ct::VoxelOccupancy::Unmapped};
    const Position3D origin{53.0 * x_extent[cm], 47.0 * y_extent[cm], 51.0 * z_extent[cm]};
    const ct::LidarConfigData lidar = makeShortLidar();

    ctpl::ConeTemplateCache cache;
    const auto& templates = cache.get(lidar, config.resolution);
    ASSERT_GE(templates.size(), 6u);

    for (std::size_t i = 0; i < 6; ++i) {
        EXPECT_EQ(trigKeys(map, origin, templates[i].direction, lidar),
                  templateKeys(templates[i], map, origin))
            << "axis " << i;
    }
}

TEST(ConeTemplate, WalkStopsAtOccupied) {
    const ct::MapConfig config = makeSmallMapConfig();
    Map map{{11, 11, 11}, config, ct::VoxelOccupancy::Empty};
    const Position3D origin{50.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]};
    map.set(Position3D{60.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]},
            ct::VoxelOccupancy::Occupied);
    map.set(Position3D{70.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]},
            ct::VoxelOccupancy::Unmapped);

    ct::LidarConfigData lidar = makeShortLidar();
    lidar.fov_circles = 1;
    lidar.z_max = 40.0 * cm;

    ctpl::ConeTemplateCache cache;
    const auto& templates = cache.get(lidar, config.resolution);
    ASSERT_FALSE(templates.empty());

    EXPECT_EQ(trigKeys(map, origin, Orientation{}, lidar),
              templateKeys(templates.front(), map, origin));
    EXPECT_TRUE(templateKeys(templates.front(), map, origin).empty());
}

TEST(ConeTemplate, StampDeduplicatesAcrossBeamsAndGenerations) {
    const ct::MapConfig config = makeSmallMapConfig();
    Map map{{11, 11, 11}, config, ct::VoxelOccupancy::Unmapped};
    const Position3D origin{50.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]};
    const ct::LidarConfigData lidar = makeShortLidar();

    ctpl::ConeTemplateCache cache;
    const auto& templates = cache.get(lidar, config.resolution);
    ctpl::VoxelStamp stamp;
    stamp.begin(config, origin, lidar.z_max);

    std::size_t first = 0;
    (void)ctpl::walkTemplate(templates.front(), map, origin, stamp,
                             [&](const Position3D&) {
                                 ++first;
                                 return true;
                             });
    std::size_t second = 0;
    (void)ctpl::walkTemplate(templates.front(), map, origin, stamp,
                             [&](const Position3D&) {
                                 ++second;
                                 return true;
                             });
    EXPECT_GT(first, 0u);
    EXPECT_EQ(second, 0u);

    stamp.begin(config, origin, lidar.z_max);
    std::size_t third = 0;
    (void)ctpl::walkTemplate(templates.front(), map, origin, stamp,
                             [&](const Position3D&) {
                                 ++third;
                                 return true;
                             });
    EXPECT_EQ(third, first);
}

TEST(ConeTemplate, NearFieldSamplesCoverExactlyInsideZMin) {
    const ct::LidarConfigData lidar = makeShortLidar();
    const ct::MapConfig config = makeSmallMapConfig();
    ctpl::ConeTemplateCache cache;
    const auto& templates = cache.get(lidar, config.resolution);
    ASSERT_FALSE(templates.empty());

    const double step = templates.front().step_cm;
    ASSERT_GT(step, 0.0);
    std::size_t expected = 0;
    for (double dist = step; dist < 20.0 - 1e-9; dist += step) {
        ++expected;
    }
    EXPECT_EQ(templates.front().near_field_samples, expected);
}

TEST(ConeTemplate, CacheReturnsSameTemplatesForSameLidarAndResolution) {
    const ct::LidarConfigData lidar = makeShortLidar();
    ctpl::ConeTemplateCache cache;
    const auto& a = cache.get(lidar, 10.0 * cm);
    const auto& b = cache.get(lidar, 10.0 * cm);
    EXPECT_EQ(&a, &b);
    EXPECT_EQ(a.size(), lc::fibonacciSphereOrientations(
                            lc::directionCountForHalfAngle(lc::coneHalfAngleRad(lidar)))
                            .size());
}

TEST(ConeTemplate, NearFieldContainsSolidWhenOccupiedInsideZMin) {
    const ct::MapConfig config = makeSmallMapConfig();
    Map map{{11, 11, 11}, config, ct::VoxelOccupancy::Empty};
    const Position3D origin{50.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]};
    map.set(Position3D{60.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]},
            ct::VoxelOccupancy::Occupied);  // 10 cm, inside z_min = 20 cm

    ct::LidarConfigData lidar = makeShortLidar();
    lidar.fov_circles = 1;
    ctpl::ConeTemplateCache cache;
    const auto& templates = cache.get(lidar, config.resolution);
    EXPECT_TRUE(ctpl::nearFieldContainsSolid(templates.front(), map, origin));
}
```

Add `tests/test_cone_template.cpp` to the `algorithm_test` source list in `Algorithm/CMakeLists.txt`.

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cmake --build build -j && ./build/Algorithm/algorithm_test --gtest_filter='ConeTemplate.*'`

Expected: compile error — `ConeTemplate.h` does not exist.

- [ ] **Step 3: Implement `ConeTemplate.h`**

Create `UserCommon/include/user_common_207190406_209543255/ConeTemplate.h`. Full file:

```cpp
#pragma once

// Precomputed lidar-cone geometry. A walk is origin + k*step*u, no trig, no hashing.
// Voxel set equals lidar_cone::countUnresolvedVoxels, verified by test_cone_template.

#include <user_common_207190406_209543255/LidarCone.h>

#include <Common/IMap3D.h>
#include <Common/Units.h>
#include <Common/types/LidarTypes.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace user_common_207190406_209543255::cone_template {

namespace lc = user_common_207190406_209543255::lidar_cone;
namespace bm = user_common_207190406_209543255::beam_math;
using common::Orientation;
using common::PhysicalLength;
using common::Position3D;
using common::cm;
using common::x_extent;
using common::y_extent;
using common::z_extent;

struct BeamRun {
    double ux = 0.0;
    double uy = 0.0;
    double uz = 0.0;
    std::size_t sample_count = 0;
};

struct ConeTemplate {
    Orientation direction{};
    std::vector<BeamRun> beams{};
    double step_cm = 0.0;
    std::size_t near_field_samples = 0;
};

class VoxelStamp {
public:
    void begin(const common::types::MapConfig& config,
               const Position3D& origin,
               PhysicalLength z_max) {
        const double step = config.resolution.force_numerical_value_in(cm);
        const double range = z_max.force_numerical_value_in(cm);
        const int radius =
            (step > 0.0) ? static_cast<int>(std::ceil(range / step) + 2.0) : 1;
        const int span = 2 * radius + 1;
        const std::size_t needed =
            static_cast<std::size_t>(span) * static_cast<std::size_t>(span) *
            static_cast<std::size_t>(span);
        if (cells_.size() < needed) {
            cells_.assign(needed, 0);
        }
        radius_ = radius;
        span_ = span;
        origin_qx_ = quant(origin.x.force_numerical_value_in(cm),
                           config.offset.x.force_numerical_value_in(cm), step);
        origin_qy_ = quant(origin.y.force_numerical_value_in(cm),
                           config.offset.y.force_numerical_value_in(cm), step);
        origin_qz_ = quant(origin.z.force_numerical_value_in(cm),
                           config.offset.z.force_numerical_value_in(cm), step);
        if (++generation_ == 0) {
            std::fill(cells_.begin(), cells_.end(), 0);
            generation_ = 1;
        }
    }

    [[nodiscard]] bool mark(const common::types::MapConfig& config,
                            const Position3D& p) {
        const double step = config.resolution.force_numerical_value_in(cm);
        const int qx = quant(p.x.force_numerical_value_in(cm),
                             config.offset.x.force_numerical_value_in(cm), step);
        const int qy = quant(p.y.force_numerical_value_in(cm),
                             config.offset.y.force_numerical_value_in(cm), step);
        const int qz = quant(p.z.force_numerical_value_in(cm),
                             config.offset.z.force_numerical_value_in(cm), step);
        const int dx = qx - origin_qx_ + radius_;
        const int dy = qy - origin_qy_ + radius_;
        const int dz = qz - origin_qz_ + radius_;
        if (dx < 0 || dy < 0 || dz < 0 || dx >= span_ || dy >= span_ ||
            dz >= span_) {
            return true;  // outside the cone box: treat as unseen, still count
        }
        const std::size_t idx =
            (static_cast<std::size_t>(dx) * static_cast<std::size_t>(span_) +
             static_cast<std::size_t>(dy)) *
                static_cast<std::size_t>(span_) +
            static_cast<std::size_t>(dz);
        if (cells_[idx] == generation_) {
            return false;
        }
        cells_[idx] = generation_;
        return true;
    }

private:
    [[nodiscard]] static int quant(double value, double origin, double step) {
        if (!(step > 0.0)) {
            return 0;
        }
        return static_cast<int>(std::llround((value - origin) / step));
    }

    std::vector<std::uint32_t> cells_{};
    std::uint32_t generation_ = 0;
    int origin_qx_ = 0;
    int origin_qy_ = 0;
    int origin_qz_ = 0;
    int radius_ = 0;
    int span_ = 0;
};

class ConeTemplateCache {
public:
    [[nodiscard]] const std::vector<ConeTemplate>& get(
        const common::types::LidarConfigData& lidar,
        PhysicalLength resolution) {
        const double res_cm = resolution.force_numerical_value_in(cm);
        const double z_min = lidar.z_min.force_numerical_value_in(cm);
        const double z_max = lidar.z_max.force_numerical_value_in(cm);
        const double d = lidar.d.force_numerical_value_in(cm);
        if (built_ && res_cm == res_cm_ && z_min == z_min_ && z_max == z_max_ &&
            d == d_ && lidar.fov_circles == fov_circles_) {
            return templates_;
        }
        templates_ = build(lidar, res_cm);
        res_cm_ = res_cm;
        z_min_ = z_min;
        z_max_ = z_max;
        d_ = d;
        fov_circles_ = lidar.fov_circles;
        built_ = true;
        return templates_;
    }

private:
    [[nodiscard]] static std::vector<ConeTemplate> build(
        const common::types::LidarConfigData& lidar, double res_cm) {
        std::vector<ConeTemplate> out;
        if (!(res_cm > 0.0)) {
            return out;
        }
        const double alpha = lc::coneHalfAngleRad(lidar);
        const auto dirs =
            lc::fibonacciSphereOrientations(lc::directionCountForHalfAngle(alpha));
        const double step_cm = 0.5 * res_cm;
        const double z_max = lidar.z_max.force_numerical_value_in(cm);
        const double z_min = lidar.z_min.force_numerical_value_in(cm);
        std::size_t near_field = 0;
        if (step_cm > 0.0) {
            for (double dist = step_cm; dist < z_min - 1e-9; dist += step_cm) {
                ++near_field;
            }
        }
        out.reserve(dirs.size());
        for (const Orientation& dir : dirs) {
            ConeTemplate cone;
            cone.direction = dir;
            cone.step_cm = step_cm;
            cone.near_field_samples = near_field;
            lc::forEachConeBeam(lidar, dir, [&](const Orientation& beam) {
                const Position3D unit = bm::pointAlongBeam(
                    Position3D{}, bm::normalizeOrientation(beam), 1.0 * cm);
                BeamRun run;
                run.ux = unit.x.force_numerical_value_in(cm);
                run.uy = unit.y.force_numerical_value_in(cm);
                run.uz = unit.z.force_numerical_value_in(cm);
                run.sample_count = 0;
                if (step_cm > 0.0) {
                    for (double dist = step_cm; dist <= z_max + 1e-9;
                         dist += step_cm) {
                        ++run.sample_count;
                    }
                }
                cone.beams.push_back(run);
                return true;
            });
            out.push_back(std::move(cone));
        }
        return out;
    }

    std::vector<ConeTemplate> templates_{};
    bool built_ = false;
    double res_cm_ = 0.0;
    double z_min_ = 0.0;
    double z_max_ = 0.0;
    double d_ = 0.0;
    std::size_t fov_circles_ = 0;
};

template <typename Fn>
inline std::size_t walkTemplate(const ConeTemplate& cone,
                                const common::IMap3D& map,
                                const Position3D& origin,
                                VoxelStamp& stamp,
                                Fn&& on_unresolved) {
    const auto config = map.getMapConfig();
    const double ox = origin.x.force_numerical_value_in(cm);
    const double oy = origin.y.force_numerical_value_in(cm);
    const double oz = origin.z.force_numerical_value_in(cm);
    std::size_t added = 0;
    for (const BeamRun& beam : cone.beams) {
        for (std::size_t i = 1; i <= beam.sample_count; ++i) {
            const double dist = static_cast<double>(i) * cone.step_cm;
            const Position3D p{
                (ox + dist * beam.ux) * x_extent[cm],
                (oy + dist * beam.uy) * y_extent[cm],
                (oz + dist * beam.uz) * z_extent[cm],
            };
            const auto occ = map.atVoxel(p);
            if (occ == common::types::VoxelOccupancy::Occupied ||
                occ == common::types::VoxelOccupancy::OutOfBounds) {
                break;
            }
            if (occ != common::types::VoxelOccupancy::Unmapped) {
                continue;
            }
            if (!stamp.mark(config, p)) {
                continue;
            }
            ++added;
            if (!on_unresolved(p)) {
                return added;
            }
        }
    }
    return added;
}

[[nodiscard]] inline bool nearFieldContainsSolid(const ConeTemplate& cone,
                                                 const common::IMap3D& map,
                                                 const Position3D& origin) {
    const double ox = origin.x.force_numerical_value_in(cm);
    const double oy = origin.y.force_numerical_value_in(cm);
    const double oz = origin.z.force_numerical_value_in(cm);
    const std::size_t n = cone.near_field_samples;
    if (n == 0) {
        return false;
    }
    for (const BeamRun& beam : cone.beams) {
        const std::size_t limit = std::min(n, beam.sample_count);
        for (std::size_t i = 1; i <= limit; ++i) {
            const double dist = static_cast<double>(i) * cone.step_cm;
            const Position3D p{
                (ox + dist * beam.ux) * x_extent[cm],
                (oy + dist * beam.uy) * y_extent[cm],
                (oz + dist * beam.uz) * z_extent[cm],
            };
            const auto occ = map.atVoxel(p);
            if (occ == common::types::VoxelOccupancy::Occupied ||
                occ == common::types::VoxelOccupancy::OutOfBounds) {
                return true;
            }
        }
    }
    return false;
}

} // namespace user_common_207190406_209543255::cone_template
```

Add `#include <algorithm>` for `std::min` / `std::fill`.

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cmake --build build -j && ./build/Algorithm/algorithm_test --gtest_filter='ConeTemplate.*:LidarCone.*'`

Expected: all PASS. If `WalkMatchesTrigVoxelSet*` fails, the sample loop or unit-vector construction diverges from `walkBeam` / `pointAlongBeam` — fix the template, not the oracle.

- [ ] **Step 5: Propose the commit**

Stage: `UserCommon/include/user_common_207190406_209543255/ConeTemplate.h`, `Algorithm/tests/test_cone_template.cpp`, `Algorithm/CMakeLists.txt`.

Proposed message:

```
feat: precompute lidar cone templates for trig-free walks
```

Stop and wait for approval. Do not commit.

---

### Task 2: Wavefront clusters on `exploreReachable`

Add clustering and memoised passability. **Keep** `ReachableCell`, `candidates`, and `stride_cells` so `NbvPlanner` still compiles. Switch the frontier flag from 26 neighbours to 6 faces for both candidates and clusters (cluster size supersedes the 26-count; `unmapped_neighbours` becomes the 6-face count and NbvPlanner's prefilter gets slightly cheaper, which is fine — it is deleted in Task 5).

**Files:**
- Modify: `Algorithm/src/MappingAlgorithmFrontier.h`
- Modify: `Algorithm/src/MappingAlgorithmFrontier.cpp`
- Modify: `Algorithm/tests/test_mapping_algorithm_frontier.cpp`

**Interfaces:**
- Consumes: existing `isSpherePassable`, `occupancyAt`, `kOffsets`, `keyToPoint`, `quantizePosition`.
- Produces (added to the existing types):
  - `using FrontierCells = std::unordered_set<GridKey, GridKeyHash>;`
  - `struct FrontierCluster { std::size_t cell_count; GridKey approach_key; common::Position3D approach_position; int approach_cost; std::vector<GridKey> keys; };`
  - `ReachabilityResult` gains `std::vector<FrontierCluster> clusters;` and `FrontierCells frontier_cells;`. Existing fields stay.
  - `exploreReachable` signature **unchanged** (`stride_cells` still required).

- [ ] **Step 1: Write the failing tests**

Append to `Algorithm/tests/test_mapping_algorithm_frontier.cpp`:

```cpp
TEST(MappingAlgorithm, ExploreReachableClustersTwoRoomsSeparatedByAWall) {
    const ct::MapConfig config = makeCm10Config();
    Map map{{11, 11, 11}, config, ct::VoxelOccupancy::Empty};
    // Occupied wall at x=50 splits the volume.
    for (int y = 0; y <= 10; ++y) {
        for (int z = 0; z <= 10; ++z) {
            map.set(pointCm(50, y * 10, z * 10), ct::VoxelOccupancy::Occupied);
        }
    }
    // Unknown pocket on the start side (x=20) and one beyond the wall (x=80).
    // The far pocket is unreachable (wall), so it must not join the near cluster.
    map.set(pointCm(20, 50, 50), ct::VoxelOccupancy::Unmapped);
    map.set(pointCm(80, 50, 50), ct::VoxelOccupancy::Unmapped);

    const detail::MappingAlgorithmFrontier frontier;
    const detail::ReachabilityResult result = frontier.exploreReachable(
        map, pointCm(0, 0, 0), 4.0 * cm, {}, 1, detail::maxExpansionsForMap(map));

    EXPECT_TRUE(result.start_passable);
    ASSERT_FALSE(result.clusters.empty());
    // Only the near pocket is reachable. One cluster, not two.
    EXPECT_EQ(result.clusters.size(), 1u);
    EXPECT_GT(result.clusters.front().cell_count, 0u);
    EXPECT_EQ(result.clusters.front().cell_count,
              result.clusters.front().keys.size());
}

TEST(MappingAlgorithm, ExploreReachableClusterCountEqualsFrontierSet) {
    const ct::MapConfig config = makeCm10Config();
    Map map{{11, 11, 11}, config, ct::VoxelOccupancy::Empty};
    map.set(pointCm(50, 50, 50), ct::VoxelOccupancy::Unmapped);

    const detail::MappingAlgorithmFrontier frontier;
    const detail::ReachabilityResult result = frontier.exploreReachable(
        map, pointCm(0, 0, 0), 4.0 * cm, {}, 1, detail::maxExpansionsForMap(map));

    std::size_t summed = 0;
    for (const detail::FrontierCluster& c : result.clusters) {
        summed += c.cell_count;
        EXPECT_EQ(c.cell_count, c.keys.size());
    }
    EXPECT_EQ(summed, result.frontier_cells.size());
    EXPECT_FALSE(result.frontier_cells.empty());
}

TEST(MappingAlgorithm, ExploreReachableApproachKeyIsLowestCostMember) {
    const ct::MapConfig config = makeCm10Config();
    Map map{{11, 11, 11}, config, ct::VoxelOccupancy::Empty};
    map.set(pointCm(80, 50, 50), ct::VoxelOccupancy::Unmapped);

    const detail::MappingAlgorithmFrontier frontier;
    const Position3D start = pointCm(0, 50, 50);
    const detail::ReachabilityResult result = frontier.exploreReachable(
        map, start, 4.0 * cm, {}, 1, detail::maxExpansionsForMap(map));

    ASSERT_FALSE(result.clusters.empty());
    const detail::FrontierCluster& cluster = result.clusters.front();
    int min_cost = cluster.approach_cost;
    for (const detail::GridKey& key : cluster.keys) {
        const auto it = result.parent_of.find(key);
        ASSERT_TRUE(it != result.parent_of.end() || key == result.start_key);
        (void)it;
    }
    // The approach cell is on the near side of the pocket, not past it.
    EXPECT_LE(cluster.approach_position.x.force_numerical_value_in(cm), 80.0);
    EXPECT_GE(min_cost, 0);
}

TEST(MappingAlgorithm, FrontierFlagIgnoresDiagonalOnlyUnmapped) {
    const ct::MapConfig config = makeCm10Config();
    Map map{{11, 11, 11}, config, ct::VoxelOccupancy::Empty};
    // Only a diagonal neighbour is Unmapped — not a 6-face frontier.
    map.set(pointCm(60, 60, 60), ct::VoxelOccupancy::Unmapped);

    const detail::MappingAlgorithmFrontier frontier;
    const detail::ReachabilityResult result = frontier.exploreReachable(
        map, pointCm(50, 50, 50), 4.0 * cm, {}, 1, detail::maxExpansionsForMap(map));

    EXPECT_TRUE(result.frontier_cells.empty());
    EXPECT_TRUE(result.clusters.empty());
}

TEST(MappingAlgorithm, ExploreReachableIncludesStartWhenItBordersUnmapped) {
    const ct::MapConfig config = makeCm10Config();
    Map map{{11, 11, 11}, config, ct::VoxelOccupancy::Unmapped};
    const Position3D start = pointCm(50, 50, 50);
    map.set(start, ct::VoxelOccupancy::Empty);

    const detail::MappingAlgorithmFrontier frontier;
    const detail::ReachabilityResult result = frontier.exploreReachable(
        map, start, 4.0 * cm, {}, 1, detail::maxExpansionsForMap(map));

    ASSERT_FALSE(result.clusters.empty());
    bool start_is_approach = false;
    for (const detail::FrontierCluster& c : result.clusters) {
        if (c.approach_key == result.start_key) {
            start_is_approach = true;
        }
    }
    EXPECT_TRUE(start_is_approach);
    EXPECT_TRUE(result.frontier_cells.contains(result.start_key));
}

TEST(MappingAlgorithm, ExploreReachableClusteringIsDeterministic) {
    const ct::MapConfig config = makeCm10Config();
    Map map{{11, 11, 11}, config, ct::VoxelOccupancy::Empty};
    map.set(pointCm(80, 50, 50), ct::VoxelOccupancy::Unmapped);
    map.set(pointCm(80, 60, 50), ct::VoxelOccupancy::Unmapped);

    const detail::MappingAlgorithmFrontier frontier;
    const auto a = frontier.exploreReachable(
        map, pointCm(0, 0, 0), 4.0 * cm, {}, 1, detail::maxExpansionsForMap(map));
    const auto b = frontier.exploreReachable(
        map, pointCm(0, 0, 0), 4.0 * cm, {}, 1, detail::maxExpansionsForMap(map));

    ASSERT_EQ(a.clusters.size(), b.clusters.size());
    for (std::size_t i = 0; i < a.clusters.size(); ++i) {
        EXPECT_EQ(a.clusters[i].cell_count, b.clusters[i].cell_count);
        EXPECT_EQ(a.clusters[i].approach_cost, b.clusters[i].approach_cost);
        EXPECT_EQ(a.clusters[i].approach_key, b.clusters[i].approach_key);
    }
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cmake --build build -j && ./build/Algorithm/algorithm_test --gtest_filter='MappingAlgorithm.ExploreReachableClusters*:MappingAlgorithm.ExploreReachableCluster*:MappingAlgorithm.ExploreReachableApproach*:MappingAlgorithm.FrontierFlag*:MappingAlgorithm.ExploreReachableIncludesStart*:MappingAlgorithm.ExploreReachableClustering*'`

Expected: compile error — `FrontierCluster` / `clusters` / `frontier_cells` do not exist.

- [ ] **Step 3: Add the types and fill them in the sweep**

In `MappingAlgorithmFrontier.h`, after `ReachableCell`:

```cpp
using FrontierCells = std::unordered_set<GridKey, GridKeyHash>;

struct FrontierCluster {
    std::size_t cell_count = 0;
    GridKey approach_key{};
    common::Position3D approach_position{};
    int approach_cost = 0;
    std::vector<GridKey> keys{};
};
```

Add to `ReachabilityResult`:

```cpp
    FrontierCells frontier_cells{};
    std::vector<FrontierCluster> clusters{};
```

In `MappingAlgorithmFrontier.cpp`, inside `exploreReachable`:

1. **Memo.** Before the Dijkstra loop, `std::unordered_map<GridKey, bool, GridKeyHash> passable_memo;`. Wrap every `isSpherePassable(...)` neighbour check:

```cpp
auto passable = [&](const GridKey& key, const Position3D& pt) {
    const auto it = passable_memo.find(key);
    if (it != passable_memo.end()) {
        return it->second;
    }
    const bool ok = isSpherePassable(map, pt, radius_cm, step, blocked_cells);
    passable_memo.emplace(key, ok);
    return ok;
};
```

Use `passable` for neighbour expansion. The start check can stay a direct call (once).

2. **6-face frontier flag, including start.** Replace the 26-neighbour triple loop. After popping `current`, count `Unmapped` face neighbours with `kOffsets`. If `unmapped > 0`, insert `current` into a `std::vector<GridKey> frontier_list` and into `out.frontier_cells`, and record `cost_of[current]` / position for later clustering. Do this for `start_key` too.

3. **Keep building `candidates`** from those same frontier cells with the existing stride-bucket logic, so `NbvPlanner` still has a list. `unmapped_neighbours` is now the 6-face count.

4. **Cluster after the Dijkstra loop.** BFS over `frontier_list` using `kOffsets`, membership tested with `out.frontier_cells`. For each new cluster, walk the component, push keys, track the member with the lowest `cost_of` as `approach_*`, set `cell_count = keys.size()`. Append to `out.clusters`. Sort clusters by `approach_key` (`qx`, then `qy`, then `qz`) so the vector order is deterministic.

Do not add a `stride_cells` path into clustering.

- [ ] **Step 4: Run the new tests and the existing frontier suite**

Run: `cmake --build build -j && ./build/Algorithm/algorithm_test --gtest_filter='MappingAlgorithm.*'`

Expected: PASS, including `ExploreReachableFindsFrontierAdjacentCandidates`, `ExploreReachableStrideDeduplicatesCandidates`, `ExploreReachableRespectsExpansionCap`, `ExploreReachableTerminatesWithoutOccupancyBound`. If `FrontierFlagIgnoresDiagonalOnlyUnmapped` fails, the 26-neighbour loop is still live.

Then: `./build/Algorithm/algorithm_test --gtest_filter='NbvPlanner.*'`

Expected: PASS — `NbvPlanner` still compiles and behaves against `candidates`.

- [ ] **Step 5: Propose the commit**

Stage: `Algorithm/src/MappingAlgorithmFrontier.h`, `Algorithm/src/MappingAlgorithmFrontier.cpp`, `Algorithm/tests/test_mapping_algorithm_frontier.cpp`.

Proposed message:

```
feat: cluster reachable frontiers in the Dijkstra sweep
```

Stop and wait for approval. Do not commit.

---

### Task 3: `WavefrontPlanner` — rank clusters, no cones

`NbvPlanner` stays on disk. The new planner is a second policy module; Task 4 switches the executor; Task 5 deletes NBV.

**Files:**
- Create: `Algorithm/src/WavefrontPlanner.h`, `Algorithm/src/WavefrontPlanner.cpp`
- Create: `Algorithm/tests/test_wavefront_planner.cpp`
- Modify: `Algorithm/CMakeLists.txt` (add `.cpp` to both targets, test to `algorithm_test`)

**Interfaces:**
- Consumes: `MappingAlgorithmFrontier::exploreReachable`, `reconstructPathTo`, `maxExpansionsForMap`, `stringPullConstantAltitude`, `stepCostForPath`, `lidar_cone::{coneHalfAngleRad, directionCountForHalfAngle, fibonacciSphereOrientations}`.
- Produces, in `namespace algorithm_207190406_209543255::detail`:
  - `struct ExplorationPlan` **new fields live here**. Do not edit `NbvPlanner.h`'s `ExplorationPlan` — that is a different type in the same namespace today. **Name collision.** Resolve it by moving the shared plan type out in this task:
    - Cut `ExplorationPlan` and `NbvInputs` from `NbvPlanner.h` into a new `Algorithm/src/ExplorationPlan.h`.
    - `NbvPlanner.h` includes it and keeps its own `plan()` returning that struct (with `terminal_scans` / `expected_gain` still present).
    - `WavefrontPlanner` fills `waypoints`, `valid`, `expected_rate`, `target_cluster_cells`, `target_keys` and leaves `terminal_scans` empty and `expected_gain` unused.
  - `struct WavefrontInputs` — same fields as `NbvInputs` (map, state, lidar, drone, remaining_steps, blocked, ignore_blocked).
  - `class WavefrontPlanner` with `ExplorationPlan plan(const WavefrontInputs&) const;`

`ExplorationPlan` after the move (single definition):

```cpp
struct ExplorationPlan {
    std::vector<common::Position3D> waypoints{};
    std::vector<common::Orientation> terminal_scans{};  // NBV only; empty under WFD
    double expected_gain = 0.0;                         // NBV only
    std::size_t target_cluster_cells = 0;
    double expected_rate = 0.0;
    std::vector<GridKey> target_keys{};
    FrontierCells frontier_cells{};
    bool valid = false;
};
```

Putting new fields on the shared struct keeps `NbvPlanner` compiling (it ignores them). Do not invent a second `ExplorationPlan` type.

- [ ] **Step 1: Write the failing tests**

Create `Algorithm/tests/test_wavefront_planner.cpp`:

```cpp
// test_wavefront_planner.cpp — cluster ranking, budget filter, determinism.

#include "FakeMap3D.h"
#include "WavefrontPlanner.h"

#include <gtest/gtest.h>

namespace detail = algorithm_207190406_209543255::detail;
using Map = AlgorithmTest::FakeMap3D;

namespace {

using common::Orientation;
using common::Position3D;
using common::cm;
using common::deg;
using common::x_extent;
using common::y_extent;
using common::z_extent;
namespace ct = common::types;

[[nodiscard]] ct::MapConfig makeConfig() {
    ct::MapConfig config{};
    config.resolution = 10.0 * cm;
    config.offset = Position3D{};
    config.boundaries.min_x = 0.0 * x_extent[cm];
    config.boundaries.max_x = 200.0 * x_extent[cm];
    config.boundaries.min_y = 0.0 * y_extent[cm];
    config.boundaries.max_y = 200.0 * y_extent[cm];
    config.boundaries.min_height = 0.0 * z_extent[cm];
    config.boundaries.max_height = 200.0 * z_extent[cm];
    return config;
}

[[nodiscard]] ct::LidarConfigData makeLidar() {
    ct::LidarConfigData cfg{};
    cfg.z_min = 20.0 * cm;
    cfg.z_max = 80.0 * cm;
    cfg.d = 2.5 * cm;
    cfg.fov_circles = 4;
    return cfg;
}

[[nodiscard]] ct::DroneConfigData makeDrone() {
    ct::DroneConfigData cfg{};
    cfg.radius = 4.0 * cm;
    cfg.max_rotate = 90.0 * deg;
    cfg.max_advance = 30.0 * cm;
    cfg.max_elevate = 20.0 * cm;
    return cfg;
}

[[nodiscard]] Position3D at(double x, double y, double z) {
    return Position3D{x * x_extent[cm], y * y_extent[cm], z * z_extent[cm]};
}

[[nodiscard]] ct::DroneState stateAt(const Position3D& p, std::size_t step_index = 0) {
    return ct::DroneState{p, Orientation{0.0 * deg, 0.0 * deg}, step_index};
}

void fillUnmappedBox(Map& map, int x0, int x1, int y0, int y1, int z0, int z1) {
    for (int x = x0; x <= x1; ++x) {
        for (int y = y0; y <= y1; ++y) {
            for (int z = z0; z <= z1; ++z) {
                map.set(at(x * 10.0, y * 10.0, z * 10.0), ct::VoxelOccupancy::Unmapped);
            }
        }
    }
}

} // namespace

TEST(WavefrontPlanner, PrefersDistantRoomOverNearbyCrumb) {
    Map map{{21, 21, 21}, makeConfig(), ct::VoxelOccupancy::Empty};
    // Crumb: three Unmapped cells one step along +Y.
    map.set(at(50.0, 110.0, 0.0), ct::VoxelOccupancy::Unmapped);
    map.set(at(50.0, 120.0, 0.0), ct::VoxelOccupancy::Unmapped);
    map.set(at(50.0, 130.0, 0.0), ct::VoxelOccupancy::Unmapped);
    // Distant room: 8×8×6 Unmapped at +X, ~400 cells.
    fillUnmappedBox(map, 14, 21, 6, 13, 0, 5);

    const detail::WavefrontPlanner planner;
    const ct::DroneState state = stateAt(at(50.0, 100.0, 0.0));
    const detail::BlockedCells blocked;
    const detail::ExplorationPlan plan = planner.plan(
        {map, state, makeLidar(), makeDrone(), 1000, blocked, false});

    ASSERT_TRUE(plan.valid);
    ASSERT_FALSE(plan.waypoints.empty());
    EXPECT_GT(plan.waypoints.back().x.force_numerical_value_in(cm), 100.0);
    EXPECT_GT(plan.target_cluster_cells, 20u);
    EXPECT_TRUE(plan.terminal_scans.empty());
}

TEST(WavefrontPlanner, StaysPutWhenStartIsTheCheapestFrontier) {
    Map map{{21, 21, 21}, makeConfig(), ct::VoxelOccupancy::Unmapped};
    const detail::WavefrontPlanner planner;
    const ct::DroneState state = stateAt(at(100.0, 100.0, 100.0));
    const detail::BlockedCells blocked;
    const detail::ExplorationPlan plan = planner.plan(
        {map, state, makeLidar(), makeDrone(), 1000, blocked, false});

    ASSERT_TRUE(plan.valid);
    EXPECT_TRUE(plan.waypoints.empty());
    EXPECT_GT(plan.target_cluster_cells, 0u);
    EXPECT_GT(plan.expected_rate, 0.0);
}

TEST(WavefrontPlanner, DiscardsClustersTheRemainingBudgetCannotAfford) {
    Map map{{21, 21, 21}, makeConfig(), ct::VoxelOccupancy::Empty};
    fillUnmappedBox(map, 18, 20, 8, 12, 0, 2);

    const detail::WavefrontPlanner planner;
    const ct::DroneState state = stateAt(at(0.0, 100.0, 0.0));
    const detail::BlockedCells blocked;

    const detail::ExplorationPlan rich = planner.plan(
        {map, state, makeLidar(), makeDrone(), 1000, blocked, false});
    const detail::ExplorationPlan poor = planner.plan(
        {map, state, makeLidar(), makeDrone(), 2, blocked, false});

    EXPECT_TRUE(rich.valid);
    EXPECT_FALSE(rich.waypoints.empty());
    if (poor.valid) {
        EXPECT_TRUE(poor.waypoints.empty());
    }
}

TEST(WavefrontPlanner, IsDeterministicAcrossIdenticalCalls) {
    Map map{{21, 21, 21}, makeConfig(), ct::VoxelOccupancy::Empty};
    fillUnmappedBox(map, 16, 20, 8, 12, 0, 2);

    const detail::WavefrontPlanner planner;
    const ct::DroneState state = stateAt(at(50.0, 100.0, 0.0));
    const detail::BlockedCells blocked;
    const auto a = planner.plan({map, state, makeLidar(), makeDrone(), 1000, blocked, false});
    const auto b = planner.plan({map, state, makeLidar(), makeDrone(), 1000, blocked, false});

    ASSERT_EQ(a.waypoints.size(), b.waypoints.size());
    for (std::size_t i = 0; i < a.waypoints.size(); ++i) {
        EXPECT_DOUBLE_EQ(a.waypoints[i].x.force_numerical_value_in(cm),
                         b.waypoints[i].x.force_numerical_value_in(cm));
        EXPECT_DOUBLE_EQ(a.waypoints[i].y.force_numerical_value_in(cm),
                         b.waypoints[i].y.force_numerical_value_in(cm));
        EXPECT_DOUBLE_EQ(a.waypoints[i].z.force_numerical_value_in(cm),
                         b.waypoints[i].z.force_numerical_value_in(cm));
    }
    EXPECT_EQ(a.target_cluster_cells, b.target_cluster_cells);
    EXPECT_DOUBLE_EQ(a.expected_rate, b.expected_rate);
    ASSERT_EQ(a.target_keys.size(), b.target_keys.size());
}

TEST(WavefrontPlanner, ReportsInvalidWhenNothingIsUnresolved) {
    Map map{{21, 21, 21}, makeConfig(), ct::VoxelOccupancy::Empty};
    const detail::WavefrontPlanner planner;
    const ct::DroneState state = stateAt(at(100.0, 100.0, 100.0));
    const detail::BlockedCells blocked;
    const detail::ExplorationPlan plan = planner.plan(
        {map, state, makeLidar(), makeDrone(), 1000, blocked, false});

    EXPECT_FALSE(plan.valid);
    EXPECT_EQ(plan.target_cluster_cells, 0u);
}

TEST(WavefrontPlanner, IgnoreBlockedRecoversWhenTheBlockedSetSealsTheDrone) {
    Map map{{21, 21, 21}, makeConfig(), ct::VoxelOccupancy::Empty};
    fillUnmappedBox(map, 16, 20, 8, 12, 0, 2);
    const Position3D start = at(100.0, 100.0, 0.0);

    detail::BlockedCells blocked;
    const ct::MapConfig config = makeConfig();
    for (const Position3D& nb : {at(110.0, 100.0, 0.0), at(90.0, 100.0, 0.0),
                                 at(100.0, 110.0, 0.0), at(100.0, 90.0, 0.0),
                                 at(100.0, 100.0, 10.0), at(100.0, 100.0, -10.0)}) {
        blocked.insert(detail::quantizePosition(nb, config));
    }

    const detail::WavefrontPlanner planner;
    const ct::DroneState state = stateAt(start);
    const auto sealed = planner.plan({map, state, makeLidar(), makeDrone(), 1000, blocked, false});
    const auto recovered = planner.plan({map, state, makeLidar(), makeDrone(), 1000, blocked, true});

    EXPECT_TRUE(sealed.waypoints.empty());
    EXPECT_TRUE(recovered.valid);
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cmake --build build -j && ./build/Algorithm/algorithm_test --gtest_filter='WavefrontPlanner.*'`

Expected: FAIL — `WavefrontPlanner.h` does not exist.

- [ ] **Step 3: Move `ExplorationPlan` / `NbvInputs` and implement the planner**

Create `Algorithm/src/ExplorationPlan.h` with the struct shown above plus:

```cpp
#pragma once

#include "MappingAlgorithmFrontier.h"

#include <Common/IMap3D.h>
#include <Common/Units.h>
#include <Common/types/DroneTypes.h>
#include <Common/types/LidarTypes.h>

#include <cstddef>
#include <vector>

namespace algorithm_207190406_209543255::detail {

struct ExplorationPlan { /* fields as above */ };

struct NbvInputs {
    const common::IMap3D& map;
    const common::types::DroneState& state;
    const common::types::LidarConfigData& lidar;
    const common::types::DroneConfigData& drone;
    std::size_t remaining_steps = 0;
    const BlockedCells& blocked;
    bool ignore_blocked = false;
};

using WavefrontInputs = NbvInputs;

} // namespace
```

`NbvPlanner.h` drops its own struct definitions and `#include "ExplorationPlan.h"`. `MappingAlgorithmImpl.cpp` keeps compiling.

`Algorithm/src/WavefrontPlanner.h`:

```cpp
#pragma once

#include "ExplorationPlan.h"
#include "MappingAlgorithmFrontier.h"
#include "PathShaping.h"

namespace algorithm_207190406_209543255::detail {

class WavefrontPlanner {
public:
    [[nodiscard]] ExplorationPlan plan(const WavefrontInputs& in) const;

private:
    static constexpr std::size_t kRankedClusters = 8;

    MappingAlgorithmFrontier frontier_{};
};

} // namespace
```

`Algorithm/src/WavefrontPlanner.cpp` — put `kMaxSweepReserve` in the anonymous namespace (the header's `kRankedClusters` stays private on the class). Implement:

```cpp
namespace {

constexpr std::size_t kMaxSweepReserve = 8;

[[nodiscard]] MovementLimits limitsFrom(const types::DroneConfigData& drone) {
    return MovementLimits{drone.max_advance, drone.max_elevate, drone.max_rotate};
}

[[nodiscard]] std::size_t reserveFor(const types::LidarConfigData& lidar) {
    const double alpha = lc::coneHalfAngleRad(lidar);
    const std::size_t n = lc::directionCountForHalfAngle(alpha);
    return std::min(n, kMaxSweepReserve);
}

} // namespace

ExplorationPlan WavefrontPlanner::plan(const WavefrontInputs& in) const {
    const BlockedCells empty_blocked;
    const BlockedCells& blocked = in.ignore_blocked ? empty_blocked : in.blocked;
    const types::MapConfig config = in.map.getMapConfig();
    const MovementLimits limits = limitsFrom(in.drone);
    const std::size_t reserve = reserveFor(in.lidar);

    const ReachabilityResult reach = frontier_.exploreReachable(
        in.map, in.state.position, in.drone.radius, blocked,
        /*stride_cells=*/3, maxExpansionsForMap(in.map));
    if (!reach.start_passable || reach.clusters.empty()) {
        return {};
    }

    std::vector<const FrontierCluster*> ranked;
    ranked.reserve(reach.clusters.size());
    for (const FrontierCluster& c : reach.clusters) {
        ranked.push_back(&c);
    }
    std::stable_sort(ranked.begin(), ranked.end(),
                     [](const FrontierCluster* a, const FrontierCluster* b) {
                         if (a->cell_count != b->cell_count) {
                             return a->cell_count > b->cell_count;
                         }
                         if (a->approach_cost != b->approach_cost) {
                             return a->approach_cost < b->approach_cost;
                         }
                         if (a->approach_key.qx != b->approach_key.qx) {
                             return a->approach_key.qx < b->approach_key.qx;
                         }
                         if (a->approach_key.qy != b->approach_key.qy) {
                             return a->approach_key.qy < b->approach_key.qy;
                         }
                         return a->approach_key.qz < b->approach_key.qz;
                     });
    if (ranked.size() > kRankedClusters) {
        ranked.resize(kRankedClusters);
    }

    ExplorationPlan best;
    double best_rate = -1.0;
    for (const FrontierCluster* cluster : ranked) {
        FrontierPathResult raw = reconstructPathTo(
            reach.parent_of, reach.start_key, cluster->approach_key, config);
        std::vector<common::Position3D> waypoints;
        if (cluster->approach_key == reach.start_key) {
            waypoints.clear();
        } else {
            if (!raw.found) {
                continue;
            }
            waypoints = stringPullConstantAltitude(in.map, raw.path, in.drone.radius);
        }
        const std::size_t travel =
            waypoints.empty()
                ? 0
                : stepCostForPath(waypoints, in.state.position, in.state.heading, limits);
        if (travel + reserve > in.remaining_steps) {
            continue;
        }
        const double rate = static_cast<double>(cluster->cell_count) /
                            static_cast<double>(travel + reserve);
        if (rate > best_rate) {
            best_rate = rate;
            best.valid = true;
            best.waypoints = std::move(waypoints);
            best.terminal_scans.clear();
            best.expected_gain = 0.0;
            best.target_cluster_cells = cluster->cell_count;
            best.expected_rate = rate;
            best.target_keys = cluster->keys;
            best.frontier_cells = reach.frontier_cells;
        }
    }
    return best;
}
```

`WavefrontPlanner.h` keeps `kRankedClusters` as a private static constexpr; `kMaxSweepReserve` lives in the `.cpp` only.

Add `src/WavefrontPlanner.cpp` to both CMake targets and `tests/test_wavefront_planner.cpp` to `algorithm_test`. `ExplorationPlan.h` is a header, no CMake line.

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cmake --build build -j && ./build/Algorithm/algorithm_test --gtest_filter='WavefrontPlanner.*:NbvPlanner.*:MappingAlgorithm.*'`

Expected: PASS. If `PrefersDistantRoomOverNearbyCrumb` loses to the crumb, print both cluster `cell_count`s — the room box may be hanging off the map dims (`{21,21,21}` covers 0..200 cm). Shrink the box to `x=14..19` if `set` is going `OutOfBounds`.

- [ ] **Step 5: Propose the commit**

Stage: `Algorithm/src/ExplorationPlan.h`, `Algorithm/src/NbvPlanner.h`, `Algorithm/src/WavefrontPlanner.h`, `Algorithm/src/WavefrontPlanner.cpp`, `Algorithm/tests/test_wavefront_planner.cpp`, `Algorithm/CMakeLists.txt`.

Proposed message:

```
feat: rank frontier clusters by cells per step
```

Stop and wait for approval. Do not commit.

---

### Task 4: Score-aware scanning and the executor

This is the task that changes behaviour the drone actually flies. After it, D's NBV policy is no longer on the `nextStep` path.

**Files:**
- Create: `Algorithm/src/ScanPlanning.h`, `Algorithm/src/ScanPlanning.cpp`
- Create: `Algorithm/tests/test_scan_planning.cpp`
- Modify: `Algorithm/include/Algorithm/MappingAlgorithmImpl.h`
- Modify: `Algorithm/src/MappingAlgorithmImpl.cpp`
- Modify: `Algorithm/tests/test_mapping_algorithm.cpp`
- Modify: `Algorithm/CMakeLists.txt`

**Interfaces:**
- Consumes: `cone_template::{ConeTemplateCache, VoxelStamp, walkTemplate, nearFieldContainsSolid}`, `detail::{ExplorationPlan, WavefrontPlanner, WavefrontInputs, FrontierCells, GridKey, quantizePosition, hasAnyNotMappedInBounds}`.
- Produces:
  - `bool isGainMasked(const GridKey& key, const FrontierCells& frontier);` — `true` if `key` is in `frontier` or a 6-face neighbour is.
  - `std::vector<common::Orientation> buildSweepDirections(const IMap3D&, const Position3D& origin, const LidarConfigData&, const FrontierCells&, const std::vector<ConeTemplate>&, VoxelStamp&);`
  - `std::optional<common::Orientation> bestTravelScan(const IMap3D&, const DroneState& predicted, const Position3D& next_waypoint, const LidarConfigData&, const FrontierCells&, const std::vector<ConeTemplate>&, VoxelStamp&);`
  - `MappingAlgorithmImpl` uses `WavefrontPlanner`, stores `FrontierCells last_frontier_`, owns `ConeTemplateCache` + `VoxelStamp`, builds the sweep on arrival, terminates on `kLowRateReplans` consecutive sub-floor rates.

- [ ] **Step 1: Write the failing ScanPlanning tests**

Create `Algorithm/tests/test_scan_planning.cpp`:

```cpp
// test_scan_planning.cpp — frontier mask, sweep order, pass-2 gate.

#include "FakeMap3D.h"
#include "MappingAlgorithmFrontier.h"
#include "ScanPlanning.h"

#include <user_common_207190406_209543255/ConeTemplate.h>

#include <gtest/gtest.h>

namespace detail = algorithm_207190406_209543255::detail;
namespace ctpl = user_common_207190406_209543255::cone_template;
using Map = AlgorithmTest::FakeMap3D;

namespace {

using common::Orientation;
using common::Position3D;
using common::cm;
using common::deg;
using common::x_extent;
using common::y_extent;
using common::z_extent;
namespace ct = common::types;

[[nodiscard]] ct::MapConfig makeConfig() {
    ct::MapConfig config{};
    config.resolution = 10.0 * cm;
    config.offset = Position3D{};
    config.boundaries.min_x = 0.0 * x_extent[cm];
    config.boundaries.max_x = 100.0 * x_extent[cm];
    config.boundaries.min_y = 0.0 * y_extent[cm];
    config.boundaries.max_y = 100.0 * y_extent[cm];
    config.boundaries.min_height = 0.0 * z_extent[cm];
    config.boundaries.max_height = 100.0 * z_extent[cm];
    return config;
}

[[nodiscard]] ct::LidarConfigData makeLidar() {
    ct::LidarConfigData cfg{};
    cfg.z_min = 20.0 * cm;
    cfg.z_max = 80.0 * cm;
    cfg.d = 2.5 * cm;
    cfg.fov_circles = 4;
    return cfg;
}

[[nodiscard]] Position3D at(double x, double y, double z) {
    return Position3D{x * x_extent[cm], y * y_extent[cm], z * z_extent[cm]};
}

} // namespace

TEST(ScanPlanning, MaskRejectsUnmappedBehindOccupied) {
    const ct::MapConfig config = makeConfig();
    const detail::GridKey free{5, 5, 5};
    const detail::GridKey wall{6, 5, 5};
    const detail::GridKey behind{7, 5, 5};
    detail::FrontierCells frontier;
    frontier.insert(free);
    EXPECT_TRUE(detail::isGainMasked(free, frontier));
    EXPECT_TRUE(detail::isGainMasked(wall, frontier));  // face-adjacent
    EXPECT_FALSE(detail::isGainMasked(behind, frontier));
}

TEST(ScanPlanning, SweepOrdersByIndependentGainNotEnumerationOrder) {
    const ct::MapConfig config = makeConfig();
    Map map{{11, 11, 11}, config, ct::VoxelOccupancy::Empty};
    // Unmapped only along +Z, so +Z must rank first even though axes[+Z] is index 4.
    for (int z = 6; z <= 10; ++z) {
        map.set(at(50.0, 50.0, z * 10.0), ct::VoxelOccupancy::Unmapped);
    }
    const Position3D origin = at(50.0, 50.0, 50.0);
    map.set(origin, ct::VoxelOccupancy::Empty);

    const detail::MappingAlgorithmFrontier frontier;
    const auto reach = frontier.exploreReachable(
        map, origin, 4.0 * cm, {}, 1, detail::maxExpansionsForMap(map));

    ctpl::ConeTemplateCache cache;
    const auto& templates = cache.get(makeLidar(), config.resolution);
    ctpl::VoxelStamp stamp;
    const auto dirs = detail::buildSweepDirections(
        map, origin, makeLidar(), reach.frontier_cells, templates, stamp);

    ASSERT_FALSE(dirs.empty());
    EXPECT_NEAR(dirs.front().altitude.force_numerical_value_in(deg), 90.0, 1e-6);
}

TEST(ScanPlanning, MarginalPassDropsFullyClaimedDirections) {
    const ct::MapConfig config = makeConfig();
    Map map{{11, 11, 11}, config, ct::VoxelOccupancy::Empty};
    map.set(at(60.0, 50.0, 50.0), ct::VoxelOccupancy::Unmapped);
    const Position3D origin = at(50.0, 50.0, 50.0);

    const detail::MappingAlgorithmFrontier frontier;
    const auto reach = frontier.exploreReachable(
        map, origin, 4.0 * cm, {}, 1, detail::maxExpansionsForMap(map));

    ctpl::ConeTemplateCache cache;
    const auto& templates = cache.get(makeLidar(), config.resolution);
    ctpl::VoxelStamp stamp;
    const auto dirs = detail::buildSweepDirections(
        map, origin, makeLidar(), reach.frontier_cells, templates, stamp);

    EXPECT_FALSE(dirs.empty());
    EXPECT_LT(dirs.size(), templates.size());
}

TEST(ScanPlanning, TravelScanRejectsDirectionWithOccupiedNearField) {
    const ct::MapConfig config = makeConfig();
    Map map{{11, 11, 11}, config, ct::VoxelOccupancy::Empty};
    const Position3D origin = at(50.0, 50.0, 50.0);
    const Position3D ahead = at(60.0, 50.0, 50.0);
    map.set(ahead, ct::VoxelOccupancy::Occupied);

    detail::FrontierCells frontier;
    frontier.insert(detail::quantizePosition(origin, config));

    ctpl::ConeTemplateCache cache;
    const auto& templates = cache.get(makeLidar(), config.resolution);
    ctpl::VoxelStamp stamp;
    ct::DroneState predicted{origin, Orientation{0.0 * deg, 0.0 * deg}, 0};
    const auto scan = detail::bestTravelScan(
        map, predicted, at(80.0, 50.0, 50.0), makeLidar(), frontier, templates, stamp);

    // +X is blocked in the near field; ±Z have no masked Unmapped. Omit the scan.
    EXPECT_FALSE(scan.has_value());
}
```

- [ ] **Step 2: Run ScanPlanning tests to verify they fail**

Run: `cmake --build build -j && ./build/Algorithm/algorithm_test --gtest_filter='ScanPlanning.*'`

Expected: compile error — `ScanPlanning.h` does not exist.

- [ ] **Step 3: Implement ScanPlanning**

`Algorithm/src/ScanPlanning.h`:

```cpp
#pragma once

#include "MappingAlgorithmFrontier.h"

#include <user_common_207190406_209543255/ConeTemplate.h>

#include <Common/IMap3D.h>
#include <Common/types/DroneTypes.h>
#include <Common/types/LidarTypes.h>

#include <optional>
#include <vector>

namespace algorithm_207190406_209543255::detail {

[[nodiscard]] bool isGainMasked(const GridKey& key, const FrontierCells& frontier);

[[nodiscard]] std::vector<common::Orientation> buildSweepDirections(
    const common::IMap3D& map,
    const common::Position3D& origin,
    const common::types::LidarConfigData& lidar,
    const FrontierCells& frontier,
    const std::vector<user_common_207190406_209543255::cone_template::ConeTemplate>&
        templates,
    user_common_207190406_209543255::cone_template::VoxelStamp& stamp);

[[nodiscard]] std::optional<common::Orientation> bestTravelScan(
    const common::IMap3D& map,
    const common::types::DroneState& predicted,
    const common::Position3D& next_waypoint,
    const common::types::LidarConfigData& lidar,
    const FrontierCells& frontier,
    const std::vector<user_common_207190406_209543255::cone_template::ConeTemplate>&
        templates,
    user_common_207190406_209543255::cone_template::VoxelStamp& stamp);

[[nodiscard]] bool clusterStillFrontier(const common::IMap3D& map,
                                        const std::vector<GridKey>& keys);

} // namespace
```

`Algorithm/src/ScanPlanning.cpp` — implement:

`isGainMasked`: return true if `frontier.contains(key)` or any of the 6 face neighbours is in `frontier`.

`maskedGain` helper: `stamp.begin(...)`; `walkTemplate` and count only when `isGainMasked(quantizePosition(p, config), frontier)`.

`buildSweepDirections`:
1. For each template, `stamp.begin`, score `maskedGain` independently, keep `(gain, direction)` where `gain > 0`.
2. `stable_sort` by gain descending, then by `horizontal` then `altitude` numerical values for a deterministic tiebreak.
3. Second pass: `stamp.begin` once; walk each sorted direction sharing the stamp; keep the direction only if the walk adds `> 0` new masked voxels.

`bestTravelScan`:
1. Build three probe orientations: heading toward `next_waypoint` in the XY plane (`atan2(dy,dx)`, altitude 0), `+Z` (0, +90), `-Z` (0, -90). If `dx==dy==0`, skip the horizontal probe.
2. For each probe, pick the cached template with the smallest angular distance to that orientation (compare unit vectors).
3. Skip the template if `nearFieldContainsSolid`.
4. `stamp.begin`; `walkTemplate` with early-exit on the first masked Unmapped. Return the first probe that hits, converted to the drone frame (`world - predicted.heading`) by the **caller** — this function returns **world-frame** orientation so tests stay simple. Document that. `MappingAlgorithmImpl` subtracts heading, as it does today.

`clusterStillFrontier`: for each key, convert to a point and check any 6-face neighbour is `Unmapped`. Return true if any key still borders Unmapped.

Add both new sources to both CMake targets and the test to `algorithm_test`.

- [ ] **Step 4: Run ScanPlanning tests**

Run: `cmake --build build -j && ./build/Algorithm/algorithm_test --gtest_filter='ScanPlanning.*'`

Expected: PASS.

- [ ] **Step 5: Write the failing executor tests**

Append to `Algorithm/tests/test_mapping_algorithm.cpp`:

```cpp
TEST(MappingAlgorithm, FinishesAfterConsecutiveLowRateReplans) {
    const ct::MapConfig config = makeCorridorConfig();
    Map output_map{{11, 11, 11}, config, ct::VoxelOccupancy::Empty};
    // A single leftover Unmapped cell: cluster size 1, rate 1/8 = 0.125 < 0.25.
    output_map.set(gridPoint(5, 5, 5, config), ct::VoxelOccupancy::Unmapped);
    fillStartBubble(output_map, 1, 1, 1, config);

    const auto mc = makeMissionConfig();
    const auto lc = makeLidarConfig();
    const auto dc = makeDroneConfig();
    Impl algorithm{common::MappingAlgorithmDependencies{mc, lc, dc, output_map}};

    const ct::DroneState state{gridPoint(1, 1, 1, config), Orientation{0.0 * deg, 0.0 * deg}, 0};
    const ct::AlgorithmStatus status = runUntilTerminal(algorithm, state, 200);
    EXPECT_EQ(status, ct::AlgorithmStatus::FinishedWithUnmappableVoxels);
}

TEST(MappingAlgorithm, AbandonsPlanWhenTargetClusterIsResolved) {
    const ct::MapConfig config = makeCorridorConfig();
    Map output_map{{11, 11, 11}, config};
    fillEmptyBox(output_map, 0, 8, 0, 10, 0, 10, config);
    output_map.set(gridPoint(10, 5, 5, config), ct::VoxelOccupancy::Unmapped);

    auto lc = makeLidarConfig();
    lc.z_max = 40.0 * cm;
    Impl algorithm{common::MappingAlgorithmDependencies{
        makeMissionConfig(), lc, makeDroneConfig(), output_map}};

    const ct::DroneState state{gridPoint(2, 5, 5, config), Orientation{0.0 * deg, 0.0 * deg}, 0};
    const auto first = algorithm.nextStep(state, nullptr);
    ASSERT_EQ(first.status, ct::AlgorithmStatus::Working);

    // Resolve the only unknown. The stored cluster is no longer a frontier.
    output_map.set(gridPoint(10, 5, 5, config), ct::VoxelOccupancy::Empty);

    ct::AlgorithmStatus status = ct::AlgorithmStatus::Working;
    for (int i = 0; i < 30 && status == ct::AlgorithmStatus::Working; ++i) {
        status = algorithm.nextStep(state, nullptr).status;
    }
    EXPECT_NE(status, ct::AlgorithmStatus::Working);
}
```

Existing tests that must keep passing without edits: `EmitsMovementAndScanInTheSameCommand`, `DoesNotFinishWhileUnresolvedSpaceRemainsAndBudgetIsLarge`, `FinishesCleanlyWhenNothingIsUnmapped`, `TerminatesWhenBudgetIsExhausted`, `FirstStepRequestsScanWithNullLatestScan`.

- [ ] **Step 6: Rewrite the executor**

In `MappingAlgorithmImpl.h`:

- Change the class comment to Wavefront Frontier Detection over the reachability substrate.
- Replace `bestTravelScan` declaration with no public/private change if it stays as a method that forwards to `detail::bestTravelScan`.
- Add `void buildArrivalSweep(const types::DroneState& state);` and `bool targetClusterAlive() const;`.
- Constants:

```cpp
    static constexpr int kMaxMovingStallTicks = 2;
    static constexpr std::size_t kReplanIntervalSteps = 25;
    static constexpr std::size_t kBlockedTtlSteps = 50;
    static constexpr int kRecoveryAttempts = 3;
    static constexpr int kLowRateReplans = 3;
    static constexpr double kMinInformationRate = 0.25;
```

Delete `kTravelScanProbes` from this header (the probe set lives in `ScanPlanning`).

In `MappingAlgorithmImpl.cpp`:

- `#include "WavefrontPlanner.h"` and `#include "ScanPlanning.h"` and `ConeTemplate.h`. Drop `#include "NbvPlanner.h"`.
- `Impl` members:

```cpp
    detail::WavefrontPlanner planner{};
    detail::ExplorationPlan plan{};
    detail::FrontierCells last_frontier{};
    user_common_207190406_209543255::cone_template::ConeTemplateCache templates{};
    user_common_207190406_209543255::cone_template::VoxelStamp stamp{};
    std::vector<common::Orientation> arrival_scans{};
    std::size_t arrival_scan_index = 0;
    std::size_t waypoint_index = 0;
    std::size_t steps_since_replan = 0;
    bool has_plan = false;
    int moving_stall_ticks = 0;
    Position3D last_position{};
    bool has_last_position = false;
    detail::BlockedCells blocked_cells{};
    detail::GridIntMap blocked_since{};
    int recovery_attempts = 0;
    int low_rate_replans = 0;
    bool finished = false;
    bool planning_initialized = false;
```

- `replan` calls `impl_->planner.plan(WavefrontInputs{...})`, copies `plan.target_keys` / rate, stores `last_frontier` by running `exploreReachable` once more **is wasteful**. Do not. Have `WavefrontPlanner::plan` return the frontier set on the plan, or add `FrontierCells frontier_used` to `ExplorationPlan`.

`replan` copies `impl_->last_frontier = impl_->plan.frontier_cells` (Task 3 already puts the set on the plan).

- After a successful replan: if `expected_rate < kMinInformationRate` or `!valid`, increment `low_rate_replans`; else reset it to 0. Finish when `low_rate_replans >= kLowRateReplans` after the recovery attempt (same shape as today's recovery block).
- `targetClusterAlive`: `detail::clusterStillFrontier(output_map_, impl_->plan.target_keys)`.
- Replan when `plan_exhausted || interval_elapsed || (has_plan && !targetClusterAlive())`.
- `plan_exhausted` is: no plan, or (waypoints done **and** `arrival_scans` done). When waypoints become done, call `buildArrivalSweep` once (`arrival_scans = buildSweepDirections(...)`, `arrival_scan_index = 0`). When `arrival_scans` is exhausted, **rebuild once**; if the rebuild is empty, mark the plan exhausted so the next `nextStep` replans. Do not rebuild in a loop on the same call.
- Travel: `movementToward` unchanged; `scan_orientation = detail::bestTravelScan(...)` then convert to the drone frame (`world - predicted.heading`), omit when nullopt.
- Arrival: emit `arrival_scans[arrival_scan_index++]` converted to the drone frame. No movement.
- `predictPose`, `movementToward`, stall TTL, `remainingSteps` stay.

`buildArrivalSweep`:

```cpp
void MappingAlgorithmImpl_207190406_209543255::buildArrivalSweep(
    const types::DroneState& state) {
    const auto& templates =
        impl_->templates.get(lidar_config_, output_map_.getMapConfig().resolution);
    impl_->arrival_scans = detail::buildSweepDirections(
        output_map_, state.position, lidar_config_, impl_->last_frontier,
        templates, impl_->stamp);
    impl_->arrival_scan_index = 0;
}
```

`nextStep` skeleton (replace the current function body after stall / waypoint-advance):

```cpp
    const bool waypoints_done =
        impl_->has_plan && impl_->waypoint_index >= impl_->plan.waypoints.size();
    if (waypoints_done && impl_->arrival_scans.empty() &&
        impl_->arrival_scan_index == 0) {
        buildArrivalSweep(state);
    }

    const bool scans_done =
        impl_->arrival_scan_index >= impl_->arrival_scans.size();
    const bool plan_exhausted =
        !impl_->has_plan || (waypoints_done && scans_done);
    const bool interval_elapsed = impl_->steps_since_replan >= kReplanIntervalSteps;
    const bool cluster_dead =
        impl_->has_plan && !targetClusterAlive();

    if (plan_exhausted || interval_elapsed || cluster_dead) {
        const bool have = replan(state, false);
        const bool low = !have || impl_->plan.expected_rate < kMinInformationRate;
        if (low) {
            ++impl_->low_rate_replans;
            if (!have && impl_->recovery_attempts < kRecoveryAttempts &&
                replan(state, true)) {
                ++impl_->recovery_attempts;
                if (impl_->plan.expected_rate >= kMinInformationRate) {
                    impl_->low_rate_replans = 0;
                }
            } else if (impl_->low_rate_replans >= kLowRateReplans) {
                impl_->finished = true;
                types::MappingStepCommand cmd{};
                cmd.status = detail::hasAnyNotMappedInBounds(output_map_)
                                 ? types::AlgorithmStatus::FinishedWithUnmappableVoxels
                                 : types::AlgorithmStatus::Finished;
                return cmd;
            }
        } else {
            impl_->low_rate_replans = 0;
            impl_->recovery_attempts = 0;
        }
        impl_->arrival_scans.clear();
        impl_->arrival_scan_index = 0;
    }

    ++impl_->steps_since_replan;

    types::MappingStepCommand cmd{};
    cmd.status = types::AlgorithmStatus::Working;

    if (impl_->waypoint_index < impl_->plan.waypoints.size()) {
        const Position3D& target = impl_->plan.waypoints[impl_->waypoint_index];
        cmd.movement = movementToward(state, target);
        if (cmd.movement.has_value()) {
            const types::DroneState predicted = predictPose(state, *cmd.movement);
            const auto& templates = impl_->templates.get(
                lidar_config_, output_map_.getMapConfig().resolution);
            const std::optional<Orientation> world = detail::bestTravelScan(
                output_map_, predicted, target, lidar_config_, impl_->last_frontier,
                templates, impl_->stamp);
            if (world.has_value()) {
                cmd.scan_orientation =
                    Orientation{world->horizontal - predicted.heading.horizontal,
                                world->altitude - predicted.heading.altitude};
            }
        }
        return cmd;
    }

    if (impl_->arrival_scans.empty()) {
        buildArrivalSweep(state);
    }
    if (impl_->arrival_scan_index < impl_->arrival_scans.size()) {
        const Orientation& world = impl_->arrival_scans[impl_->arrival_scan_index++];
        cmd.scan_orientation = Orientation{world.horizontal - state.heading.horizontal,
                                           world.altitude - state.heading.altitude};
        return cmd;
    }

    impl_->has_plan = false;
    cmd.movement = types::MovementCommand{};
    return cmd;
```

`replan` must reset `waypoint_index`, `arrival_scans`, `arrival_scan_index`, `steps_since_replan`, and copy `impl_->last_frontier = impl_->plan.frontier_cells`.

- [ ] **Step 7: Run the executor tests**

Run: `cmake --build build -j && ./build/Algorithm/algorithm_test --gtest_filter='MappingAlgorithm.*:ScanPlanning.*:WavefrontPlanner.*'`

Expected: PASS. Existing `DoesNotFinishWhileUnresolvedSpaceRemainsAndBudgetIsLarge` must still hold — that map's unknown wall is a large cluster, rate well above 0.25.

If `FinishesAfterConsecutiveLowRateReplans` never finishes, the leftover cell is being treated as a high-rate cluster (start may be adjacent). Move the Unmapped cell to `gridPoint(9, 9, 9)` so travel + reserve keeps the rate below 0.25, or raise the isolation (no `fillStartBubble` next to it).

- [ ] **Step 8: Early `house_full` gate (do not skip)**

The spec's runtime estimate was derived from the code; D's was too and was wrong. Measure now, not in Task 6.

```bash
cmake -S . -B build/opt -DCMAKE_BUILD_TYPE=Release
cmake --build build/opt -j --target Algorithm_207190406_209543255 simulator_207190406_209543255 MissionControl_207190406_209543255
```

Use the existing single-cell compose if present (`tmp/profiling/compose_house_full.yaml`) or write one: `house_simulation` + `house_mission_full` + `drone_small` + `lidar_short`. Run the simulator under `timeout 60` (Docker) or a 60 s wall-clock cap. Record score / steps / status / wall seconds.

**Gate:** the process must exit before 60 s. If it does not, stop the plan and diagnose — do not continue to Task 5/6 on a binary that still misses the spec's success criterion 1.

- [ ] **Step 9: Propose the commit**

Stage: `Algorithm/src/ScanPlanning.h`, `Algorithm/src/ScanPlanning.cpp`, `Algorithm/tests/test_scan_planning.cpp`, `Algorithm/src/ExplorationPlan.h`, `Algorithm/src/WavefrontPlanner.cpp`, `Algorithm/include/Algorithm/MappingAlgorithmImpl.h`, `Algorithm/src/MappingAlgorithmImpl.cpp`, `Algorithm/tests/test_mapping_algorithm.cpp`, `Algorithm/CMakeLists.txt`.

Proposed message:

```
feat: execute wavefront plans with score-aware scans
```

Stop and wait for approval. Do not commit.

---

### Task 5: Delete NBV and the leftover candidate API

**Files:**
- Delete: `Algorithm/src/NbvPlanner.h`, `Algorithm/src/NbvPlanner.cpp`, `Algorithm/tests/test_nbv_planner.cpp`
- Modify: `Algorithm/src/MappingAlgorithmFrontier.h` — remove `struct ReachableCell`; remove `candidates` from `ReachabilityResult`; remove `stride_cells` from `exploreReachable`
- Modify: `Algorithm/src/MappingAlgorithmFrontier.cpp` — drop stride-bucket candidate construction; `exploreReachable` loses the `stride_cells` parameter
- Modify: `Algorithm/src/WavefrontPlanner.cpp` — call `exploreReachable` without a stride
- Modify: `Algorithm/tests/test_mapping_algorithm_frontier.cpp` — every `exploreReachable(..., stride, cap)` becomes `exploreReachable(..., cap)`; delete `ExploreReachableStrideDeduplicatesCandidates`; rewrite candidate loops to use `clusters` / `frontier_cells`
- Modify: `Algorithm/tests/test_scan_planning.cpp` and `Algorithm/tests/test_wavefront_planner.cpp` if they pass a stride
- Modify: `Algorithm/src/ExplorationPlan.h` — drop `terminal_scans` and `expected_gain`
- Modify: `Algorithm/CMakeLists.txt` — remove `NbvPlanner.cpp` and `test_nbv_planner.cpp`

**Interfaces:**
- Consumes: Task 2–4 types.
- Produces: `ReachabilityResult exploreReachable(const IMap3D&, const Position3D& start, PhysicalLength drone_radius, const BlockedCells&, std::size_t max_expansions) const;`

- [ ] **Step 1: Write the failing signature tests by updating call sites first**

Change the declaration and every call. Existing tests that iterate `result.candidates` become:

```cpp
TEST(MappingAlgorithm, ExploreReachableFindsFrontierAdjacentCandidates) {
    const ct::MapConfig config = makeCm10Config();
    Map map{{11, 11, 11}, config, ct::VoxelOccupancy::Empty};
    map.set(pointCm(50, 50, 50), ct::VoxelOccupancy::Unmapped);

    const detail::MappingAlgorithmFrontier frontier;
    const Position3D start = pointCm(0, 0, 0);
    const detail::ReachabilityResult result = frontier.exploreReachable(
        map, start, 4.0 * cm, {}, detail::maxExpansionsForMap(map));

    EXPECT_TRUE(result.start_passable);
    EXPECT_FALSE(result.truncated);
    ASSERT_FALSE(result.clusters.empty());
    EXPECT_FALSE(result.frontier_cells.empty());
    for (const detail::FrontierCluster& cluster : result.clusters) {
        EXPECT_GT(cluster.cell_count, 0u);
        EXPECT_GT(cluster.approach_cost, 0);
    }
}
```

Same rewrite for `FrontierFindsPathAlongEmptyCorridor`, `FrontierFindsFrontierInsideEmptyCube`, `FrontierFindExplorePathMovesTowardUnknown` — assert on `clusters` / `frontier_cells` / `approach_position` instead of `ReachableCell`.

`ExploreReachableRespectsExpansionCap` and `ExploreReachableTerminatesWithoutOccupancyBound` drop the stride argument (the `3` / `1` that was before the cap).

`ExploreReachableReportsStartPassabilityWithCapOfOne` and the C-era start-passable tests: `exploreReachable(map, pos, radius, {}, 1)` — cap is now the last argument.

Delete `TEST(MappingAlgorithm, ExploreReachableStrideDeduplicatesCandidates)` entirely.

Delete the three NBV files and their CMake lines.

- [ ] **Step 2: Run the build to verify NBV is gone and the new signature fails until the .cpp matches**

Run: `cmake --build build -j`

Expected: compile error in `MappingAlgorithmFrontier.cpp` (still takes `stride_cells`) and in `WavefrontPlanner.cpp` / leftover NBV includes, until Step 3.

- [ ] **Step 3: Apply the signature change and strip dead fields**

`exploreReachable` implementation: delete `stride`, `bucket_of`, and the `candidates.push_back` block. Keep frontier-list insertion and the clustering BFS.

`WavefrontPlanner::plan` call:

```cpp
    const ReachabilityResult reach = frontier_.exploreReachable(
        in.map, in.state.position, in.drone.radius, blocked,
        maxExpansionsForMap(in.map));
```

`ExplorationPlan`: remove `terminal_scans` and `expected_gain`. Grep the tree for both names and for `ReachableCell`, `kCandidateStrideCells`, `kScoredCandidates`, `NbvPlanner` — only docs and this plan may mention them.

- [ ] **Step 4: Run the full algorithm suite**

Run: `cmake --build build -j && ./build/Algorithm/algorithm_test`

Expected: all PASS. Zero tests named `NbvPlanner.*`.

Then: `ctest --test-dir build` (or the project's usual ctest invocation).

Expected: green. If a test-discovery line still lists `test_nbv_planner.cpp`, CMake cache is stale — reconfigure.

- [ ] **Step 5: Propose the commit**

Stage the deletions and the signature/test updates.

Proposed message:

```
refactor: remove NBV planner and strided candidate API
```

Stop and wait for approval. Do not commit.

---

### Task 6: Measure and document

**Files:**
- Create: `docs/benchmarks/2026-08-31-post_f_honest.csv`, `docs/benchmarks/2026-08-31-post_f_honest.md`
- Modify: `docs/mapping-algorithm-analysis.md`
- Modify: `docs/HLD.md`
- Modify: `docs/superpowers/specs/2026-08-29-mapping-algorithm-roadmap.md`
- Modify: `docs/mapping-algorithm-rewrite-pickup.md`
- Modify: `docs/known-issues.md` (issue #20, if the new step profile changes the note)
- Modify: `docs/superpowers/specs/2026-08-31-wavefront-frontier-exploration-design.md` — status already Accepted; add the measured `kMinInformationRate` if the sweep changes it

**Interfaces:**
- Consumes: `scripts/benchmark/run_benchmark.py`, `docs/benchmarks/ex2-reference.csv`, `docs/benchmarks/2026-08-29-post_c_honest.csv`.
- Produces: the post-F honest (and adversarial) CSVs plus the doc updates listed in the spec.

- [ ] **Step 1: Release build**

```bash
cmake -S . -B build/opt -DCMAKE_BUILD_TYPE=Release
cmake --build build/opt -j
```

Confirm `CMAKE_BUILD_TYPE` is `Release` (`-O2`). Do not measure a `build/` debug tree.

- [ ] **Step 2: Sweep `kMinInformationRate` on the six profiling cells**

Cells: `house_lower`, `small_room`, `large_room`, `small_out`, `house_full`, `large_out` (same compose files as the D investigation: `tmp/profiling/compose_*.yaml`). Values: `{0.10, 0.25, 0.50}`. 60 s cap per cell. Record score and wall time.

Keep `0.25` unless another value is strictly better on `small_room` (must reach ≥ 82.57) **and** does not give back `small_out`'s 83-point class improvement. If you change the constant, change only `kMinInformationRate` in `MappingAlgorithmImpl.h` and note the measured table in the spec.

- [ ] **Step 3: Honest 24-cell column**

From the repo root, with the venv at `scripts/benchmark/.venv`:

```bash
python scripts/benchmark/run_benchmark.py --build-dir build/opt --columns honest
```

(Use the flag names the script actually exposes — read `run_benchmark.py --help` if they differ.) Copy the labelled CSV to `docs/benchmarks/2026-08-31-post_f_honest.csv` and write `docs/benchmarks/2026-08-31-post_f_honest.md` with per-group band verdicts against ex2, plus totals vs post-C 1331.0 and lawnmower 646.4.

Success criteria to report, verbatim:

1. Every honest cell finishes within 60 s wall clock at `-O2`.
2. Honest score sum > 1331.0; `small_room` ≥ 82.57; every cell D finished is at least as good as D's number.
3. `small_out` stays in the 80s, not back at post-C's 25.
4. `house_full` moves up from 6.30 toward 56–62.
5. Zero `ERROR` in either column.

If criterion 1 fails, stop and diagnose. Do not write "done" docs over a timeout.

- [ ] **Step 4: Adversarial column**

Same harness, `--columns adversarial`. Confirm zero `ERROR`. Record the CSV next to the honest one if the script emits it; otherwise note the result in the markdown summary.

- [ ] **Step 5: Documentation updates**

`docs/mapping-algorithm-analysis.md`:
- In "What the assignment actually measures", add that `MapsComparison` pass 2 counts target-known voxels outside the spawn-reachable set and credits only `Empty` — resolving wall interiors / `PotentiallyOccupied` there lowers the score.
- In "Better known algorithms", mark item 3 (WFD as primary) as adopted by project F.

`docs/HLD.md` lines 89–92, replace the BFS/NBV sentence with:

```
  Uses Wavefront Frontier Detection over a reachability substrate
  (`MappingAlgorithmFrontier`) to pick a cluster, then emits movement plus a
  score-aware scan toward that cluster.
```

`docs/superpowers/specs/2026-08-29-mapping-algorithm-roadmap.md`:
- Title becomes A/B/C/D/F (E remains the deferred mp-units conversion).
- Add a Project F row pointing at this spec and plan.
- Record D's measured outcome honestly: `-O2` `house_full` / `large_out` killed at 120 s; `small_room` 76.37 vs post-C 82.57; `small_out` 83.49 vs 25.16. D's candidate-scoring half is superseded by F.

`docs/mapping-algorithm-rewrite-pickup.md`:
- Verdict: A/B/C done; D implemented and measured as a regression on runtime and `small_room`; F is the current work (or done, once this task lands).
- Next queue item: project E (mp-units on the substrate) and packaging.

`docs/known-issues.md` #20: update the step-profile note if the new travel+scan pattern changes the foreign-MC comparison.

- [ ] **Step 6: Propose the commit**

Stage the benchmark artefacts and the doc files. Do not stage `tmp/profiling/**`, venv, or `build/`.

Proposed message:

```
docs: record post-F wavefront benchmark against ex2
```

Stop and wait for approval. Do not commit.

---

## Self-review

**Spec coverage**

| Spec requirement | Task |
|------------------|------|
| Cone templates, exact voxel-set equivalence, `VoxelStamp`, `near_field_samples` | 1 |
| 6-face frontier flag, memoised `isSpherePassable`, clusters, start included | 2 |
| Rank top 8 by `cell_count / (travel + reserve)`, zero cones at replan, current pose wins when it is the cheapest member | 3 |
| Gain masked to frontier set; arrival-time independent-then-marginal sweep; pass-2 travel gate; `kReplanIntervalSteps = 25`; cluster invalidation; `kMinInformationRate` / `kLowRateReplans` | 4 |
| Delete `NbvPlanner`, `ReachableCell`, stride, `terminal_scans` | 5 |
| 60 s `-O2` gate, honest + adversarial columns, rate sweep, analysis/HLD/roadmap/pickup | 6 |
| ALG28 expansion bound | 2 (retained), 5 (signature only) |
| Co-emission | 4 (executor + existing test) |
| `Unmapped` soft cost, safety invariant, no RNG, mp-units, no `common/` edits | Global Constraints |

**Placeholders:** none. The only value left to measurement is `kMinInformationRate`, and Task 6 says how to choose it.

**Type consistency**

- `ExplorationPlan` is defined once in `ExplorationPlan.h`. Fields after Task 5: `waypoints`, `target_cluster_cells`, `expected_rate`, `target_keys`, `frontier_cells`, `valid`. Task 3 temporarily keeps `terminal_scans` / `expected_gain` for `NbvPlanner`; Task 5 deletes them.
- `WavefrontInputs` is an alias of `NbvInputs` until Task 5, after which `NbvInputs` can be renamed to `WavefrontInputs` in `ExplorationPlan.h` (do that rename in Task 5 if `NbvInputs` has no remaining callers).
- `exploreReachable(..., stride, cap)` through Task 4; `exploreReachable(..., cap)` from Task 5.
- `walkTemplate(cone, map, origin, stamp, on_unresolved)` and `isGainMasked(key, frontier)` keep the same signatures from the task that introduces them.
- `bestTravelScan` returns a **world-frame** orientation; the executor converts to the drone frame. Do not convert twice.

**Compile-safety:** Task 2 does not break `NbvPlanner`. Task 3 does not switch `nextStep`. Task 4 switches the executor. Task 5 deletes the old policy.

**Known test-fixture trap:** `FakeMap3D` `{21,21,21}` with 10 cm resolution covers 0..200 cm inclusive. A box that writes `x=21` is the last in-bounds cell (`210` cm is out). Keep `fillUnmappedBox` upper bounds at `20`.
