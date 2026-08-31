# Sensor Model + Clearance Fix Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fix the `isSpherePassable` / `sphereContainsNotMapped` sphere–grid intersection no-op, and replace the hardcoded 26-direction scan sweep with lidar-cone-derived orientations plus gain-gating against `output_map_`.

**Architecture:** Keep loop bounds as `ceil(radius/step)` so 1 cm test grids still work; change only the probe inclusion test to nearest-point-in-box vs sphere (physically correct). Put shared beam stepping in `UserCommon/` (`namespace user_common_207190406_209543255`). Add lidar cone helpers (half-angle from MockLidar geometry, Fibonacci sphere directions, cone-vs-map Unmapped walk). Wire `buildScanOrientations` + scanning-phase gain-gate. No new belief map. Re-measure `honest` harness column after.

**Tech Stack:** C++20, gtest, Docker `drone-mapper-ex3-dev`, `scripts/benchmark/run_benchmark.py`.

## Global Constraints

- Spec: `docs/superpowers/specs/2026-08-29-sensor-model-clearance-belief-design.md`
- **Never `git commit` without explicit human approval** in this chat (propose message; wait).
- **No new belief-map data structure**; operate on `output_map_` only.
- **No Algorithm policy rewrite** (frontier/Dijkstra/blacklists) — that is project D.
- Shared code namespace: `user_common_207190406_209543255`. MissionControl wrappers may thin-include UserCommon.
- Branch: continue `algorithm-benchmark-harness`.
- Authoritative cone half-angle (from `Simulator/src/MockLidar.cpp` / `HostLidar.cpp`):
  `α = atan2((fov_circles - 1) * d, z_min)` when `fov_circles ≥ 1` and `z_min > 0`; else treat as degenerate (no directions / no gain).

### Spec clarification (implementation must follow this, not a naïve fixed 3×3×3)

The design text suggested a fixed `dx,dy,dz ∈ {-1,0,1}` loop because shipped missions use 10 cm grids with radius ≤ 7.5 cm. **Existing Algorithm tests use 1 cm grids with radius 5 cm**, which need probes out to ~5 cells. **Keep** `rx = rh = ceil(radius_cm / step_cm)` for loop bounds; **only** replace the centre-distance gate `ox²+oy²+oz² > r²` with nearest-point-of-cell-box-to-origin ≤ `r`. That restores the shipped 10 cm / 7.5 cm face-neighbour case without breaking 1 cm tests.

For radius 4 cm on a 10 cm grid, face-neighbour nearest distance is 5 cm > 4 cm — still centre-only; that is correct physics, not a remaining bug.

---

### Task 1: Sphere–box clearance fix + frontier tests

**Files:**
- Modify: `Algorithm/src/MappingAlgorithmFrontier.cpp` (`isSpherePassable`, `sphereContainsNotMapped`)
- Modify: `Algorithm/tests/test_mapping_algorithm_frontier.cpp`

**Interfaces:**
- Consumes: existing `diagnose` / `hasNotMappedInSphere` public API
- Produces: corrected passability / unmapped-in-sphere geometry (no new public symbols required)

- [ ] **Step 1: Add a shared anonymous-namespace helper** used by both functions:

```cpp
// True iff the axis-aligned voxel box centered at (dx,dy,dz)*step_cm with
// half-extent step_cm/2 intersects the closed sphere of radius radius_cm at the origin.
[[nodiscard]] bool sphereIntersectsCellBox(int dx, int dy, int dz,
                                           double step_cm,
                                           double radius_cm) {
    if (dx == 0 && dy == 0 && dz == 0) {
        return true;
    }
    const double half = step_cm * 0.5;
    const double ox = static_cast<double>(dx) * step_cm;
    const double oy = static_cast<double>(dy) * step_cm;
    const double oz = static_cast<double>(dz) * step_cm;
    // Nearest point in [o-half, o+half] to 0 on each axis:
    const auto nearest1d = [half](double o) {
        if (0.0 < o - half) {
            return o - half;
        }
        if (0.0 > o + half) {
            return o + half;
        }
        return 0.0;
    };
    const double nx = nearest1d(ox);
    const double ny = nearest1d(oy);
    const double nz = nearest1d(oz);
    return (nx * nx + ny * ny + nz * nz) <= (radius_cm * radius_cm);
}
```

In both `isSpherePassable` and `sphereContainsNotMapped`, keep the existing `rx`/`rh` ceil loops; replace the `ox*ox+… > r*r` continue with `if (!sphereIntersectsCellBox(dx,dy,dz,step_cm,radius_cm)) continue;`. Keep computing probe at cell **centre** for `occupancyAt` (map is cell-valued).

- [ ] **Step 2: Write failing tests** (10 cm grid — the no-op case). Append to `test_mapping_algorithm_frontier.cpp`:

```cpp
[[nodiscard]] ct::MapConfig makeCm10Config() {
    ct::MapConfig config{};
    config.resolution = 10.0 * cm;
    config.offset = Position3D{};
    config.boundaries.min_x = -50.0 * x_extent[cm];
    config.boundaries.max_x = 50.0 * x_extent[cm];
    config.boundaries.min_y = -50.0 * y_extent[cm];
    config.boundaries.max_y = 50.0 * y_extent[cm];
    config.boundaries.min_height = 0.0 * z_extent[cm];
    config.boundaries.max_height = 100.0 * z_extent[cm];
    return config;
}

// radius 7.5, step 10: face neighbour box nearest=5 ≤ 7.5 → must reject Occupied neighbour
TEST(MappingAlgorithm, FrontierRejectsOccupiedFaceNeighbourOnCm10Grid) {
    const ct::MapConfig config = makeCm10Config();
    Map map{{11, 11, 11}, config}; // enough for ±50cm
    // Prefer sizing FakeMap3D to match bounds: use dims that cover indices.
    // Use same pattern as other tests — size {11,11,11} with offset 0 and bounds ±50 may be wrong.
    // Safer: fill via set at absolute cm positions; FakeMap3D dims must cover quantized indices.
}
```

Concrete failing test body (use map dims consistent with FakeMap3D — mirror how `test_mapping_algorithm.cpp` builds 10 cm maps; read `FakeMap3D.h` / existing corridor tests for sizing):

1. Centre at `(0,0,50)` Empty; neighbour at `(10,0,50)` Occupied; `diagnose(..., 7.5 * cm)` → `start_passable == false`.
2. Centre Empty; neighbour at `(10,0,50)` Occupied; `diagnose(..., 4.0 * cm)` → `start_passable == true` (nearest 5 > 4).
3. `hasNotMappedInSphere(map, centre, 7.5 * cm)` true when only the face neighbour is Unmapped and centre Empty (same geometry as passability).

Before the fix, test (1) fails (`start_passable` stays true). After, it passes.

- [ ] **Step 3: Implement helper + wire both functions.**

- [ ] **Step 4: Build + test in Docker**

```bash
docker run --rm -e VCPKG_ROOT=/usr/local/vcpkg -v "${PWD}:/work" -w /work drone-mapper-ex3-dev \
  bash -lc 'cmake --build --preset default -j$(nproc) --target algorithm_test && ./build/default/Algorithm/algorithm_test --gtest_filter=MappingAlgorithm.Frontier*'
```

Expected: all `MappingAlgorithm.Frontier*` PASS (including new tests). If FakeMap sizing is wrong, fix the test map dimensions first.

- [ ] **Step 5: Propose commit** (do not commit until approved)

```
fix: use sphere-vs-voxel-box clearance in frontier passability
```

---

### Task 2: Move beam math into `UserCommon/`

**Files:**
- Create: `UserCommon/include/user_common_207190406_209543255/BeamMath.h` (header-only OK, matching current `BeamMath.hpp`)
- Modify: `MissionControl/src/ScanResultToVoxels.cpp`, `MissionControl/src/DroneControlImpl.cpp` — include UserCommon header; drop local `BeamMath.hpp` **or** leave a one-line forwarding header that includes UserCommon (prefer delete private header once both compile).
- Modify: `MissionControl/CMakeLists.txt` — add `${CMAKE_SOURCE_DIR}/UserCommon/include` to MC plugin + `test_drone_control` + `test_mission_control` include dirs.
- Modify: `docs/component-placement.md` UserCommon table — add BeamMath row.

**Interfaces:**
- Produces: `user_common_207190406_209543255::beam_math::{isZeroDistance,isMissDistance,absoluteBeamOrientation,wrapDeg,normalizeOrientation,pointAlongBeam}` — same signatures as today's MissionControl helpers, namespace changed.
- Consumes: `common::Types` / Units (already available via `common::common`).

- [ ] **Step 1: Create UserCommon header** by copying `MissionControl/src/BeamMath.hpp` and renaming the namespace to `user_common_207190406_209543255::beam_math`.

- [ ] **Step 2: Update MC sources** to `#include <user_common_207190406_209543255/BeamMath.h>` and `namespace bm = user_common_207190406_209543255::beam_math;`. Delete `MissionControl/src/BeamMath.hpp`.

- [ ] **Step 3: CMake include dirs** for MissionControl targets as above.

- [ ] **Step 4: Docker build** `test_drone_control` + `test_mission_control`; run both. Expected PASS.

- [ ] **Step 5: Update `docs/component-placement.md`**; propose commit:

```
refactor: share BeamMath via UserCommon for Algorithm and MissionControl
```

---

### Task 3: Lidar cone helpers in `UserCommon/` + unit tests

**Files:**
- Create: `UserCommon/include/user_common_207190406_209543255/LidarCone.h` (header-only preferred)
- Create: `Algorithm/tests/test_lidar_cone.cpp` (or `UserCommon` tests hosted under Algorithm / MissionControl — put under `Algorithm/tests/` and add to `algorithm_test` for one binary)
- Modify: `Algorithm/CMakeLists.txt` — add UserCommon include dir; add `test_lidar_cone.cpp` to `algorithm_test` sources

**Interfaces:**

```cpp
namespace user_common_207190406_209543255::lidar_cone {

[[nodiscard]] inline double coneHalfAngleRad(const common::types::LidarConfigData& cfg);
// atan2((fov_circles-1)*d_cm, z_min_cm); returns 0 if fov_circles==0 or z_min<=0

[[nodiscard]] inline std::size_t directionCountForHalfAngle(double half_angle_rad,
                                                            double overlap = 0.85);
// spacing = 2 * half_angle * overlap; n ~= ceil(4π / spacing²), clamp to [6, 64]

[[nodiscard]] inline std::vector<common::Orientation>
fibonacciSphereOrientations(std::size_t count);
// Deterministic Fibonacci lattice → (azimuth, elevation) in degrees as Orientation
// relative to +X forward / world axes (caller subtracts drone heading like today)

[[nodiscard]] inline bool coneCoversUnresolved(
    const common::IMap3D& map,
    const common::Position3D& origin,
    const common::Orientation& drone_heading,
    const common::Orientation& relative_scan,
    const common::types::LidarConfigData& cfg);
// Sample MockLidar-equivalent beams (centre + rings 1..fov_circles-1) out to z_max
// with step 0.5*resolution; return true if any voxel along any beam is Unmapped
// (or OutOfBounds beyond map — treat as resolved / not gain). Stop a beam early on Occupied.

} // namespace
```

Half-angle for shipped configs (assert in tests):

| Config | Values | α |
|--------|--------|---|
| lidar_short | fov=4, d=2.5, z_min=20 | `atan2(7.5, 20) ≈ 0.3588 rad ≈ 20.56°` |
| lidar_long | fov=3, d=2.5, z_min=20 | `atan2(5.0, 20) ≈ 0.2450 rad ≈ 14.04°` |

Direction counts must **differ** between short and long (not both 26).

- [ ] **Step 1: Write failing unit tests** for half-angle (±1e-6), directionCountShort != directionCountLong, fibonacci size == requested count, determinism (two calls equal), and `coneCoversUnresolved` true/false on a tiny FakeMap (one Unmapped cell along +X beam vs all Empty).

- [ ] **Step 2: Implement `LidarCone.h`** using `beam_math::pointAlongBeam` / `absoluteBeamOrientation`. For ring sampling mirror `MockLidar.cpp` circle loop (polar via `atan2(circle*d, z_min)` style used by HostLidar, or MockLidar's offset angles — pick **HostLidar/MockLidar outer half-angle consistency**; prefer HostLidar's `atan2(circle*d, z_min)` polar for beam directions inside the cone).

- [ ] **Step 3: Build + run** `algorithm_test` filtered to new tests. PASS.

- [ ] **Step 4: Propose commit**

```
feat: add lidar cone geometry helpers in UserCommon
```

---

### Task 4: Wire Algorithm scan orientations + gain-gating

**Files:**
- Modify: `Algorithm/src/MappingAlgorithmImpl.cpp` — `buildScanOrientations`, `handleScanningPhase`
- Modify: `Algorithm/include/Algorithm/MappingAlgorithmImpl.h` — comment (no longer "26-direction")
- Modify: `Algorithm/CMakeLists.txt` — UserCommon include on shared lib + `algorithm_test` (if not already from Task 3)
- Modify: `Algorithm/tests/test_mapping_algorithm.cpp` — replace any hard assert of exactly 26 scans if present; add gain-gate / lidar-diff tests

**Interfaces:**
- Consumes: `lidar_cone::*`, `lidar_config_`
- Produces: scan list sized from lidar; scanning phase skips resolved cones

- [ ] **Step 1: Replace `buildScanOrientations` body:**

```cpp
void MappingAlgorithmImpl_...::buildScanOrientations(const Orientation& heading,
                                                     const Position3D& /*position*/) {
    impl_->scan_orientations.clear();
    const double alpha = user_common_207190406_209543255::lidar_cone::coneHalfAngleRad(lidar_config_);
    if (alpha <= 0.0) {
        return;
    }
    const std::size_t n =
        user_common_207190406_209543255::lidar_cone::directionCountForHalfAngle(alpha);
    const auto world = user_common_207190406_209543255::lidar_cone::fibonacciSphereOrientations(n);
    impl_->scan_orientations.reserve(world.size());
    for (const Orientation& abs_dir : world) {
        impl_->scan_orientations.push_back(Orientation{
            abs_dir.horizontal - heading.horizontal,
            abs_dir.altitude - heading.altitude,
        });
    }
}
```

- [ ] **Step 2: Gain-gate in `handleScanningPhase`:** before emitting `scan_orientations[scan_index]`, while index < size, if `!coneCoversUnresolved(output_map_, state.position, state.heading, orientation, lidar_config_)` then `++scan_index` and continue; if all skipped, clear and transition as today when sweep completes. If at least one remains, emit it.

- [ ] **Step 3: Tests**

1. Construct Impl with lidar_short vs lidar_long deps; drive Scanning until Planning (or count scan commands); assert short_count != long_count and neither equals the old hardcoded assumption blindly — both in a reasonable band e.g. `[6, 64]`.
2. Gain-gate: map fully Empty in a large box around spawn + fill all voxels Empty in bounds; after build, scanning phase should skip quickly / emit fewer scans than direction count (or zero then leave Scanning) — assert that with a fully-resolved map, number of emitted scan commands in one sweep `< directionCount` (ideally 0).
3. Partial Unmapped along one axis: at least one scan still emitted.

Grep existing tests for `26` / scan count assumptions and update.

- [ ] **Step 4: Full `algorithm_test` + `ctest` smoke in Docker.**

- [ ] **Step 5: Propose commit**

```
feat: derive scan directions from lidar cone and gain-gate
```

---

### Task 5: Docs + honest harness re-measure

**Files:**
- `docs/mapping-algorithm-analysis.md` — mark clearance + scan-geometry findings resolved; record α formula and note box-intersection fix + 3×3×3 clarification
- `docs/superpowers/specs/2026-08-29-mapping-algorithm-roadmap.md` — Project C → implemented; notes for D
- `docs/mapping-algorithm-rewrite-pickup.md` — C done; next = D
- `docs/superpowers/specs/2026-08-29-sensor-model-clearance-belief-design.md` — Status → Accepted
- Create: `docs/benchmarks/2026-08-29-post_c_honest.{csv,md}` (date stamp OK if different)

- [ ] **Step 1: Rebuild plugins; run harness**

```bash
docker run --rm -e VCPKG_ROOT=/usr/local/vcpkg -v "${PWD}:/work" -w /work drone-mapper-ex3-dev \
  bash -lc 'apt-get update -qq && apt-get install -y -qq python3-venv python3-pip >/dev/null
    cmake --build --preset default -j$(nproc)
    python3 -m venv scripts/benchmark/.venv
    scripts/benchmark/.venv/bin/pip install -q -r scripts/benchmark/requirements.txt
    scripts/benchmark/.venv/bin/python scripts/benchmark/run_benchmark.py \
      --columns honest --label post_c_honest --num-threads 8'
```

Compare totals to `docs/benchmarks/2026-08-29-post_b_honest.md`. Explain deltas in the new `.md` (steps may drop from gain-gating; clearance may add detours).

- [ ] **Step 2: Optionally `--columns adversarial` smoke** (or shorter subset if full sweep is long) — must not introduce hard Errors from illegal moves.

- [ ] **Step 3: Doc edits** per list above.

- [ ] **Step 4: Propose commit**

```
docs: record post-C honest benchmark and resolve sensor-model findings
```

---

## Spec coverage

| Requirement | Task |
|-------------|------|
| Sphere–box clearance fix (`isSpherePassable`) | 1 |
| Same fix `sphereContainsNotMapped` | 1 |
| Tests for 10 cm / 7.5 cm neighbour reject | 1 |
| Beam math in UserCommon | 2 |
| Cone half-angle from MockLidar geometry | 3 |
| Fibonacci directions; short ≠ long count | 3–4 |
| Gain-gating vs `output_map_` | 4 |
| No belief map | (all — omitted by design) |
| Docs + honest harness | 5 |

## Self-review notes

- Placeholder scan: none intentional; FakeMap sizing in Task 1 must be verified against `FakeMap3D.h` at implement time.
- Type consistency: all shared APIs under `user_common_207190406_209543255::{beam_math,lidar_cone}`.
- Spec's fixed 3×3×3 is **superseded** by the Global Constraints clarification — update analysis/spec status text in Task 5 to match.
