# AdvCpp rubric review (judgment, not a gate)

**Written to this file:** 2026-09-03.  
**Re-verify:** 2026-09-03 (four explore groups A–D against the live tree after Tasks 1–7 on `fix-advcpp-rubric-findings`, plus leftover-site cleanup in Task 8).  
**Score-parity follow-up:** 2026-09-03 (libm unwrap in movement/beam/sphere/scoring/planner hot paths so all 24 `sim_compose.yaml` scores match `main` `9374aea`; see `docs/benchmarks/2026-09-03-main-score-parity.md`).  
**How to read this:** `e01`–`e23` are grader judgment from `context/Error Code Key.xlsx` / `docs/review-error-codes.md`. A site here is “worth a second look,” not an automated FAIL. `b*` codes are runtime deductions and are **not** in this table.

Severity: **m** = minor, **n** = normal, **s** = severe (spreadsheet columns).

Do **not** put this markdown in the submission zip. If we will not fix a row before the deadline, file it via `populate-known-issues`.

Source files confirmed before the pass: `docs/review-error-codes.md`, `context/AdvCpp Review Guideline.docx`, `context/Error Code Key.xlsx`.

---

## Re-verify 2026-09-03

**Versus the 2026-09-02 combined table:** none of those sites remain. The old 100+ line functions, static-only I/O/orchestrator classes, string `"blocked"` / `"boundary"` handlers, raw `dlopen` `void*`, UserCommon→Simulator include, `step_cm` / `walkBeam(double)` APIs, and the missing HLD CLI / YAML / report / DI nodes are gone. Line numbers that drifted did so because the finding itself was removed, not because the same smell moved.

Task 8 also cleared four leftovers the 2026-09-02 table still named after Tasks 1–7:

- `e22` `ConeTemplate.h` public `BeamRun` / `ConeTemplate` aliases (dropped; callers use `detail::`).
- `e22` `ExplorationPlan` `target_keys` / `frontier_cells` (moved into `plan_detail::Internals`).
- `e21` `nextStep` progress path walking the volume again in `finishIfUnmapped` (reuses the cached unmapped count).
- `e08` no-op `ensurePlanningReady` / `planning_initialized`.

Cheap new nit also dropped: `e17` `BeamMath.h` umbrella `<Common/Types.h>` → `<Common/Units.h>`.

**e07 / e11:** no obvious violations.

Rows below are **new** nits this pass named. They are not the 2026-09-02 combined-table sites. Not chasing a second plan for them.

`HLD.pdf` is at the repo root (**205,402** bytes).

### Score / libm parity (accepted e03 risk)

Full quantity-path trig (`si::cos` / `si::sin` + `quantity_cast` multiplies) changed outdoor float order vs `main` and broke score parity on long missions. We restored **unwrap → libm → rewrap** inside hot `.cpp` files while keeping strong-typed APIs and stored fields (e16). That is an intentional e03 soft risk documented in `.cursor/rules/mp-units-strong-types.mdc` (“Score / libm parity”). Do **not** “fix” those sites back to quantity-path math without re-running the 24-cell matrix.

**Intentional unwrap sites (keep):** `MockMovement` advance/elevate/rotate limits+trig, `MockGPS` snap, `BeamMath::pointAlongBeam` dir×distance, `forEachSphereSample` cm loop, `MapsComparison` quantize/compare scalars, Algorithm planner `atan2`/distance unwraps in `MappingAlgorithmImpl` / `PathShaping` / `ScanPlanning` / `WavefrontPlanner` / Frontier helpers.

**Still open e03 (not required for score parity):** LidarCone / ConeTemplate / `normalizeOrientation` / MockLidar local normalize / factory resolution factor — quantity cleanup remains desirable there.

Clean fair timing 2026-09-03: branch vs `main` **24/24 scores identical**; walls ~176s vs ~171s (noise). Details: `docs/benchmarks/2026-09-03-main-score-parity.md`.

---

## Combined findings table

| Code | File : Line | Sev | Finding | Suggested fix |
|------|-------------|-----|---------|---------------|
| e01 | *(our headers)* | — | No obvious violations. | — |
| e02 | *(our headers)* | — | No obvious violations. I/O is three headers (`SimulatorPaths`, `SimulatorReports`, `YamlConfigParsers`). | — |
| e03 | `Algorithm/src/MappingAlgorithmImpl.cpp` | s | `movementToward` unwraps for `std::atan2` (score/libm parity). | **Accepted** — see Score / libm parity. |
| e03 | `Algorithm/src/PathShaping.cpp` | s | `stepCostForPath` unwraps advance/elevate/rotate + `atan2` (parity). | **Accepted** — see Score / libm parity. |
| e03 | `Algorithm/src/ScanPlanning.cpp` | s | Travel/angular helpers unwrap planar/`clamp` math (parity). | **Accepted** — see Score / libm parity. |
| e03 | `Simulator/src/MockMovement.cpp` | s | Advance uses libm `cos`/`sin`(deg·π/180); limits via unwrapped cm/deg. | **Accepted** — see Score / libm parity. |
| e03 | `Simulator/src/MockGPS.cpp` | s | Snap rounds in cm `double` then rewraps axes. | **Accepted** — see Score / libm parity. |
| e03 | `UserCommon/src/BeamMath.cpp` | s | `pointAlongBeam` multiplies dir×distance in cm doubles; `normalizeOrientation` still unwraps into `wrapDeg(double)`. | Beam step: **Accepted** (parity). Wrap: prefer typed angles if cheap. |
| e03 | `UserCommon/.../SimulationCoordUtil.h` | s | Sphere sample loop unwraps cm for index/`r2` then rewraps `Position3D`. | **Accepted** — see Score / libm parity. |
| e03 | `Simulator/src/MapsComparison.cpp` | s | Quantize/compare use cm doubles (parity with main scorer). | **Accepted** — see Score / libm parity. |
| e03 | `UserCommon/include/.../LidarCone.h` | s | `forEachConeBeam` unwraps lengths/angles, then cone basis/trig in `double`. | Same construction with `si::sin` / `si::cos` / `si::atan2`. |
| e03 | `UserCommon/src/LidarCone.cpp` | s | `coneHalfAngleRad` / fibonacci sphere use unwrapped trig. | `si::atan2` / quantity angles; reuse `beam_math::normalizeOrientation`. |
| e03 | `Simulator/src/MockLidar.cpp` | s | Local `normalize_orientation` duplicates unwrap. | Call `beam_math::normalizeOrientation`. |
| e03 | `UserCommon/src/ConeTemplate.cpp` | s | `build` unwraps `pointAlongBeam(..., 1.0 * cm)` into `BeamRun` doubles. | Store `PhysicalLength` components. |
| e03 | `Simulator/src/SimulationRunFactoryImpl.cpp` | s | Output resolution is `(map_resolution.force_numerical_value_in(cm) / factor) * cm`. | `sim.map_resolution / factor`. |
| e04 | `UserCommon/src/ConeTemplate.cpp:89` | n | Cache hit test is `==` on unwrapped `double` keys. | Compare `PhysicalLength` members. |
| e04 | `Algorithm/src/ScanPlanning.cpp:44` | n | Neighbor table is a C array `kFaceOffsets[6]`. | `constexpr std::array`. |
| e04 | `Algorithm/src/MappingAlgorithmFrontier.cpp:49` | n | Same C-array 6-neighbor table. | `std::array`. |
| e04 | `Algorithm/src/WavefrontPlanner.cpp:65` | n | Horizontal offsets are `const Position3D offsets[4]`. | `std::array<Position3D, 4>`. |
| e04 | `UserCommon/src/LidarCone.cpp:51` | n | Axis orientations are `const Orientation axes[]`. | `std::array<Orientation, 6>`. |
| e05 | *(shippable sources)* | — | No obvious violations. Handlers take `ErrorRef`; `"blocked"` / `"boundary"` string matching is gone. | — |
| e06 | `MissionControl/include/MissionControl/DroneControlImpl.h:19` | n | `ILidar&` / `IGPS&` are mutable; `step()` only calls `const` `scan` / `position` / `heading`. | `const common::ILidar&` and `const common::IGPS&`. |
| e06 | `UserCommon/include/user_common_207190406_209543255/SimulationCoordUtil.h:13` | m | `worldInitialDronePosition` takes `Position3D` by value, then mutates `z`. | `const Position3D&` and return a copy with adjusted `z`. |
| e06 | `UserCommon/include/user_common_207190406_209543255/BeamMath.h:20` | m | `normalizeOrientation` takes `Orientation` by value and only reads it. | `const Orientation&`. |
| e07 | *(our headers)* | — | No obvious violations. Query methods that can be `const` already are. | — |
| e08 | `MissionControl/include/MissionControl/DroneControlImpl.h:17` | n | Ctor takes `MissionConfigData` and stores `mission_`; the `.cpp` never reads it. | Drop the param and the member. |
| e08 | `Algorithm/include/Algorithm/MappingAlgorithmImpl.h:51` | n | `predictPose` / `samePosition` / `reachedWaypoint` use no instance state. | Move to an anonymous namespace in the `.cpp`. |
| e08 | `MissionControl/include/MissionControl/MissionControlImpl.h:25` | m | `mission_` copies the full config; `runMission` only uses `max_steps`. | Store `std::size_t max_steps_`. |
| e09 | `Simulator/src/MapsComparison.cpp:171` | n | `compareMaps` body ~80 lines; two full-grid passes plus reachability. | Extract pass-1 / pass-2 visitors. |
| e09 | `Simulator/src/io/SimulationCli.cpp:148` | n | `addShapeErrors` body ~75 lines. | One helper per check class. |
| e09 | `Simulator/src/SimulationRunImpl.cpp:75` | n | `SimulationRunImpl::run` body ~75 lines. | Split contain-mission / save-map / score. |
| e09 | `Algorithm/src/MappingAlgorithmFrontier.cpp:278` | n | `clusterFrontierCells` body ~73 lines. | Split cluster-walk from surface-approach. |
| e09 | `Simulator/src/SimulationRunFactoryImpl.cpp:125` | n | `create` body ~71 lines. | Extract map-pair build and plugin wiring. |
| e09 | `Simulator/src/RunMatrixOrchestrator.cpp:75` | n | `runPluginMatrix` body ~65 lines. | Extract per-cell runner. |
| e09 | `Algorithm/src/WavefrontPlanner.cpp:115` | n | `buildCandidatePlans` body ~64 lines. | Extract stay-and-drop vs travel candidate. |
| e09 | `Algorithm/src/MappingAlgorithmFrontier.cpp:206` | n | `runBoundedSearch` body ~62 lines. | Extract neighbor expand / frontier mark. |
| e09 | `Algorithm/src/MappingAlgorithmFrontier.cpp:434` | n | `findPathTo` body ~58 lines; unused by the production plan path. | Delete or route through `runBoundedSearch`. |
| e09 | `Algorithm/src/MappingAlgorithmImpl.cpp:369` | n | `nextStep` body ~55 lines after the Task 4 split. | Optional: peel stall / waypoint advance. |
| e09 | `Simulator/src/io/CompositionYamlParser.cpp:81` | n | `parseCompositionFile` body ~54 lines. | Extract drone/lidar list load. |
| e09 | `Simulator/src/io/ComparativeReportWriter.cpp:25` | n | `writeComparativeReport` body ~54 lines. | Share YAML dump/errors with competitive writer. |
| e09 | `Algorithm/src/WavefrontPlanner.cpp:208` | n | `WavefrontPlanner::plan` body ~53 lines. | Extract unstick / forced-descend. |
| e09 | `Simulator/src/main.cpp:155` | n | `main` body ~53 lines. | Extract output-dir + composition bootstrap. |
| e09 | `Algorithm/src/ScanPlanning.cpp:253` | n | `bestTravelScan` body ~50 lines. | Extract probe-list build from cone test. |
| e10 | `Simulator/src/PluginLoader.cpp:84` | n | `loadOneAlgorithm` / `loadOneMissionControl` are the same flow. | One templated/callback loader. |
| e10 | `Simulator/src/MockLidar.cpp:40` | n | `normalize_orientation` / `wrap_deg` copy `beam_math::normalizeOrientation` + `BeamMath.cpp` `wrapDeg`; `PathShaping.cpp` has a third `wrapDeg`. | Call `beam_math::normalizeOrientation`; one wrap helper. |
| e10 | `Algorithm/src/ScanPlanning.cpp:55` | n | `keyToPoint` copies `MappingAlgorithmFrontier.cpp:54`. | One helper next to `quantizePosition`. |
| e10 | `Simulator/src/Map3DImpl.cpp:32` | n | `isUnsetBoundaries` duplicated in `SimulationRunFactoryImpl.cpp:39`. | One shared Simulator helper. |
| e10 | `Simulator/src/MapsComparison.cpp:34` | n | `GridKeyHash` uses the same mix constants as `MappingAlgorithmFrontier.h:26`. | Share the hash (or the constants). |
| e10 | `Algorithm/src/MappingAlgorithmFrontier.cpp:434` | n | `findPathTo` repeats the Dijkstra in `runBoundedSearch`; no production caller in `Algorithm/src`. | Route through `runBoundedSearch` or drop. |
| e10 | `UserCommon/src/LidarCone.cpp:110` | n | Shippable `countUnresolvedVoxels` / `coneCoversUnresolved` / `walkBeam` parallel `walkTemplate`. | Keep one walk API in UserCommon. |
| e10 | `Simulator/src/MockLidar.cpp:74` | n | `scan` rebuilds the fov_circles / 4^i cone that `forEachConeBeam` already generates. | Drive MockLidar rings through `forEachConeBeam`. |
| e10 | `Simulator/src/main.cpp:30` | n | `loadPlugins` comparative vs competition factory-binding loops match. | One bind-loop over the varying folder. |
| e10 | `Simulator/src/io/SimulationCli.cpp:226` | n | `applyFilesystemChecks` comparative vs competition file/folder checks are copies. | Shared `requireOpenableFile` / `requireSoFolder`. |
| e11 | `vcpkg.json` + root `CMakeLists.txt` | — | No obvious violations. `find_package(... CONFIG REQUIRED)`; deps in `vcpkg.json`. | — |
| e13 | `Simulator/include/Simulator/RunMatrixTypes.h:18` | n | `MatrixCell` holds four nullable `const T*` into composition vectors. | `std::reference_wrapper` or store indices. |
| e13 | `Algorithm/src/WavefrontPlanner.h:18` | n | `plan(..., std::vector<ExplorationPlan>* alternates = nullptr)` is a raw optional out-parameter. | `std::optional<std::reference_wrapper<...>>` or return `{best, alternates}`. |
| e14 | `docs/hld/class-overview.mmd:86` | n | `RunErrorLog` is a node; shipped `IRunErrorLog` and `RunErrorLog ..|> IRunErrorLog` are absent. | Add `IRunErrorLog` and the inheritance/use edges. |
| e14 | `docs/hld/class-overview.mmd:82` | n | `MatrixCell` is shown; sibling PODs `PluginMatrixBinding` / `PluginMatrixResult` are not. | Add both next to `MatrixCell`. |
| e14 | `docs/hld/class-overview.mmd:20` | n | `YamlConfigParsers` has no `ConfigParseResult`. | Add `ConfigParseResult` and parsers → result / `IRunErrorLog`. |
| e15 | `docs/hld/seq-comparative-cell.mmd:22` | n | Loop shows `Main->>Dist: distributeWork` per cell. Live `runPluginMatrix` expands once and calls `distributeWork` once. | One `expand` then one `distributeWork`; workers call `create`/`run`. |
| e15 | `docs/hld/seq-comparative-cell.mmd:27` | n | Comparative cell skips per-run `errorLogPathFromOutputMap` + `RunErrorLog` before `runMission`. | Insert those calls at the start of `run()`. |
| e15 | `docs/hld/seq-drone-step.mmd:16` | n | Recoverable-throw alt says “Continue (no scan write)”. Live `step()` still scans after `Continue`. | Scan after movement unless `Error`; drop “no scan write”. |
| e15 | `docs/hld/seq-drone-step.mmd:12` | n | Step omits `markDroneFootprintEmpty` / `forEachSphereSample` before `nextStep`. | Show footprint carve, then GPS/`nextStep`. |
| e16 | `UserCommon/include/user_common_207190406_209543255/ConeTemplate.h:29` | n | `detail::BeamRun` stores unit direction as `double ux, uy, uz`. | `PhysicalLength` components so `ux * dist` stays typed. |
| e16 | `UserCommon/include/user_common_207190406_209543255/ConeTemplate.h:82` | n | `ConeTemplateCache` identity is `res_cm_`, `z_min_`, `z_max_`, `d_` as `double`. | Store `PhysicalLength` and compare those. |
| e16 | `UserCommon/include/user_common_207190406_209543255/LidarCone.h:30` | n | `coneHalfAngleRad` / `directionCountForHalfAngle` are raw-`double` radian APIs. | Return/take an mp-units angle (or keep radians only inside `.cpp`). |
| e17 | `Simulator/include/Simulator/RunMatrixTypes.h:3` | n | Includes `ISimulationRunFactory.h` for `reference_wrapper<ISimulationRunFactory>`. | Forward-declare `ISimulationRunFactory`. |
| e17 | `Simulator/include/Simulator/SimulationRunImpl.h:5` | n | Six complete `I*` headers so the implicit dtor can destroy `unique_ptr` members. | Declare the dtor in the header; define it in the `.cpp`; forward-declare. |
| e17 | `MissionControl/include/MissionControl/DroneControlImpl.h:5` | n | Five complete interface headers for reference members. | Forward-declare those five; include in the `.cpp`. |
| e17 | `Simulator/include/Simulator/MockLidar.h:3` | m | `IGPS` / `IMap3D` are only `const` reference members. | Forward-declare `IGPS` and `IMap3D`. |
| e17 | `Simulator/include/Simulator/MockMovement.h:4` | m | `IMap3D` is only a `const` reference member. | Forward-declare `IMap3D`. |
| e17 | `UserCommon/include/user_common_207190406_209543255/RunErrorLog.h:5` | m | `<Common/types/MissionTypes.h>` is only needed for `const ErrorRef&` on `log()`. | Drop it; `ErrorRef` can stay incomplete. |
| e17 | `Simulator/include/Simulator/io/YamlConfigParsers.h:8` | m | `<Common/types/MissionTypes.h>` is already pulled by `ConfigParseResult.h`. | Remove the redundant include. |
| e21 | `Algorithm/src/ScanPlanning.cpp:218` | n | `buildSweepDirections` walks every cone in `scoreTemplate` then walks again for the incremental stamp. | Score and keep in one walk, or reuse first-pass gain. Intentional two-phase filter — not cheap to merge. |
| e21 | `Algorithm/src/PathShaping.cpp:65` | n | String-pull calls `hasClearLineOfSight(anchor, probe)` for each farther probe, rewalking the prefix. | Check only the new segment, or cache prefix clearance. |
| e21 | `MissionControl/src/ScanResultToVoxels.cpp:116` | n | `applyScanToMap` traces every beam, then `supplementGridAlignedFusion` traces again. | One walk that marks both sample sets. |
| e21 | `Algorithm/src/WavefrontPlanner.cpp:244` | n | `plan` calls `unmappedInColumnBelow`; `buildCandidatePlans` may call it again for the start cell. | Compute once and pass in. |
| e21 | `Simulator/src/MapsComparison.cpp:190` | n | Pass 1 and pass 2 both `forEachVoxelIndex` the same grid and recompute `voxelCenter`. | Single walk with both tallies. |
| e21 | `Algorithm/src/MappingAlgorithmFrontier.cpp:484` | n | `findPathTo` has no passable memo; `runBoundedSearch` already memos `isSpherePassable`. | Same memo if `findPathTo` stays. |
| e22 | `UserCommon/include/user_common_207190406_209543255/ConeTemplate.h:72` | n | `ConeTemplateCache::get` returns `const std::vector<detail::ConeTemplate>&` to the cache. | Span, or a lookup-by-orientation API. |
| e22 | `Algorithm/src/WavefrontPlanner.h:21` | m | Class declares unused `kRankedClusters`; `rankClusters` uses a second local `8`. | One private constant; use it from the `.cpp`. |
| e23 | `Simulator/src/io/DroneConfigYamlParser.cpp:15` | n | Diameter→radius is `*v / 2.0`. | Named half-dimension constant (or `*v / 2`). |
| e23 | `Algorithm/src/ScanPlanning.cpp:279` | n | Travel probes use bare `90.0 * deg` / `-90.0 * deg`. | `kZenith` / `kNadir`. |
| e23 | `MissionControl/src/ScanResultToVoxels.cpp:20` | n | Occupancy ranks are bare `3`/`2`/`1`/`0`. | Named priority constants. |
| e23 | `UserCommon/src/ConeTemplate.cpp:23` | n | Stamp radius padding is `ceil(range/step) + 2.0`. | `kStampRadiusPad`. |
| e23 | `Algorithm/src/ScanPlanning.cpp:94` | n | Ceiling test uses bare `1e-6 * z_extent[cm]` (also `WavefrontPlanner.cpp:51`, `:152`). | Shared `kHeightEpsilon`. |
| e23 | `Algorithm/src/MappingAlgorithmFrontier.cpp:90` | n | Half-voxel is `step * 0.5` (again at `:563`). | `kHalfVoxelFactor` or `step / 2`. |
| e23 | `Algorithm/src/WavefrontPlanner.cpp:231` | n | Escape/forced plans set `expected_rate = 1.0` (again `:253`). | `kForcedEscapeRate`. |

**No obvious violations reported:** e01, e02, e05, e07, e11. No production `new`/`delete`/`malloc`/`free` (only `= delete` on special members). Frozen `common/` / `common_*` headers were not scored for e01/e08 design.

No function in scope is still 100+ body lines. Largest remaining: `compareMaps` ~80, `addShapeErrors` ~75, `SimulationRunImpl::run` ~75, `clusterFrontierCells` ~73.

---

## Appendix — HLD (e14 / e15)

`HLD.pdf` **is** at the repo root (**205,402** bytes, dated 2026-09-03 after Task 7).

**Task 7 nodes confirmed present:** `parseSimulationCliArgs` / `SimulationCliArgs`, YAML parsers, report writers, `createOutputDir` / `errorLogPathFromOutputMap`, `applyScanToMap`, DI structs, factory aliases, registration structs, `MatrixCell` / loaded-plugin PODs, `RunErrorLog`, `SimulationCoordUtil`, `WavefrontPlanner` / `MappingAlgorithmFrontier`, `ConeTemplateCache` / `VoxelStamp`, `runPluginMatrix(bindings, composition, output_root, num_threads)`, expand inside that call, save map before `compareMaps`, `applyScanToMap` in the drone-step diagram.

**In code, still absent from the class diagram (supporting types, not the 2026-09-02 core-class gap):** `IRunErrorLog`, `ConfigParseResult`, `PluginMatrixBinding`, `PluginMatrixResult`. Weaker internals not drawn: `PluginLoadOutcome`, `DlCloser`, `SimulationCliParseResult`, report-input PODs, `countRunMatrixCells` / `appendLoadErrors`, `ExplorationPlan` / `PathShaping` / `ScanPlanning`, `LidarCone` / `BeamMath`.

**Comparative sequence vs `main.cpp` / `SimulationRunImpl`:** CLI → output dir → composition parse → plugin load → one `runPluginMatrix` (one expand, one `distributeWork`) → per-slot `create` / `run` (per-run `RunErrorLog`, `runMission`, save, `worldInitialDronePosition`, `compareMaps`) → YAML writers. Diagram still shows `distributeWork` inside the per-cell loop and skips the per-run error-log path.

**Drone-step vs `DroneControlImpl::step`:** live order is footprint carve → `nextStep` → movement → `applyScanIfRequested` even after recoverable `Continue`. Diagram omits the carve and still says “no scan write” on the throw alt (`docs/HLD.md` mirrors that).

README vs CMake: presets, `simulator_207190406_209543255`, both `.so` names, and `ctest --test-dir build/default` match.

---

## Guideline extras (not individual `e*` rows)

- Helpers that could live in anonymous namespaces: `samePosition` / `predictPose` / `reachedWaypoint`; `lidar_cone::detail::beamsOnCircle` in a public header (`MockLidar.cpp` duplicates it).
- `MappingAlgorithmFrontier` is a stateless class — methods could be free functions in the `.cpp`.
- Happy-flow / invalid-config / runtime (`RG1`–`RG5`, `b03`–`b08`) were **not** re-executed in this review; that is `run_all.sh` / cell-runtime.

---

## After we edit

Re-run unit tests for the touched project (`ctest` filters). Do **not** treat a full Debug `run_all.sh` as the inner loop. If Algorithm/`nextStep` changes, re-time cells with `verify-cell-runtime` (Release serial) so we do not regress the 24/24 COMPLETED + ~60 s bar. Then re-scan this file’s line numbers.
