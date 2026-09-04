// SimulationRunFactoryImpl.cpp
// Builds the full per-run dependency graph and returns a SimulationRunImpl.
//
// Key ex2 fix that must NOT regress:
//   Mission boundaries in YAML are in local (pre-offset) map coordinates.
//   outputMapConfig() adds simulation.map_offset to every bound so the output
//   map's world-space coordinates align with the drone's GPS position reports.
//   Without this shift the house scenario (height_offset: 150) writes all scan
//   results outside the output map and the score collapses to near-zero.

#include <Simulator/SimulationRunFactoryImpl.h>

#include "Map3DNpy.h"
#include <Simulator/MockGPS.h>
#include <Simulator/MockLidar.h>
#include <Simulator/MockMovement.h>
#include <Simulator/SimulationRunImpl.h>

#include <user_common_207190406_209543255/RunErrorLog.h>
#include <user_common_207190406_209543255/SimulationCoordUtil.h>

#include <TinyNPY.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <utility>

namespace simulator {

namespace {

using namespace common;
using namespace common::types;
namespace UC = user_common_207190406_209543255;

[[nodiscard]] bool isUnsetBoundaries(const MappingBounds& bounds) {
    return bounds.min_x == 0.0 * x_extent[cm] && bounds.max_x == 0.0 * x_extent[cm] &&
           bounds.min_y == 0.0 * y_extent[cm] && bounds.max_y == 0.0 * y_extent[cm] &&
           bounds.min_height == 0.0 * z_extent[cm] && bounds.max_height == 0.0 * z_extent[cm];
}

[[nodiscard]] MapConfig hiddenMapConfig(const simulator::types::SimulationConfigData& sim) {
    return MapConfig{
        .boundaries = {},
        .offset     = sim.map_offset,
        .resolution = sim.map_resolution,
    };
}

[[nodiscard]] MapConfig outputMapConfig(const simulator::types::SimulationConfigData& sim,
                                         const MissionConfigData& mission,
                                         const Map3DImpl& hidden_map) {
    MapConfig config = hidden_map.getMapConfig();

    const double factor = mission.output_mapping_resolution_factor >= 1.0
                              ? mission.output_mapping_resolution_factor
                              : 1.0;
    config.resolution =
        (sim.map_resolution.force_numerical_value_in(cm) / factor) * cm;

    if (!isUnsetBoundaries(mission.mission_bounds)) {
        // Shift mission-local bounds into world coordinates by adding map_offset.
        const Position3D& off      = sim.map_offset;
        config.boundaries.min_x      = mission.mission_bounds.min_x      + off.x;
        config.boundaries.max_x      = mission.mission_bounds.max_x      + off.x;
        config.boundaries.min_y      = mission.mission_bounds.min_y      + off.y;
        config.boundaries.max_y      = mission.mission_bounds.max_y      + off.y;
        config.boundaries.min_height = mission.mission_bounds.min_height + off.z;
        config.boundaries.max_height = mission.mission_bounds.max_height + off.z;
        config.offset = Position3D{
            config.boundaries.min_x,
            config.boundaries.min_y,
            config.boundaries.min_height,
        };
    }
    return config;
}

[[nodiscard]] std::size_t voxelAxisCount(PhysicalLength min_bound,
                                          PhysicalLength max_bound,
                                          PhysicalLength resolution) {
    const double res_cm = resolution.force_numerical_value_in(cm);
    if (res_cm <= 0.0) {
        return 0;
    }
    const double span_cm = (max_bound - min_bound).force_numerical_value_in(cm);
    if (span_cm < 0.0) {
        return 0;
    }
    return static_cast<std::size_t>(std::lround(span_cm / res_cm)) + 1;
}

[[nodiscard]] std::shared_ptr<NpyArray> makeEmptyOutputArray(const MapConfig& config) {
    const std::size_t nx =
        voxelAxisCount(config.boundaries.min_x, config.boundaries.max_x, config.resolution);
    const std::size_t ny =
        voxelAxisCount(config.boundaries.min_y, config.boundaries.max_y, config.resolution);
    const std::size_t nz =
        voxelAxisCount(config.boundaries.min_height, config.boundaries.max_height, config.resolution);
    if (nx == 0 || ny == 0 || nz == 0) {
        return std::make_shared<NpyArray>();
    }
    auto map = std::make_shared<NpyArray>(NpyArray::shape_t{nx, ny, nz},
                                          sizeof(std::int8_t),
                                          NpyArray::GetTypeChar(typeid(std::int8_t)));
    map->Allocate();
    std::fill_n(map->Data<std::int8_t>(), map->NumValue(),
                static_cast<std::int8_t>(VoxelOccupancy::Unmapped));
    return map;
}

} // namespace

SimulationRunFactoryImpl::SimulationRunFactoryImpl(
    const common::MappingAlgorithmFactory& algorithm_factory,
    const common::MissionControlFactory&   mission_control_factory,
    bool                                   verbose)
    : algorithm_factory_(algorithm_factory),
      mission_control_factory_(mission_control_factory),
      verbose_(verbose) {}

std::unique_ptr<ISimulationRun>
SimulationRunFactoryImpl::create(const types::SimulationConfigData&      simulation_config,
                                  const common::types::MissionConfigData& mission_config,
                                  const common::types::DroneConfigData&   drone_config,
                                  const common::types::LidarConfigData&   lidar_config,
                                  const std::filesystem::path&            output_path) {
    std::vector<common::types::ErrorRef> startup_errors;

    // Load hidden map
    auto map_array = std::make_shared<NpyArray>();
    const LPCSTR load_err = map_array->LoadNPY(simulation_config.map_filename.string());
    if (load_err != nullptr) {
        startup_errors.push_back({"MAP_FILE_NOT_FOUND", simulation_config.map_filename.string()});
        // Use an empty placeholder so object construction still succeeds.
        map_array = std::make_shared<NpyArray>();
    }

    auto hidden_map = std::make_unique<Map3DImpl>(
        makeMap3D(map_array, MapRole::Hidden, hiddenMapConfig(simulation_config)));

    // Compute and allocate output map
    const MapConfig out_cfg = outputMapConfig(simulation_config, mission_config, *hidden_map);
    auto output_map = std::make_unique<Map3DImpl>(
        makeMap3D(makeEmptyOutputArray(out_cfg), MapRole::Output, out_cfg));

    // Compute world spawn position (local + map_axes_offset)
    const Position3D world_spawn = UC::worldInitialDronePosition(
        simulation_config.initial_drone_position, simulation_config.map_offset.z);

    if (startup_errors.empty()) {
        if (!UC::isDroneSpawnPassable(*hidden_map, drone_config.radius, world_spawn)) {
            startup_errors.push_back({"SPAWN_NOT_PASSABLE",
                                       "initial drone position is blocked by occupied voxels or boundary"});
        }
    }

    // Build mocks
    auto gps = std::make_unique<simulator::MockGPS>(
        world_spawn,
        common::Orientation{simulation_config.initial_angle,
                            0.0 * common::altitude_angle[common::deg]},
        mission_config.gps_resolution);
    auto movement  = std::make_unique<simulator::MockMovement>(*gps, *hidden_map, drone_config);
    auto lidar     = std::make_unique<simulator::MockLidar>(lidar_config, *hidden_map, *gps);

    // Invoke plugin factories
    auto mapping_algorithm = algorithm_factory_(common::MappingAlgorithmDependencies{
        .mission_config = mission_config,
        .lidar_config   = lidar_config,
        .drone_config   = drone_config,
        .output_map     = *output_map,
    });

    auto mission_control = mission_control_factory_(common::MissionControlDependencies{
        .mission_config    = mission_config,
        .drone_config      = drone_config,
        .lidar             = *lidar,
        .gps               = *gps,
        .movement          = *movement,
        .output_map        = *output_map,
        .mapping_algorithm = *mapping_algorithm,
        .output_map_file   = output_path,
        .verbose           = verbose_,
    });

    return std::make_unique<SimulationRunImpl>(
        std::move(hidden_map),
        std::move(output_map),
        std::move(gps),
        std::move(movement),
        std::move(lidar),
        std::move(mapping_algorithm),
        std::move(mission_control),
        simulation_config,
        mission_config,
        output_path,
        std::move(startup_errors));
}

} // namespace simulator
