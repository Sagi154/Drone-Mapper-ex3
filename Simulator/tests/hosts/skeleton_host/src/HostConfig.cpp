#include "HostConfig.h"

#include "HostUnits.h"

#include <TinyNPY.h>
#include <yaml-cpp/yaml.h>

#include <stdexcept>
#include <string>

namespace skeleton_host {

namespace {

[[nodiscard]] std::string requireString(const YAML::Node& node, const char* key) {
    if (!node[key]) {
        throw std::runtime_error(std::string("YAML missing key: ") + key);
    }
    return node[key].as<std::string>();
}

[[nodiscard]] double requireDouble(const YAML::Node& node, const char* key) {
    if (!node[key]) {
        throw std::runtime_error(std::string("YAML missing key: ") + key);
    }
    return node[key].as<double>();
}

[[nodiscard]] std::size_t requireSize(const YAML::Node& node, const char* key) {
    if (!node[key]) {
        throw std::runtime_error(std::string("YAML missing key: ") + key);
    }
    return node[key].as<std::size_t>();
}

[[nodiscard]] YAML::Node require(const YAML::Node& node, const char* key) {
    if (!node[key]) {
        throw std::runtime_error(std::string("YAML missing key: ") + key);
    }
    return node[key];
}

[[nodiscard]] YAML::Node loadYamlFile(const std::filesystem::path& path) {
    if (!std::filesystem::is_regular_file(path)) {
        throw std::runtime_error("missing or unreadable file: " + path.string());
    }
    try {
        return YAML::LoadFile(path.string());
    } catch (const YAML::Exception& ex) {
        throw std::runtime_error("failed to parse YAML '" + path.string() + "': " + ex.what());
    }
}

[[nodiscard]] std::filesystem::path resolveMapPath(const std::filesystem::path& simulation_yaml,
                                                   const std::filesystem::path& map_filename) {
    if (map_filename.is_absolute()) {
        return map_filename;
    }
    const auto sim_dir = simulation_yaml.parent_path();
    const auto beside_sim = sim_dir / map_filename;
    if (std::filesystem::exists(beside_sim)) {
        return beside_sim;
    }
    const auto inputs_root = sim_dir.parent_path();
    const auto under_inputs = inputs_root / map_filename;
    if (std::filesystem::exists(under_inputs)) {
        return under_inputs;
    }
    const auto cwd = std::filesystem::current_path() / map_filename;
    if (std::filesystem::exists(cwd)) {
        return cwd;
    }
    return under_inputs;
}

simulator::types::SimulationConfigData parseSimulation(const YAML::Node& root) {
    const auto cfg = require(root, "simulation_config");
    simulator::types::SimulationConfigData out;
    out.map_filename = requireString(cfg, "map_filename");
    out.map_resolution = length_cm(requireDouble(cfg, "map_resolution_cm"));
    const auto pos = require(cfg, "initial_drone_position");
    out.initial_drone_position = pos_cm(requireDouble(pos, "x_cm"), requireDouble(pos, "y_cm"),
                                        requireDouble(pos, "height_cm"));
    out.initial_angle = horiz_deg(requireDouble(cfg, "initial_angle_deg"));
    const auto off = require(cfg, "map_axes_offset");
    out.map_offset = pos_cm(requireDouble(off, "x_offset"), requireDouble(off, "y_offset"),
                            requireDouble(off, "height_offset"));
    return out;
}

common::types::MissionConfigData parseMission(const YAML::Node& root) {
    const auto cfg = require(root, "mission_config");
    common::types::MissionConfigData out;
    out.max_steps = requireSize(cfg, "max_steps");
    out.gps_resolution = length_cm(requireDouble(cfg, "gps_resolution_cm"));
    if (cfg["output_mapping_resolution_factor"]) {
        out.output_mapping_resolution_factor =
            cfg["output_mapping_resolution_factor"].as<double>();
    }
    const auto bounds = require(cfg, "boundaries");
    const auto xb = require(bounds, "x_boundary");
    const auto yb = require(bounds, "y_boundary");
    const auto zb = require(bounds, "height_boundary");
    out.mission_bounds.min_x = x_cm(requireDouble(xb, "min_cm"));
    out.mission_bounds.max_x = x_cm(requireDouble(xb, "max_cm"));
    out.mission_bounds.min_y = y_cm(requireDouble(yb, "min_cm"));
    out.mission_bounds.max_y = y_cm(requireDouble(yb, "max_cm"));
    out.mission_bounds.min_height = z_cm(requireDouble(zb, "min_cm"));
    out.mission_bounds.max_height = z_cm(requireDouble(zb, "max_cm"));
    return out;
}

common::types::DroneConfigData parseDrone(const YAML::Node& root) {
    const auto cfg = require(root, "drone_config");
    common::types::DroneConfigData out;
    const double diameter = requireDouble(cfg, "dimensions_cm");
    out.radius = length_cm(diameter * 0.5);
    out.max_rotate = horiz_deg(requireDouble(cfg, "max_rotate_deg"));
    out.max_advance = length_cm(requireDouble(cfg, "max_advance_cm"));
    out.max_elevate = length_cm(requireDouble(cfg, "max_elevate_cm"));
    return out;
}

common::types::LidarConfigData parseLidar(const YAML::Node& root) {
    const auto cfg = require(root, "lidar_config");
    common::types::LidarConfigData out;
    out.z_min = length_cm(requireDouble(cfg, "z_min_cm"));
    out.z_max = length_cm(requireDouble(cfg, "z_max_cm"));
    out.d = length_cm(requireDouble(cfg, "d_cm"));
    out.fov_circles = requireSize(cfg, "fov_circles");
    return out;
}

}  // namespace

LoadedConfigs loadStaffConfigs(const std::filesystem::path& simulation_yaml,
                               const std::filesystem::path& mission_yaml,
                               const std::filesystem::path& drone_yaml,
                               const std::filesystem::path& lidar_yaml) {
    LoadedConfigs loaded;
    loaded.simulation = parseSimulation(loadYamlFile(simulation_yaml));
    loaded.mission = parseMission(loadYamlFile(mission_yaml));
    loaded.drone = parseDrone(loadYamlFile(drone_yaml));
    loaded.lidar = parseLidar(loadYamlFile(lidar_yaml));
    loaded.map_path = resolveMapPath(simulation_yaml, loaded.simulation.map_filename);
    if (!std::filesystem::is_regular_file(loaded.map_path)) {
        throw std::runtime_error("missing map file: " + loaded.map_path.string());
    }
    return loaded;
}

}  // namespace skeleton_host
