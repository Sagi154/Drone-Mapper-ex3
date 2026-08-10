#include "YamlParseUtil.hpp"

#include <sstream>

namespace simulator::io::detail {

using namespace common;
using namespace common::types;

void logRecoverable(UserCommon_207190406_209543255::IRunErrorLog& log,
                    const std::string& code,
                    const std::string& message) {
    log.log({code, message});
}

std::optional<YAML::Node> loadYamlFile(const std::filesystem::path& path,
                                        UserCommon_207190406_209543255::IRunErrorLog& log,
                                        const std::string& context) {
    if (!std::filesystem::exists(path)) {
        std::ostringstream msg;
        msg << context << " could not open file \"" << path.string() << "\" — using defaults";
        logRecoverable(log, "CONFIG_FILE_NOT_FOUND", msg.str());
        return std::nullopt;
    }
    try {
        return YAML::LoadFile(path.string());
    } catch (const YAML::Exception& ex) {
        std::ostringstream msg;
        msg << context << " parse error in \"" << path.string() << "\": " << ex.what();
        logRecoverable(log, "CONFIG_PARSE_ERROR", msg.str());
        return std::nullopt;
    }
}

YAML::Node configRoot(const YAML::Node& root, const char* wrapper_key) {
    if (root[wrapper_key]) {
        return root[wrapper_key];
    }
    return root;
}

std::optional<double> readScalarDouble(const YAML::Node& node, const char* key) {
    const YAML::Node child = node[key];
    if (!child || !child.IsScalar()) {
        return std::nullopt;
    }
    try {
        return child.as<double>();
    } catch (const YAML::Exception&) {
        return std::nullopt;
    }
}

std::optional<PhysicalLength> readLengthCm(const YAML::Node& node, const char* key) {
    const auto value = readScalarDouble(node, key);
    if (!value.has_value()) {
        return std::nullopt;
    }
    return *value * cm;
}

std::optional<HorizontalAngle> readHorizontalAngleDeg(const YAML::Node& node, const char* key) {
    const auto value = readScalarDouble(node, key);
    if (!value.has_value()) {
        return std::nullopt;
    }
    return *value * horizontal_angle[deg];
}

std::optional<Position3D> readPosition3D(const YAML::Node& node) {
    if (!node || !node.IsMap()) {
        return std::nullopt;
    }
    const auto x      = readScalarDouble(node, "x_cm");
    const auto y      = readScalarDouble(node, "y_cm");
    const auto height = readScalarDouble(node, "height_cm");
    if (!x || !y || !height) {
        return std::nullopt;
    }
    return Position3D{
        *x * x_extent[cm],
        *y * y_extent[cm],
        *height * z_extent[cm],
    };
}

std::optional<Position3D> readMapOffset(const YAML::Node& node) {
    if (!node || !node.IsMap()) {
        return std::nullopt;
    }
    Position3D offset{};

    if (const auto v = readScalarDouble(node, "x_cm")) {
        offset.x = *v * x_extent[cm];
    } else if (const auto v2 = readScalarDouble(node, "x_offset")) {
        offset.x = *v2 * x_extent[cm];
    } else {
        return std::nullopt;
    }

    if (const auto v = readScalarDouble(node, "y_cm")) {
        offset.y = *v * y_extent[cm];
    } else if (const auto v2 = readScalarDouble(node, "y_offset")) {
        offset.y = *v2 * y_extent[cm];
    } else {
        return std::nullopt;
    }

    if (const auto v = readScalarDouble(node, "z_cm")) {
        offset.z = *v * z_extent[cm];
    } else if (const auto v2 = readScalarDouble(node, "height_offset")) {
        offset.z = *v2 * z_extent[cm];
    } else {
        return std::nullopt;
    }

    return offset;
}

std::optional<MappingBounds> readMissionBoundaries(const YAML::Node& node) {
    const YAML::Node boundaries = node["boundaries"];
    if (!boundaries || !boundaries.IsMap()) {
        return std::nullopt;
    }
    MappingBounds bounds{};

    if (const YAML::Node axis = boundaries["x_boundary"]; axis && axis.IsMap()) {
        if (const auto v = readScalarDouble(axis, "min_cm")) { bounds.min_x = *v * x_extent[cm]; }
        if (const auto v = readScalarDouble(axis, "max_cm")) { bounds.max_x = *v * x_extent[cm]; }
    }
    if (const YAML::Node axis = boundaries["y_boundary"]; axis && axis.IsMap()) {
        if (const auto v = readScalarDouble(axis, "min_cm")) { bounds.min_y = *v * y_extent[cm]; }
        if (const auto v = readScalarDouble(axis, "max_cm")) { bounds.max_y = *v * y_extent[cm]; }
    }
    if (const YAML::Node axis = boundaries["height_boundary"]; axis && axis.IsMap()) {
        if (const auto v = readScalarDouble(axis, "min_cm")) { bounds.min_height = *v * z_extent[cm]; }
        if (const auto v = readScalarDouble(axis, "max_cm")) { bounds.max_height = *v * z_extent[cm]; }
    }
    return bounds;
}

} // namespace simulator::io::detail
