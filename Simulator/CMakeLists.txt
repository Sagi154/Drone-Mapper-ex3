find_package(GTest CONFIG REQUIRED)

add_executable(test_simulator_mocks
    tests/test_simulator_mocks.cpp
    src/MockGPS.cpp
    src/MockLidar.cpp
    src/MockMovement.cpp
)

target_include_directories(test_simulator_mocks PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(test_simulator_mocks PRIVATE
    GTest::gtest GTest::gtest_main common::common
)

drone_warnings(test_simulator_mocks)

add_executable(test_map3d_impl
    tests/test_map3d_impl.cpp
    src/Map3DImpl.cpp
)

target_include_directories(test_map3d_impl PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(test_map3d_impl PRIVATE
    GTest::gtest GTest::gtest_main common::common TinyNPY::TinyNPY
)

drone_warnings(test_map3d_impl)

add_executable(test_usercommon_time_error_log
    tests/test_usercommon_time_error_log.cpp
    ${CMAKE_SOURCE_DIR}/UserCommon/src/TimeFormat.cpp
    ${CMAKE_SOURCE_DIR}/UserCommon/src/RunErrorLog.cpp
)
target_include_directories(test_usercommon_time_error_log PRIVATE
    ${CMAKE_SOURCE_DIR}/UserCommon/include
    ${CMAKE_SOURCE_DIR}/common/include
)
target_link_libraries(test_usercommon_time_error_log PRIVATE GTest::gtest GTest::gtest_main common::common)
drone_warnings(test_usercommon_time_error_log)

add_executable(test_usercommon_config_parse_result
    tests/test_usercommon_config_parse_result.cpp
)
target_include_directories(test_usercommon_config_parse_result PRIVATE
    ${CMAKE_SOURCE_DIR}/UserCommon/include
    ${CMAKE_SOURCE_DIR}/common/include
)
target_link_libraries(test_usercommon_config_parse_result PRIVATE
    GTest::gtest GTest::gtest_main common::common
)
drone_warnings(test_usercommon_config_parse_result)

add_executable(test_usercommon_simulation_coord_util
    tests/test_usercommon_simulation_coord_util.cpp
    ${CMAKE_SOURCE_DIR}/UserCommon/src/SimulationCoordUtil.cpp
)
target_include_directories(test_usercommon_simulation_coord_util PRIVATE
    ${CMAKE_SOURCE_DIR}/UserCommon/include
    ${CMAKE_SOURCE_DIR}/common/include
    ${CMAKE_SOURCE_DIR}/Simulator/common_simulator/include
)
target_link_libraries(test_usercommon_simulation_coord_util PRIVATE
    GTest::gtest GTest::gtest_main common::common
)
drone_warnings(test_usercommon_simulation_coord_util)

include(CTest)
add_test(NAME test_simulator_mocks COMMAND test_simulator_mocks)
add_test(NAME test_map3d_impl COMMAND test_map3d_impl)
add_test(NAME test_usercommon_time_error_log COMMAND test_usercommon_time_error_log)
add_test(NAME test_usercommon_config_parse_result COMMAND test_usercommon_config_parse_result)
add_test(NAME test_usercommon_simulation_coord_util COMMAND test_usercommon_simulation_coord_util)

set(YAML_PARSER_SOURCES
    src/io/YamlParseUtil.cpp
    src/io/PathResolver.cpp
    src/io/DroneConfigYamlParser.cpp
    src/io/LidarConfigYamlParser.cpp
    src/io/MissionConfigYamlParser.cpp
    src/io/SimulationConfigYamlParser.cpp
    src/io/CompositionYamlParser.cpp
)

add_executable(test_yaml_config_parsers
    tests/test_yaml_config_parsers.cpp
    ${YAML_PARSER_SOURCES}
    ${CMAKE_SOURCE_DIR}/UserCommon/src/TimeFormat.cpp
    ${CMAKE_SOURCE_DIR}/UserCommon/src/RunErrorLog.cpp
)

target_include_directories(test_yaml_config_parsers PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/common_simulator/include
    ${CMAKE_SOURCE_DIR}/UserCommon/include
)

target_link_libraries(test_yaml_config_parsers PRIVATE
    GTest::gtest GTest::gtest_main common::common yaml-cpp::yaml-cpp
)

drone_warnings(test_yaml_config_parsers)
add_test(NAME test_yaml_config_parsers COMMAND test_yaml_config_parsers)

set(SIMULATOR_CORE_SOURCES
    src/MockGPS.cpp
    src/MockLidar.cpp
    src/MockMovement.cpp
    src/Map3DImpl.cpp
    src/SimulationRunImpl.cpp
    src/SimulationRunFactoryImpl.cpp
    ${CMAKE_SOURCE_DIR}/UserCommon/src/TimeFormat.cpp
    ${CMAKE_SOURCE_DIR}/UserCommon/src/RunErrorLog.cpp
    ${CMAKE_SOURCE_DIR}/UserCommon/src/SimulationCoordUtil.cpp
)

add_executable(test_simulation_run_factory
    tests/test_simulation_run_factory.cpp
    ${SIMULATOR_CORE_SOURCES}
)

target_include_directories(test_simulation_run_factory PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/common_simulator/include
    ${CMAKE_SOURCE_DIR}/UserCommon/include
)

target_link_libraries(test_simulation_run_factory PRIVATE
    GTest::gtest GTest::gtest_main common::common TinyNPY::TinyNPY
)

drone_warnings(test_simulation_run_factory)
add_test(NAME test_simulation_run_factory COMMAND test_simulation_run_factory)

# CLI parse/validation is self-contained (no plugins / dlopen / threading).
add_library(simulator_cli STATIC
    src/io/SimulationCli.cpp
)
add_library(simulator::cli ALIAS simulator_cli)
target_include_directories(simulator_cli PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
drone_warnings(simulator_cli)

# Registrar + registration ctor bodies (Simulator only — never link into a plugin).
add_library(simulator_registrar STATIC
    src/PluginRegistrar.cpp
    src/MappingAlgorithmRegistration.cpp
    src/MissionControlRegistration.cpp
)
add_library(simulator::registrar ALIAS simulator_registrar)
target_link_libraries(simulator_registrar PUBLIC common::common)
target_include_directories(simulator_registrar PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
drone_warnings(simulator_registrar)

add_executable(simulator_cli_test
    tests/test_simulation_cli.cpp
)
target_link_libraries(simulator_cli_test PRIVATE
    simulator::cli
    GTest::gtest_main
)
drone_warnings(simulator_cli_test)

add_executable(simulator_registrar_test
    tests/test_plugin_registrar.cpp
)
target_link_libraries(simulator_registrar_test PRIVATE
    simulator::registrar
    GTest::gtest_main
)
drone_warnings(simulator_registrar_test)

include(GoogleTest)
gtest_discover_tests(simulator_cli_test)
gtest_discover_tests(simulator_registrar_test)

# TODO: Build the simulator executable and its private implementation sources.
# Link the executable with common::common and ${CMAKE_DL_LIBS}, and export the
# registration-constructor symbols needed by dynamically loaded plugins.
