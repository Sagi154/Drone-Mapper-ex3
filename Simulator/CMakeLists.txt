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

set(YAML_PARSER_SOURCES
    src/io/YamlParseUtil.cpp
    src/io/PathResolver.cpp
    src/io/DroneConfigYamlParser.cpp
    src/io/LidarConfigYamlParser.cpp
    src/io/MissionConfigYamlParser.cpp
    src/io/SimulationConfigYamlParser.cpp
    src/io/CompositionYamlParser.cpp
)

add_executable(test_usercommon_simulation_coord_util
    tests/test_usercommon_simulation_coord_util.cpp
    src/Map3DImpl.cpp
    ${YAML_PARSER_SOURCES}
    ${CMAKE_SOURCE_DIR}/UserCommon/src/SimulationCoordUtil.cpp
    ${CMAKE_SOURCE_DIR}/UserCommon/src/TimeFormat.cpp
    ${CMAKE_SOURCE_DIR}/UserCommon/src/RunErrorLog.cpp
)
target_include_directories(test_usercommon_simulation_coord_util PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/UserCommon/include
    ${CMAKE_SOURCE_DIR}/common/include
    ${CMAKE_SOURCE_DIR}/Simulator/common_simulator/include
)
target_link_libraries(test_usercommon_simulation_coord_util PRIVATE
    GTest::gtest GTest::gtest_main common::common TinyNPY::TinyNPY yaml-cpp::yaml-cpp
)
drone_warnings(test_usercommon_simulation_coord_util)

include(CTest)
add_test(NAME test_simulator_mocks COMMAND test_simulator_mocks)
add_test(NAME test_map3d_impl COMMAND test_map3d_impl)
add_test(NAME test_usercommon_time_error_log COMMAND test_usercommon_time_error_log)
add_test(NAME test_usercommon_config_parse_result COMMAND test_usercommon_config_parse_result)
add_test(NAME test_usercommon_simulation_coord_util COMMAND test_usercommon_simulation_coord_util)

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
    tests/test_simulation_run_impl.cpp
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

# Registrar singleton (no registration ctor bodies — those are an OBJECT lib so
# the linker cannot drop them from a static archive before ENABLE_EXPORTS).
add_library(simulator_registrar STATIC
    src/PluginRegistrar.cpp
)
add_library(simulator::registrar ALIAS simulator_registrar)
target_link_libraries(simulator_registrar PUBLIC common::common)
target_include_directories(simulator_registrar PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
drone_warnings(simulator_registrar)

# Registration ctor bodies — OBJECT so every consumer that links them exports
# the symbols (required for dlopen'd plugins to resolve the undefined ctor).
add_library(simulator_registration OBJECT
    src/MappingAlgorithmRegistration.cpp
    src/MissionControlRegistration.cpp
)
target_link_libraries(simulator_registration PUBLIC simulator::registrar)
drone_warnings(simulator_registration)

# dlopen/dlclose loader (depends on registrar).
add_library(simulator_loader STATIC
    src/PluginLoader.cpp
)
add_library(simulator::loader ALIAS simulator_loader)
target_link_libraries(simulator_loader PUBLIC simulator::registrar ${CMAKE_DL_LIBS})
drone_warnings(simulator_loader)

# Work matrix distributor (threads + atomic index).
find_package(Threads REQUIRED)
add_library(simulator_work STATIC
    src/WorkDistributor.cpp
)
add_library(simulator::work ALIAS simulator_work)
target_include_directories(simulator_work PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(simulator_work PUBLIC Threads::Threads)
drone_warnings(simulator_work)

# Map scoring (stub body returns -1 until full BFS port).
add_library(simulator_scoring STATIC
    src/MapsComparison.cpp
)
add_library(simulator::scoring ALIAS simulator_scoring)
target_link_libraries(simulator_scoring PUBLIC common::common)
target_include_directories(simulator_scoring PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
drone_warnings(simulator_scoring)

# SimulationRunImpl (in test_simulation_run_factory) calls MapsComparison.
target_link_libraries(test_simulation_run_factory PRIVATE simulator::scoring)

# Report YAML writers (comparative / competitive / per-plugin).
add_library(simulator_report_writers STATIC
    src/io/SimulationOutputYamlWriter.cpp
    src/io/ComparativeReportWriter.cpp
    src/io/CompetitiveReportWriter.cpp
)
add_library(simulator::report_writers ALIAS simulator_report_writers)
target_include_directories(simulator_report_writers PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/common_simulator/include
)
target_link_libraries(simulator_report_writers PUBLIC common::common yaml-cpp::yaml-cpp)
drone_warnings(simulator_report_writers)

add_executable(simulator_report_writers_test
    tests/test_simulation_output_yaml_writer.cpp
    tests/test_comparative_report_writer.cpp
    tests/test_competitive_report_writer.cpp
)
target_link_libraries(simulator_report_writers_test PRIVATE
    simulator::report_writers
    GTest::gtest_main
)
drone_warnings(simulator_report_writers_test)

add_executable(simulator_output_path_util_test
    tests/test_output_path_util.cpp
)
target_include_directories(simulator_output_path_util_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(simulator_output_path_util_test PRIVATE
    GTest::gtest_main
)
drone_warnings(simulator_output_path_util_test)

# Collision-checked output directory helper.
add_library(simulator_output_dir STATIC
    src/io/OutputDirHelper.cpp
    ${CMAKE_SOURCE_DIR}/UserCommon/src/TimeFormat.cpp
)
add_library(simulator::output_dir ALIAS simulator_output_dir)
target_include_directories(simulator_output_dir PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/UserCommon/include
)
drone_warnings(simulator_output_dir)

add_executable(simulator_output_dir_test
    tests/test_output_dir_helper.cpp
)
target_link_libraries(simulator_output_dir_test PRIVATE
    simulator::output_dir
    GTest::gtest_main
)
drone_warnings(simulator_output_dir_test)

# Run-matrix orchestrator (frozen ISimulationRunFactory / ISimulationRun).
add_library(simulator_orchestrator STATIC
    src/RunMatrixOrchestrator.cpp
)
add_library(simulator::orchestrator ALIAS simulator_orchestrator)
target_include_directories(simulator_orchestrator PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/common_simulator/include
)
target_link_libraries(simulator_orchestrator PUBLIC
    simulator::work
    common::common
)
drone_warnings(simulator_orchestrator)

# ── Unit tests ────────────────────────────────────────────────────────────────
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
    $<TARGET_OBJECTS:simulator_registration>
)
target_link_libraries(simulator_registrar_test PRIVATE
    simulator::registrar
    GTest::gtest_main
)
drone_warnings(simulator_registrar_test)

# Test-only .so fixtures for the loader (never shipped). Registration ctor stays
# UNDEFINED in these libs and is resolved from the test executable via ENABLE_EXPORTS.
set(PLUGIN_FIXTURES_OUT "${CMAKE_CURRENT_BINARY_DIR}/tests/fixtures")
file(MAKE_DIRECTORY "${PLUGIN_FIXTURES_OUT}")

add_library(valid_algorithm_plugin SHARED
    tests/fixtures/valid_algorithm_plugin.cpp
)
set_target_properties(valid_algorithm_plugin PROPERTIES
    PREFIX ""
    LIBRARY_OUTPUT_DIRECTORY "${PLUGIN_FIXTURES_OUT}"
)
target_link_libraries(valid_algorithm_plugin PRIVATE common::common)
drone_warnings(valid_algorithm_plugin)

add_library(unregistered_plugin SHARED
    tests/fixtures/unregistered_plugin.cpp
)
set_target_properties(unregistered_plugin PROPERTIES
    PREFIX ""
    LIBRARY_OUTPUT_DIRECTORY "${PLUGIN_FIXTURES_OUT}"
)
drone_warnings(unregistered_plugin)

add_library(valid_mission_control_plugin SHARED
    tests/fixtures/valid_mission_control_plugin.cpp
)
set_target_properties(valid_mission_control_plugin PROPERTIES
    PREFIX ""
    LIBRARY_OUTPUT_DIRECTORY "${PLUGIN_FIXTURES_OUT}"
)
target_link_libraries(valid_mission_control_plugin PRIVATE common::common)
drone_warnings(valid_mission_control_plugin)

add_library(faulty_wall_algorithm_plugin SHARED
    tests/fixtures/faulty_wall_algorithm_plugin.cpp
)
set_target_properties(faulty_wall_algorithm_plugin PROPERTIES
    PREFIX ""
    LIBRARY_OUTPUT_DIRECTORY "${PLUGIN_FIXTURES_OUT}"
)
target_link_libraries(faulty_wall_algorithm_plugin PRIVATE common::common)
drone_warnings(faulty_wall_algorithm_plugin)

add_executable(simulator_loader_test
    tests/test_plugin_loader.cpp
    $<TARGET_OBJECTS:simulator_registration>
)
set_target_properties(simulator_loader_test PROPERTIES ENABLE_EXPORTS ON)
target_link_libraries(simulator_loader_test PRIVATE
    simulator::loader
    GTest::gtest_main
)
target_compile_definitions(simulator_loader_test PRIVATE
    PLUGIN_FIXTURES_DIR="${PLUGIN_FIXTURES_OUT}"
)
add_dependencies(simulator_loader_test
    valid_algorithm_plugin
    unregistered_plugin
    valid_mission_control_plugin
)
drone_warnings(simulator_loader_test)

add_executable(simulator_work_test
    tests/test_work_distributor.cpp
)
target_link_libraries(simulator_work_test PRIVATE
    simulator::work
    GTest::gtest_main
)
drone_warnings(simulator_work_test)

add_executable(simulator_scoring_test
    tests/test_maps_comparison.cpp
)
target_link_libraries(simulator_scoring_test PRIVATE
    simulator::scoring
    GTest::gtest_main
)
drone_warnings(simulator_scoring_test)

add_executable(simulator_orchestrator_test
    tests/test_run_matrix_orchestrator.cpp
)
target_link_libraries(simulator_orchestrator_test PRIVATE
    simulator::orchestrator
    GTest::gtest_main
)
drone_warnings(simulator_orchestrator_test)

include(GoogleTest)
gtest_discover_tests(simulator_cli_test)
gtest_discover_tests(simulator_registrar_test)
gtest_discover_tests(simulator_loader_test)
gtest_discover_tests(simulator_work_test)
gtest_discover_tests(simulator_scoring_test)
gtest_discover_tests(simulator_orchestrator_test)
gtest_discover_tests(simulator_report_writers_test)
gtest_discover_tests(simulator_output_dir_test)
gtest_discover_tests(simulator_output_path_util_test)

set(SIMULATOR_MAIN_SOURCES
    src/main.cpp
    src/Map3DImpl.cpp
    src/MockGPS.cpp
    src/MockLidar.cpp
    src/MockMovement.cpp
    src/SimulationRunImpl.cpp
    src/SimulationRunFactoryImpl.cpp
    src/io/SimulationOutputYamlWriter.cpp
    src/io/ComparativeReportWriter.cpp
    src/io/CompetitiveReportWriter.cpp
    src/io/OutputDirHelper.cpp
    ${YAML_PARSER_SOURCES}
    ${CMAKE_SOURCE_DIR}/UserCommon/src/TimeFormat.cpp
    ${CMAKE_SOURCE_DIR}/UserCommon/src/RunErrorLog.cpp
    ${CMAKE_SOURCE_DIR}/UserCommon/src/SimulationCoordUtil.cpp
    $<TARGET_OBJECTS:simulator_registration>
)

add_executable(simulator_207190406_209543255 ${SIMULATOR_MAIN_SOURCES})
set_target_properties(simulator_207190406_209543255 PROPERTIES ENABLE_EXPORTS ON)

target_include_directories(simulator_207190406_209543255 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/common_simulator/include
    ${CMAKE_SOURCE_DIR}/UserCommon/include
)

target_link_libraries(simulator_207190406_209543255 PRIVATE
    common::common
    simulator::loader
    simulator::registrar
    simulator::orchestrator
    simulator::cli
    simulator::scoring
    TinyNPY::TinyNPY
    yaml-cpp::yaml-cpp
    Threads::Threads
    ${CMAKE_DL_LIBS}
)

drone_warnings(simulator_207190406_209543255)
add_dependencies(simulator_207190406_209543255 faulty_wall_algorithm_plugin)
