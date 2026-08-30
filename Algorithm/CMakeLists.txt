add_library(Algorithm_207190406_209543255 SHARED
    src/MappingAlgorithmFrontier.cpp
    src/MappingAlgorithmImpl.cpp
    src/PathShaping.cpp
    src/NbvPlanner.cpp
    src/WavefrontPlanner.cpp
)
set_target_properties(Algorithm_207190406_209543255 PROPERTIES PREFIX "")
target_link_libraries(Algorithm_207190406_209543255 PRIVATE common::common)
drone_warnings(Algorithm_207190406_209543255)
target_include_directories(Algorithm_207190406_209543255 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/UserCommon/include)

# ── Algorithm unit tests ──────────────────────────────────────────────────────
add_executable(algorithm_test
    tests/test_mapping_algorithm.cpp
    tests/test_mapping_algorithm_frontier.cpp
    tests/test_lidar_cone.cpp
    tests/test_cone_template.cpp
    tests/test_path_shaping.cpp
    tests/test_nbv_planner.cpp
    tests/test_wavefront_planner.cpp
    # Compile sources directly so tests are self-contained (no dlopen/shared lib).
    src/MappingAlgorithmFrontier.cpp
    src/MappingAlgorithmImpl.cpp
    src/PathShaping.cpp
    src/NbvPlanner.cpp
    src/WavefrontPlanner.cpp
    # Stub: satisfies MappingAlgorithmRegistration ctor (provided by Simulator in prod).
    tests/StubPluginRegistration.cpp
)
target_link_libraries(algorithm_test PRIVATE
    GTest::gtest_main
    common::common
)
drone_warnings(algorithm_test)
target_include_directories(algorithm_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include  # Algorithm/MappingAlgorithmImpl.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src      # MappingAlgorithmFrontier.h (internal)
    ${CMAKE_CURRENT_SOURCE_DIR}/tests    # FakeMap3D.h
    ${CMAKE_SOURCE_DIR}/UserCommon/include
)
include(GoogleTest)
gtest_discover_tests(algorithm_test)
