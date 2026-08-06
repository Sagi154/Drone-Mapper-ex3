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

include(CTest)
add_test(NAME test_simulator_mocks COMMAND test_simulator_mocks)
add_test(NAME test_map3d_impl COMMAND test_map3d_impl)
