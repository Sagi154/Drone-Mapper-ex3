# MissionControl plugin — SHARED library loaded by the simulator via dlopen.
add_library(MissionControl_207190406_209543255 SHARED
    src/ScanResultToVoxels.cpp
    src/DroneControlImpl.cpp
    src/MissionControlImpl.cpp
    src/MissionControlPluginRegistration.cpp
)
set_target_properties(MissionControl_207190406_209543255 PROPERTIES PREFIX "")

target_link_libraries(MissionControl_207190406_209543255 PRIVATE common::common)

target_include_directories(MissionControl_207190406_209543255
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/common_mission_control/include
        ${CMAKE_SOURCE_DIR}/UserCommon/include
)

drone_warnings(MissionControl_207190406_209543255)

add_executable(test_drone_control
    tests/test_drone_control.cpp
    src/ScanResultToVoxels.cpp
    src/DroneControlImpl.cpp
)
target_include_directories(test_drone_control PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/common_mission_control/include
    ${CMAKE_SOURCE_DIR}/UserCommon/include
)
target_link_libraries(test_drone_control PRIVATE
    GTest::gtest GTest::gtest_main common::common
)
drone_warnings(test_drone_control)
add_test(NAME test_drone_control COMMAND test_drone_control)

add_executable(test_mission_control
    tests/test_mission_control.cpp
    src/ScanResultToVoxels.cpp
    src/DroneControlImpl.cpp
    src/MissionControlImpl.cpp
)
target_include_directories(test_mission_control PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/common_mission_control/include
    ${CMAKE_SOURCE_DIR}/UserCommon/include
)
target_link_libraries(test_mission_control PRIVATE
    GTest::gtest GTest::gtest_main common::common
)
drone_warnings(test_mission_control)
add_test(NAME test_mission_control COMMAND test_mission_control)