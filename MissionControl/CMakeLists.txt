# MissionControl plugin — SHARED library loaded by the simulator via dlopen.
add_library(MissionControl_207190406_209543255 SHARED
    src/plugin_placeholder.cpp
)
set_target_properties(MissionControl_207190406_209543255 PROPERTIES PREFIX "")

target_link_libraries(MissionControl_207190406_209543255 PRIVATE common::common)

target_include_directories(MissionControl_207190406_209543255
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/common_mission_control/include
)

drone_warnings(MissionControl_207190406_209543255)
