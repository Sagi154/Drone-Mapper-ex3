cmake_minimum_required(VERSION 3.15)
project(skeleton_host LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    get_filename_component(SKELETON_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
    find_package(mp-units CONFIG REQUIRED)
else()
    set(SKELETON_ROOT "${CMAKE_SOURCE_DIR}")
endif()

find_package(yaml-cpp CONFIG REQUIRED)
find_package(TinyNPY CONFIG REQUIRED)

if(TARGET yaml-cpp::yaml-cpp)
    set(SKELETON_HOST_YAML yaml-cpp::yaml-cpp)
else()
    set(SKELETON_HOST_YAML yaml-cpp)
endif()

add_executable(skeleton_host
    src/main.cpp
    src/HostMap3D.cpp
    src/HostGPS.cpp
    src/HostLidar.cpp
    src/HostMovement.cpp
    src/HostRegistrar.cpp
    src/HostConfig.cpp
    src/HostNpy.cpp
    src/MappingAlgorithmRegistration.cpp
    src/MissionControlRegistration.cpp
)

target_include_directories(skeleton_host PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${SKELETON_ROOT}/common/include"
    "${SKELETON_ROOT}/Simulator/common_simulator/include"
)

target_link_libraries(skeleton_host PRIVATE
    mp-units::mp-units
    ${SKELETON_HOST_YAML}
    TinyNPY::TinyNPY
    ${CMAKE_DL_LIBS}
)

set_target_properties(skeleton_host PROPERTIES
    ENABLE_EXPORTS ON
    PREFIX ""
)

if(NOT MSVC)
    target_compile_options(skeleton_host PRIVATE -Wall -Wextra -Werror -pedantic)
    # PIE executables do not put their symbols on the plugin search path
    # unless the dynamic symbol table is exported.
    target_link_options(skeleton_host PRIVATE "-Wl,--export-dynamic")
endif()
