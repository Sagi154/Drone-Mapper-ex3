add_library(Algorithm_207190406_209543255 SHARED
    src/MappingAlgorithmFrontier.cpp
    src/MappingAlgorithmImpl.cpp
)
set_target_properties(Algorithm_207190406_209543255 PROPERTIES PREFIX "")
target_link_libraries(Algorithm_207190406_209543255 PRIVATE common::common)
drone_warnings(Algorithm_207190406_209543255)
target_include_directories(Algorithm_207190406_209543255 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
