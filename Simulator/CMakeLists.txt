# TODO: Build the simulator executable and its private implementation sources.
# Link the executable with common::common and ${CMAKE_DL_LIBS}, and export the
# registration-constructor symbols needed by dynamically loaded plugins.

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
add_test(NAME test_usercommon_time_error_log COMMAND test_usercommon_time_error_log)
