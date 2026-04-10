# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

get_filename_component(_motor_protocol_build_dir_name "${CMAKE_BINARY_DIR}" NAME)
set(_motor_protocol_test_runner OFF)

if(_motor_protocol_build_dir_name STREQUAL "built-test")
    set(_motor_protocol_test_runner ON)
    set(EXTRA_COMPONENT_DIRS test)
endif()

set(MOTOR_PROTOCOL_TEST_RUNNER
    ${_motor_protocol_test_runner}
    CACHE BOOL "Build firmware with Unity test runner app_main"
    FORCE)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(brakingMotor)
