cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/../common/protocol"
    "${CMAKE_CURRENT_LIST_DIR}/../common/drivers"
    "${CMAKE_CURRENT_LIST_DIR}/../common/logic"
    "${CMAKE_CURRENT_LIST_DIR}/../common/services"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(safety_esp32)
