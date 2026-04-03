cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/../common/protocol"
    "${CMAKE_CURRENT_LIST_DIR}/../common/drivers"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(can_sim_esp32c6)
