cmake_minimum_required(VERSION 3.16)

# Pick up shared bridge components from the per-stack folder.
set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/../../../components"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ble_advertise_minimal)
