# SPDX-License-Identifier: Apache-2.0
# Linux-side IDF-shape project — mirrors examples/<scenario>/mcu_host/.
cmake_minimum_required(VERSION 3.19)
list(APPEND EXTRA_COMPONENT_DIRS
    "$ENV{EH_PATH}/examples/common_components/eh_example_common")
include($ENV{EH_PATH}/tools/cmake/hosted_project.cmake)
project(network_split_station_c_app C)
add_subdirectory(main)
