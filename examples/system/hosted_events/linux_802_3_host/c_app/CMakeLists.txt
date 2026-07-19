# SPDX-License-Identifier: Apache-2.0
# Linux-side IDF-shape project — mirrors examples/<scenario>/mcu_host/.
cmake_minimum_required(VERSION 3.19)
list(APPEND EXTRA_COMPONENT_DIRS
    "$ENV{EH_PATH}/examples/common_components/esp_hosted_examples_common")
include($ENV{EH_PATH}/tools/cmake/hosted_project.cmake)
project(system_hosted_events_c_app C)
add_subdirectory(main)
