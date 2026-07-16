# SPDX-License-Identifier: Apache-2.0
# Linux-side IDF-shape project — mirrors examples/<scenario>/mcu_host/.
cmake_minimum_required(VERSION 3.19)
include($ENV{EH_PATH}/tools/cmake/hosted_project.cmake)
project(wifi_softap_host_c_app C)
add_subdirectory(main)
