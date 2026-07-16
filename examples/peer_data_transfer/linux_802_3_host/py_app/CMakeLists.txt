# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.19)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
include($ENV{EH_PATH}/tools/cmake/hosted_project.cmake)
project(peer_data_transfer_echo_host_py_app C)
add_subdirectory(main)
