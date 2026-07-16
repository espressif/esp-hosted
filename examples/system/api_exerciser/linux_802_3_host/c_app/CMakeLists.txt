# SPDX-License-Identifier: Apache-2.0
# Linux-side project — mirrors examples/system/get_cp_fw_version/.../c_app.
cmake_minimum_required(VERSION 3.19)
include($ENV{EH_PATH}/tools/cmake/hosted_project.cmake)
project(system_api_exerciser_c_app C)
add_subdirectory(main)
