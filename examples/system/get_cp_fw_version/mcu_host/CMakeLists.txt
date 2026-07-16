# SPDX-License-Identifier: Apache-2.0
#
# system/get_cp_fw_version/mcu_host/2_host -- ESP-IDF MCU host project
# entrypoint.  Mirrors examples/wifi/mcu/CMakeLists.txt: the esp_hosted target
# `esp_hosted` component is reached via idf_component.yml's
# override_path mechanism (see main/idf_component.yml).  No
# EXTRA_COMPONENT_DIRS needed -- the managed-components flow handles
# discovery from the repo root automatically.

cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(system_get_cp_fw_version_mcu_host)
