# SPDX-License-Identifier: Apache-2.0
#
# ESP-IDF MCU host project entrypoint. The esp_hosted component is reached via
# main/idf_component.yml's override_path (repo root); no EXTRA_COMPONENT_DIRS.

cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(system_api_exerciser_mcu_host)
