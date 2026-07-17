# SPDX-License-Identifier: Apache-2.0
# Linux-side IDF-shape project — mirrors examples/<scenario>/mcu_host/.
cmake_minimum_required(VERSION 3.19)
# esp_hosted_examples_common: single shared copy in the repo checkout; copied
# into components/ at registry-publish time. Add the checkout path only when it
# exists, so an extracted project (no repo siblings) doesn't hard-error.
if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/../../../../common_components/esp_hosted_examples_common")
    list(APPEND EXTRA_COMPONENT_DIRS
        "${CMAKE_CURRENT_LIST_DIR}/../../../../common_components/esp_hosted_examples_common")
endif()
include($ENV{EH_PATH}/tools/cmake/hosted_project.cmake)
project(wifi_sta_host_c_app C)
add_subdirectory(main)
