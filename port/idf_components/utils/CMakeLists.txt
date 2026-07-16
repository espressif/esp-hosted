# eh_host_extra/utils — IDF cross-cutting helpers.
# SPDX-License-Identifier: Apache-2.0
#
# Per-port enablement: on for any non-IDF port.
#   - esp_idf port: real IDF supplies them natively.
#   - linux_user / stm32 / zephyr ports: vendor sources here.
# Tiny self-contained component (≈674 LOC at upstream STM32).
#
# esp_assert.h, esp_check.h, esp_macros.h.
#
# Status: SLOT — not yet vendored.  See README.md.

if(ESP_PLATFORM)
    return()
endif()

add_library(utils INTERFACE)
target_include_directories(utils INTERFACE
    "${CMAKE_CURRENT_LIST_DIR}/include"
)
target_link_libraries(utils INTERFACE eh_host_config log)
