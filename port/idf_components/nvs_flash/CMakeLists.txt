# eh_host_extra/nvs_flash — IDF NVS API for hosts without a flash
# partition.  Filesystem-backed impl adapted from
# _upstream_posix_port/posix/src/os_posix_nvs.c.
# SPDX-License-Identifier: Apache-2.0
#
# Per-port enablement:
#   - esp_idf:    off — real IDF NVS over flash partition.
#   - linux_user: on  — filesystem-backed (records persist under
#                       .nvs/ — see src/nvs_flash.c).
#   - stm32 / zephyr: deferred — would need a non-volatile backend
#                                (file / EEPROM / battery-backed RAM).

if(ESP_PLATFORM)
    return()
endif()
if(CONFIG_ESP_HOSTED_HOST_PORT_TYPE_STM32 OR CONFIG_ESP_HOSTED_HOST_PORT_TYPE_ZEPHYR)
    return()
endif()

add_library(nvs_flash STATIC
    "src/nvs_flash.c"
)

target_include_directories(nvs_flash
    PUBLIC "${CMAKE_CURRENT_LIST_DIR}/include"
)
target_link_libraries(nvs_flash PUBLIC utils log esp_system eh_host_port_api_headers)
