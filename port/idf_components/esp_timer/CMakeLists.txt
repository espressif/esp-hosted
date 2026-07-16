# port/esp_idf_port/esp_timer — IDF esp_timer shadow.
# SPDX-License-Identifier: Apache-2.0
#
# Per-port enablement:
#   - esp_idf:    OFF (early return) — IDF supplies esp_timer natively.
#   - linux_user: STATIC, esp_timer_get_time over clock_gettime(CLOCK_MONOTONIC).
#   - stm32 / zephyr: deferred — would need a hardware-timer backend.

if(ESP_PLATFORM)
    return()
endif()
if(CONFIG_ESP_HOSTED_HOST_PORT_TYPE_STM32 OR CONFIG_ESP_HOSTED_HOST_PORT_TYPE_ZEPHYR)
    return()
endif()

add_library(esp_timer STATIC
    "src/esp_timer_linux.c"
)
target_include_directories(esp_timer
    PUBLIC "${CMAKE_CURRENT_LIST_DIR}/include")
target_link_libraries(esp_timer PUBLIC utils eh_host_config)
