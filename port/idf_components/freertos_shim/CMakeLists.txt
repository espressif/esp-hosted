# port/esp_idf_port/freertos_shim — FreeRTOS-shape header shim.
# SPDX-License-Identifier: Apache-2.0
#
# Consumed only by vendored IDF-internal headers (esp_private/wifi.h,
# lwip/port/freertos/) that include <FreeRTOS.h> directly.  Upper
# layers (examples, features, compat) use eh_host_port_* — they never
# pull from this slot.
#
# Per-port enablement:
#   - esp_idf port:        OFF (early return) — IDF supplies real FreeRTOS.
#   - posix / linux_user:  ON  — typedef-only stubs (this dir's include/).
#   - real FreeRTOS / CMSIS-FreeRTOS ports (future stm32 / zephyr-cmsis):
#                          OFF — integrator's toolchain supplies the headers.

if(ESP_PLATFORM)
    return()
endif()

# Real-FreeRTOS ports use system include path; skip the shim there.
if(CONFIG_ESP_HOSTED_HOST_PORT_TYPE_FREERTOS OR
   CONFIG_ESP_HOSTED_HOST_PORT_TYPE_CMSIS_FREERTOS)
    return()
endif()

add_library(freertos_shim INTERFACE)
target_include_directories(freertos_shim
    INTERFACE "${CMAKE_CURRENT_LIST_DIR}/include"
)
# os_header.h (BaseType_t / TickType_t typedefs) is in the utils slot;
# we include it from FreeRTOS.h.
target_link_libraries(freertos_shim INTERFACE utils)
