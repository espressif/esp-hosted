# SPDX-License-Identifier: Apache-2.0
# console slot. IDF: supplies esp_console natively. Linux: a minimal stdin-REPL
# port of the esp_console API subset (cmd_register / new_repl_uart / start_repl),
# so console-driven host apps build unchanged on both. Cycle-free: links only
# eh_host_port_api_headers (headers), never eh_host_port_api.
if(ESP_PLATFORM)
    return()
endif()

add_library(console STATIC src/esp_console_posix.c)
target_include_directories(console PUBLIC "${CMAKE_CURRENT_LIST_DIR}/include")
target_link_libraries(console PUBLIC eh_host_port_api_headers)
