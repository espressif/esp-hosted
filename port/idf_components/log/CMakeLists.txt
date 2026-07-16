# eh_host_extra/log — IDF esp_log impl, ported per non-IDF host.
# SPDX-License-Identifier: Apache-2.0
#
# Per-port enablement:
#   - esp_idf port:   OFF (early return) — IDF supplies log natively.
#   - posix / linux:  ON — stderr-backed esp_log_write / esp_log_writev,
#                          IDF-shape format prefix (color + level + ms + tag).
#   - stm32 / zephyr: ON — same source today; per-port override via
#                          PRE_FORMAT_NEWLINE_CHAR macro if needed.

if(ESP_PLATFORM)
    return()
endif()

add_library(log STATIC
    src/esp_log.c
    src/esp_err.c
)
target_include_directories(log
    PUBLIC  "${CMAKE_CURRENT_LIST_DIR}/include"
    PRIVATE "src"
)
