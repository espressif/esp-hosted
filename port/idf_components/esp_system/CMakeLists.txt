# SPDX-License-Identifier: Apache-2.0

if(ESP_PLATFORM)
    return()
endif()

add_library(esp_system INTERFACE)
target_include_directories(esp_system INTERFACE
    "${CMAKE_CURRENT_LIST_DIR}/include"
)
