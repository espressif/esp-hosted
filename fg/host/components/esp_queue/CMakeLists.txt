cmake_minimum_required(VERSION 3.16)
project(esp_queue C)

add_library(esp_queue STATIC
    src/esp_queue.c
)

target_include_directories(esp_queue PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Enable position-independent code for shared library compatibility
set_target_properties(esp_queue PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)
