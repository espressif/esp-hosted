# SPDX-License-Identifier: Apache-2.0
# eh_host_autoinit — feature descriptor registry + ordered init/deinit walker.

if(NOT CONFIG_ESP_HOSTED_HOST_CORE_READY)
    return()
endif()

add_library(eh_host_autoinit STATIC
    "src/eh_host_auto_init.c"
)

target_include_directories(eh_host_autoinit
    PUBLIC "include"
)

target_link_libraries(eh_host_autoinit
    PUBLIC
        eh_host_port_api
        eh_host_config
)
