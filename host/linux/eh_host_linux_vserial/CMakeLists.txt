# eh_host_linux_vserial — Linux /dev/esps0 byte-stream adapter
# SPDX-License-Identifier: Apache-2.0

if(NOT CONFIG_ESP_HOSTED_HOST_VSERIAL_LINUX_READY)
    return()
endif()

add_library(eh_host_linux_vserial STATIC
    "src/eh_host_linux_vserial.c"
)

target_include_directories(eh_host_linux_vserial
    PUBLIC "include"
    PRIVATE "src"
)

target_link_libraries(eh_host_linux_vserial
    PUBLIC
        eh_common
        eh_frame
        eh_host_feat_rpc
)
