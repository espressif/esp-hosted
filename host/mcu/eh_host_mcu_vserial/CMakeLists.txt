# eh_host_mcu_vserial — MCU byte-stream channel adapter
# SPDX-License-Identifier: Apache-2.0

if(NOT CONFIG_ESP_HOSTED_HOST_VSERIAL_MCU_READY)
    return()
endif()

add_library(eh_host_mcu_vserial STATIC
    "src/eh_host_mcu_vserial.c"
)

target_include_directories(eh_host_mcu_vserial
    PUBLIC "include"
)

target_link_libraries(eh_host_mcu_vserial
    PUBLIC
        eh_common
        eh_host_feat_rpc
        eh_host_mcu_transport
        eh_host_port_api
)
