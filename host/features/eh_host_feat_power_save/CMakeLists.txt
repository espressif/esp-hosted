# SPDX-License-Identifier: Apache-2.0

if(NOT CONFIG_ESP_HOSTED_HOST_FEAT_POWER_SAVE_READY)
    return()
endif()

add_library(eh_host_feat_power_save STATIC
    "src/eh_host_feat_power_save.c"
)

target_include_directories(eh_host_feat_power_save
    PUBLIC  "include"
    PRIVATE "src"
)

target_link_libraries(eh_host_feat_power_save
    PUBLIC
        eh_host_compat_types
        eh_host_port_api
        eh_common
        eh_host_core
    PRIVATE
        eh_host_config
)

if(CONFIG_ESP_HOSTED_HOST_TRANSPORT_MCU_READY)
    target_link_libraries(eh_host_feat_power_save PRIVATE eh_host_mcu_transport)
endif()

if(CONFIG_ESP_HOSTED_HOST_FEAT_POWER_SAVE_AUTO_INIT)
    target_link_options(eh_host_feat_power_save INTERFACE
        "-Wl,--undefined=eh_host_power_save_init")
endif()
