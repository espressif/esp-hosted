# SPDX-License-Identifier: Apache-2.0

if(NOT CONFIG_ESP_HOSTED_HOST_CORE_READY)
    return()
endif()

add_library(eh_host_core STATIC
    "src/eh_host_core.c"
    "src/eh_host_core_auto_init.c"
    "src/eh_host_core_event_bases.c"
    "src/eh_host_core_bringup_glue.c"
    "src/eh_host_idf_idempotent_wraps.c"
)

# CP-bringup policy (eh_host_core_bringup()) — link-time per-port impl.
if(CONFIG_ESP_HOSTED_HOST_TYPE_MCU)
    target_sources(eh_host_core PRIVATE
        "src/mcu/eh_host_core_bringup_mcu.c")
elseif(CONFIG_ESP_HOSTED_HOST_TYPE_POSIX)
    target_sources(eh_host_core PRIVATE
        "src/linux/eh_host_core_bringup_linux.c")
endif()
target_include_directories(eh_host_core
    PUBLIC "include"
    PRIVATE "src"
)

target_link_libraries(eh_host_core
    PUBLIC
        eh_host_feat_rpc
        eh_host_feat_rpc_ext_v2
        eh_host_feat_system
        eh_host_port_api
        eh_common
        eh_host_autoinit
    PRIVATE
        eh_host_config
)

# Force-retain the defining TU: ESP_HOSTED_EVENT used by address only.
target_link_options(eh_host_core INTERFACE
    "-Wl,--undefined=ESP_HOSTED_EVENT")

# Idempotent wraps for two IDF init functions that examples and hosted
# both call.  See src/eh_host_idf_idempotent_wraps.c for rationale.
target_link_options(eh_host_core INTERFACE
    "-Wl,--wrap=esp_netif_init"
    "-Wl,--wrap=esp_event_loop_create_default")

if(CONFIG_ESP_HOSTED_AUTO_CALL_INIT_BEFORE_APP_MAIN)
    target_link_options(eh_host_core INTERFACE
        "-Wl,--undefined=eh_host_auto_init_ctor")
endif()

if(CONFIG_ESP_HOSTED_HOST_VSERIAL_MCU_READY)
    target_link_libraries(eh_host_core PUBLIC eh_host_mcu_vserial)
endif()
if(CONFIG_ESP_HOSTED_HOST_VSERIAL_LINUX_READY)
    target_link_libraries(eh_host_core PUBLIC eh_host_linux_vserial)
endif()
