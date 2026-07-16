# SPDX-License-Identifier: Apache-2.0
# eh_host_feat_bt — CP controller lifecycle + HCI byte pass-through.

if(NOT CONFIG_ESP_HOSTED_HOST_FEAT_BT_READY)
    return()
endif()

add_library(eh_host_feat_bt STATIC
    "src/eh_host_feat_bt.c"
)

target_include_directories(eh_host_feat_bt
    PUBLIC "include"
    PRIVATE "src"
)

target_link_libraries(eh_host_feat_bt
    PUBLIC
        eh_host_port_api
        eh_common
)

# Force-pull TU so EH_HOST_FEAT_REGISTER descriptor lands in feat_descs section.
if(CONFIG_ESP_HOSTED_HOST_FEAT_BT_AUTO_INIT)
    target_link_options(eh_host_feat_bt INTERFACE
        "-Wl,--undefined=eh_host_feat_bt_init")
endif()


target_link_libraries(eh_host_feat_bt PRIVATE eh_host_autoinit)

# HCI byte pipe is MCU-host-only (Linux user-space BT is kmod/BlueZ).
if(CONFIG_ESP_HOSTED_HOST_TRANSPORT_MCU_READY)
    target_sources(eh_host_feat_bt PRIVATE "src/eh_host_feat_bt_mcu.c")
    target_link_libraries(eh_host_feat_bt PRIVATE eh_host_mcu_transport)
endif()

target_sources(eh_host_feat_bt PRIVATE
    "src/eh_host_bt.c"
)
target_link_libraries(eh_host_feat_bt PRIVATE
    eh_host_feat_rpc_ext_v2
    eh_host_feat_rpc
    eh_proto_v2
)

# AUTO_SET_MAC reaches into the system feature for eh_host_iface_mac_addr_set.
if(CONFIG_ESP_HOSTED_HOST_FEAT_BT_AUTO_SET_MAC)
    target_link_libraries(eh_host_feat_bt PRIVATE eh_host_feat_system)
endif()
