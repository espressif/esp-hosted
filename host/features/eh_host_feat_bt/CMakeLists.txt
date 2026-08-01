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

# No BT auto-init: the esp_hosted_bt adapter calls setup explicitly.

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

# eh_host_bt_apply_mac() uses the system feature's MAC setter.
target_link_libraries(eh_host_feat_bt PRIVATE eh_host_feat_system)
