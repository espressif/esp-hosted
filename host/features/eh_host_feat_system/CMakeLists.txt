# SPDX-License-Identifier: Apache-2.0
# eh_host_feat_system — fw version, MAC get/set, heartbeat.

if(NOT CONFIG_ESP_HOSTED_HOST_FEAT_SYSTEM_READY)
    return()
endif()

add_library(eh_host_feat_system STATIC
    "src/eh_host_feat_system.c"
)

target_include_directories(eh_host_feat_system
    PUBLIC "include"
    PRIVATE "src"
)

target_link_libraries(eh_host_feat_system
    PUBLIC
        eh_host_port_api
        eh_common
)

target_sources(eh_host_feat_system PRIVATE
    "src/eh_host_sys.c"
    "src/eh_host_cp_ota.c"
    "src/eh_host_feat_ota_v2_req.c"
    "src/eh_host_feat_ota_v2_resp_evt.c"
    "src/eh_host_feat_system_v2_req.c"
    "src/eh_host_feat_system_v2_resp_evt.c"
)
target_link_libraries(eh_host_feat_system PRIVATE
    eh_host_feat_rpc_ext_v2
    eh_host_feat_rpc
    eh_proto_v2
)
