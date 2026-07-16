# SPDX-License-Identifier: Apache-2.0

if(NOT CONFIG_ESP_HOSTED_HOST_FEAT_HEARTBEAT_READY)
    return()
endif()

add_library(eh_host_feat_heartbeat STATIC
    "src/eh_host_feat_heartbeat.c"
)

target_include_directories(eh_host_feat_heartbeat
    PUBLIC "include"
    PRIVATE "src"
)

target_link_libraries(eh_host_feat_heartbeat
    PUBLIC
        eh_host_port_api
        eh_common
)

if(CONFIG_ESP_HOSTED_HOST_FEAT_HEARTBEAT_AUTO_INIT)
    target_link_options(eh_host_feat_heartbeat INTERFACE
        "-Wl,--undefined=eh_host_feat_heartbeat_init")
endif()


target_link_libraries(eh_host_feat_heartbeat PRIVATE eh_host_autoinit)

target_sources(eh_host_feat_heartbeat PRIVATE
    "src/eh_host_heartbeat.c"
    "src/eh_host_feat_heartbeat_v2_req.c"
    "src/eh_host_feat_heartbeat_v2_resp_evt.c"
)
target_link_libraries(eh_host_feat_heartbeat PRIVATE
    eh_host_feat_rpc_ext_v2
    eh_host_feat_rpc
    eh_proto_v2
)
