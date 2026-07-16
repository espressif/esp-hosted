# SPDX-License-Identifier: Apache-2.0

if(NOT CONFIG_ESP_HOSTED_HOST_FEAT_PEER_DATA_READY)
    return()
endif()

add_library(eh_host_feat_peer_data STATIC
    "src/eh_host_feat_peer_data.c"
    "src/eh_host_feat_peer_data_dispatch.c"
)

target_include_directories(eh_host_feat_peer_data
    PUBLIC "include"
    PRIVATE "src"
)

target_link_libraries(eh_host_feat_peer_data
    PUBLIC
        eh_host_port_api
        eh_common
)

if(CONFIG_ESP_HOSTED_HOST_FEAT_PEER_DATA_AUTO_INIT)
    target_link_options(eh_host_feat_peer_data INTERFACE
        "-Wl,--undefined=eh_host_feat_peer_data_init")
endif()


target_link_libraries(eh_host_feat_peer_data PRIVATE eh_host_autoinit)

target_sources(eh_host_feat_peer_data PRIVATE
    "src/eh_host_peer_data.c"
    "src/eh_host_feat_peer_data_v2_req.c"
    "src/eh_host_feat_peer_data_v2_resp_evt.c"
)
target_link_libraries(eh_host_feat_peer_data PRIVATE
    eh_host_feat_rpc_ext_v2
    eh_host_feat_rpc
    eh_proto_v2
)
