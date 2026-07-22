# SPDX-License-Identifier: Apache-2.0

if(NOT CONFIG_ESP_HOSTED_HOST_FEAT_NW_SPLIT_READY)
    return()
endif()

add_library(eh_host_feat_nw_split STATIC
    "src/eh_host_feat_nw_split.c"
)

target_include_directories(eh_host_feat_nw_split
    PUBLIC "include"
    PRIVATE "src"
)

target_link_libraries(eh_host_feat_nw_split
    PUBLIC
        eh_host_port_api
        eh_common
)

if(CONFIG_ESP_HOSTED_HOST_FEAT_NW_SPLIT_AUTO_INIT)
    target_link_options(eh_host_feat_nw_split INTERFACE
        "-Wl,--undefined=eh_host_feat_nw_split_init")
endif()


target_link_libraries(eh_host_feat_nw_split PRIVATE eh_host_autoinit)

target_sources(eh_host_feat_nw_split PRIVATE
    "src/eh_host_nw_split.c"
    "src/eh_host_feat_nw_split_v2_req.c"
    "src/eh_host_feat_nw_split_v2_resp_evt.c"
)
target_link_libraries(eh_host_feat_nw_split PRIVATE
    eh_host_feat_rpc_ext_v2
    eh_host_feat_rpc
    eh_proto_v2
    eh_host_feat_wifi
)
