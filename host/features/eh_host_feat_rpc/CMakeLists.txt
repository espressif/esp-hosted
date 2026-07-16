# SPDX-License-Identifier: Apache-2.0
# eh_host_feat_rpc — Host RPC base (uid/correlation/IO binding).

if(NOT CONFIG_ESP_HOSTED_HOST_FEAT_RPC_READY)
    return()
endif()

add_library(eh_host_feat_rpc STATIC
    "src/eh_host_feat_rpc.c"
    "src/eh_host_feat_rpc_tx.c"
    "src/eh_host_feat_rpc_rx.c"
    "src/eh_host_feat_rpc_uid.c"
    "src/eh_host_feat_rpc_evt.c"
)

target_include_directories(eh_host_feat_rpc
    PUBLIC "include"
    PRIVATE "src"
)

target_link_libraries(eh_host_feat_rpc
    PUBLIC
        eh_common
        eh_frame
        eh_tlv
        eh_host_port_api
)

