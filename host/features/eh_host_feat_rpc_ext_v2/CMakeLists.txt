# SPDX-License-Identifier: Apache-2.0
# eh_host_feat_rpc_ext_v2 — MCU V1 Rpc mappings.

if(NOT CONFIG_ESP_HOSTED_HOST_FEAT_RPC_EXT_V2_READY)
    return()
endif()

set(SRCS
    "src/eh_host_feat_rpc_ext_v2.c"
    "src/eh_host_feat_rpc_ext_v2_pack.c"
    "src/eh_host_feat_rpc_ext_v2_decode.c"
    "src/eh_host_feat_rpc_ext_v2_api_common.c"
)

# Shared RpcReqFeatureControl wire op — used by both feat_bt and feat_openthread.
if(CONFIG_ESP_HOSTED_HOST_FEAT_BT_READY OR CONFIG_ESP_HOSTED_HOST_FEAT_OPENTHREAD_READY)
    list(APPEND SRCS "src/feature_control_req.c")
    list(APPEND SRCS "src/feature_control_resp_evt.c")
endif()

add_library(eh_host_feat_rpc_ext_v2 STATIC ${SRCS})

target_include_directories(eh_host_feat_rpc_ext_v2
    PUBLIC "include"
    PRIVATE "src"
)

target_link_libraries(eh_host_feat_rpc_ext_v2
    PUBLIC
        eh_host_feat_rpc
        eh_proto_v2
        eh_host_port_api
        eh_common
        eh_frame
        eh_tlv
    PRIVATE
        eh_host_config
)

# Per-feature PRIVATE links are wired in host/CMakeLists.txt AFTER feature
# targets exist; static-lib cycle is handled via --start-group on final link.
