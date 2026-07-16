# eh_host_feat_wifi_ext_itwt — host Wi-Fi extension typed wrappers.
# SPDX-License-Identifier: Apache-2.0

if(NOT CONFIG_ESP_HOSTED_HOST_FEAT_WIFI_EXT_ITWT_READY)
    return()
endif()

add_library(eh_host_feat_wifi_ext_itwt STATIC
    "src/eh_host_feat_wifi_ext_itwt.c"
)

target_include_directories(eh_host_feat_wifi_ext_itwt
    PUBLIC "include"
    PRIVATE "src"
)

target_link_libraries(eh_host_feat_wifi_ext_itwt
    PUBLIC
        eh_host_port_api
        eh_common
    PRIVATE
        eh_host_config                # eh_host_master_config.h
)

# Force-pull this feature's TU so its EH_HOST_FEAT_REGISTER descriptor
# lands in the eh_host_feat_descs section. INTERFACE option propagates
# through the esp_hosted INTERFACE chain to the final executable.
if(CONFIG_ESP_HOSTED_HOST_FEAT_WIFI_EXT_ITWT_AUTO_INIT)
    target_link_options(eh_host_feat_wifi_ext_itwt INTERFACE
        "-Wl,--undefined=eh_host_feat_wifi_ext_itwt_init")
endif()


target_link_libraries(eh_host_feat_wifi_ext_itwt PRIVATE eh_host_autoinit)  # eh_host_auto_init.h (EH_HOST_FEAT_REGISTER)

target_sources(eh_host_feat_wifi_ext_itwt PRIVATE
    "src/eh_host_wifi_itwt.c"
    "src/eh_host_feat_wifi_ext_itwt_v2_req.c"
    "src/eh_host_feat_wifi_ext_itwt_v2_resp_evt.c"
)
target_link_libraries(eh_host_feat_wifi_ext_itwt PRIVATE
    eh_host_feat_rpc_ext_v2  # eh_rpc_ctrl_cmd_t / types / api_common
    eh_host_feat_rpc          # request_sync
    eh_proto_v2              # pb-c types + RPC_ID__* constants
)
