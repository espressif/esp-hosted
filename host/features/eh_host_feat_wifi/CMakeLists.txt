# SPDX-License-Identifier: Apache-2.0
# eh_host_feat_wifi — host Wi-Fi RPC API + event fan-out.

if(NOT CONFIG_ESP_HOSTED_HOST_FEAT_WIFI_READY)
    return()
endif()

add_library(eh_host_feat_wifi STATIC
    "src/eh_host_feat_wifi.c"
)

target_include_directories(eh_host_feat_wifi
    PUBLIC "include"
    PRIVATE "src"
)

target_link_libraries(eh_host_feat_wifi
    PUBLIC
        eh_host_port_api
        eh_common
    PRIVATE
        eh_host_config
)

# Force-pull TU so EH_HOST_FEAT_REGISTER descriptor lands in feat_descs section.
if(CONFIG_ESP_HOSTED_HOST_FEAT_WIFI_AUTO_INIT)
    target_link_options(eh_host_feat_wifi INTERFACE
        "-Wl,--undefined=eh_host_feat_wifi_init")
endif()


target_link_libraries(eh_host_feat_wifi PRIVATE eh_host_autoinit)

target_sources(eh_host_feat_wifi PRIVATE
    "src/eh_host_wifi.c"
    "src/eh_host_feat_wifi_v2_req.c"
    "src/eh_host_feat_wifi_v2_resp_evt.c"
)
target_link_libraries(eh_host_feat_wifi PRIVATE
    eh_host_feat_rpc_ext_v2
    eh_host_feat_rpc
    eh_proto_v2
)
