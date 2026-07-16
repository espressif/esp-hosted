# SPDX-License-Identifier: Apache-2.0

if(NOT CONFIG_ESP_HOSTED_CP_FEAT_RPC_EXT_V1)
    return()
endif()

add_library(eh_cp_feat_rpc_ext_v1 STATIC
    src/eh_cp_feat_rpc_ext_v1_system_reqs.c
    src/eh_cp_feat_rpc_ext_v1_wifi_reqs.c
    src/eh_cp_feat_rpc_ext_v1_ota_reqs.c
    src/eh_cp_feat_rpc_ext_v1_core.c
    src/eh_cp_feat_rpc_ext_v1_event_handler.c
    src/eh_cp_feat_rpc_ext_v1_protobuf_req_hook.c
    src/eh_cp_feat_rpc_ext_v1_protocomm_hook.c
    src/eh_cp_feat_rpc_ext_v1_protobuf_evt_hook.c
)

target_include_directories(eh_cp_feat_rpc_ext_v1
    PUBLIC  "include"
    PRIVATE "private_include"
)

set(required_components
    idf::esp_event
    idf::app_update
)
if(CONFIG_ESP_HOSTED_CP_FEAT_WIFI_READY)
    list(APPEND required_components idf::esp_wifi)
endif()
if(CONFIG_ESP_WIFI_REMOTE_EAP_ENABLED OR CONFIG_ESP_WIFI_ENTERPRISE_SUPPORT)
    list(APPEND required_components idf::wpa_supplicant)
endif()

target_link_libraries(eh_cp_feat_rpc_ext_v1
    PUBLIC
        eh_cp_core
        eh_cp_feat_rpc
        eh_common
        eh_proto_v1
        ${required_components}
)

if(CONFIG_ESP_HOSTED_CP_FEAT_NW_SPLIT_READY)
    target_link_libraries(eh_cp_feat_rpc_ext_v1 PUBLIC eh_cp_feat_nw_split)
endif()
if(CONFIG_ESP_HOSTED_CP_FEAT_SYSTEM_READY)
    target_link_libraries(eh_cp_feat_rpc_ext_v1 PUBLIC eh_cp_feat_system)
endif()
if(CONFIG_ESP_HOSTED_CP_FEAT_PEER_DATA_READY)
    target_link_libraries(eh_cp_feat_rpc_ext_v1 PUBLIC eh_cp_feat_peer_data)
    target_include_directories(eh_cp_feat_rpc_ext_v1 PRIVATE
        $<TARGET_PROPERTY:eh_cp_feat_peer_data,INTERFACE_INCLUDE_DIRECTORIES>)
endif()
if(CONFIG_ESP_HOSTED_CP_FEAT_WIFI_READY)
    target_link_libraries(eh_cp_feat_rpc_ext_v1 PUBLIC eh_cp_feat_wifi)
endif()
if(CONFIG_ESP_HOSTED_CP_FEAT_BT_READY)
    target_link_libraries(eh_cp_feat_rpc_ext_v1 PUBLIC eh_cp_feat_bt)
endif()

if(CONFIG_ESP_HOSTED_CP_FEAT_RPC_EXT_V1_AUTO_INIT)
    target_link_options(eh_cp_feat_rpc_ext_v1 INTERFACE
        "-Wl,--undefined=eh_cp_feat_rpc_ext_v1_init")
endif()

if(TARGET eh_cp)
    target_link_libraries(eh_cp INTERFACE eh_cp_feat_rpc_ext_v1)
endif()
