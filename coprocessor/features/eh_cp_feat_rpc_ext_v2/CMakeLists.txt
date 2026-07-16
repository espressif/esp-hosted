# SPDX-License-Identifier: Apache-2.0

if(NOT CONFIG_ESP_HOSTED_CP_FEAT_RPC_EXT_V2_READY)
    return()
endif()

set(srcs
    src/eh_cp_feat_rpc_ext_v2.c
    src/eh_cp_feat_rpc_ext_v2_event_handler.c
    src/eh_cp_feat_rpc_ext_v2_hooks.c
    src/eh_cp_feat_rpc_ext_v2_dispatcher_evt.c
    src/eh_cp_feat_rpc_ext_v2_handler_evt.c
    src/eh_cp_feat_rpc_ext_v2_dispatcher_req.c
)

if(CONFIG_ESP_HOSTED_CP_FEAT_WIFI_READY)
    list(APPEND srcs src/eh_cp_feat_rpc_ext_v2_handler_req_wifi.c)
endif()

if(CONFIG_ESP_HOSTED_CP_FEAT_BT_READY)
    list(APPEND srcs src/eh_cp_feat_rpc_ext_v2_handler_req_bt.c)
endif()

if(CONFIG_ESP_HOSTED_CP_FEAT_GPIO_EXP_READY)
    list(APPEND srcs src/eh_cp_feat_rpc_ext_v2_handler_req_gpio.c)
endif()

if(CONFIG_ESP_HOSTED_CP_FEAT_DEBUG_MEM_MONITOR_READY)
    list(APPEND srcs src/eh_cp_feat_rpc_ext_v2_handler_req_mem_monitor.c)
endif()

if(CONFIG_ESP_HOSTED_CP_FEAT_CP_EXT_COEX_READY)
    list(APPEND srcs src/eh_cp_feat_rpc_ext_v2_handler_req_cp_ext_coex.c)
endif()

if(CONFIG_ESP_HOSTED_CP_FEAT_SYSTEM_READY)
    list(APPEND srcs src/eh_cp_feat_rpc_ext_v2_handler_req_system.c)
endif()

if(CONFIG_ESP_HOSTED_CP_FEAT_NW_SPLIT_READY)
    list(APPEND srcs src/eh_cp_feat_rpc_ext_v2_handler_req_nw_split.c)
endif()

if(CONFIG_ESP_HOSTED_CP_FEAT_PEER_DATA_READY)
    list(APPEND srcs src/eh_cp_feat_rpc_ext_v2_handler_req_peer_data.c)
endif()

if(CONFIG_ESP_HOSTED_CP_FEAT_OPENTHREAD_READY)
    list(APPEND srcs src/eh_cp_feat_rpc_ext_v2_handler_req_openthread.c)
endif()

add_library(eh_cp_feat_rpc_ext_v2 STATIC ${srcs})

target_include_directories(eh_cp_feat_rpc_ext_v2
	PUBLIC
		"include"
	PRIVATE
		"private_include"
)

set(required_components
    idf::esp_event
    idf::app_update
)

if(CONFIG_ESP_HOSTED_CP_FEAT_NW_SPLIT_READY)
    list(APPEND required_components eh_cp_feat_nw_split)
endif()

if(CONFIG_ESP_HOSTED_CP_FEAT_SYSTEM_READY)
    list(APPEND required_components eh_cp_feat_system)
endif()

if(CONFIG_ESP_HOSTED_CP_FEAT_CP_EXT_COEX_READY)
    list(APPEND required_components idf::esp_coex)
endif()

if(CONFIG_ESP_HOSTED_CP_FEAT_PEER_DATA_READY)
    list(APPEND required_components eh_cp_feat_peer_data)
endif()

if(CONFIG_ESP_WIFI_REMOTE_EAP_ENABLED OR CONFIG_ESP_WIFI_ENTERPRISE_SUPPORT)
    list(APPEND required_components idf::wpa_supplicant)
endif()

target_link_libraries(eh_cp_feat_rpc_ext_v2
	PUBLIC
		eh_cp_core
		eh_cp_feat_rpc
		eh_common
		eh_proto_v2
		${required_components}
)

if(CONFIG_ESP_HOSTED_CP_FEAT_RPC_EXT_V2_AUTO_INIT)
    target_link_options(eh_cp_feat_rpc_ext_v2 INTERFACE
        "-Wl,--undefined=eh_cp_feat_rpc_ext_v2_init")
endif()
if(CONFIG_ESP_HOSTED_CP_FEAT_WIFI_READY)
    target_link_options(eh_cp_feat_rpc_ext_v2 INTERFACE
        "-Wl,--wrap=esp_wifi_init")
endif()

if(CONFIG_ESP_HOSTED_CP_FEAT_WIFI_READY)
    target_link_libraries(eh_cp_feat_rpc_ext_v2 PUBLIC eh_cp_feat_wifi)
endif()
if(CONFIG_ESP_HOSTED_CP_FEAT_WIFI_EXT_ENT_READY)
    target_link_libraries(eh_cp_feat_rpc_ext_v2 PUBLIC eh_cp_feat_wifi_ext_ent)
endif()
if(CONFIG_ESP_HOSTED_CP_FEAT_WIFI_EXT_ITWT_READY)
    target_link_libraries(eh_cp_feat_rpc_ext_v2 PUBLIC eh_cp_feat_wifi_ext_itwt)
endif()
if(CONFIG_ESP_HOSTED_CP_FEAT_WIFI_EXT_DPP_READY)
    target_link_libraries(eh_cp_feat_rpc_ext_v2 PUBLIC eh_cp_feat_wifi_ext_dpp)
endif()
if(CONFIG_ESP_HOSTED_CP_FEAT_BT_READY)
    target_link_libraries(eh_cp_feat_rpc_ext_v2 PUBLIC eh_cp_feat_bt)
endif()
if(CONFIG_ESP_HOSTED_CP_FEAT_GPIO_EXP_READY)
    target_link_libraries(eh_cp_feat_rpc_ext_v2 PUBLIC eh_cp_feat_gpio_exp)
endif()
if(CONFIG_ESP_HOSTED_CP_FEAT_OPENTHREAD_READY)
    target_link_libraries(eh_cp_feat_rpc_ext_v2 PUBLIC eh_cp_feat_openthread)
endif()

if(TARGET eh_cp)
    target_link_libraries(eh_cp INTERFACE eh_cp_feat_rpc_ext_v2)
endif()
