# SPDX-License-Identifier: Apache-2.0
# eh_cp_feat_rpc_ext_mcu — MCU RPC adapter
#

if(NOT CONFIG_ESP_HOSTED_CP_FEAT_RPC_MCU_READY)
    return()
endif()

set(srcs
    src/eh_cp_feat_rpc_ext_mcu.c
    src/eh_cp_feat_rpc_ext_mcu_event_handler.c
    src/eh_cp_feat_rpc_ext_mcu_hooks.c
    src/eh_cp_feat_rpc_ext_mcu_dispatcher_evt.c
    src/eh_cp_feat_rpc_ext_mcu_handler_evt.c
    src/eh_cp_feat_rpc_ext_mcu_dispatcher_req.c
)

if(CONFIG_ESP_HOSTED_CP_FEAT_WIFI_READY)
    list(APPEND srcs src/eh_cp_feat_rpc_ext_mcu_handler_req_wifi.c)
endif()

if(CONFIG_ESP_HOSTED_CP_FEAT_BT_READY)
    list(APPEND srcs src/eh_cp_feat_rpc_ext_mcu_handler_req_bt.c)
endif()

if(CONFIG_ESP_HOSTED_CP_FEAT_GPIO_EXP_READY)
    list(APPEND srcs src/eh_cp_feat_rpc_ext_mcu_handler_req_gpio.c)
endif()

if(CONFIG_ESP_HOSTED_CP_FEAT_MEM_MONITOR_READY)
    list(APPEND srcs src/eh_cp_feat_rpc_ext_mcu_handler_req_mem_monitor.c)
endif()

if(CONFIG_ESP_HOSTED_CP_FEAT_EXT_COEX_READY)
    list(APPEND srcs src/eh_cp_feat_rpc_ext_mcu_handler_req_ext_coex.c)
endif()

if(CONFIG_ESP_HOSTED_CP_FEAT_SYSTEM_READY)
    list(APPEND srcs src/eh_cp_feat_rpc_ext_mcu_handler_req_system.c)
endif()

if(CONFIG_ESP_HOSTED_CP_FEAT_NW_SPLIT_READY)
    list(APPEND srcs src/eh_cp_feat_rpc_ext_mcu_handler_req_nw_split.c)
endif()

if(CONFIG_ESP_HOSTED_CP_FEAT_PEER_DATA_TRANSFER_READY)
    list(APPEND srcs src/eh_cp_feat_rpc_ext_mcu_handler_req_peer_data.c)
endif()

add_library(eh_cp_feat_rpc_ext_mcu STATIC ${srcs})

target_include_directories(eh_cp_feat_rpc_ext_mcu
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

if(CONFIG_ESP_HOSTED_CP_FEAT_EXT_COEX_READY)
    list(APPEND required_components idf::esp_coex)
endif()

if(CONFIG_ESP_HOSTED_CP_FEAT_PEER_DATA_TRANSFER_READY)
    list(APPEND required_components eh_cp_feat_peer_data)
endif()


if(CONFIG_ESP_WIFI_REMOTE_EAP_ENABLED OR CONFIG_ESP_WIFI_ENTERPRISE_SUPPORT)
    list(APPEND required_components idf::wpa_supplicant)
endif()

target_link_libraries(eh_cp_feat_rpc_ext_mcu
	PUBLIC
		eh_cp_core
		eh_cp_feat_rpc
		eh_common
		eh_proto_mcu_v1
		${required_components}
)

target_link_options(eh_cp_feat_rpc_ext_mcu INTERFACE
    "-Wl,--undefined=eh_cp_feat_rpc_ext_mcu_init")
if(CONFIG_ESP_HOSTED_CP_FEAT_WIFI_READY)
    target_link_options(eh_cp_feat_rpc_ext_mcu INTERFACE
        "-Wl,--wrap=esp_wifi_init")
endif()
