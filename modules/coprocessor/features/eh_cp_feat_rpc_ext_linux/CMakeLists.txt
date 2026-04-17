# SPDX-License-Identifier: Apache-2.0
# eh_cp_feat_rpc_ext_linux — Linux FG RPC adapter
#
set(req_srcs)
set(core_srcs)
set(protocomm_hook_srcs)

if(CONFIG_ESP_HOSTED_CP_FEAT_RPC_LINUX)
	list(APPEND req_srcs
		src/eh_cp_feat_rpc_ext_linux_system_reqs.c
		src/eh_cp_feat_rpc_ext_linux_wifi_reqs.c
		src/eh_cp_feat_rpc_ext_linux_ota_reqs.c
	)

	list(APPEND core_srcs
		src/eh_cp_feat_rpc_ext_linux_core.c
		src/eh_cp_feat_rpc_ext_linux_event_handler.c
	)

	list(APPEND protocomm_hook_srcs
		src/eh_cp_feat_rpc_ext_linux_protobuf_req_hook.c
		src/eh_cp_feat_rpc_ext_linux_protocomm_hook.c
		src/eh_cp_feat_rpc_ext_linux_protobuf_evt_hook.c
	)
endif()

list(APPEND srcs ${req_srcs} ${core_srcs} ${protocomm_hook_srcs})

if(CONFIG_ESP_HOSTED_CP_FEAT_RPC_LINUX)
	add_library(eh_cp_feat_rpc_ext_linux STATIC ${srcs})
else()
	add_library(eh_cp_feat_rpc_ext_linux INTERFACE)
endif()

target_include_directories(eh_cp_feat_rpc_ext_linux
	PUBLIC
		"include"
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

if(CONFIG_ESP_HOSTED_CP_FEAT_RPC_LINUX)
	target_include_directories(eh_cp_feat_rpc_ext_linux
		PRIVATE
			"private_include"
	)

	target_link_libraries(eh_cp_feat_rpc_ext_linux
		PUBLIC
			eh_cp_core
			eh_cp_feat_rpc
			eh_common
			eh_proto_linux_v1
			${required_components}
	)
endif()

if(CONFIG_ESP_HOSTED_CP_FEAT_NW_SPLIT_READY AND TARGET eh_cp_feat_nw_split)
    target_link_libraries(eh_cp_feat_rpc_ext_linux PUBLIC eh_cp_feat_nw_split)
endif()

if(CONFIG_ESP_HOSTED_CP_FEAT_SYSTEM_READY AND TARGET eh_cp_feat_system)
    target_link_libraries(eh_cp_feat_rpc_ext_linux PUBLIC eh_cp_feat_system)
endif()

if(CONFIG_ESP_HOSTED_CP_FEAT_PEER_DATA_TRANSFER_READY AND TARGET eh_cp_feat_peer_data)
    target_link_libraries(eh_cp_feat_rpc_ext_linux PUBLIC eh_cp_feat_peer_data)
    target_include_directories(eh_cp_feat_rpc_ext_linux PRIVATE
        $<TARGET_PROPERTY:eh_cp_feat_peer_data,INTERFACE_INCLUDE_DIRECTORIES>)
endif()
