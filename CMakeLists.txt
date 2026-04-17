cmake_minimum_required(VERSION 3.16)

set(esp_hosted_public_includes
    "modules/coprocessor/eh_cp_core/include"
    "modules/coprocessor/eh_cp_transport/include"
    "modules/coprocessor/features/eh_cp_feat_rpc/include"
    "modules/coprocessor/features/eh_cp_feat_rpc_ext_linux/include"
    "modules/coprocessor/features/eh_cp_feat_rpc_ext_mcu/include"
    "modules/coprocessor/features/eh_cp_feat_host_ps/include"
    "modules/coprocessor/features/eh_cp_feat_nw_split/include"
	"modules/coprocessor/features/eh_cp_feat_wifi/include"
    "modules/coprocessor/features/eh_cp_feat_cli/include"
    "modules/coprocessor/features/eh_cp_feat_peer_data/include"
    "modules/common/eh_common/include"
    "modules/common/eh_frame/include"
    "modules/common/eh_mempool/include"
)

idf_component_register(NAME esp_hosted INCLUDE_DIRS ${esp_hosted_public_includes})

if(CONFIG_ESP_HOSTED_CP_FEAT_WIFI_READY)
    # Ensure esp_wifi_connect is wrapped at the final link for all apps.
    idf_build_set_property(LINK_OPTIONS "-Wl,--wrap=esp_wifi_connect" APPEND)
endif()

if(CONFIG_ESP_HOSTED)
    add_subdirectory(modules/common ${CMAKE_BINARY_DIR}/common)
endif()

if(CONFIG_ESP_HOSTED_CP)
    add_subdirectory(modules/coprocessor ${CMAKE_BINARY_DIR}/coprocessor)
endif()

if(CONFIG_ESP_HOSTED_HOST)
    add_subdirectory(modules/host ${CMAKE_BINARY_DIR}/host)
endif()
