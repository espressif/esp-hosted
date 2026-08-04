if(NOT CONFIG_ESP_HOSTED_HOST)
    return()
endif()

if(CONFIG_ESP_HOST_WIFI_ENABLED)
	message(FATAL_ERROR
		"esp_hosted and CONFIG_ESP_HOST_WIFI_ENABLED (native host Wi-Fi) "
		"cannot both be enabled; both use the host SDIO bus. Disable one.")
endif()

# Removed in 3.0 — legacy hosted-BT enables. Fail loud on a stale sdkconfig.
if(CONFIG_ESP_HOSTED_ENABLE_BT_BLUEDROID OR CONFIG_ESP_HOSTED_BLUEDROID_HCI_VHCI
   OR CONFIG_ESP_HOSTED_ENABLE_BT_NIMBLE OR CONFIG_ESP_HOSTED_NIMBLE_HCI_VHCI)
	message(FATAL_ERROR
		"Hosted BT Kconfig changed in 3.0. See "
		"https://github.com/espressif/esp-hosted/blob/unified_release/docs/migration.md")
endif()


if(CONFIG_ESP_HOSTED_HOST_FEAT_RPC_READY)
    add_subdirectory(features/eh_host_feat_rpc
        ${CMAKE_BINARY_DIR}/eh_host_feat_rpc)
    target_link_libraries(eh_host INTERFACE eh_host_feat_rpc)
endif()
if(CONFIG_ESP_HOSTED_HOST_FEAT_RPC_EXT_V2_READY)
    add_subdirectory(features/eh_host_feat_rpc_ext_v2
        ${CMAKE_BINARY_DIR}/eh_host_feat_rpc_ext_v2)
    target_link_libraries(eh_host INTERFACE eh_host_feat_rpc_ext_v2)
endif()

foreach(feat
        bt openthread cp_ext_coex gpio_exp peer_data nw_split heartbeat
        system wifi wifi_ext_ent wifi_ext_itwt wifi_ext_dpp mem_monitor
        power_save cli)
    string(TOUPPER "CONFIG_ESP_HOSTED_HOST_FEAT_${feat}_READY" _eh_cfg)
    if(${_eh_cfg})
        add_subdirectory(features/eh_host_feat_${feat}
            ${CMAKE_BINARY_DIR}/eh_host_feat_${feat})
        target_link_libraries(eh_host INTERFACE eh_host_feat_${feat})
    endif()
endforeach()

# Cycle-completing PRIVATE links so feat_X/include/ propagates onto
# rpc_ext_v2's compile via target_link_libraries. Forms a static
# cycle with feat_X PRIVATE → rpc_ext_v2; CMake handles via
# --start-group / --end-group at final link.
if(TARGET eh_host_feat_rpc_ext_v2)
    foreach(_rpc_feat
            bt openthread cp_ext_coex gpio_exp peer_data nw_split heartbeat
            system wifi wifi_ext_ent wifi_ext_itwt wifi_ext_dpp mem_monitor)
        if(TARGET eh_host_feat_${_rpc_feat})
            target_link_libraries(eh_host_feat_rpc_ext_v2
                PRIVATE eh_host_feat_${_rpc_feat})
        endif()
    endforeach()
endif()

if(CONFIG_ESP_HOSTED_HOST_CORE_READY AND CONFIG_ESP_HOSTED_HOST_AUTO_FEAT_INIT)
    add_subdirectory(eh_host_autoinit
        ${CMAKE_BINARY_DIR}/eh_host_autoinit)
    target_link_libraries(eh_host INTERFACE eh_host_autoinit)
endif()

# IO layer
if(CONFIG_ESP_HOSTED_HOST_VSERIAL_LINUX_READY)
    add_subdirectory(linux/eh_host_linux_vserial
        ${CMAKE_BINARY_DIR}/eh_host_linux_vserial)
    target_link_libraries(eh_host INTERFACE eh_host_linux_vserial)
endif()
if(CONFIG_ESP_HOSTED_HOST_VSERIAL_MCU_READY)
    add_subdirectory(mcu/eh_host_mcu_vserial
        ${CMAKE_BINARY_DIR}/eh_host_mcu_vserial)
    target_link_libraries(eh_host INTERFACE eh_host_mcu_vserial)
endif()
if(CONFIG_ESP_HOSTED_HOST_TRANSPORT_MCU_READY)
    add_subdirectory(mcu/eh_host_mcu_transport
        ${CMAKE_BINARY_DIR}/eh_host_mcu_transport)
    target_link_libraries(eh_host INTERFACE eh_host_mcu_transport)
endif()

# Core (depends on vserial above)
if(CONFIG_ESP_HOSTED_HOST_CORE_READY)
    add_subdirectory(eh_host_core ${CMAKE_BINARY_DIR}/eh_host_core)
    target_link_libraries(eh_host INTERFACE eh_host_core)
endif()

# Compat surface — always built.
add_subdirectory(compat ${CMAKE_BINARY_DIR}/esp_hosted_compat)
target_link_libraries(eh_host INTERFACE esp_hosted_compat)

# wifi_remote_glue: strong-body side of esp_wifi_remote_* hook contract.
if(CONFIG_ESP_HOSTED_HOST_FEAT_WIFI_READY)
    add_subdirectory(wifi_remote_glue
        ${CMAKE_BINARY_DIR}/wifi_remote_glue)
    target_link_libraries(eh_host INTERFACE esp_wifi_remote_glue)
endif()

# IDF-library port slots. Each slot's CMakeLists early-returns on ports
# it doesn't apply to; header-only reservations skipped via TARGET test.
foreach(piece utils freertos_shim esp_event esp_netif lwip log esp_system esp_timer nvs_flash console)
    add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../port/idf_components/${piece}
        ${CMAKE_BINARY_DIR}/${piece})
    if(TARGET ${piece})
        target_link_libraries(eh_host INTERFACE ${piece})
    endif()
endforeach()

# IDF vs non-IDF bifurcation and self-attaches to eh_host.
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../port/idf_components/esp_wifi_remote
    ${CMAKE_BINARY_DIR}/esp_wifi_remote)

# Force every host-side lib to attach eh_host_config (sdkconfig.h +
# master config + eh_common). Idempotent for libs that attach themselves.
foreach(_t IN ITEMS
        eh_host_core
        eh_host_autoinit
        esp_hosted_compat eh_host_compat_types
        esp_wifi_remote_glue
        eh_host_mcu_transport eh_host_mcu_vserial
        eh_host_linux_vserial eh_host_so
        eh_host_feat_rpc eh_host_feat_rpc_ext_v2
        eh_host_feat_bt eh_host_feat_openthread
        eh_host_feat_cp_ext_coex eh_host_feat_gpio_exp
        eh_host_feat_peer_data
        eh_host_feat_nw_split eh_host_feat_heartbeat
        eh_host_feat_system eh_host_feat_wifi
        eh_host_feat_wifi_ext_ent eh_host_feat_wifi_ext_itwt
        eh_host_feat_wifi_ext_dpp eh_host_feat_mem_monitor
        eh_host_feat_power_save eh_host_feat_cli)
    if(TARGET ${_t})
        get_target_property(_type ${_t} TYPE)
        # INTERFACE_LIBRARY rejects PRIVATE link; use INTERFACE.
        if(_type STREQUAL "INTERFACE_LIBRARY")
            target_link_libraries(${_t} INTERFACE eh_host_config)
        else()
            target_link_libraries(${_t} PRIVATE eh_host_config)
        endif()
    endif()
endforeach()

# eh_host_feat_descs placement:
#  - ESP_PLATFORM: ldgen maps the section into flash_rodata via the component
#    LDFRAGMENTS (host/eh_host_feat_descs.lf, wired in the root CMakeLists).
#  - non-ESP (linux/posix): the GNU .ld below provides the section bounds.
if(NOT ESP_PLATFORM)
    set(_eh_host_descs_linux_ld "${CMAKE_CURRENT_BINARY_DIR}/eh_host_feat_descs_linux.ld")
    configure_file(
        "${CMAKE_CURRENT_LIST_DIR}/eh_host_feat_descs_linux.ld.in"
        "${_eh_host_descs_linux_ld}"
        @ONLY)
    target_link_options(eh_host INTERFACE "-Wl,-T${_eh_host_descs_linux_ld}")
endif()
