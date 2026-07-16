
set(common_shared_inc
    "${CMAKE_CURRENT_LIST_DIR}/../../common/eh_common/include")

set(CORE_SRCS
    "src/eh_cp_core.c"
    "src/eh_cp_registries.c"
    "src/esp_hosted_init.c"
    "src/eh_cp_idem_wraps.c"
)
set(PROTOCOL_SRCS
    "src/eh_cp_event_base.c"
)
set(UTIL_SRCS
    "src/eh_cp_utils.c"
    "src/eh_cp_transport_test.c"
)
add_library(eh_cp_core STATIC
    ${CORE_SRCS}
    ${PROTOCOL_SRCS}
    ${UTIL_SRCS}
)

target_include_directories(eh_cp_core
    PUBLIC
        "include"
        "${common_shared_inc}"
)

set(cp_core_requires
    idf::esp_event
    idf::driver
    idf::esp_timer
    idf::esp_common
    eh_cp_transport
    eh_common
    eh_frame
)

if(CONFIG_ESP_WIFI_ENABLED)
    list(APPEND cp_core_requires idf::esp_wifi)
endif()

target_link_libraries(eh_cp_core
    PUBLIC
        ${cp_core_requires}
        eh_tlv
)

if(CONFIG_ESP_HOSTED_CP_FEAT_RPC_READY)
    target_link_libraries(eh_cp_core PUBLIC eh_cp_feat_rpc)
endif()
if(CONFIG_ESP_HOSTED_CP_FEAT_HOST_PS_READY)
    target_link_libraries(eh_cp_core PUBLIC eh_cp_feat_host_ps)
endif()
if(CONFIG_ESP_HOSTED_CP_FEAT_BT_READY)
    target_link_libraries(eh_cp_core PUBLIC eh_cp_feat_bt)
endif()

if(CONFIG_ESP_HOSTED_AUTO_CALL_INIT_BEFORE_APP_MAIN)
    target_link_options(eh_cp_core INTERFACE
        "-Wl,--undefined=eh_cp_auto_init_ctor")
endif()

target_link_options(eh_cp_core INTERFACE
    "LINKER:--wrap=esp_event_loop_create_default"
    "LINKER:--wrap=esp_netif_init"
    "LINKER:--undefined=__wrap_esp_event_loop_create_default"
    "LINKER:--undefined=__wrap_esp_netif_init"
)

if(TARGET eh_cp)
    target_link_libraries(eh_cp INTERFACE eh_cp_core)
    target_include_directories(eh_cp INTERFACE
        "${CMAKE_CURRENT_LIST_DIR}/include")
endif()
