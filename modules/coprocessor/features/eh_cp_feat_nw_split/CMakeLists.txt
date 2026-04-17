# eh_cp_feat_nw_split — network split extension

# Build gate: only compile this component when explicitly enabled.
if(NOT CONFIG_ESP_HOSTED_CP_FEAT_NW_SPLIT)
    return()
endif()

add_library(eh_cp_feat_nw_split STATIC
    "src/eh_cp_feat_nw_split.c"
)

target_include_directories(eh_cp_feat_nw_split
    PUBLIC
        "include"
        "."
)

set(reqs eh_cp_core eh_common)
set(priv_reqs idf::esp_timer idf::esp_wifi)

if(CONFIG_ESP_HOSTED_CP_FEAT_NW_SPLIT_READY)
    list(APPEND reqs idf::esp_wifi idf::esp_netif idf::esp_event idf::lwip)
endif()

target_link_libraries(eh_cp_feat_nw_split
    PUBLIC
        ${reqs}
        ${priv_reqs}
)

if(CONFIG_ESP_HOSTED_CP_FEAT_HOST_PS_READY AND TARGET eh_cp_feat_host_ps)
    target_link_libraries(eh_cp_feat_nw_split PUBLIC eh_cp_feat_host_ps)
endif()

# Wrap esp_wifi_internal_reg_rxcb so nw_split can suppress STA rxcb overrides
# from ESP-IDF default handlers (wifi_default.c) and WiFi feature event publisher.
target_link_libraries(eh_cp_feat_nw_split INTERFACE
    -Wl,--wrap=esp_wifi_internal_reg_rxcb
)

# Configure LWIP hooks for network split mode
if(TARGET lwip)
    target_include_directories(lwip PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
    target_compile_definitions(lwip PRIVATE
        "ESP_IDF_LWIP_HOOK_FILENAME=\"eh_cp_feat_nw_split_lwip_hook.h\""
    )
endif()
