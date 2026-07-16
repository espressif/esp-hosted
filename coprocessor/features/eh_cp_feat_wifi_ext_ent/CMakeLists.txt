if(NOT CONFIG_ESP_HOSTED_CP_FEAT_WIFI_EXT_ENT_READY)
    return()
endif()

set(srcs
    "src/eh_cp_feat_wifi_ext_ent.c"
)

add_library(eh_cp_feat_wifi_ext_ent STATIC ${srcs})
target_include_directories(eh_cp_feat_wifi_ext_ent
    PUBLIC
        "include"
)
target_link_libraries(eh_cp_feat_wifi_ext_ent
    PUBLIC
        eh_cp_core
        eh_common
        idf::esp_wifi
        idf::wpa_supplicant
)

if(TARGET eh_cp)
    target_link_libraries(eh_cp INTERFACE eh_cp_feat_wifi_ext_ent)
endif()
