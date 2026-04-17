# eh_cp_feat_wifi_ext_dpp — WiFi DPP feature extension

if(CONFIG_ESP_HOSTED_CP_FEAT_WIFI_EXT_DPP_READY)
    set(srcs
        "src/eh_cp_feat_wifi_ext_dpp.c"
        "src/eh_cp_feat_wifi_ext_dpp_event_publisher.c"
    )

    add_library(eh_cp_feat_wifi_ext_dpp STATIC ${srcs})
    target_include_directories(eh_cp_feat_wifi_ext_dpp
        PUBLIC
            "include"
    )
    target_link_libraries(eh_cp_feat_wifi_ext_dpp
        PUBLIC
            eh_cp_core
            eh_common
            idf::esp_event
            idf::esp_wifi
    )
else()
    add_library(eh_cp_feat_wifi_ext_dpp INTERFACE)
    target_include_directories(eh_cp_feat_wifi_ext_dpp
        INTERFACE
            "include"
    )
endif()
