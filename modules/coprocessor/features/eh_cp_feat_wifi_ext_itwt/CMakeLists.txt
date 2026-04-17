# eh_cp_feat_wifi_ext_itwt — WiFi iTWT feature extension

if(CONFIG_ESP_HOSTED_CP_FEAT_WIFI_EXT_ITWT_READY)
    set(srcs
        "src/eh_cp_feat_wifi_ext_itwt.c"
        "src/eh_cp_feat_wifi_ext_itwt_event_publisher.c"
    )

    add_library(eh_cp_feat_wifi_ext_itwt STATIC ${srcs})
    target_include_directories(eh_cp_feat_wifi_ext_itwt
        PUBLIC
            "include"
    )
    target_link_libraries(eh_cp_feat_wifi_ext_itwt
        PUBLIC
            eh_cp_core
            eh_common
            idf::esp_event
            idf::esp_wifi
    )
else()
    add_library(eh_cp_feat_wifi_ext_itwt INTERFACE)
    target_include_directories(eh_cp_feat_wifi_ext_itwt
        INTERFACE
            "include"
    )
endif()
