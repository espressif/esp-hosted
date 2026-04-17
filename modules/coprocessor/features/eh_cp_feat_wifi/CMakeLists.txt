# eh_cp_feat_wifi — WiFi feature extension

if(CONFIG_ESP_HOSTED_CP_FEAT_WIFI_READY)
    set(srcs
        "src/eh_cp_feat_wifi.c"
        "src/eh_cp_feat_wifi_event_publisher.c"
    )

    add_library(eh_cp_feat_wifi STATIC ${srcs})
    target_include_directories(eh_cp_feat_wifi
        PUBLIC
            "include"
    )
    target_link_libraries(eh_cp_feat_wifi
        PUBLIC
            eh_cp_core
            eh_common
            idf::esp_common
            idf::esp_event
            idf::esp_wifi
    )
    target_link_options(eh_cp_feat_wifi INTERFACE
        "-Wl,--wrap=esp_wifi_connect")
else()
    add_library(eh_cp_feat_wifi INTERFACE)
    target_include_directories(eh_cp_feat_wifi
        INTERFACE
            "include"
    )
endif()
