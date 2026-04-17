# eh_cp_feat_bt — Bluetooth feature extension

if(CONFIG_ESP_HOSTED_CP_FEAT_BT_READY)
    set(srcs
        "src/eh_cp_feat_bt.c"
        "src/eh_cp_feat_bt_core.c"
    )
    if(CONFIG_IDF_TARGET_ESP32)
        list(APPEND srcs "src/eh_cp_feat_bt_uart_esp32.c")
    elseif(CONFIG_IDF_TARGET_ESP32C3 OR CONFIG_IDF_TARGET_ESP32S3)
        list(APPEND srcs "src/eh_cp_feat_bt_uart_esp32c3_s3.c")
    else()
        list(APPEND srcs "src/eh_cp_feat_bt_uart_esp32xx.c")
    endif()

    add_library(eh_cp_feat_bt STATIC ${srcs})
    target_include_directories(eh_cp_feat_bt
        PUBLIC
            "include"
        PRIVATE
            "include/priv"
    )
    set(bt_deps
        eh_cp_core
        eh_cp_transport
        eh_common
        idf::esp_common
        idf::driver
    )
    if(TARGET idf::bt)
        list(APPEND bt_deps idf::bt)
    endif()

    target_link_libraries(eh_cp_feat_bt
        PUBLIC
            ${bt_deps}
    )
else()
    add_library(eh_cp_feat_bt INTERFACE)
    target_include_directories(eh_cp_feat_bt
        INTERFACE
            "include"
    )
endif()
