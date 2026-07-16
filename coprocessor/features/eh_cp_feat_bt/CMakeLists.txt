if(NOT CONFIG_ESP_HOSTED_CP_FEAT_BT_READY)
    return()
endif()

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
target_link_libraries(eh_cp_feat_bt
    PUBLIC
        eh_cp_core
        eh_cp_transport
        eh_common
        idf::esp_common
        idf::driver
        idf::bt
)
if(CONFIG_ESP_HOSTED_CP_FEAT_BT_AUTO_INIT)
    target_link_options(eh_cp_feat_bt INTERFACE
    "-Wl,--undefined=eh_cp_feat_bt_init")
endif()

if(TARGET eh_cp)
    target_link_libraries(eh_cp INTERFACE eh_cp_feat_bt)
endif()
