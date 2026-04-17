# eh_cp_ext_cli — diagnostic CLI commands extension

set(srcs "src/eh_cp_cli_stub.c")
if(CONFIG_ESP_HOSTED_CP_FEAT_CLI_READY)
    list(APPEND srcs "src/eh_cp_cli.c")
endif()

add_library(eh_cp_feat_cli STATIC ${srcs})

target_include_directories(eh_cp_feat_cli
    PUBLIC
        "include"
)

target_link_libraries(eh_cp_feat_cli
    PUBLIC
        eh_cp_core
        idf::console
        idf::lwip
        idf::esp_wifi
        idf::esp_timer
)

