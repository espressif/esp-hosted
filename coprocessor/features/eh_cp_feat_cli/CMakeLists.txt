
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
        eh_cp_feat_debug
        idf::console
        idf::lwip
        idf::esp_wifi
        idf::esp_timer
)

if(CONFIG_ESP_HOSTED_CP_FEAT_HOST_PS_READY)
    target_link_libraries(eh_cp_feat_cli PUBLIC eh_cp_feat_host_ps)
endif()

if(CONFIG_ESP_HOSTED_CP_FEAT_CLI_AUTO_INIT)
    target_link_options(eh_cp_feat_cli INTERFACE
        "-Wl,--undefined=eh_cp_feat_cli_register_commands")
endif()

if(TARGET eh_cp)
    target_link_libraries(eh_cp INTERFACE eh_cp_feat_cli)
endif()
