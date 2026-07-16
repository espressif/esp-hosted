if(CONFIG_EH_USE_LEGACY_API)
    set(_main_src "legacy/main.c")
else()
    set(_main_src "main.c")
endif()

set(srcs "${_main_src}" "station_example.c")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS "."
                    REQUIRES esp_hosted esp_event
                    PRIV_REQUIRES nvs_flash esp_wifi esp_netif)
