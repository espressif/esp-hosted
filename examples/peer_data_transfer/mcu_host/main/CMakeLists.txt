if(CONFIG_EH_USE_LEGACY_API)
    set(_main_src "legacy/main.c")
else()
    set(_main_src "main.c")
endif()

idf_component_register(SRCS "${_main_src}"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES esp_wifi nvs_flash esp_event)
