# Simple component selection based on Kconfig
if(CONFIG_EH_USE_LEGACY_API)
    set(_main_src "legacy/main.c")
else()
    set(_main_src "main.c")
endif()

# main.c #if-selects the header of the chosen OTA method's component. Kconfig
# values are NOT available during component-requirements expansion, so require
# all three method components unconditionally — main.c picks the one to use.
idf_component_register(SRCS "${_main_src}"
                       INCLUDE_DIRS "."
                       REQUIRES esp_hosted console esp_timer
                       PRIV_REQUIRES esp_app_format ota_https ota_littlefs ota_partition)
