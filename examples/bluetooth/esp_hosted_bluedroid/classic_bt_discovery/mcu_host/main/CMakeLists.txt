idf_component_register(
    SRCS         "classic_bt_discovery.c"
    INCLUDE_DIRS "."
    REQUIRES     esp_hosted nvs_flash bt
)
