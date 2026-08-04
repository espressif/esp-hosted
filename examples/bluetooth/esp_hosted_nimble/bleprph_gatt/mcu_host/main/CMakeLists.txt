idf_component_register(
    SRCS         "bleprph_gatt.c"
    INCLUDE_DIRS "."
    REQUIRES     esp_hosted nvs_flash bt
)
