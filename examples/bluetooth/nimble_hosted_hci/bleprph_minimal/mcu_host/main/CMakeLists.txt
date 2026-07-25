idf_component_register(
    SRCS         "bleprph_minimal.c"
    INCLUDE_DIRS "."
    REQUIRES     esp_hosted_hci_nimble nvs_flash bt
)
