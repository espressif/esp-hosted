idf_component_register(
    SRCS         "main.c"
    INCLUDE_DIRS "."
    REQUIRES     esp_hosted_hci_bluedroid nvs_flash bt
)
