idf_component_register(
    SRCS         "ble_advertise_minimal.c"
    INCLUDE_DIRS "."
    REQUIRES     esp_hosted_hci_bluedroid nvs_flash bt
)
