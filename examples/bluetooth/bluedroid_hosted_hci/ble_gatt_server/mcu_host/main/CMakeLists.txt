idf_component_register(
    SRCS         "ble_gatt_server.c"
    INCLUDE_DIRS "."
    REQUIRES     esp_hosted_hci_bluedroid nvs_flash bt
)
