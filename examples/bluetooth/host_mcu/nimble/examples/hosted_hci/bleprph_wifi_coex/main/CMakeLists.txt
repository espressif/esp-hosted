idf_component_register(SRCS "main.c" "gatt_svr.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_hosted_hci_nimble nvs_flash bt eh_example_common
                    PRIV_REQUIRES esp_wifi esp_event esp_netif)
