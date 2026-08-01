idf_component_register(SRCS "main.c" "gatt_svr.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_hosted nvs_flash bt esp_hosted_examples_common
                    PRIV_REQUIRES esp_wifi esp_event esp_netif)
