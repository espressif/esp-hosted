idf_component_register(
    SRCS         "init.c" "sta.c" "scan.c" "ap.c"
    INCLUDE_DIRS "include"
    REQUIRES     esp_event esp_netif esp_wifi nvs_flash
    PRIV_REQUIRES esp_hosted)
