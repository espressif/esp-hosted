set(srcs "main.c"
         "gatt_svr.c"
         "uart_driver.c")

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "."
                       REQUIRES bt esp_driver_uart nvs_flash nimble_peripheral_utils)
