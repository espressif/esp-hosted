if(CONFIG_ESP_HOSTED)

zephyr_library()
zephyr_include_directories(
        common/include
        host/components/include
        host/control_lib/include
        host/control_lib/src/include
        host/virtual_serial_if/include
)

zephyr_library_sources(
        common/esp_hosted_config.pb-c.c
        host/components/src/esp_queue.c
        host/control_lib/src/ctrl_api.c
        host/control_lib/src/ctrl_core.c
        host/virtual_serial_if/src/serial_if.c
)

endif() #CONFIG_ESP_HOSTED
