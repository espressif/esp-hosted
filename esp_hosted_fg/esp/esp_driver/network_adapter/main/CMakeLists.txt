set(common_dir "../../../../common")

set(COMPONENT_SRCS
    "slave_control.c"
    "${common_dir}/esp_hosted_config.pb-c.c"
    "${common_dir}/utils/esp_hosted_cli.c"
    "protocomm_pserial.c"
    "esp_hosted_coprocessor.c"
    "slave_bt.c"
    "mempool.c"
    "stats.c"
    "mempool_ll.c"
    "host_power_save.c"
    "lwip_filter.c"
)

if(CONFIG_ESP_HOSTED_COPROCESSOR_EXAMPLE_MQTT)
    list(APPEND COMPONENT_SRCS mqtt_example.c)
endif()

if(CONFIG_ESP_HOSTED_COPROCESSOR_EXAMPLE_HTTP_CLIENT)
    list(APPEND COMPONENT_SRCS http_req.c)
endif()

set(COMPONENT_ADD_INCLUDEDIRS
    "."
    "${common_dir}/include"
    "${common_dir}/utils"
)

if(CONFIG_ESP_SDIO_HOST_INTERFACE)
    list(APPEND COMPONENT_SRCS sdio_slave_api.c)
else(CONFIG_ESP_SPI_HOST_INTERFACE)
    list(APPEND COMPONENT_SRCS spi_slave_api.c)
endif()

register_component()
target_compile_definitions(${COMPONENT_LIB} PRIVATE)

# Add directory of protocomm_priv.h to include paths
idf_component_get_property(protocomm_dir protocomm COMPONENT_DIR)
target_include_directories(${COMPONENT_LIB} PRIVATE "${protocomm_dir}/src/common")
