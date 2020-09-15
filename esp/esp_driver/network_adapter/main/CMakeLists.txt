set(COMPONENT_SRCS "slave_commands.c" "../../../../common/esp_hosted_config.pb-c.c" "protocomm_pserial.c" "app_main.c")
set(COMPONENT_ADD_INCLUDEDIRS "." "../../../../common/include")

if(CONFIG_ESP_SDIO_HOST_INTERFACE)
    list(APPEND COMPONENT_SRCS sdio_slave_api.c)
else(CONFIG_ESP_SPI_HOST_INTERFACE)
    list(APPEND COMPONENT_SRCS spi_slave_api.c)
endif()


register_component()
set(PROJECT_VERSION "-DPROJECT_VERSION=0.2")
target_compile_definitions(${COMPONENT_LIB} PRIVATE ${PROJECT_VERSION})
