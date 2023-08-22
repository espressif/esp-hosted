set(COMPONENT_SRCS "slave_control.c" "../../../../common/esp_hosted_config.pb-c.c" "protocomm_pserial.c" "app_main.c" "slave_bt.c" "mempool.c" "stats.c" "mempool_ll.c")
set(COMPONENT_ADD_INCLUDEDIRS "." "../../../../common/include")

if(CONFIG_ESP_SDIO_HOST_INTERFACE)
    list(APPEND COMPONENT_SRCS sdio_slave_api.c)
else(CONFIG_ESP_SPI_HOST_INTERFACE)
    list(APPEND COMPONENT_SRCS spi_slave_api.c)
endif()


register_component()
set(PROJECT_VERSION_MAJOR_1 "-DPROJECT_VERSION_MAJOR_1=0")
set(PROJECT_VERSION_MAJOR_2 "-DPROJECT_VERSION_MAJOR_2=0")
set(PROJECT_VERSION_MINOR "-DPROJECT_VERSION_MINOR=5")
target_compile_definitions(${COMPONENT_LIB} PRIVATE ${PROJECT_VERSION_MAJOR_1} ${PROJECT_VERSION_MAJOR_2} ${PROJECT_VERSION_MINOR})

# Add directory of protocomm_priv.h to include paths
idf_component_get_property(protocomm_dir protocomm COMPONENT_DIR)
target_include_directories(${COMPONENT_LIB} PRIVATE "${protocomm_dir}/src/common")
