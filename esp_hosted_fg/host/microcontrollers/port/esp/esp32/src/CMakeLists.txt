# app & driver files
#set(COMPONENT_SRCS "../../../../app/app_main.c" "../../../../app/app_main_api.c" "../../../../app/data/arp_server_stub.c" "../../../../app/control/control.c" "../../../../app/control/control_utils.c" "../../../../drivers/transport/transport_drv.c"  "../../../../drivers/network/netdev_api.c" "../../../../drivers/network/netdev_stub.c" "../../../../drivers/serial/serial_ll_if.c" "../../../../utils/common.c" "../../../../utils/util.c" "../../../../utils/stats.c" "os_wrapper.c" "../../../../drivers/serial/serial_drv.c")
set(COMPONENT_SRCS "../../hosted_app_main.c" "../../../../app/control/control_utils.c" "../../../../drivers/transport/transport_drv.c"  "../../../../drivers/network/netdev_api.c" "../../../../drivers/network/netdev_stub.c" "../../../../drivers/serial/serial_ll_if.c" "../../../../utils/common.c" "../../../../utils/util.c" "../../../../utils/stats.c" "os_wrapper.c" "../../../../drivers/serial/serial_drv.c" "../../../../drivers/wifi/src/wifi_drv.c")

set(COMPONENT_ADD_INCLUDEDIRS "." "../../../../app" "../../../../app/data" "../../../../app/control" "../../../../drivers/transport" "../../../../drivers/transport/spi" "../../../../drivers/transport/sdio" "../../../../drivers/netif" "../../../../drivers/network" "../../../../drivers/serial" "../../../../utils" "../../../../../../common/include" "../include" "." "../../.." "../../../../drivers/wifi/include")

#components
# control path lib files
list(APPEND COMPONENT_SRCS  "../../../../../common/control_lib/src/ctrl_core.c" "../../../../../common/control_lib/src/ctrl_api.c" "../../../../../common/virtual_serial_if/src/serial_if.c")
list(APPEND COMPONENT_ADD_INCLUDEDIRS "../../../../../common/control_lib/src/include" "../../../../../common/control_lib/include" "../../../../../common/virtual_serial_if/include")
# esp_queue
list(APPEND COMPONENT_SRCS "../../../../../common/esp_queue/src/esp_queue.c")
list(APPEND COMPONENT_ADD_INCLUDEDIRS "../../../../../common/esp_queue/include")
# mempool
list(APPEND COMPONENT_SRCS "../../../../../common/mempool/src/mempool.c")
list(APPEND COMPONENT_ADD_INCLUDEDIRS "../../../../../common/mempool/include" )

# protobuf files
list(APPEND COMPONENT_SRCS "../../../../../../common/protobuf-c/protobuf-c/protobuf-c.c" "../../../../../../common/esp_hosted_config.pb-c.c" )
list(APPEND COMPONENT_ADD_INCLUDEDIRS "../../../../../../common/protobuf-c" )

# transport files
if(CONFIG_ESP_SDIO_HOST_INTERFACE)
    list(APPEND COMPONENT_SRCS "../../../../drivers/transport/sdio/sdio_ll.c" "../../../../drivers/transport/sdio/sdio_api.c" "../../../../drivers/transport/sdio/sdio_host.c" "../../../../drivers/transport/sdio/sdio_drv.c")
else(CONFIG_ESP_SPI_HOST_INTERFACE)
	list(APPEND COMPONENT_SRCS "../../../../drivers/transport/spi/spi_drv.c")
endif()


register_component()

# Add needed components to include path
idf_component_get_property(freertos_dir freertos COMPONENT_DIR)
target_include_directories(${COMPONENT_LIB} PRIVATE "${freertos_dir}/include/freertos")

idf_component_get_property(driver_dir driver COMPONENT_DIR)
#target_include_directories(${COMPONENT_LIB} PRIVATE "${driver_dir}/include")
target_include_directories(${COMPONENT_LIB} PRIVATE "${driver_dir}/include" "${driver_dir}/spi/include" "${driver_dir}/gpio/include")

idf_component_get_property(esp_timer_dir esp_timer COMPONENT_DIR)
target_include_directories(${COMPONENT_LIB} PRIVATE "${esp_timer_dir}/include")

idf_component_get_property(esp_event_dir esp_event COMPONENT_DIR)
target_include_directories(${COMPONENT_LIB} PRIVATE "${esp_event_dir}/include")

idf_component_get_property(nvs_flash_dir nvs_flash COMPONENT_DIR)
target_include_directories(${COMPONENT_LIB} PRIVATE "${nvs_flash_dir}/include")

idf_component_get_property(esp_partition_dir esp_partition COMPONENT_DIR)
target_include_directories(${COMPONENT_LIB} PRIVATE "${esp_partition_dir}/include")
