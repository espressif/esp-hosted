#
# SPDX-License-Identifier: Apache-2.0
#
if(CONFIG_ESP_HOSTED)
zephyr_library_named(esp_hosted_glue)
zephyr_interface_library_named(esp_hosted_glue_interface)

add_subdirectory(${ZEPHYR_CURRENT_MODULE_DIR} esp-hosted)

zephyr_include_directories_ifdef(CONFIG_ESP_HOSTED
    csk/app/include
    csk/common
    csk/port/include
    csk/driver/netif
    csk/driver/network
    csk/driver/serial
    csk/driver/spi
    csk/driver/gpio
    csk/driver/vhci
    csk/os/include
)

zephyr_library_sources_ifdef(CONFIG_ESP_HOSTED
    csk/app/src/hosted.c
    csk/common/common.c
    csk/common/util.c
    csk/port/src/platform_wrapper.c
    csk/driver/network/netdev_api.c
    csk/driver/network/netdev_stub.c
    csk/driver/serial/serial_ll_if.c
    csk/driver/spi/hosted_spi.c
    csk/driver/spi/spi_drv.c
    csk/driver/gpio/hosted_gpio.c
    csk/driver/vhci/vhci_if.c
    csk/os/src/csk_os_memory.c
    csk/os/src/csk_os_queue.c
    csk/os/src/csk_os_semaphore.c
    csk/os/src/csk_os_mutex.c
    csk/os/src/csk_os_thread.c
    csk/os/src/csk_os_timer.c
)

endif()
