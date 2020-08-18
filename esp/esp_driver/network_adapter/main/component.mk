#
# Main component makefile.
#
# This Makefile can be left empty. By default, it will take the sources in the 
# src/ directory, compile them and link them into lib(subdirectory_name).a 
# in the build directory. This behaviour is entirely configurable,
# please read the ESP-IDF documents if you need to do this.
#
$(call compile_only_if,$(CONFIG_ESP_SDIO_HOST_INTERFACE),sdio_slave_api.o)
$(call compile_only_if,$(CONFIG_ESP_SPI_HOST_INTERFACE),spi_slave_api.o)
