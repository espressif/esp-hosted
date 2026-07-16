# SPDX-License-Identifier: Apache-2.0
idf_component_register(
    SRCS "host_gpio_expander_example.c"
    INCLUDE_DIRS "."
    REQUIRES esp_hosted
    PRIV_REQUIRES nvs_flash esp_event)
