# SPDX-License-Identifier: Apache-2.0
if(CONFIG_EH_USE_LEGACY_API)
    set(_main_src "legacy/main.c")
else()
    set(_main_src "main.c")
endif()

idf_component_register(
    SRCS "${_main_src}"
    INCLUDE_DIRS "."
    REQUIRES esp_hosted
    PRIV_REQUIRES nvs_flash esp_event)
