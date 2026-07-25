# SPDX-License-Identifier: Apache-2.0
#
# Bridge component: ESP-Hosted <-> Bluedroid HCI driver interface.
# Stack-specific glue lives here; ESP-Hosted core knows nothing about
# Bluedroid.

idf_component_register(
    SRCS         "src/esp_hosted_hci_bluedroid.c"
    INCLUDE_DIRS "include"
    REQUIRES     esp_hosted bt
)
