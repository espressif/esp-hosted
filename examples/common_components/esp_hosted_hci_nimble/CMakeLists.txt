# SPDX-License-Identifier: Apache-2.0
#
# Bridge component: ESP-Hosted <-> NimBLE HCI transport.
# Provides strong overrides for NimBLE's weak transport hooks; the
# ESP-Hosted core component knows nothing about NimBLE.

idf_component_register(
    SRCS         "src/esp_hosted_hci_nimble.c"
    INCLUDE_DIRS "include"
    REQUIRES     esp_hosted bt
)
