# SPDX-License-Identifier: Apache-2.0
#
# esp_hosted_bt — one adapter binding a BT host stack (NimBLE / Bluedroid /
# custom) to eh_host_feat_bt's HCI byte-pipe. Not a standalone IDF component:
# an internal STATIC lib of esp_hosted, add_subdirectory'd from the root
# CMakeLists and gated by the BT feature. The stack is chosen by the IDF BT
# Kconfig (BT_NIMBLE_ENABLED / BT_BLUEDROID_ENABLED); neither => the custom path.
# feat_bt stays stack-agnostic.

if(NOT CONFIG_ESP_HOSTED_HOST_FEAT_BT)
    return()
endif()

add_library(esp_hosted_bt STATIC
    "src/esp_hosted_bt.c"
    "src/esp_hosted_bt_nimble.c"
    "src/esp_hosted_bt_bluedroid.c"
)

target_include_directories(esp_hosted_bt PUBLIC "include")

# Native feat headers (controller + HCI byte-pipe + MAC helper). Below compat.
file(GLOB _eh_bt_adapter_incs LIST_DIRECTORIES true
    "${EH_PATH}/host/features/eh_host_feat_*/include")
target_include_directories(esp_hosted_bt PRIVATE ${_eh_bt_adapter_incs})

target_link_libraries(esp_hosted_bt
    PUBLIC
        eh_host_feat_bt          # controller RPC + eh_host_bt_mcu_hci_register
    PRIVATE
        eh_host_config           # sdkconfig.h + master config header
)

# idf::bt only when a built-in stack is compiled (the nimble/bluedroid glue
# needs its headers/symbols); a custom build (neither stack) pulls no IDF stack.
if(CONFIG_BT_NIMBLE_ENABLED OR CONFIG_BT_BLUEDROID_ENABLED)
    target_link_libraries(esp_hosted_bt PRIVATE idf::bt)
endif()

# Force the TU into the final link: NimBLE's strong ble_transport_* overrides,
# and the public esp_hosted_bt_stack_setup entry point.
target_link_options(esp_hosted_bt INTERFACE
    "-Wl,--undefined=esp_hosted_bt_stack_setup")

# Self-attach to the host umbrella.
if(TARGET eh_host)
    target_link_libraries(eh_host INTERFACE esp_hosted_bt)
endif()
