# SPDX-License-Identifier: Apache-2.0
#
# Bluedroid hosted-HCI port — stack glue on top of eh_host_feat_bt's HCI
# byte-pipe. Not a standalone IDF component: an internal STATIC lib of the
# esp_hosted component, add_subdirectory'd from the root CMakeLists and
# gated by its Kconfig. feat_bt stays stack-agnostic.

if(NOT CONFIG_ESP_HOSTED_HOST_BT_PORT_BLUEDROID)
    return()
endif()

add_library(eh_host_bluedroid STATIC
    "src/eh_host_bluedroid.c"
)

target_include_directories(eh_host_bluedroid PUBLIC "include")

# Mirror the esp_hosted component's own header visibility (compat + feature
# headers) so this sub-lib resolves esp_hosted.h and its include chain, just
# like a REQUIRES esp_hosted consumer.
file(GLOB _eh_bluedroid_feat_incs LIST_DIRECTORIES true
    "${EH_PATH}/host/features/eh_host_feat_*/include")
target_include_directories(eh_host_bluedroid PRIVATE
    "${EH_PATH}/host/compat/include" ${_eh_bluedroid_feat_incs})

target_link_libraries(eh_host_bluedroid
    PUBLIC
        eh_host_feat_bt          # HCI byte-pipe: eh_host_bt_mcu_hci_register
    PRIVATE
        eh_host_config           # sdkconfig.h + master config header
        esp_hosted_compat        # esp_hosted_connect_to_slave (compat symbol)
        idf::bt                  # Bluedroid host stack + esp_bluedroid_hci.h
)

# Force our TU into the final link so the auto-init descriptor (when enabled)
# lands even without a direct reference. (Bluedroid ops are referenced by the
# app, but keep this symmetric with the nimble port.)
target_link_options(eh_host_bluedroid INTERFACE
    "-Wl,--undefined=eh_host_bluedroid_init")

# Self-attach to the host umbrella.
if(TARGET eh_host)
    target_link_libraries(eh_host INTERFACE eh_host_bluedroid)
endif()
