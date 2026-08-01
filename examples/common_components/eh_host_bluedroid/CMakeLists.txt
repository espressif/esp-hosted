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

# This port sits BELOW compat: it uses only native eh_host_feat_bt headers
# (HCI byte-pipe + controller). No esp_hosted.h / compat dependency.
file(GLOB _eh_bluedroid_feat_incs LIST_DIRECTORIES true
    "${EH_PATH}/host/features/eh_host_feat_*/include")
target_include_directories(eh_host_bluedroid PRIVATE ${_eh_bluedroid_feat_incs})

target_link_libraries(eh_host_bluedroid
    PUBLIC
        eh_host_feat_bt          # HCI byte-pipe + eh_host_bt_controller_*
    PRIVATE
        eh_host_config           # sdkconfig.h + master config header
        idf::bt                  # Bluedroid host stack + esp_bluedroid_hci.h
)

# Only when auto-init is chosen: EH_HOST_FEAT_REGISTER + eh_host_auto_init.h.
if(CONFIG_ESP_HOSTED_HOST_BT_PORT_BLUEDROID_AUTO_INIT)
    target_link_libraries(eh_host_bluedroid PRIVATE eh_host_autoinit)
endif()

# Force our TU into the final link so the auto-init descriptor (when enabled)
# lands even without a direct reference. (Bluedroid ops are referenced by the
# app, but keep this symmetric with the nimble port.)
target_link_options(eh_host_bluedroid INTERFACE
    "-Wl,--undefined=eh_host_bluedroid_init")

# Self-attach to the host umbrella.
if(TARGET eh_host)
    target_link_libraries(eh_host INTERFACE eh_host_bluedroid)
endif()
