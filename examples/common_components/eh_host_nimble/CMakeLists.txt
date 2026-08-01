# SPDX-License-Identifier: Apache-2.0
#
# NimBLE hosted-HCI port — stack glue on top of eh_host_feat_bt's HCI
# byte-pipe. Not a standalone IDF component: an internal STATIC lib of the
# esp_hosted component, add_subdirectory'd from the root CMakeLists and
# gated by its Kconfig. feat_bt stays stack-agnostic.

if(NOT CONFIG_ESP_HOSTED_HOST_BT_PORT_NIMBLE)
    return()
endif()

add_library(eh_host_nimble STATIC
    "src/eh_host_nimble.c"
)

target_include_directories(eh_host_nimble PUBLIC "include")

# This port sits BELOW compat: it uses only native eh_host_feat_bt headers
# (HCI byte-pipe + controller). No esp_hosted.h / compat dependency.
file(GLOB _eh_nimble_feat_incs LIST_DIRECTORIES true
    "${EH_PATH}/host/features/eh_host_feat_*/include")
target_include_directories(eh_host_nimble PRIVATE ${_eh_nimble_feat_incs})

target_link_libraries(eh_host_nimble
    PUBLIC
        eh_host_feat_bt          # HCI byte-pipe + eh_host_bt_controller_*
    PRIVATE
        eh_host_config           # sdkconfig.h + master config header
        idf::bt                  # NimBLE host stack + transport headers
)

# Only when auto-init is chosen: EH_HOST_FEAT_REGISTER + eh_host_auto_init.h.
if(CONFIG_ESP_HOSTED_HOST_BT_PORT_NIMBLE_AUTO_INIT)
    target_link_libraries(eh_host_nimble PRIVATE eh_host_autoinit)
endif()

# Weak-override lands from a STATIC lib only if the object is pulled in.
# Force our TU into the final link so the strong ble_transport_* overrides
# (and, when enabled, the auto-init descriptor) take effect.
target_link_options(eh_host_nimble INTERFACE
    "-Wl,--undefined=eh_host_nimble_init")

# Self-attach to the host umbrella.
if(TARGET eh_host)
    target_link_libraries(eh_host INTERFACE eh_host_nimble)
endif()
