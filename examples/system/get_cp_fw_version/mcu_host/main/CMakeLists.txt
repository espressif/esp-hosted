# SPDX-License-Identifier: Apache-2.0
#
# Single esp_hosted REQUIRES brings in the host stack; every
# selected sub-library (eh_host_core, eh_host_feat_system, RPC base/
# ext, transport_mcu, port_host_esp_idf, vserial_mcu) flows in via
# INTERFACE-link propagation from esp_hosted.  Mirrors
# examples/wifi/mcu/main/CMakeLists.txt's REQUIRES list -- the only
# difference is the host-side does NOT need esp_wifi (this scenario
# is system feature only).
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
