# SPDX-License-Identifier: Apache-2.0
#
# Single esp_hosted REQUIRES brings in the host stack; every
# selected sub-library (eh_host_core, eh_host_feat_system, RPC base/
# ext, transport_mcu, port_host_esp_idf, vserial_mcu) flows in via
# INTERFACE-link propagation from esp_hosted.  esp_wifi is required
# here (this scenario calls esp_wifi_*) -- on esp32p4/h2 esp_wifi
# routes its API into the esp_wifi_remote hooks our esp_hosted target
# implements (see host/compat/esp_wifi_remote_glue/).
idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES esp_hosted
    PRIV_REQUIRES esp_wifi nvs_flash esp_event esp_netif)
