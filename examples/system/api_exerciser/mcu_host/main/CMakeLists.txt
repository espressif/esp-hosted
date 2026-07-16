# SPDX-License-Identifier: Apache-2.0
#
# esp_hosted brings in the host stack; console gives the REPL; esp_wifi provides
# the wifi_* types the eh_host_wifi_* API takes (routed to esp_wifi_remote on p4).
# eh_api_cmd.c is the shared command corpus in ../../common (also used by the
# linux_802_3_host role). The build cache copies common/ into the scratch so
# ../../common resolves both in-tree and in the COW build.
idf_component_register(
    SRCS "main.c" "../../common/eh_api_cmd.c"
    INCLUDE_DIRS "." "../../common"
    REQUIRES esp_hosted console esp_wifi
    PRIV_REQUIRES nvs_flash esp_event esp_netif)
