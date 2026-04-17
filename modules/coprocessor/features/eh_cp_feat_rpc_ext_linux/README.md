# EH_CP Base RPC Linux Component

#### WiFi Handlers
- `req_set_wifi_mode_handler`
- `req_connect_ap_handler`
- `req_get_ap_config_handler`
- `req_disconnect_ap_handler`
- `req_get_softap_config_handler`
- `req_start_softap_handler`
- `req_get_ap_scan_list_handler`
- `req_stop_softap_handler`
- `get_connected_sta_list_handler`
- `req_set_mac_address_handler`
- `req_set_power_save_mode_handler`
- `req_get_power_save_mode_handler`
- `req_set_wifi_max_tx_power_handler`
- `req_get_wifi_curr_tx_power_handler`
- `req_set_country_code_handler`
- `req_get_country_code_handler`
- `req_set_softap_vender_specific_ie_handler`

#### System Handlers
- `req_config_heartbeat`
- `req_enable_disable`
- `req_get_dhcp_dns_status`
- `req_set_dhcp_dns_status`

#### OTA Handlers
- `req_ota_begin_handler`
- `req_ota_write_handler`
- `req_ota_end_handler`

### Dependencies
- ✅ `esp_hosted_cp_lin_fg_pbuf` - Correctly configured in CMakeLists.txt
- ✅ `esp_hosted_cp` - For core functionality
- ✅ `esp_wifi`, `esp_netif`, `esp_event`, `esp_ota` - For respective functionalities

## Usage
This component will be automatically initialized when the ESP Hosted CP system starts, registering all the base RPC handlers with the command registry.
