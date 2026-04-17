<!-- %% sp.sy.ve-fp.o %% - context -->
---
type: spec
last_verified: 2026-04-02
---

# Feature Parity Map: esp_hosted_mcu → eh_hosted

## Legend
- ✅ Done — handler present, guarded, tested
- 🔲 Missing — exists in legacy, not yet in new
- ⚠️ Partial — handler exists but needs work
- N/A — not applicable for that RPC path

<!-- %% sp.sy.ve-fp.wifi.o %% - context -->
## WiFi Standard (43 legacy handlers)

| RPC ID | Handler | MCU v1 | FG v1 | v2 | Guard |
|--------|---------|--------|-------|-----|-------|
| Req_WifiInit | req_wifi_init | ✅ | N/A | N/A | EH_CP_FEAT_WIFI_READY |
| Req_WifiDeinit | req_wifi_deinit | ✅ | N/A | N/A | EH_CP_FEAT_WIFI_READY |
| Req_WifiStart | req_wifi_start | ✅ | N/A | N/A | EH_CP_FEAT_WIFI_READY |
| Req_WifiStop | req_wifi_stop | ✅ | N/A | N/A | EH_CP_FEAT_WIFI_READY |
| Req_WifiConnect | req_wifi_connect | ✅ | ✅ ConnectAP | N/A | EH_CP_FEAT_WIFI_READY |
| Req_WifiDisconnect | req_wifi_disconnect | ✅ | ✅ DisconnectAP | N/A | EH_CP_FEAT_WIFI_READY |
| Req_WifiSetConfig | req_wifi_set_config | ✅ | ✅ StartSoftAP | N/A | EH_CP_FEAT_WIFI_READY |
| Req_WifiGetConfig | req_wifi_get_config | ✅ | ✅ GetAPConfig/GetSoftAPConfig | N/A | EH_CP_FEAT_WIFI_READY |
| Req_GetMACAddress | req_wifi_get_mac | ✅ | ✅ | N/A | EH_CP_FEAT_WIFI_READY |
| Req_SetMacAddress | req_wifi_set_mac | ✅ | ✅ | N/A | EH_CP_FEAT_WIFI_READY |
| Req_GetWifiMode | req_wifi_get_mode | ✅ | ✅ | N/A | EH_CP_FEAT_WIFI_READY |
| Req_SetWifiMode | req_wifi_set_mode | ✅ | ✅ | N/A | EH_CP_FEAT_WIFI_READY |
| Req_WifiSetPs | req_wifi_set_ps | ✅ | ✅ SetPowerSaveMode | N/A | EH_CP_FEAT_WIFI_READY |
| Req_WifiGetPs | req_wifi_get_ps | ✅ | ✅ GetPowerSaveMode | N/A | EH_CP_FEAT_WIFI_READY |
| Req_WifiSetMaxTxPower | req_wifi_set_max_tx_power | ✅ | ✅ | N/A | EH_CP_FEAT_WIFI_READY |
| Req_WifiGetMaxTxPower | req_wifi_get_max_tx_power | ✅ | ✅ GetWifiCurrTxPower | N/A | EH_CP_FEAT_WIFI_READY |
| Req_WifiScanStart | req_wifi_scan_start | ✅ | ✅ GetAPScanList | N/A | EH_CP_FEAT_WIFI_READY |
| Req_WifiScanStop | req_wifi_scan_stop | ✅ | N/A | N/A | EH_CP_FEAT_WIFI_READY |
| Req_WifiScanParams | req_wifi_scan_params | ✅ | N/A | N/A | EH_CP_FEAT_WIFI_READY |
| Req_WifiScanGetApNum | req_wifi_scan_get_ap_num | ✅ | N/A | N/A | EH_CP_FEAT_WIFI_READY |
| Req_WifiScanGetApRecord | req_wifi_scan_get_ap_record | ✅ | N/A | N/A | EH_CP_FEAT_WIFI_READY |
| Req_WifiScanGetApRecords | req_wifi_scan_get_ap_records | ✅ | N/A | N/A | EH_CP_FEAT_WIFI_READY |
| Req_WifiClearApList | req_wifi_clear_ap_list | ✅ | N/A | N/A | EH_CP_FEAT_WIFI_READY |
| Req_WifiRestore | req_wifi_restore | ✅ | N/A | N/A | EH_CP_FEAT_WIFI_READY |
| Req_WifiClearFastConnect | req_wifi_clear_fast_connect | ✅ | N/A | N/A | EH_CP_FEAT_WIFI_READY |
| Req_WifiStaGetApInfo | req_wifi_sta_get_ap_info | ✅ | N/A | N/A | EH_CP_FEAT_WIFI_READY |
| Req_WifiDeauthSta | req_wifi_deauth_sta | ✅ | N/A | N/A | EH_CP_FEAT_WIFI_READY |
| Req_WifiSetStorage | req_wifi_set_storage | ✅ | N/A | N/A | EH_CP_FEAT_WIFI_READY |
| Req_WifiSetProtocol | req_wifi_set_protocol | ✅ | N/A | N/A | EH_CP_FEAT_WIFI_READY |
| Req_WifiGetProtocol | req_wifi_get_protocol | ✅ | N/A | N/A | EH_CP_FEAT_WIFI_READY |
| Req_WifiSetBandwidth | req_wifi_set_bandwidth | ✅ | N/A | N/A | EH_CP_FEAT_WIFI_READY |
| Req_WifiGetBandwidth | req_wifi_get_bandwidth | ✅ | N/A | N/A | EH_CP_FEAT_WIFI_READY |
| Req_WifiSetChannel | req_wifi_set_channel | ✅ | N/A | N/A | EH_CP_FEAT_WIFI_READY |
| Req_WifiGetChannel | req_wifi_get_channel | ✅ | N/A | N/A | EH_CP_FEAT_WIFI_READY |
| Req_WifiSetCountryCode | req_wifi_set_country_code | ✅ | ✅ | N/A | EH_CP_FEAT_WIFI_READY |
| Req_WifiGetCountryCode | req_wifi_get_country_code | ✅ | ✅ | N/A | EH_CP_FEAT_WIFI_READY |
| Req_WifiSetCountry | req_wifi_set_country | ✅ | N/A | N/A | EH_CP_FEAT_WIFI_READY |
| Req_WifiGetCountry | req_wifi_get_country | ✅ | N/A | N/A | EH_CP_FEAT_WIFI_READY |
| Req_WifiApGetStaList | req_wifi_ap_get_sta_list | ✅ | ✅ GetSoftAPConnectedSTAList | N/A | EH_CP_FEAT_WIFI_READY |
| Req_WifiApGetStaAid | req_wifi_ap_get_sta_aid | ✅ | N/A | N/A | EH_CP_FEAT_WIFI_READY |
| Req_WifiStaGetRssi | req_wifi_sta_get_rssi | ✅ | N/A | N/A | EH_CP_FEAT_WIFI_READY |
| Req_WifiStaGetAid | req_wifi_sta_get_aid | ✅ | N/A | N/A | EH_CP_FEAT_WIFI_READY |
| Req_WifiStaGetNegotiatedPhymode | req_wifi_sta_get_negotiated_phymode | ✅ | N/A | N/A | EH_CP_FEAT_WIFI_READY |
| Req_WifiSetInactiveTime | req_wifi_set_inactive_time | ✅ | N/A | N/A | EH_CP_FEAT_WIFI_READY |
| Req_WifiGetInactiveTime | req_wifi_get_inactive_time | ✅ | N/A | N/A | EH_CP_FEAT_WIFI_READY |
| Req_SetSoftAPVendorSpecificIE | req_set_softap_vender_specific_ie | N/A | ✅ | N/A | EH_CP_FEAT_WIFI_READY |

### WiFi PHY 5.4.0+ (8 handlers)

| RPC ID | Handler | MCU v1 | FG v1 | Guard |
|--------|---------|--------|-------|-------|
| Req_WifiSetProtocols | req_wifi_set_protocols | ✅ | N/A | H_PRESENT_IN_ESP_IDF_5_4_0 |
| Req_WifiGetProtocols | req_wifi_get_protocols | ✅ | N/A | H_PRESENT_IN_ESP_IDF_5_4_0 |
| Req_WifiSetBandwidths | req_wifi_set_bandwidths | ✅ | N/A | H_PRESENT_IN_ESP_IDF_5_4_0 |
| Req_WifiGetBandwidths | req_wifi_get_bandwidths | ✅ | N/A | H_PRESENT_IN_ESP_IDF_5_4_0 |
| Req_WifiSetBand | req_wifi_set_band | ✅ | N/A | H_PRESENT_IN_ESP_IDF_5_4_0 |
| Req_WifiGetBand | req_wifi_get_band | ✅ | N/A | H_PRESENT_IN_ESP_IDF_5_4_0 |
| Req_WifiSetBandMode | req_wifi_set_band_mode | ✅ | N/A | H_PRESENT_IN_ESP_IDF_5_4_0 |
| Req_WifiGetBandMode | req_wifi_get_band_mode | ✅ | N/A | H_PRESENT_IN_ESP_IDF_5_4_0 |
<!-- %% sp.sy.ve-fp.wifi.c %% -->

<!-- %% sp.sy.ve-fp.ent.o %% - context -->
## WiFi Enterprise (22 legacy handlers)

| RPC ID | Handler | MCU v1 | FG v1 | Guard |
|--------|---------|--------|-------|-------|
| Req_WifiStaEnterpriseEnable | req_wifi_sta_enterprise_enable | ✅ | N/A | H_WIFI_ENTERPRISE_SUPPORT |
| Req_WifiStaEnterpriseDisable | req_wifi_sta_enterprise_disable | ✅ | N/A | H_WIFI_ENTERPRISE_SUPPORT |
| Req_EapSetIdentity | req_eap_set_identity | ✅ | N/A | H_WIFI_ENTERPRISE_SUPPORT |
| Req_EapClearIdentity | req_eap_clear_identity | ✅ | N/A | H_WIFI_ENTERPRISE_SUPPORT |
| Req_EapSetUsername | req_eap_set_username | ✅ | N/A | H_WIFI_ENTERPRISE_SUPPORT |
| Req_EapClearUsername | req_eap_clear_username | ✅ | N/A | H_WIFI_ENTERPRISE_SUPPORT |
| Req_EapSetPassword | req_eap_set_password | ✅ | N/A | H_WIFI_ENTERPRISE_SUPPORT |
| Req_EapClearPassword | req_eap_clear_password | ✅ | N/A | H_WIFI_ENTERPRISE_SUPPORT |
| Req_EapSetNewPassword | req_eap_set_new_password | ✅ | N/A | H_WIFI_ENTERPRISE_SUPPORT |
| Req_EapClearNewPassword | req_eap_clear_new_password | ✅ | N/A | H_WIFI_ENTERPRISE_SUPPORT |
| Req_EapSetCaCert | req_eap_set_ca_cert | ✅ | N/A | H_WIFI_ENTERPRISE_SUPPORT |
| Req_EapClearCaCert | req_eap_clear_ca_cert | ✅ | N/A | H_WIFI_ENTERPRISE_SUPPORT |
| Req_EapSetCertificateAndKey | req_eap_set_certificate_and_key | ✅ | N/A | H_WIFI_ENTERPRISE_SUPPORT |
| Req_EapClearCertificateAndKey | req_eap_clear_certificate_and_key | ✅ | N/A | H_WIFI_ENTERPRISE_SUPPORT |
| Req_EapSetDisableTimeCheck | req_eap_set_disable_time_check | ✅ | N/A | H_WIFI_ENTERPRISE_SUPPORT |
| Req_EapGetDisableTimeCheck | req_eap_get_disable_time_check | ✅ | N/A | H_WIFI_ENTERPRISE_SUPPORT |
| Req_EapSetTtlsPhase2Method | req_eap_set_ttls_phase2_method | ✅ | N/A | H_WIFI_ENTERPRISE_SUPPORT |
| Req_EapSetSuitebCertification | req_eap_set_suiteb_certification | ✅ | N/A | H_WIFI_ENTERPRISE_SUPPORT |
| Req_EapSetPacFile | req_eap_set_pac_file | ✅ | N/A | H_WIFI_ENTERPRISE_SUPPORT |
| Req_EapSetFastParams | req_eap_set_fast_params | ✅ | N/A | H_WIFI_ENTERPRISE_SUPPORT |
| Req_EapUseDefaultCertBundle | req_eap_use_default_cert_bundle | ✅ | N/A | H_WIFI_ENTERPRISE_SUPPORT |
| Req_WifiSetOkcSupport | req_wifi_set_okc_support | ✅ | N/A | H_GOT_EAP_OKC_SUPPORT |
| Req_EapSetDomainName | req_eap_set_domain_name | ✅ | N/A | H_GOT_EAP_SET_DOMAIN_NAME |
| Req_EapSetEapMethods | req_eap_set_eap_methods | ✅ | N/A | H_GOT_SET_EAP_METHODS_API |

**Extension status**: ✅ `eh_cp_feat_wifi_enterprise` — scaffold + cert ownership + cap bits. RPC handlers in rpc_mcu.
<!-- %% sp.sy.ve-fp.ent.c %% -->

<!-- %% sp.sy.ve-fp.itwt.o %% - context -->
## WiFi ITWT (7 legacy handlers + 4 events)

| RPC ID | Handler | MCU v1 | FG v1 | Guard |
|--------|---------|--------|-------|-------|
| Req_WifiStaTwtConfig | req_wifi_sta_twt_config | ✅ | N/A | CONFIG_SOC_WIFI_HE_SUPPORT |
| Req_WifiStaItwtSetup | req_wifi_sta_itwt_setup | ✅ | N/A | CONFIG_SOC_WIFI_HE_SUPPORT |
| Req_WifiStaItwtTeardown | req_wifi_sta_itwt_teardown | ✅ | N/A | CONFIG_SOC_WIFI_HE_SUPPORT |
| Req_WifiStaItwtSuspend | req_wifi_sta_itwt_suspend | ✅ | N/A | CONFIG_SOC_WIFI_HE_SUPPORT |
| Req_WifiStaItwtGetFlowIdStatus | req_wifi_sta_itwt_get_flow_id_status | ✅ | N/A | CONFIG_SOC_WIFI_HE_SUPPORT |
| Req_WifiStaItwtSendProbeReq | req_wifi_sta_itwt_send_probe_req | ✅ | N/A | CONFIG_SOC_WIFI_HE_SUPPORT |
| Req_WifiStaItwtSetTargetWakeTimeOffset | req_wifi_sta_itwt_set_target_wake_time_offset | ✅ | N/A | CONFIG_SOC_WIFI_HE_SUPPORT |

**Events**: ✅ Own event base `ESP_HOSTED_CP_EXT_WIFI_ITWT_EVENT` + publisher.
**Extension status**: ✅ `eh_cp_feat_wifi_itwt` — scaffold + events + cap bits. RPC handlers in rpc_mcu.
<!-- %% sp.sy.ve-fp.itwt.c %% -->

<!-- %% sp.sy.ve-fp.dpp.o %% - context -->
## WiFi DPP (5 legacy handlers + 6 events)

| RPC ID | Handler | MCU v1 | FG v1 | Guard |
|--------|---------|--------|-------|-------|
| Req_SuppDppInit | req_supp_dpp_init | ✅ | N/A | H_DPP_SUPPORT |
| Req_SuppDppDeinit | req_supp_dpp_deinit | ✅ | N/A | H_DPP_SUPPORT |
| Req_SuppDppBootstrapGen | req_supp_dpp_bootstrap_gen | ✅ | N/A | H_DPP_SUPPORT |
| Req_SuppDppStartListen | req_supp_dpp_start_listen | ✅ | N/A | H_DPP_SUPPORT |
| Req_SuppDppStopListen | req_supp_dpp_stop_listen | ✅ | N/A | H_DPP_SUPPORT |

**Events**: ✅ Own event base `ESP_HOSTED_CP_EXT_WIFI_DPP_EVENT` + publisher (supp + wifi paths).
**Extension status**: ✅ `eh_cp_feat_wifi_dpp` — scaffold + events + cap bits. RPC handlers in rpc_mcu.
<!-- %% sp.sy.ve-fp.dpp.c %% -->

<!-- %% sp.sy.ve-fp.bt.o %% - context -->
## Bluetooth

| Feature | Status | Guard |
|---------|--------|-------|
| BT init/deinit/enable/disable | ✅ via FeatureControl RPC | EH_CP_FEAT_BT_READY |
| HCI (VHCI) path | ✅ | BLUETOOTH_HCI |
| UART path (ESP32) | ✅ | BLUETOOTH_UART + IDF_TARGET_ESP32 |
| UART path (C3/S3) | ✅ | BLUETOOTH_UART + BT_OVER_C3_S3 |
| UART path (generic) | ✅ | BLUETOOTH_UART (others) |
| BLE / BR_EDR / Dual mode | ✅ | CONFIG_BTDM_* |
| NimBLE controller | ✅ | SOC_ESP_NIMBLE_CONTROLLER |
| Cap bits | ✅ | Dynamic via get_capabilities() |
| Auto-start/stop | ✅ (new) | EH_CP_AUTO_START_BT / EH_CP_AUTO_STOP_BT |
| Extension scaffold | ✅ | Files renamed to spec |
<!-- %% sp.sy.ve-fp.bt.c %% -->

<!-- %% sp.sy.ve-fp.sys.o %% - context -->
## System

| RPC ID | Handler | MCU v1 | FG v1 | Guard |
|--------|---------|--------|-------|-------|
| Req_OTABegin | req_ota_begin_handler | ✅ | ✅ | EH_CP_FEAT_SYSTEM_READY |
| Req_OTAWrite | req_ota_write_handler | ✅ | ✅ | EH_CP_FEAT_SYSTEM_READY |
| Req_OTAEnd | req_ota_end_handler | ✅ | ✅ | EH_CP_FEAT_SYSTEM_READY |
| Req_OTAActivate | req_ota_activate_handler | ✅ | N/A | EH_CP_FEAT_SYSTEM_READY |
| Req_ConfigHeartbeat | req_config_heartbeat | ✅ | ✅ | EH_CP_FEAT_SYSTEM_READY |
| Req_GetCoprocessorFwVersion | req_get_coprocessor_fw_version | ✅ | ✅ GetFwVersion | EH_CP_FEAT_SYSTEM_READY |
| Req_IfaceMacAddrSetGet | req_iface_mac_addr_set_get | ✅ | N/A | EH_CP_FEAT_SYSTEM_READY |
| Req_IfaceMacAddrLenGet | req_iface_mac_addr_len_get | ✅ | N/A | EH_CP_FEAT_SYSTEM_READY |
| Req_FeatureControl | req_feature_control | ✅ | ✅ EnableDisable | EH_CP_FEAT_SYSTEM_READY |
| Req_AppGetDesc | req_app_get_desc | ✅ | N/A | EH_CP_FEAT_SYSTEM_READY |
<!-- %% sp.sy.ve-fp.sys.c %% -->

<!-- %% sp.sy.ve-fp.missing.o %% - always -->
## Missing Extensions (legacy features not yet as standalone extensions)

| Feature | Legacy source | RPC handlers | Extension | Status |
|---------|--------------|-------------|-----------|--------|
| WiFi Enterprise | slave_wifi_enterprise.c | 22 in rpc_mcu | ✅ eh_cp_feat_wifi_enterprise | Done |
| ITWT | slave_wifi_std.c (HE section) | 7 in rpc_mcu | ✅ eh_cp_feat_wifi_itwt | Done — events + publisher |
| DPP | slave_wifi_std.c (DPP section) | 5 in rpc_mcu | ✅ eh_cp_feat_wifi_dpp | Done — events + publisher |
| GPIO Expander | slave_gpio_expander.c | 7 in rpc_mcu | ✅ eh_cp_feat_gpio_exp | Done — handlers + dispatcher + example |
| Memory Monitor | slave_control.c | 1 in rpc_mcu | ✅ eh_cp_feat_mem_monitor | Done — handler + timer + event |
| External Coex | slave_ext_coex.c | 1 in rpc_mcu | ✅ eh_cp_feat_ext_coex | Done — handler + dispatcher |
| Light Sleep | slave_light_sleep.c | 0 (local PM) | ✅ eh_cp_feat_light_sleep | Done — PM lock functions ported |
<!-- %% sp.sy.ve-fp.missing.c %% -->

<!-- %% sp.sy.ve-fp.events.o %% - context -->
## Event Parity

| Event | Legacy | New publisher | New RPC handler | Status |
|-------|--------|--------------|----------------|--------|
| STA Connected | ✅ | ✅ WiFi ext | ✅ rpc_mcu | ✅ |
| STA Disconnected | ✅ | ✅ WiFi ext | ✅ rpc_mcu | ✅ |
| STA Started/Stopped | ✅ | ✅ WiFi ext | ✅ rpc_mcu | ✅ |
| SoftAP Started/Stopped | ✅ | ✅ WiFi ext | ✅ rpc_mcu | ✅ |
| SoftAP STA Connect/Disconnect | ✅ | ✅ WiFi ext | ✅ rpc_mcu | ✅ |
| Scan Done | ✅ | ✅ WiFi ext | ✅ rpc_mcu | ✅ |
| ITWT Setup/Teardown/Suspend/Probe | ✅ | ✅ ITWT ext | ✅ rpc_mcu evt handler | ✅ |
| DPP URI/Cfg/Fail (supp) | ✅ | ✅ DPP ext | ✅ rpc_mcu evt handler | ✅ |
| DPP URI/Cfg/Fail (wifi) | ✅ | ✅ DPP ext | ✅ rpc_mcu evt handler | ✅ |
| DHCP/DNS Status | ✅ | ✅ nw_split ext | ✅ rpc_mcu | ✅ |
| Heartbeat | ✅ | ✅ system ext | ✅ rpc_mcu | ✅ |
| ESP Init | ✅ | ✅ core | ✅ rpc_mcu | ✅ |
| Memory Monitor | ✅ | ✅ timer in handler | ✅ rpc_mcu | ✅ |
<!-- %% sp.sy.ve-fp.events.c %% -->

<!-- %% sp.sy.ve-fp.summary.o %% - context -->
## Handler Count Summary (verified against legacy slave_control.c)

| Category | Legacy | New | File |
|----------|--------|-----|------|
| OTA | 4 | 4 | handler_req_system.c |
| System (heartbeat/FW/MAC/feature) | 5 | 5 | handler_req_system.c |
| WiFi Standard | 43 | 43 | handler_req_wifi.c |
| WiFi Enterprise | 22 | 22 | handler_req_wifi.c |
| WiFi DPP | 5 | 5 | handler_req_wifi.c |
| WiFi ITWT | 7 | 7 | handler_req_wifi.c |
| GPIO Expander | 7 | 7 | handler_req_gpio.c |
| External Coex | 1 | 1 | handler_req_ext_coex.c |
| Memory Monitor | 1 | 1 | handler_req_mem_monitor.c |
| Custom RPC | 1 | 1 | handler_req_peer_data.c |
| **Total** | **96** | **96** | |

RX callbacks: wlan_sta, wlan_ap, bt_hci — all registered in eh_cp_core.c
Event senders: 15 events — all ported to handler_evt.c + dispatcher_evt.c
<!-- %% sp.sy.ve-fp.summary.c %% -->

<!-- %% sp.sy.ve-fp.hwtest.o %% - context -->
## Hardware Test Results (ESP32-P4 host ↔ ESP32-C6 CP, SDIO)

Verified via pytest-embedded dual-DUT, 2026-04-05.

| Test | CP Example | Host Example | Tests | Result |
|------|-----------|-------------|-------|--------|
| Boot + FW ver + WiFi scan | minimal/wifi | host_minimal_test | 7 | ✅ PASS |
| GPIO toggle + read | extensions/gpio_exp | host_gpio_expander | 3 | ✅ PASS |
| Custom RPC echo (3 sizes + GHOST) | extensions/peer_data_transfer | host_peer_data_transfer | 5 | ✅ PASS |
| Heap stats + monitor config | extensions/mem_monitor | host_hosted_cp_meminfo | 5 | ✅ PASS |
| BLE NimBLE VHCI advertising | minimal/bt | host_nimble_bleprph_host_only_vhci | 5 | ✅ PASS |
| BT MAC address query | minimal/bt | host_bt_controller_mac_addr | 2 | ✅ PASS |
| WiFi connect + GOT_IP + events | minimal/wifi | host_hosted_events | 7 | ✅ PASS |
| Heartbeat INIT + FW ver + tick | minimal/wifi | host_hosted_events | 6 | ✅ PASS |
| Real OTA 992KB + activate + reboot | minimal/wifi | host_performs_slave_ota | 6 | ✅ PASS |
| **Total** | | | **46** | **46/46 PASS** |

Not tested (requires external HW/setup):
- WiFi Enterprise (RADIUS server), DPP (configurator), iTWT (802.11ax AP)
- Power save (function EV board), ext coex (coex HW)
- BT HID (peer device), UART HCI (different transport)
<!-- %% sp.sy.ve-fp.hwtest.c %% -->

<!-- %% sp.sy.ve-fp.c %% -->
