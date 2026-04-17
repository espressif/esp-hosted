/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2025 Espressif Systems (Shanghai) PTE LTD */

#ifndef __EH_CP_BASE_RPC_LINUX_PRIV_H__
#define __EH_CP_BASE_RPC_LINUX_PRIV_H__
#include "esp_err.h"
#include "eh_cp_feat_rpc_ext_linux_pbuf.h"

#define PCOM_EP_REQ  "ctrlResp"
#define PCOM_EP_EVT  "ctrlEvnt"

/* Protocomm callbacks for protobuf processing */
esp_err_t linux_rpc_event_handler(uint32_t session_id, const uint8_t *inbuf, ssize_t inlen,
    uint8_t **outbuf, ssize_t *outlen, void *priv_data);

esp_err_t linux_rpc_req_handler(uint32_t session_id, const uint8_t *inbuf, ssize_t inlen,
    uint8_t **outbuf, ssize_t *outlen, void *priv_data);

/* Internal functions */
/* Event Dispatcher */
esp_err_t eh_cp_feat_rpc_ext_linux_rpc_evt_dispatcher(CtrlMsg *ntfy, void *priv_data, const uint8_t *inbuf, size_t inlen);
/* Command Dispatcher */
esp_err_t eh_cp_feat_rpc_ext_linux_rpc_req_dispatcher(const CtrlMsg *req, CtrlMsg *resp, void *priv_data);

/* Event handlers */
esp_err_t eh_cp_feat_rpc_ext_linux_register_wifi_evt_handlers(void);
esp_err_t eh_cp_feat_rpc_ext_linux_unregister_wifi_evt_handlers(void);
esp_err_t eh_cp_feat_rpc_ext_linux_register_system_evt_handlers(void);
esp_err_t eh_cp_feat_rpc_ext_linux_unregister_system_evt_handlers(void);
/* Handlers for cross-extension events on the core bus.
 * Called from fg_core init/deinit — NOT from the extension that posts them. */
esp_err_t eh_cp_feat_register_nw_split_evt_handlers(void);
esp_err_t eh_cp_feat_unregister_nw_split_evt_handlers(void);

/* RPC Request Handlers */
esp_err_t req_get_dhcp_dns_status(const CtrlMsg *req, CtrlMsg *resp, void *priv_data);
esp_err_t req_set_dhcp_dns_status(const CtrlMsg *req, CtrlMsg *resp, void *priv_data);
esp_err_t req_get_fw_version_handler(const CtrlMsg *req, CtrlMsg *resp, void *priv_data);
#if EH_CP_FEAT_PEER_DATA_TRANSFER_READY
esp_err_t req_custom_unserialised_rpc_msg_handler(const CtrlMsg *req, CtrlMsg *resp, void *priv_data);
#endif
esp_err_t req_config_heartbeat(const CtrlMsg *req, CtrlMsg *resp, void *priv_data);
esp_err_t req_enable_disable(const CtrlMsg *req, CtrlMsg *resp, void *priv_data);

/* Wifi RPC Request Handlers */
esp_err_t req_get_mac_address_handler(const CtrlMsg *req, CtrlMsg *resp, void *priv_data);
esp_err_t req_set_mac_address_handler(const CtrlMsg *req, CtrlMsg *resp, void *priv_data);
esp_err_t req_get_wifi_mode_handler(const CtrlMsg *req, CtrlMsg *resp, void *priv_data);
esp_err_t req_set_wifi_mode_handler(const CtrlMsg *req, CtrlMsg *resp, void *priv_data);
esp_err_t req_get_ap_config_handler(const CtrlMsg *req, CtrlMsg *resp, void *priv_data);
esp_err_t req_connect_ap_handler(const CtrlMsg *req, CtrlMsg *resp, void *priv_data);
esp_err_t req_disconnect_ap_handler(const CtrlMsg *req, CtrlMsg *resp, void *priv_data);
esp_err_t req_get_softap_config_handler(const CtrlMsg *req, CtrlMsg *resp, void *priv_data);
esp_err_t req_start_softap_handler(const CtrlMsg *req, CtrlMsg *resp, void *priv_data);
esp_err_t req_stop_softap_handler(const CtrlMsg *req, CtrlMsg *resp, void *priv_data);
esp_err_t req_set_power_save_mode_handler(const CtrlMsg *req, CtrlMsg *resp, void *priv_data);
esp_err_t req_get_power_save_mode_handler(const CtrlMsg *req, CtrlMsg *resp, void *priv_data);
esp_err_t req_set_softap_vender_specific_ie_handler(const CtrlMsg *req, CtrlMsg *resp, void *priv_data);
esp_err_t req_set_wifi_max_tx_power_handler(const CtrlMsg *req, CtrlMsg *resp, void *priv_data);
esp_err_t req_get_wifi_curr_tx_power_handler(const CtrlMsg *req, CtrlMsg *resp, void *priv_data);
esp_err_t req_get_country_code_handler(const CtrlMsg *req, CtrlMsg *resp, void *priv_data);
esp_err_t req_set_country_code_handler(const CtrlMsg *req, CtrlMsg *resp, void *priv_data);
esp_err_t req_get_ap_scan_list_handler(const CtrlMsg *req, CtrlMsg *resp, void *priv_data);
esp_err_t req_get_connected_sta_list_handler(const CtrlMsg *req, CtrlMsg *resp, void *priv_data);


/* OTA RPC Request Handlers */
esp_err_t req_ota_begin_handler(const CtrlMsg *req, CtrlMsg *resp, void *priv_data);
esp_err_t req_ota_write_handler(const CtrlMsg *req, CtrlMsg *resp, void *priv_data);
esp_err_t req_ota_end_handler(const CtrlMsg *req, CtrlMsg *resp, void *priv_data);

#endif /* __EH_CP_BASE_RPC_LINUX_PRIV_H__ */
