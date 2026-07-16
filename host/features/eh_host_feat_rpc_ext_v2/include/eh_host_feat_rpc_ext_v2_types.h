/* SPDX-License-Identifier: Apache-2.0 */
/* PUBLIC types for the MCU V1 Rpc extension (ctrl_cmd + payloads). */

#ifndef EH_HOST_FEAT_RPC_EXT_V2_TYPES_H_
#define EH_HOST_FEAT_RPC_EXT_V2_TYPES_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "gen_v2.h"

#include "eh_host_feat_rpc.h"  /* eh_host_rpc_rsp_cb_t, prefix layout */

#ifdef __cplusplus
extern "C" {
#endif

#define EH_RPC_MAC_LEN                  6u
#define EH_RPC_SSID_LEN                 32u
#define EH_RPC_PASSWORD_LEN             64u
#define EH_RPC_COUNTRY_CC_LEN           3u
#define EH_RPC_IP4_LEN                  4u
#define EH_RPC_OTA_CHUNK_MAX            1536u
#define EH_RPC_IDF_TARGET_LEN           32u   /* e.g. "esp32c6" */
#define EH_RPC_SAE_H2E_IDENTIFIER_LEN   32u

typedef struct {
    uint8_t mac[EH_RPC_MAC_LEN];
    int32_t mode;                 /* WIFI_IF_STA / WIFI_IF_AP */
} eh_rpc_mac_addr_t;

/* Req/Resp_IfaceMacAddrSetGet — full esp_mac_type_t surface. */
typedef struct {
    uint32_t set;
    uint32_t type;                /* esp_mac_type_t */
    uint8_t  mac[8];              /* 6 MAC-48 / 8 IEEE802154 EUI-64 */
    uint32_t mac_len;
} eh_rpc_iface_mac_addr_t;

typedef struct {
    int32_t mode;                 /* WIFI_MODE_* */
} eh_rpc_wifi_mode_t;

typedef struct {
    RpcWifiPowerSave ps_mode;
} eh_rpc_wifi_ps_t;

typedef struct {
    int32_t power;                /* dBm * 4 */
} eh_rpc_wifi_tx_power_t;

/* Plain-C mirror of IDF wifi_config_t (STA + AP), gated by `iface`. */
typedef struct {
    int32_t iface;                /* WIFI_IF_STA / WIFI_IF_AP */

    /* shared (STA + AP) */
    uint8_t  ssid[EH_RPC_SSID_LEN + 1];
    uint32_t ssid_len;
    uint8_t  password[EH_RPC_PASSWORD_LEN + 1];
    uint32_t channel;
    int32_t  authmode;
    uint8_t  pmf_capable;
    uint8_t  pmf_required;
    int32_t  sae_pwe_h2e;

    /* STA only */
    uint8_t  bssid[EH_RPC_MAC_LEN];
    bool     bssid_set;
    int32_t  scan_method;
    uint32_t listen_interval;
    int32_t  sort_method;
    int8_t   threshold_rssi;
    uint32_t threshold_rssi_5g_adjustment;
    int32_t  sae_pk_mode;
    uint32_t failure_retry_cnt;
    uint8_t  sae_h2e_identifier[EH_RPC_SAE_H2E_IDENTIFIER_LEN];
    uint32_t sae_h2e_identifier_len;
    /* Raw STA bitmasks; wrapper packs/unpacks via eh_rpc_bitmasks.h. */
    uint32_t bitmask;
    uint32_t he_bitmask;

    /* AP only */
    uint32_t ssid_hidden;
    uint32_t max_connection;
    uint32_t beacon_interval;
    int32_t  pairwise_cipher;
    bool     ftm_responder;
    uint32_t csa_count;
    uint32_t dtim_period;
    uint32_t transition_disable;
    uint32_t sae_ext;
    uint32_t bss_max_idle_period;
    bool     bss_max_idle_protected_keep_alive;
    uint32_t gtk_rekey_interval;
} eh_rpc_wifi_cfg_t;

/* Req_WifiInit: plain-C mirror of IDF wifi_init_config_t. */
typedef struct {
    int32_t  static_rx_buf_num;
    int32_t  dynamic_rx_buf_num;
    int32_t  tx_buf_type;
    int32_t  static_tx_buf_num;
    int32_t  dynamic_tx_buf_num;
    int32_t  cache_tx_buf_num;
    int32_t  csi_enable;
    int32_t  ampdu_rx_enable;
    int32_t  ampdu_tx_enable;
    int32_t  amsdu_tx_enable;
    int32_t  nvs_enable;
    int32_t  nano_enable;
    int32_t  rx_ba_win;
    int32_t  wifi_task_core_id;
    int32_t  beacon_max_len;
    int32_t  mgmt_sbuf_num;
    uint64_t feature_caps;
    bool     sta_disconnected_pm;
    int32_t  espnow_max_encrypt_num;
    int32_t  magic;
    int32_t  rx_mgmt_buf_type;
    int32_t  rx_mgmt_buf_num;
    int32_t  tx_hetb_queue_num;
    int32_t  dump_hesigb_enable;
} eh_rpc_wifi_init_cfg_t;

typedef struct {
    bool     block;
    bool     cfg_set;
} eh_rpc_wifi_scan_cfg_t;

/* Req/Resp_WifiScanParams — mirrors wifi_scan_default_params_t. */
typedef struct {
    bool     set;
    uint32_t passive;
    uint32_t active_min;
    uint32_t active_max;
    uint32_t home_chan_dwell_time;
} eh_rpc_wifi_scan_params_t;

typedef struct {
    int32_t status;
    int32_t number;
    int32_t scan_id;
} eh_rpc_scan_done_t;

typedef struct {
    uint8_t  ssid[EH_RPC_SSID_LEN + 1];
    uint32_t ssid_len;
    uint8_t  bssid[EH_RPC_MAC_LEN];
    uint32_t channel;
    int32_t  authmode;
    int32_t  aid;
} eh_rpc_sta_connected_t;

typedef struct {
    uint8_t  ssid[EH_RPC_SSID_LEN + 1];
    uint32_t ssid_len;
    uint8_t  bssid[EH_RPC_MAC_LEN];
    uint32_t reason;
    int32_t  rssi;
} eh_rpc_sta_disconnected_t;

/* AP-side Sta connect/disconnect events. */
typedef struct {
    uint8_t  mac[EH_RPC_MAC_LEN];
    uint32_t aid;
    bool     is_mesh_child;
} eh_rpc_ap_staconnected_t;

typedef struct {
    uint8_t  mac[EH_RPC_MAC_LEN];
    uint32_t aid;
    bool     is_mesh_child;
    uint32_t reason;
} eh_rpc_ap_stadisconnected_t;

typedef struct {
    uint32_t cp_reset_reason;
} eh_rpc_esp_init_t;

typedef struct {
    int32_t wifi_event_id;
} eh_rpc_wifi_simple_t;

typedef struct {
    uint8_t *data;              /* heap-owned */
    size_t   len;
} eh_rpc_ota_write_t;

typedef struct {
    bool     enable;
    int32_t  duration;
} eh_rpc_heartbeat_cfg_t;

typedef struct {
    int32_t  hb_num;
} eh_rpc_heartbeat_evt_t;

typedef struct {
    int8_t   cc[EH_RPC_COUNTRY_CC_LEN + 1];
    bool     ieee80211d_enabled;
} eh_rpc_country_code_t;

/* Full wifi_country_t mirror; cc NUL-padded for safe printing. */
typedef struct {
    int8_t   cc[EH_RPC_COUNTRY_CC_LEN + 1];
    uint32_t schan;
    uint32_t nchan;
    int32_t  max_tx_power;
    int32_t  policy;
} eh_rpc_wifi_country_full_t;

/* wifi_protocols mirror; ghz_2g/5g widened to uint32 for wire. */
typedef struct {
    int32_t  ifx;
    uint32_t ghz_2g;
    uint32_t ghz_5g;
} eh_rpc_wifi_protocols_t;

/* wifi_bandwidths mirror; enum widened to uint32 for wire. */
typedef struct {
    int32_t  ifx;
    uint32_t ghz_2g;
    uint32_t ghz_5g;
} eh_rpc_wifi_bandwidths_t;

/* wifi_twt_config_t mirror. */
typedef struct {
    bool     post_wakeup_event;
    bool     twt_enable_keep_alive;
} eh_rpc_wifi_twt_config_t;

/* wifi_sta_info mirror; decode cracks proto bitmask into bools. */
typedef struct {
    uint8_t  mac[EH_RPC_MAC_LEN];
    int32_t  rssi;
    bool     phy_11b;
    bool     phy_11g;
    bool     phy_11n;
    bool     phy_lr;
    bool     phy_11ax;
    bool     is_mesh_child;
    uint32_t reserved;
} eh_rpc_wifi_sta_info_t;

/* Resp_WifiApGetStaList: heap-owned array. */
typedef struct {
    eh_rpc_wifi_sta_info_t *entries;
    uint32_t                count;
} eh_rpc_sta_list_t;

/* wifi_ap_record mirror; decode cracks bitmasks into explicit bools. */
typedef struct {
    uint8_t  bssid[EH_RPC_MAC_LEN];
    uint8_t  ssid[EH_RPC_SSID_LEN + 1];
    uint32_t ssid_len;
    uint32_t primary;
    int32_t  second;
    int32_t  rssi;
    int32_t  authmode;
    int32_t  pairwise_cipher;
    int32_t  group_cipher;
    int32_t  ant;
    bool     phy_11b;
    bool     phy_11g;
    bool     phy_11n;
    bool     phy_lr;
    bool     phy_11a;
    bool     phy_11ac;
    bool     phy_11ax;
    bool     wps;
    bool     ftm_responder;
    bool     ftm_initiator;
    uint32_t reserved;
    eh_rpc_wifi_country_full_t country;
    uint32_t bss_color;             /* 6 bits semantically */
    bool     partial_bss_color;
    bool     bss_color_disabled;
    uint32_t he_ap_bssid_index;
    uint32_t bandwidth;
    uint32_t vht_ch_freq1;
    uint32_t vht_ch_freq2;
} eh_rpc_wifi_ap_record_t;

/* Resp_WifiScanGetApRecords: heap-owned array (Resp only). */
typedef struct {
    eh_rpc_wifi_ap_record_t *records;
    uint32_t                 number;
} eh_rpc_ap_records_t;

typedef struct {
    uint32_t major;
    uint32_t minor;
    uint32_t patch;
    int32_t  revision;
    int32_t  prerelease;
    int32_t  build;
    uint32_t chip_id;                            /* CONFIG_IDF_FIRMWARE_CHIP_ID */
    uint32_t idf_target_len;
    uint8_t  idf_target[EH_RPC_IDF_TARGET_LEN];  /* CONFIG_IDF_TARGET */
} eh_rpc_fw_version_t;

/* GPIO payloads mirror RpcGpioConfig field-by-field. drive_cap is
 * forward-compat (no wire support yet). */
typedef struct {
    uint64_t pin_bit_mask;
    RpcGpioMode mode;
    bool     pull_up_en;
    bool     pull_down_en;
    int32_t  intr_type;
    int32_t  drive_cap;            /* not on wire yet */
} eh_rpc_gpio_cfg_t;

typedef struct {
    int32_t  gpio_num;
    uint32_t level;
} eh_rpc_gpio_level_t;

typedef struct {
    int32_t  gpio_num;
    RpcGpioMode mode;
} eh_rpc_gpio_direction_t;

typedef struct {
    int32_t  gpio_num;
    RpcGpioPullMode pull;
} eh_rpc_gpio_pull_t;

typedef struct {
    int32_t  gpio_num;
} eh_rpc_gpio_num_t;

/* RpcReqExtCoex mirror; `cmd` selects which fields are used. */
typedef struct {
    RpcExtCoexCmd      cmd;
    uint32_t           set_gpio_wire_type;     /* 0..3 (WIRE_1..WIRE_4) */
    int32_t            set_gpio_request_pin;
    int32_t            set_gpio_priority_pin;
    int32_t            set_gpio_grant_pin;
    int32_t            set_gpio_tx_line_pin;
    uint32_t           set_work_mode;
    uint32_t           set_grant_delay_us;
    bool               set_validate_high;
} eh_rpc_ext_coex_cfg_t;

/* RpcReqFeatureControl — BT controller lifecycle in V1. */
typedef struct {
    RpcFeature         feature;
    RpcFeatureCommand  command;
    RpcFeatureOption   option;
} eh_rpc_feature_ctrl_t;

/* CustomRpc peer data; .data is heap-owned (Req/Resp/Event path). */
typedef struct {
    uint32_t  custom_msg_id;
    uint8_t  *data;
    size_t    len;
} eh_rpc_peer_data_blob_t;

/* Generic heap-owned byte blob. */
typedef struct {
    uint8_t  *data;
    size_t    len;
} eh_rpc_blob_t;

/* RpcReqEapSetCertificateAndKey. */
typedef struct {
    eh_rpc_blob_t  client_cert;
    eh_rpc_blob_t  private_key;
    eh_rpc_blob_t  private_key_password;
} eh_rpc_eap_cert_key_t;

/* WifiItwtSetupConfig mirror. */
typedef struct {
    uint32_t  setup_cmd;
    uint32_t  bitmask_1;
    uint32_t  min_wake_dura;
    uint32_t  wake_invl_mant;
    uint32_t  twt_id;
    uint32_t  timeout_time_ms;
} eh_rpc_itwt_setup_t;

typedef struct { int32_t  flow_id; }                         eh_rpc_itwt_teardown_t;
typedef struct { int32_t  flow_id; int32_t suspend_time_ms; } eh_rpc_itwt_suspend_t;
typedef struct { int32_t  timeout_ms; }                      eh_rpc_itwt_probe_t;
typedef struct { int32_t  offset_us; }                       eh_rpc_itwt_twt_offset_t;
typedef struct { int32_t  flow_id_bitmap; }                  eh_rpc_itwt_flow_status_t;

typedef struct { bool  cb; }                                 eh_rpc_dpp_init_t;
typedef struct {
    int32_t        type;
    eh_rpc_blob_t  chan_list;
    eh_rpc_blob_t  key;
    eh_rpc_blob_t  info;
} eh_rpc_dpp_bootstrap_t;

typedef struct { bool    enable; }                           eh_rpc_eap_bool_t;
typedef struct { int32_t value; }                            eh_rpc_eap_int_t;

typedef struct {
    int32_t fast_provisioning;
    int32_t fast_max_pac_list_len;
    bool    fast_pac_format_binary;
} eh_rpc_eap_fast_t;

/* Network-Split DHCP/DNS; blobs heap-owned on Req/Resp/Event paths. */
typedef struct {
    int32_t        iface;
    int32_t        net_link_up;
    int32_t        dhcp_up;
    eh_rpc_blob_t  dhcp_ip;
    eh_rpc_blob_t  dhcp_nm;
    eh_rpc_blob_t  dhcp_gw;
    int32_t        dns_up;
    eh_rpc_blob_t  dns_ip;
    int32_t        dns_type;
    int32_t        resp;             /* event-only */
} eh_rpc_dhcp_dns_t;

/* iTWT_Setup event: status/reason + flattened WifiItwtSetupConfig. */
typedef struct {
    int32_t   status;
    uint32_t  reason;
    uint64_t  target_wake_time;
    uint32_t  setup_cmd;
    uint32_t  bitmask_1;
    uint32_t  min_wake_dura;
    uint32_t  wake_invl_mant;
    uint32_t  twt_id;
    uint32_t  timeout_time_ms;
} eh_rpc_itwt_setup_evt_t;

typedef struct {
    uint32_t  flow_id;
    uint32_t  status;
} eh_rpc_itwt_teardown_evt_t;

typedef struct {
    int32_t   status;
    uint32_t  flow_id_bitmap;
} eh_rpc_itwt_suspend_evt_t;

typedef struct {
    int32_t   status;
    uint32_t  reason;
} eh_rpc_itwt_probe_evt_t;

/* DPP URI event: qrcode is heap-owned. */
typedef struct {
    eh_rpc_blob_t  qrcode;
} eh_rpc_dpp_uri_evt_t;

/* DPP Cfg Received event (flattened WifiStaConfig). */
typedef struct {
    uint8_t   ssid[EH_RPC_SSID_LEN + 1];
    uint32_t  ssid_len;
    uint8_t   password[EH_RPC_PASSWORD_LEN + 1];
    uint8_t   bssid[EH_RPC_MAC_LEN];
    bool      bssid_set;
    int32_t   authmode;
} eh_rpc_dpp_cfg_evt_t;

typedef struct { int32_t  reason; } eh_rpc_dpp_fail_evt_t;

/* WIFI_EVENT_DPP_* — distinct from Supplicant DPP above; routes to
 * WIFI_EVENT loop, not single-slot subscribers. */
typedef struct {
    eh_rpc_blob_t  qrcode;
} eh_rpc_wifi_dpp_uri_ready_evt_t;

typedef struct {
    uint8_t   ssid[EH_RPC_SSID_LEN + 1];
    uint32_t  ssid_len;
    uint8_t   password[EH_RPC_PASSWORD_LEN + 1];
    uint8_t   bssid[EH_RPC_MAC_LEN];
    bool      bssid_set;
    int32_t   authmode;
} eh_rpc_wifi_dpp_cfg_recvd_evt_t;

typedef struct { int32_t  reason; } eh_rpc_wifi_dpp_fail_evt_t;

/* Memory monitor event (flattened HeapInfo). */
typedef struct {
    uint32_t free_size;
    uint32_t largest_free_block;
} eh_rpc_mem_info_t;

typedef struct {
    eh_rpc_mem_info_t  cap_dma;
    eh_rpc_mem_info_t  cap_8bit;
} eh_rpc_cap_info_t;

typedef struct {
    uint32_t           curr_total_free_heap_size;
    uint32_t           curr_min_free_heap_size;
    eh_rpc_cap_info_t  curr_internal;
    eh_rpc_cap_info_t  curr_external;
} eh_rpc_mem_monitor_evt_t;

/* Req/Resp_MemMonitor. */
typedef struct {
    RpcMemMonitorConfig config;
    bool               report_always;
    uint32_t           interval_sec;
    uint32_t           internal_threshold_dma;
    uint32_t           internal_threshold_8bit;
    uint32_t           external_threshold_dma;
    uint32_t           external_threshold_8bit;
    uint32_t           curr_total_heap_size;
    eh_rpc_cap_info_t  curr_internal;
    eh_rpc_cap_info_t  curr_external;
} eh_rpc_mem_monitor_t;

typedef struct eh_rpc_ctrl_cmd_s {
    uint8_t  msg_type;
    int32_t  msg_id;
    uint32_t uid;
    int32_t  resp_event_status;
    /* Async tail; zero-init selects sync path. See eh_rpc_ctrl_cmd_prefix_t. */
    eh_host_rpc_rsp_cb_t rpc_rsp_cb;
    void                *rpc_rsp_cb_ctx;
    uint32_t             rsp_timeout_ms;

    union {
        eh_rpc_mac_addr_t          wifi_mac;
        eh_rpc_iface_mac_addr_t    iface_mac_addr;
        eh_rpc_wifi_mode_t         wifi_mode;
        eh_rpc_wifi_ps_t           wifi_ps;
        eh_rpc_wifi_tx_power_t     wifi_tx_power;
        eh_rpc_wifi_init_cfg_t     wifi_init_cfg;
        eh_rpc_wifi_cfg_t          wifi_cfg;
        eh_rpc_wifi_scan_cfg_t     wifi_scan_cfg;
        eh_rpc_wifi_scan_params_t  wifi_scan_params;
        eh_rpc_ota_write_t         ota_write;
        eh_rpc_heartbeat_cfg_t     heartbeat_cfg;
        eh_rpc_country_code_t      country_code;
        eh_rpc_wifi_country_full_t wifi_country;
        eh_rpc_fw_version_t        fw_version;
        eh_rpc_wifi_protocols_t    wifi_protocols;
        eh_rpc_wifi_bandwidths_t   wifi_bandwidths;
        eh_rpc_wifi_twt_config_t   wifi_twt_config;
        eh_rpc_sta_list_t          sta_list;
        eh_rpc_ap_records_t        ap_records;
        eh_rpc_mem_monitor_t       mem_monitor;
        eh_rpc_gpio_cfg_t          gpio_cfg;
        eh_rpc_gpio_level_t        gpio_level;
        eh_rpc_gpio_direction_t    gpio_direction;
        eh_rpc_gpio_pull_t         gpio_pull;
        eh_rpc_gpio_num_t          gpio_num;
        eh_rpc_ext_coex_cfg_t      ext_coex;
        eh_rpc_feature_ctrl_t      feat_ctrl;
        eh_rpc_peer_data_blob_t    peer_data;

        eh_rpc_blob_t              eap_blob;
        eh_rpc_eap_cert_key_t      eap_cert_key;
        eh_rpc_eap_bool_t          eap_bool;
        eh_rpc_eap_int_t           eap_int;
        eh_rpc_eap_fast_t          eap_fast;
        eh_rpc_itwt_setup_t        itwt_setup;
        eh_rpc_itwt_teardown_t     itwt_teardown;
        eh_rpc_itwt_suspend_t      itwt_suspend;
        eh_rpc_itwt_probe_t        itwt_probe;
        eh_rpc_itwt_twt_offset_t   itwt_twt_offset;
        eh_rpc_itwt_flow_status_t  itwt_flow_status;
        eh_rpc_dpp_init_t          dpp_init;
        eh_rpc_dpp_bootstrap_t     dpp_bootstrap;
        eh_rpc_dhcp_dns_t          dhcp_dns;

        eh_rpc_esp_init_t          e_init;
        eh_rpc_heartbeat_evt_t     e_heartbeat;
        eh_rpc_scan_done_t         e_scan_done;
        eh_rpc_sta_connected_t     e_sta_connected;
        eh_rpc_sta_disconnected_t  e_sta_disconnected;
        eh_rpc_ap_staconnected_t      e_wifi_ap_staconnected;
        eh_rpc_ap_stadisconnected_t   e_wifi_ap_stadisconnected;
        eh_rpc_wifi_simple_t       e_wifi_simple;
        eh_rpc_peer_data_blob_t    e_peer_data;
        eh_rpc_mem_monitor_evt_t   e_mem_monitor;

        eh_rpc_itwt_setup_evt_t    e_itwt_setup;
        eh_rpc_itwt_teardown_evt_t e_itwt_teardown;
        eh_rpc_itwt_suspend_evt_t  e_itwt_suspend;
        eh_rpc_itwt_probe_evt_t    e_itwt_probe;
        eh_rpc_dpp_uri_evt_t       e_dpp_uri;
        eh_rpc_dpp_cfg_evt_t       e_dpp_cfg;
        eh_rpc_dpp_fail_evt_t      e_dpp_fail;
        eh_rpc_wifi_dpp_uri_ready_evt_t  e_wifi_dpp_uri_ready;
        eh_rpc_wifi_dpp_cfg_recvd_evt_t  e_wifi_dpp_cfg_recvd;
        eh_rpc_wifi_dpp_fail_evt_t       e_wifi_dpp_fail;
    } u;
} eh_rpc_ctrl_cmd_t;

/* Layout-compat with eh_rpc_ctrl_cmd_prefix_t. */
_Static_assert(offsetof(struct eh_rpc_ctrl_cmd_s, msg_type) ==
               offsetof(eh_rpc_ctrl_cmd_prefix_t, msg_type),
               "ctrl_cmd prefix mismatch: msg_type offset");
_Static_assert(offsetof(struct eh_rpc_ctrl_cmd_s, msg_id) ==
               offsetof(eh_rpc_ctrl_cmd_prefix_t, msg_id),
               "ctrl_cmd prefix mismatch: msg_id offset");
_Static_assert(offsetof(struct eh_rpc_ctrl_cmd_s, uid) ==
               offsetof(eh_rpc_ctrl_cmd_prefix_t, uid),
               "ctrl_cmd prefix mismatch: uid offset");
_Static_assert(offsetof(struct eh_rpc_ctrl_cmd_s, resp_event_status) ==
               offsetof(eh_rpc_ctrl_cmd_prefix_t, resp_event_status),
               "ctrl_cmd prefix mismatch: resp_event_status offset");
_Static_assert(offsetof(struct eh_rpc_ctrl_cmd_s, rpc_rsp_cb) ==
               offsetof(eh_rpc_ctrl_cmd_prefix_t, rpc_rsp_cb),
               "ctrl_cmd prefix mismatch: rpc_rsp_cb offset");
_Static_assert(offsetof(struct eh_rpc_ctrl_cmd_s, rpc_rsp_cb_ctx) ==
               offsetof(eh_rpc_ctrl_cmd_prefix_t, rpc_rsp_cb_ctx),
               "ctrl_cmd prefix mismatch: rpc_rsp_cb_ctx offset");
_Static_assert(offsetof(struct eh_rpc_ctrl_cmd_s, rsp_timeout_ms) ==
               offsetof(eh_rpc_ctrl_cmd_prefix_t, rsp_timeout_ms),
               "ctrl_cmd prefix mismatch: rsp_timeout_ms offset");

eh_rpc_ctrl_cmd_t *eh_rpc_ctrl_cmd_alloc(void);
void               eh_rpc_ctrl_cmd_free(eh_rpc_ctrl_cmd_t *c);

#ifdef __cplusplus
}
#endif

#endif /* EH_HOST_FEAT_RPC_EXT_V2_TYPES_H_ */
