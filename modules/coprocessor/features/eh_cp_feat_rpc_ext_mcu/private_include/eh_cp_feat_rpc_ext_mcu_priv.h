/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2025 Espressif Systems (Shanghai) PTE LTD */

#ifndef __EH_CP_BASE_RPC_MCU_PRIV_H__
#define __EH_CP_BASE_RPC_MCU_PRIV_H__

/* Essential system headers */
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "esp_err.h"
#include "esp_wifi_types.h"
#include "esp_event.h"
#include "esp_idf_version.h"
#include "eh_cp_master_config.h"

/* Component headers */
#if EH_CP_FEAT_PEER_DATA_TRANSFER_READY
#include "eh_cp_feat_peer_data.h"
#endif
#if EH_CP_FEAT_NW_SPLIT_READY && defined(CONFIG_LWIP_ENABLE)
#include "eh_cp_feat_nw_split_apis.h"
#include "eh_cp_feat_nw_split_events.h"
#endif
#include "eh_cp_feat_rpc_ext_mcu.h"
#include "eh_cp_event.h"
#include "eh_cp_feat_rpc_ext_mcu_pbuf.h"
#include "eh_cp_feat_rpc_ext_mcu_bitmasks.h"
//#include "eh_cp_ext_err_code_rpc.h"
#include "eh_log.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PCOM_EP_REQ  "RPCRsp"
#define PCOM_EP_EVT  "RPCEvt"

/* Constants */
#define BSSID_BYTES_SIZE       6

/* MCU component internal state variables - WiFi related */
extern wifi_event_sta_connected_t mcu_lkg_sta_connected_event;

/* MCU component internal state variables - System/OTA related */
extern TimerHandle_t handle_heartbeat_task;
extern uint32_t hb_num;


extern esp_event_handler_instance_t instance_any_id;
extern esp_event_handler_instance_t instance_ip;
//extern volatile uint8_t mcu_station_got_ip;


#if EH_CP_FEAT_WIFI_EXT_ENT_READY
/* WiFi Enterprise cert globals — owned by eh_cp_feat_wifi_ext_ent */
extern unsigned char *g_ca_cert;
extern int g_ca_cert_len;
extern unsigned char *g_client_cert;
extern int g_client_cert_len;
extern unsigned char *g_private_key;
extern int g_private_key_len;
extern unsigned char *g_private_key_password;
extern int g_private_key_passwd_len;
#endif /* EH_CP_FEAT_WIFI_EXT_ENT_READY */


/* Constants */
#define SUCCESS                     0
#define FAILURE                     -1

/* Helper macros */
#ifndef min
#define min(a, b) ((a) < (b) ? (a) : (b))
#endif


/* Configuration lengths */
#define SSID_LENGTH             32
#define PASSWORD_LENGTH         64
#define VENDOR_OUI_BUF          3

/* MAC address string formatting */
#define MAC_STR_LEN                 17
#define MAC2STR(a)                  (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
#define MACSTR                      "%02x:%02x:%02x:%02x:%02x:%02x"

/* Memory management */
#define mem_free(x)                 \
        {                           \
            if (x) {                \
                free(x);            \
                x = NULL;           \
            }                       \
        }

/* Error codes */
#define RPC_ERR_MEMORY_FAILURE      -2

/* Use standard network split structure from extension system */

/* RPC helper macros from eh_cp_mcu/slave/main/slave_control.h */
#define RPC_TEMPLATE(RspTyPe, RspStRuCt, ReqType, ReqStruct, InIt_FuN)          \
  RspTyPe *resp_payload = NULL;                                                 \
  ReqType *req_payload = NULL;                                                  \
  if (!req || !resp || !req->ReqStruct) {                                       \
    ESP_LOGE(TAG, "Invalid parameters");                                        \
    return ESP_FAIL;                                                            \
  }                                                                             \
  req_payload = req->ReqStruct;                                                 \
  resp_payload = (RspTyPe *)calloc(1, sizeof(RspTyPe));                         \
  if (!resp_payload) {                                                          \
      ESP_LOGE(TAG, "Failed to alloc mem for resp.%s\n",#RspStRuCt);            \
      return ESP_ERR_NO_MEM;                                                    \
  }                                                                             \
  resp->RspStRuCt = resp_payload;                                               \
  InIt_FuN(resp_payload);                                                       \
  resp_payload->resp = SUCCESS;

#define RPC_TEMPLATE_SIMPLE(RspTyPe, RspStRuCt, ReqType, ReqStruct, InIt_FuN)   \
  RspTyPe *resp_payload = NULL;                                                 \
  if (!req || !resp) {                                                          \
    ESP_LOGE(TAG, "Invalid parameters");                                        \
    return ESP_FAIL;                                                            \
  }                                                                             \
  resp_payload = (RspTyPe *)calloc(1, sizeof(RspTyPe));                         \
  if (!resp_payload) {                                                          \
      ESP_LOGE(TAG, "Failed to alloc mem for resp.%s\n",#RspStRuCt);            \
      return ESP_ERR_NO_MEM;                                                    \
  }                                                                             \
  resp->RspStRuCt = resp_payload;                                               \
  InIt_FuN(resp_payload);                                                       \
  resp_payload->resp = SUCCESS;

#define RPC_RESP_ASSIGN_FIELD(PaRaM)                                            \
  resp_payload->PaRaM = PaRaM

#define RPC_RET_FAIL_IF(ConDiTiOn) do {                                         \
  int rEt = (ConDiTiOn);                                                        \
  if (rEt) {                                                                    \
    resp_payload->resp = rEt;                                                   \
    ESP_LOGE(TAG, "%s:%u failed [%s] = [%d]", __func__,__LINE__,#ConDiTiOn, rEt); \
    return ESP_OK;                                                              \
  }                                                                             \
} while(0);

#define RPC_ALLOC_ELEMENT(TyPe,MsG_StRuCt,InIt_FuN) {                         \
    TyPe *NeW_AllocN = (TyPe *)calloc(1, sizeof(TyPe));                       \
    if (!NeW_AllocN) {                                                        \
        ESP_LOGI(TAG,"Failed to allocate memory for req.%s\n",#MsG_StRuCt);   \
        resp_payload->resp = RPC_ERR_MEMORY_FAILURE;                          \
        goto err;                                                             \
    }                                                                         \
    MsG_StRuCt = NeW_AllocN;                                                  \
    InIt_FuN(MsG_StRuCt);                                                     \
}

#define RPC_REQ_COPY_BYTES(dest, src, num_bytes)                                \
  if (src.len && src.data)                                                      \
    memcpy((char*)dest, src.data, min(min(sizeof(dest), num_bytes), src.len));

#define RPC_REQ_COPY_STR RPC_REQ_COPY_BYTES

#define RPC_RESP_COPY_STR(dest, src, max_len)                                   \
  if (src) {                                                                    \
    dest.data = (uint8_t*)strndup((char*)src, max_len);                         \
    if (!dest.data) {                                                           \
      ESP_LOGE(TAG, "%s:%u Failed to duplicate bytes\n",__func__,__LINE__);     \
      resp_payload->resp = FAILURE;                                             \
      return ESP_OK;                                                            \
    }                                                                           \
    dest.len = min(max_len,strlen((char*)src)+1);                                 \
  }

#define RPC_RESP_COPY_BYTES_SRC_UNCHECKED(dest, src, num)                       \
  do {                                                                          \
    if (num) {                                                                  \
      dest.data = (uint8_t *)calloc(1, num);                                    \
      if (!dest.data) {                                                         \
        ESP_LOGE(TAG, "%s:%u Failed to duplicate bytes\n",__func__,__LINE__);   \
        resp_payload->resp = FAILURE;                                           \
        return ESP_OK;                                                          \
      }                                                                         \
      memcpy(dest.data, src, num);                                              \
      dest.len = num;                                                             \
    }                                                                           \
  } while(0)

#define RPC_RESP_COPY_BYTES(dest, src, num)                                     \
  if (src) {                                                                    \
    RPC_RESP_COPY_BYTES_SRC_UNCHECKED(dest, src, num);                          \
  }

#define RPC_COPY_STR(dest, src, max_len)                                        \
  if (src) {                                                                    \
    dest.data = (uint8_t*)strndup((char*)src, max_len);                         \
    if (!dest.data) {                                                           \
      ESP_LOGE(TAG, "%s:%u Failed to duplicate bytes\n",__func__,__LINE__);     \
      return FAILURE;                                                           \
    }                                                                           \
    dest.len = min(max_len,strlen((char*)src)+1);                                 \
  }

#define RPC_COPY_BYTES(dest, src, num)                                          \
  do {                                                                          \
    if (num) {                                                                  \
      dest.data = (uint8_t *)calloc(1, num);                                    \
      if (!dest.data) {                                                         \
        ESP_LOGE(TAG, "%s:%u Failed to duplicate bytes\n",__func__,__LINE__);   \
        return FAILURE;                                                         \
      }                                                                         \
      memcpy(dest.data, src, num);                                              \
      dest.len = num;                                                             \
    }                                                                           \
  } while(0)

  #define NTFY_TEMPLATE_SIMPLE(NtFy_MsgId, NtFy_TyPe, NtFy_StRuCt, InIt_FuN)           \
  NtFy_TyPe *ntfy_payload = NULL;                                             \
  ntfy_payload = (NtFy_TyPe*)calloc(1,sizeof(NtFy_TyPe));                     \
  if (!ntfy_payload) {                                                        \
      ESP_LOGE(TAG,"Failed to allocate memory");                                \
      return ESP_ERR_NO_MEM;                                                    \
  }                                                                           \
  InIt_FuN(ntfy_payload);                                                     \
  ntfy->payload_case = NtFy_MsgId;                                            \
  ntfy->NtFy_StRuCt = ntfy_payload;

/* Notification helper macros from eh_cp_mcu/slave/main/slave_control.h */
#define NTFY_TEMPLATE(NtFy_MsgId, NtFy_TyPe, NtFy_StRuCt, InIt_FuN)           \
    NtFy_TyPe *ntfy_payload = NULL;                                             \
    ntfy_payload = (NtFy_TyPe*)calloc(1,sizeof(NtFy_TyPe));                     \
    if (!ntfy_payload) {                                                        \
        ESP_LOGE(TAG,"Failed to allocate memory");                                \
        return ESP_ERR_NO_MEM;                                                    \
    }                                                                           \
    InIt_FuN(ntfy_payload);                                                     \
    ntfy->payload_case = NtFy_MsgId;                                            \
    ntfy->NtFy_StRuCt = ntfy_payload;                                           \
    ntfy_payload->resp = SUCCESS;

#define NTFY_ALLOC_ELEMENT(TyPe,MsG_StRuCt,InIt_FuN) {                        \
    TyPe *NeW_AllocN = (TyPe *)calloc(1, sizeof(TyPe));                       \
    if (!NeW_AllocN) {                                                        \
        ESP_LOGI(TAG,"Failed to allocate memory for req.%s\n",#MsG_StRuCt);   \
        ntfy_payload->resp = RPC_ERR_MEMORY_FAILURE;                          \
        goto err;                                                             \
    }                                                                         \
    MsG_StRuCt = NeW_AllocN;                                                  \
    InIt_FuN(MsG_StRuCt);                                                     \
}

#define NTFY_COPY_BYTES(dest, src, num)                                         \
  do {                                                                          \
    if (num) {                                                                  \
      dest.data = (uint8_t *)calloc(1, num);                                    \
      if (!dest.data) {                                                         \
        ESP_LOGE(TAG, "%s:%u Failed to duplicate bytes\n",__func__,__LINE__);   \
        ntfy_payload->resp = FAILURE;                                           \
        return ESP_OK;                                                          \
      }                                                                         \
      memcpy(dest.data, src, num);                                              \
      dest.len = num;                                                           \
    }                                                                           \
  } while(0)

  #define NTFY_COPY_STR(dest, src, max_len)                                       \
    if (src) {                                                                    \
      dest.data = (uint8_t*)strndup((char*)src, max_len);                         \
      if (!dest.data) {                                                           \
        ESP_LOGE(TAG, "%s:%u Failed to duplicate bytes\n",__func__,__LINE__);     \
        ntfy_payload->resp = FAILURE;                                             \
        return ESP_OK;                                                            \
      }                                                                           \
      dest.len = min(max_len,strlen((char*)src)+1);                               \
    }

/* Protocomm callbacks for protobuf processing - these exist in eh_cp_feat_rpc_ext_mcu_rpc_core.c */
esp_err_t mcu_rpc_event_handler(uint32_t session_id, const uint8_t *inbuf, ssize_t inlen,
    uint8_t **outbuf, ssize_t *outlen, void *priv_data);

esp_err_t mcu_rpc_req_handler(uint32_t session_id, const uint8_t *inbuf, ssize_t inlen,
    uint8_t **outbuf, ssize_t *outlen, void *priv_data);

/* Internal functions - these exist in eh_cp_feat_rpc_ext_mcu_rpc_dispatcher_*.c */
esp_err_t eh_cp_feat_rpc_ext_mcu_rpc_evt_dispatcher(Rpc *ntfy, void *priv_data, const uint8_t *inbuf, size_t inlen);
esp_err_t eh_cp_feat_rpc_ext_mcu_rpc_req_dispatcher(Rpc *req, Rpc *resp, void *priv_data);

/* System RPC Request Handlers - these exist in eh_cp_feat_rpc_ext_mcu_rpc_handler_req_system.c */
esp_err_t req_ota_begin_handler(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_ota_write_handler(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_ota_end_handler(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_ota_activate_handler(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_config_heartbeat(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_get_coprocessor_fw_version(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_iface_mac_addr_set_get(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_iface_mac_addr_len_get(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_feature_control(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_app_get_desc(Rpc *req, Rpc *resp, void *priv_data);
#if EH_CP_FEAT_PEER_DATA_TRANSFER_READY
esp_err_t req_custom_rpc_handler(Rpc *req, Rpc *resp, void *priv_data);
#endif

/* WiFi RPC Request Handlers - Core Functions (exist in eh_cp_feat_rpc_ext_mcu_rpc_handler_req_wifi.c) */
esp_err_t req_wifi_get_mac(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_wifi_set_mac(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_wifi_get_mode(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_wifi_set_mode(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_wifi_set_ps(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_wifi_get_ps(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_wifi_set_max_tx_power(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_wifi_get_max_tx_power(Rpc *req, Rpc *resp, void *priv_data);

/* WiFi RPC Request Handlers - Lifecycle Functions */
esp_err_t req_wifi_init(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_wifi_deinit(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_wifi_start(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_wifi_stop(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_wifi_connect(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_wifi_disconnect(Rpc *req, Rpc *resp, void *priv_data);

/* WiFi RPC Request Handlers - Configuration Functions */
esp_err_t req_wifi_set_config(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_wifi_get_config(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_wifi_scan_params(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_wifi_set_storage(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_wifi_set_protocol(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_wifi_get_protocol(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_wifi_set_bandwidth(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_wifi_get_bandwidth(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_wifi_set_channel(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_wifi_get_channel(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_wifi_set_country_code(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_wifi_get_country_code(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_wifi_set_country(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_wifi_get_country(Rpc *req, Rpc *resp, void *priv_data);

/* WiFi RPC Request Handlers - Scan Functions */
esp_err_t req_wifi_scan_start(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_wifi_scan_stop(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_wifi_scan_get_ap_num(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_wifi_scan_get_ap_record(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_wifi_scan_get_ap_records(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_wifi_clear_ap_list(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_wifi_restore(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_wifi_clear_fast_connect(Rpc *req, Rpc *resp, void *priv_data);

/* WiFi RPC Request Handlers - Station Functions */
esp_err_t req_wifi_sta_get_ap_info(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_wifi_sta_get_rssi(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_wifi_sta_get_aid(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_wifi_sta_get_negotiated_phymode(Rpc *req, Rpc *resp, void *priv_data);

/* WiFi RPC Request Handlers - AP Functions */
esp_err_t req_wifi_deauth_sta(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_wifi_ap_get_sta_list(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_wifi_ap_get_sta_aid(Rpc *req, Rpc *resp, void *priv_data);

/* Missing WiFi and System RPC Request Handlers */
esp_err_t req_wifi_set_inactive_time(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_wifi_get_inactive_time(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_get_coprocessor_fw_version(Rpc *req, Rpc *resp, void *priv_data);

/* Enterprise WiFi (EAP) RPC Request Handlers */
esp_err_t req_wifi_sta_enterprise_enable(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_wifi_sta_enterprise_disable(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_eap_set_identity(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_eap_clear_identity(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_eap_set_username(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_eap_clear_username(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_eap_set_password(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_eap_clear_password(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_eap_set_new_password(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_eap_clear_new_password(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_eap_set_ca_cert(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_eap_clear_ca_cert(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_eap_set_certificate_and_key(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_eap_clear_certificate_and_key(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_eap_get_disable_time_check(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_eap_set_ttls_phase2_method(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_eap_set_suiteb_certification(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_eap_set_pac_file(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_eap_set_fast_params(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_eap_use_default_cert_bundle(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_eap_set_disable_time_check(Rpc *req, Rpc *resp, void *priv_data);
#if EH_CP_WIFI_GOT_EAP_OKC
esp_err_t req_wifi_set_okc_support(Rpc *req, Rpc *resp, void *priv_data);
#endif
#if EH_CP_WIFI_GOT_EAP_SET_DOMAIN_NAME
esp_err_t req_eap_set_domain_name(Rpc *req, Rpc *resp, void *priv_data);
#endif
#if EH_CP_WIFI_GOT_SET_EAP_METHODS
esp_err_t req_eap_set_eap_methods(Rpc *req, Rpc *resp, void *priv_data);
#endif

/* External Coex RPC Request Handler */
#if EH_CP_FEAT_EXT_COEX_READY
esp_err_t req_ext_coex(Rpc *req, Rpc *resp, void *priv_data);
#endif

/* Memory Monitor RPC Request Handler */
#if EH_CP_FEAT_MEM_MONITOR_READY
esp_err_t req_mem_monitor(Rpc *req, Rpc *resp, void *priv_data);
#endif

/* GPIO Expander RPC Request Handlers */
#if EH_CP_FEAT_GPIO_EXP_READY
esp_err_t req_gpio_config(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_gpio_reset(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_gpio_set_level(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_gpio_get_level(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_gpio_set_direction(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_gpio_input_enable(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_gpio_set_pull_mode(Rpc *req, Rpc *resp, void *priv_data);
#endif

/* OTA RPC Request Handlers */
esp_err_t req_ota_begin_handler(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_ota_write_handler(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_ota_end_handler(Rpc *req, Rpc *resp, void *priv_data);

/* Conditional WiFi RPC Request Handlers */
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 0)
esp_err_t req_wifi_set_protocols(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_wifi_get_protocols(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_wifi_set_bandwidths(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_wifi_get_bandwidths(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_wifi_set_band(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_wifi_get_band(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_wifi_set_band_mode(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_wifi_get_band_mode(Rpc *req, Rpc *resp, void *priv_data);
#endif


/* Event handlers */
esp_err_t rpc_evt_ESPInit(Rpc *ntfy);
esp_err_t rpc_evt_heartbeat(Rpc *ntfy);
esp_err_t rpc_evt_sta_scan_done(Rpc *ntfy, const uint8_t *data, ssize_t len, int event_id);
esp_err_t rpc_evt_sta_connected(Rpc *ntfy, const uint8_t *data, ssize_t len, int event_id);
esp_err_t rpc_evt_sta_disconnected(Rpc *ntfy, const uint8_t *data, ssize_t len, int event_id);
esp_err_t rpc_evt_ap_staconn_conn_disconn(Rpc *ntfy, const uint8_t *data, ssize_t len, int event_id);

#if CONFIG_SOC_WIFI_HE_SUPPORT
esp_err_t rpc_evt_itwt_setup(Rpc *ntfy, const uint8_t *data, ssize_t len, int event_id);
esp_err_t rpc_evt_itwt_teardown(Rpc *ntfy, const uint8_t *data, ssize_t len, int event_id);
esp_err_t rpc_evt_itwt_suspend(Rpc *ntfy, const uint8_t *data, ssize_t len, int event_id);
esp_err_t rpc_evt_itwt_probe(Rpc *ntfy, const uint8_t *data, ssize_t len, int event_id);
#endif
esp_err_t rpc_evt_Event_WifiEventNoArgs(Rpc *ntfy, const uint8_t *data, ssize_t len);


#if CONFIG_SOC_WIFI_HE_SUPPORT
#if EH_CP_WIFI_HE_GT_IDF_5_3
esp_err_t req_wifi_sta_twt_config(Rpc *req, Rpc *resp, void *priv_data);
#endif
esp_err_t req_wifi_sta_itwt_setup(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_wifi_sta_itwt_teardown(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_wifi_sta_itwt_suspend(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_wifi_sta_itwt_get_flow_id_status(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_wifi_sta_itwt_send_probe_req(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_wifi_sta_itwt_set_target_wake_time_offset(Rpc *req, Rpc *resp, void *priv_data);
#endif

esp_err_t eh_cp_feat_rpc_ext_mcu_register_wifi_evt_handlers(void);
esp_err_t eh_cp_feat_rpc_ext_mcu_register_system_evt_handlers(void);
esp_err_t eh_cp_feat_rpc_ext_mcu_unregister_wifi_evt_handlers(void);
esp_err_t eh_cp_feat_rpc_ext_mcu_unregister_system_evt_handlers(void);


esp_err_t rpc_evt_Event_DhcpDnsStatus(Rpc *ntfy, const uint8_t *data, ssize_t len);
esp_err_t rpc_evt_custom_rpc(Rpc *ntfy, const uint8_t *data, size_t len);
esp_err_t rpc_evt_mem_monitor(Rpc *ntfy, const uint8_t *inbuf, size_t inlen);

#if EH_CP_FEAT_NW_SPLIT_READY
esp_err_t req_get_dhcp_dns_status(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_set_dhcp_dns_status(Rpc *req, Rpc *resp, void *priv_data);
#endif


typedef enum {
	PAYLOAD_TYPE_RPC_RESP_WIFI_GET_CONFIG,
	PAYLOAD_TYPE_RPC_EVENT_SUPP_DPP_GET_CONFIG,
#if EH_CP_WIFI_DPP
	PAYLOAD_TYPE_RPC_EVENT_WIFI_DPP_GET_CONFIG,
#endif
} rpc_payload_type_t;
esp_err_t copy_wifi_sta_cfg_to_rpc_struct(void *payload, rpc_payload_type_t type,
		wifi_sta_config_t *sta_cfg);
esp_err_t req_iface_mac_addr_len_get(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_iface_mac_addr_set_get(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_feature_control(Rpc *req, Rpc *resp, void *priv_data);

#if EH_CP_FEAT_WIFI_EXT_DPP_READY
#if EH_CP_WIFI_SUPP_DPP
#include "esp_dpp.h"   /* supplicant-based DPP type esp_supp_dpp_event_t */
void dpp_enrollee_event_cb(esp_supp_dpp_event_t event, void *data);
esp_err_t rpc_evt_supp_dpp_uri_ready(Rpc *ntfy,
          const uint8_t *data, ssize_t len);
esp_err_t rpc_evt_supp_dpp_cfg_recvd(Rpc *ntfy,
		const uint8_t *data, ssize_t len);
esp_err_t rpc_evt_supp_dpp_fail(Rpc *ntfy,
		const uint8_t *data, ssize_t len);
esp_err_t req_supp_dpp_init(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_supp_dpp_deinit(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_supp_dpp_bootstrap_gen(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_supp_dpp_start_listen(Rpc *req, Rpc *resp, void *priv_data);
esp_err_t req_supp_dpp_stop_listen(Rpc *req, Rpc *resp, void *priv_data);
#endif // EH_CP_WIFI_SUPP_DPP

#if EH_CP_WIFI_DPP
esp_err_t rpc_evt_wifi_dpp_uri_ready(Rpc *ntfy,
		const uint8_t *data, ssize_t len);
esp_err_t rpc_evt_wifi_dpp_cfg_recvd(Rpc *ntfy,
		const uint8_t *data, ssize_t len);
esp_err_t rpc_evt_wifi_dpp_fail(Rpc *ntfy,
		const uint8_t *data, ssize_t len);
#endif // EH_CP_WIFI_DPP
#endif

#if EH_CP_FEAT_WIFI_EXT_DPP_READY
// DPP URI length max (adjust based on your requirements)
#define DPP_URI_LEN_MAX (512 + 1)

typedef struct {
    uint32_t uri_data_len;
    char uri[DPP_URI_LEN_MAX];
} supp_wifi_event_dpp_uri_ready_t;

typedef struct {
    wifi_config_t wifi_cfg;
} supp_wifi_event_dpp_config_received_t;

typedef struct {
    int failure_reason;
} supp_wifi_event_dpp_failed_t;
#endif // EH_CP_FEAT_WIFI_EXT_DPP_READY

/* Handlers for cross-extension events on the core bus.
 * Called from rpc_mcu init/deinit — NOT from the extension that posts them. */

esp_err_t eh_cp_feat_register_nw_split_evt_handlers(void);
esp_err_t eh_cp_feat_unregister_nw_split_evt_handlers(void);

/* MCU WiFi + system event handlers */
#if EH_CP_FEAT_WIFI_READY
esp_err_t eh_cp_feat_rpc_ext_mcu_register_wifi_evt_handlers(void);
esp_err_t eh_cp_feat_rpc_ext_mcu_unregister_wifi_evt_handlers(void);
#endif
esp_err_t eh_cp_feat_rpc_ext_mcu_register_system_evt_handlers(void);
esp_err_t eh_cp_feat_rpc_ext_mcu_unregister_system_evt_handlers(void);

#ifdef __cplusplus
}
#endif

#endif /* __EH_CP_BASE_RPC_MCU_PRIV_H__ */
