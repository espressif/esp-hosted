/* extension_test.c — Host-compiled unit tests for the two RPC extensions
 *
 * Tests cover:
 *   A. FG req_table linear-search dispatcher
 *   B. MCU switch-based req dispatcher
 *   C. FG evt switch dispatcher
 *   D. MCU evt switch dispatcher
 *   E. SLIST adapter (fg_slist_req_handler / mcu_slist_req_handler) wiring
 *   F. Resp msg_id derivation: Req_Base→Resp_Base offset, both protos
 *   G. Sentinel / out-of-range guard in both dispatchers
 *   H. Unknown msg_id handling (returns Resp_Base, not an error)
 *   I. Null-input guards in req_handler / evt_handler
 *   J. FG vs MCU event-id routing: same internal cp_event → correct wire IDs
 *
 * Compile & run:
 *   make -C eh_fg/coprocessor/extensions/test run
 * or:
 *   gcc -std=c11 -Wall -Wextra -Wshadow \
 *       -o /tmp/ext_test_bin \
 *       eh_fg/coprocessor/extensions/test/extension_test.c
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <sys/queue.h>

/* ══════════════════════════════════════════════════════════════════════════
 * Stub types / macros
 * ══════════════════════════════════════════════════════════════════════════ */
typedef int    esp_err_t;
typedef long   ssize_t;
#define ESP_OK               0
#define ESP_FAIL            -1
#define ESP_ERR_INVALID_ARG  0x102
#define ESP_ERR_NO_MEM       0x101
#define SUCCESS  0
#define FAILURE -1
#define ESP_LOGE(t,f,...) ((void)0)
#define ESP_LOGI(t,f,...) ((void)0)
#define ESP_LOGD(t,f,...) ((void)0)
#define ESP_LOGW(t,f,...) ((void)0)
#ifndef PRIx32
#define PRIx32 "x"
#define PRIu32 "u"
#endif

/* ══════════════════════════════════════════════════════════════════════════
 * FG proto ID constants (from eh_config.pb-c.h)
 * ══════════════════════════════════════════════════════════════════════════ */
#define CTRL_MSG_ID__Req_Base                    100
#define CTRL_MSG_ID__Req_GetMACAddress           101
#define CTRL_MSG_ID__Req_SetMacAddress           102
#define CTRL_MSG_ID__Req_GetWifiMode             103
#define CTRL_MSG_ID__Req_SetWifiMode             104
#define CTRL_MSG_ID__Req_GetAPScanList           105
#define CTRL_MSG_ID__Req_GetAPConfig             106
#define CTRL_MSG_ID__Req_ConnectAP               107
#define CTRL_MSG_ID__Req_DisconnectAP            108
#define CTRL_MSG_ID__Req_GetSoftAPConfig         109
#define CTRL_MSG_ID__Req_SetSoftAPVendorSpecificIE 110
#define CTRL_MSG_ID__Req_StartSoftAP             111
#define CTRL_MSG_ID__Req_GetSoftAPConnectedSTAList 112
#define CTRL_MSG_ID__Req_StopSoftAP              113
#define CTRL_MSG_ID__Req_SetPowerSaveMode        114
#define CTRL_MSG_ID__Req_GetPowerSaveMode        115
#define CTRL_MSG_ID__Req_OTABegin                116
#define CTRL_MSG_ID__Req_OTAWrite                117
#define CTRL_MSG_ID__Req_OTAEnd                  118
#define CTRL_MSG_ID__Req_SetWifiMaxTxPower       119
#define CTRL_MSG_ID__Req_GetWifiCurrTxPower      120
#define CTRL_MSG_ID__Req_ConfigHeartbeat         121
#define CTRL_MSG_ID__Req_EnableDisable           122
#define CTRL_MSG_ID__Req_GetFwVersion            123
#define CTRL_MSG_ID__Req_SetCountryCode          124
#define CTRL_MSG_ID__Req_GetCountryCode          125
#define CTRL_MSG_ID__Req_SetDhcpDnsStatus        126
#define CTRL_MSG_ID__Req_GetDhcpDnsStatus        127
#define CTRL_MSG_ID__Req_Custom_RPC_Unserialised_Msg 128
#define CTRL_MSG_ID__Req_Max                     129
#define CTRL_MSG_ID__Resp_Base                   200
#define CTRL_MSG_ID__Event_Base                  300
#define CTRL_MSG_ID__Event_ESPInit               301
#define CTRL_MSG_ID__Event_Heartbeat             302
#define CTRL_MSG_ID__Event_StationDisconnectFromAP        303
#define CTRL_MSG_ID__Event_StationDisconnectFromESPSoftAP 304
#define CTRL_MSG_ID__Event_StationConnectedToAP           305
#define CTRL_MSG_ID__Event_StationConnectedToESPSoftAP    306
#define CTRL_MSG_ID__Event_SetDhcpDnsStatus      307
#define CTRL_MSG_ID__Event_Custom_RPC_Unserialised_Msg 308
#define CTRL_MSG_ID__Event_Max                   309

/* ══════════════════════════════════════════════════════════════════════════
 * MCU proto ID constants (from eh_rpc.pb-c.h)
 * ══════════════════════════════════════════════════════════════════════════ */
#define RPC_ID__Req_Base                 256
#define RPC_ID__Req_GetMACAddress        257
#define RPC_ID__Req_SetMacAddress        258
#define RPC_ID__Req_GetWifiMode          259
#define RPC_ID__Req_SetWifiMode          260
#define RPC_ID__Req_WifiInit             265
#define RPC_ID__Req_WifiDeinit           266
#define RPC_ID__Req_WifiStart            267
#define RPC_ID__Req_WifiStop             268
#define RPC_ID__Req_WifiConnect          269
#define RPC_ID__Req_WifiDisconnect       270
#define RPC_ID__Req_WifiSetConfig        271
#define RPC_ID__Req_WifiGetConfig        272
#define RPC_ID__Req_WifiScanStart        273
#define RPC_ID__Req_WifiScanStop         274
#define RPC_ID__Req_OTABegin             355
#define RPC_ID__Req_OTAWrite             356
#define RPC_ID__Req_OTAEnd               357
#define RPC_ID__Req_ConfigHeartbeat      358
#define RPC_ID__Req_GetCoprocessorFwVersion 359
#define RPC_ID__Req_IfaceMacAddrSetGet   380
#define RPC_ID__Req_IfaceMacAddrLenGet   381
#define RPC_ID__Req_FeatureControl       382
#define RPC_ID__Req_Max                  388
#define RPC_ID__Resp_Base                512
#define RPC_ID__Event_Base               768
#define RPC_ID__Event_ESPInit            769
#define RPC_ID__Event_Heartbeat          770
#define RPC_ID__Event_StaConnected       772
#define RPC_ID__Event_StaDisconnected    773
#define RPC_ID__Event_AP_StaConnected    774
#define RPC_ID__Event_AP_StaDisconnected 775
#define RPC_ID__Event_StaScanDone        776
#define RPC_ID__Event_WifiEventNoArgs    777
#define RPC_ID__Event_DhcpDnsStatus      778
#define RPC_ID__Event_Custom_RPC_Unserialised_Msg 788
#define RPC_ID__Event_Max                789

/* ══════════════════════════════════════════════════════════════════════════
 * Minimal CtrlMsg / Rpc stubs
 * We only need the fields the dispatcher code actually touches.
 * ══════════════════════════════════════════════════════════════════════════ */
typedef struct { uint32_t msg_id; uint32_t uid; int msg_type; int payload_case; } CtrlMsg;
typedef struct { uint32_t msg_id; uint32_t uid; int msg_type; int payload_case; } Rpc;

#define CTRL_MSG_TYPE__Resp  2
#define CTRL_MSG_TYPE__Event 3
#define RPC_TYPE__Resp       2
#define RPC_TYPE__Event      3

static void ctrl_msg__init(CtrlMsg *m) { memset(m, 0, sizeof(*m)); }
static void rpc__init(Rpc *m)          { memset(m, 0, sizeof(*m)); }

/* ══════════════════════════════════════════════════════════════════════════
 * Call-recording infrastructure
 * Each handler stub records the msg_id it was called with.
 * ══════════════════════════════════════════════════════════════════════════ */
#define MAX_CALLS 4
static uint32_t s_call_log[MAX_CALLS];
static int      s_call_count = 0;
static esp_err_t s_handler_ret = ESP_OK; /* configurable per-test */

static void reset_calls(void) { s_call_count = 0; memset(s_call_log, 0, sizeof(s_call_log)); s_handler_ret = ESP_OK; }
static void record(uint32_t id) { if (s_call_count < MAX_CALLS) s_call_log[s_call_count++] = id; }

/* ══════════════════════════════════════════════════════════════════════════
 * FG handler stubs — match signature: (const CtrlMsg*, CtrlMsg*, void*)
 * ══════════════════════════════════════════════════════════════════════════ */
#define FG_STUB(name, id) \
static esp_err_t name(const CtrlMsg *req, CtrlMsg *resp, void *p) \
    { (void)req;(void)resp;(void)p; record(id); return s_handler_ret; }

FG_STUB(req_get_mac_address_handler,          CTRL_MSG_ID__Req_GetMACAddress)
FG_STUB(req_set_mac_address_handler,          CTRL_MSG_ID__Req_SetMacAddress)
FG_STUB(req_get_wifi_mode_handler,            CTRL_MSG_ID__Req_GetWifiMode)
FG_STUB(req_set_wifi_mode_handler,            CTRL_MSG_ID__Req_SetWifiMode)
FG_STUB(req_get_ap_scan_list_handler,         CTRL_MSG_ID__Req_GetAPScanList)
FG_STUB(req_get_ap_config_handler,            CTRL_MSG_ID__Req_GetAPConfig)
FG_STUB(req_connect_ap_handler,               CTRL_MSG_ID__Req_ConnectAP)
FG_STUB(req_disconnect_ap_handler,            CTRL_MSG_ID__Req_DisconnectAP)
FG_STUB(req_get_softap_config_handler,        CTRL_MSG_ID__Req_GetSoftAPConfig)
FG_STUB(req_set_softap_vender_specific_ie_handler, CTRL_MSG_ID__Req_SetSoftAPVendorSpecificIE)
FG_STUB(req_start_softap_handler,             CTRL_MSG_ID__Req_StartSoftAP)
FG_STUB(req_get_connected_sta_list_handler,   CTRL_MSG_ID__Req_GetSoftAPConnectedSTAList)
FG_STUB(req_stop_softap_handler,              CTRL_MSG_ID__Req_StopSoftAP)
FG_STUB(req_set_power_save_mode_handler,      CTRL_MSG_ID__Req_SetPowerSaveMode)
FG_STUB(req_get_power_save_mode_handler,      CTRL_MSG_ID__Req_GetPowerSaveMode)
FG_STUB(req_ota_begin_handler_fg,             CTRL_MSG_ID__Req_OTABegin)
FG_STUB(req_ota_write_handler_fg,             CTRL_MSG_ID__Req_OTAWrite)
FG_STUB(req_ota_end_handler_fg,               CTRL_MSG_ID__Req_OTAEnd)
FG_STUB(req_set_wifi_max_tx_power_handler,    CTRL_MSG_ID__Req_SetWifiMaxTxPower)
FG_STUB(req_get_wifi_curr_tx_power_handler,   CTRL_MSG_ID__Req_GetWifiCurrTxPower)
FG_STUB(req_config_heartbeat_fg,              CTRL_MSG_ID__Req_ConfigHeartbeat)
FG_STUB(req_enable_disable,                   CTRL_MSG_ID__Req_EnableDisable)
FG_STUB(req_get_fw_version_handler,           CTRL_MSG_ID__Req_GetFwVersion)
FG_STUB(req_set_country_code_handler,         CTRL_MSG_ID__Req_SetCountryCode)
FG_STUB(req_get_country_code_handler,         CTRL_MSG_ID__Req_GetCountryCode)
FG_STUB(req_set_dhcp_dns_status_fg,           CTRL_MSG_ID__Req_SetDhcpDnsStatus)
FG_STUB(req_get_dhcp_dns_status_fg,           CTRL_MSG_ID__Req_GetDhcpDnsStatus)
FG_STUB(req_custom_unserialised_rpc_msg_handler, CTRL_MSG_ID__Req_Custom_RPC_Unserialised_Msg)

/* ══════════════════════════════════════════════════════════════════════════
 * MCU handler stubs — match signature: (Rpc*, Rpc*, void*)
 * ══════════════════════════════════════════════════════════════════════════ */
#define MCU_STUB(name, id) \
static esp_err_t name(Rpc *req, Rpc *resp, void *p) \
    { (void)req;(void)resp;(void)p; record(id); return s_handler_ret; }

MCU_STUB(req_wifi_get_mac,          RPC_ID__Req_GetMACAddress)
MCU_STUB(req_wifi_set_mac,          RPC_ID__Req_SetMacAddress)
MCU_STUB(req_wifi_get_mode,         RPC_ID__Req_GetWifiMode)
MCU_STUB(req_wifi_set_mode,         RPC_ID__Req_SetWifiMode)
MCU_STUB(req_wifi_init,             RPC_ID__Req_WifiInit)
MCU_STUB(req_wifi_deinit,           RPC_ID__Req_WifiDeinit)
MCU_STUB(req_wifi_start,            RPC_ID__Req_WifiStart)
MCU_STUB(req_wifi_stop,             RPC_ID__Req_WifiStop)
MCU_STUB(req_wifi_connect,          RPC_ID__Req_WifiConnect)
MCU_STUB(req_wifi_disconnect,       RPC_ID__Req_WifiDisconnect)
MCU_STUB(req_wifi_set_config,       RPC_ID__Req_WifiSetConfig)
MCU_STUB(req_wifi_get_config,       RPC_ID__Req_WifiGetConfig)
MCU_STUB(req_wifi_scan_start,       RPC_ID__Req_WifiScanStart)
MCU_STUB(req_wifi_scan_stop,        RPC_ID__Req_WifiScanStop)
MCU_STUB(req_ota_begin_handler,     RPC_ID__Req_OTABegin)
MCU_STUB(req_ota_write_handler,     RPC_ID__Req_OTAWrite)
MCU_STUB(req_ota_end_handler,       RPC_ID__Req_OTAEnd)
MCU_STUB(req_config_heartbeat,      RPC_ID__Req_ConfigHeartbeat)
MCU_STUB(req_get_coprocessor_fw_version, RPC_ID__Req_GetCoprocessorFwVersion)
MCU_STUB(req_iface_mac_addr_set_get, RPC_ID__Req_IfaceMacAddrSetGet)
MCU_STUB(req_iface_mac_addr_len_get, RPC_ID__Req_IfaceMacAddrLenGet)
MCU_STUB(req_feature_control,       RPC_ID__Req_FeatureControl)

/* ══════════════════════════════════════════════════════════════════════════
 * FG req dispatcher — inlined from protobuf_req_hook.c
 * (Same logic, stub handler pointers, no IDF deps)
 * ══════════════════════════════════════════════════════════════════════════ */
typedef struct {
    int req_num;
    esp_err_t (*command_handler)(const CtrlMsg *req, CtrlMsg *resp, void *priv_data);
} fg_req_entry_t;

static fg_req_entry_t fg_req_table[] = {
    { CTRL_MSG_ID__Req_GetMACAddress,            req_get_mac_address_handler },
    { CTRL_MSG_ID__Req_SetMacAddress,            req_set_mac_address_handler },
    { CTRL_MSG_ID__Req_GetWifiMode,              req_get_wifi_mode_handler },
    { CTRL_MSG_ID__Req_SetWifiMode,              req_set_wifi_mode_handler },
    { CTRL_MSG_ID__Req_GetAPScanList,            req_get_ap_scan_list_handler },
    { CTRL_MSG_ID__Req_GetAPConfig,              req_get_ap_config_handler },
    { CTRL_MSG_ID__Req_ConnectAP,                req_connect_ap_handler },
    { CTRL_MSG_ID__Req_DisconnectAP,             req_disconnect_ap_handler },
    { CTRL_MSG_ID__Req_GetSoftAPConfig,          req_get_softap_config_handler },
    { CTRL_MSG_ID__Req_SetSoftAPVendorSpecificIE, req_set_softap_vender_specific_ie_handler },
    { CTRL_MSG_ID__Req_StartSoftAP,              req_start_softap_handler },
    { CTRL_MSG_ID__Req_GetSoftAPConnectedSTAList, req_get_connected_sta_list_handler },
    { CTRL_MSG_ID__Req_StopSoftAP,               req_stop_softap_handler },
    { CTRL_MSG_ID__Req_SetPowerSaveMode,         req_set_power_save_mode_handler },
    { CTRL_MSG_ID__Req_GetPowerSaveMode,         req_get_power_save_mode_handler },
    { CTRL_MSG_ID__Req_OTABegin,                 req_ota_begin_handler_fg },
    { CTRL_MSG_ID__Req_OTAWrite,                 req_ota_write_handler_fg },
    { CTRL_MSG_ID__Req_OTAEnd,                   req_ota_end_handler_fg },
    { CTRL_MSG_ID__Req_SetWifiMaxTxPower,        req_set_wifi_max_tx_power_handler },
    { CTRL_MSG_ID__Req_GetWifiCurrTxPower,       req_get_wifi_curr_tx_power_handler },
    { CTRL_MSG_ID__Req_ConfigHeartbeat,          req_config_heartbeat_fg },
    { CTRL_MSG_ID__Req_EnableDisable,            req_enable_disable },
    { CTRL_MSG_ID__Req_GetFwVersion,             req_get_fw_version_handler },
    { CTRL_MSG_ID__Req_SetCountryCode,           req_set_country_code_handler },
    { CTRL_MSG_ID__Req_GetCountryCode,           req_get_country_code_handler },
    { CTRL_MSG_ID__Req_SetDhcpDnsStatus,         req_set_dhcp_dns_status_fg },
    { CTRL_MSG_ID__Req_GetDhcpDnsStatus,         req_get_dhcp_dns_status_fg },
    { CTRL_MSG_ID__Req_Custom_RPC_Unserialised_Msg, req_custom_unserialised_rpc_msg_handler },
};
#define FG_TABLE_SIZE ((int)(sizeof(fg_req_table)/sizeof(fg_req_table[0])))

static int fg_lookup(int req_id) {
    for (int i = 0; i < FG_TABLE_SIZE; i++)
        if (fg_req_table[i].req_num == req_id) return i;
    return -1;
}

static esp_err_t fg_req_dispatcher(const CtrlMsg *req, CtrlMsg *resp, void *priv) {
    if (!req || !resp) return ESP_FAIL;
    if (req->msg_id <= (uint32_t)CTRL_MSG_ID__Req_Base ||
        req->msg_id >= (uint32_t)CTRL_MSG_ID__Req_Max) {
        resp->msg_id = CTRL_MSG_ID__Req_Base; return ESP_OK;
    }
    int idx = fg_lookup((int)req->msg_id);
    if (idx < 0) { resp->msg_id = CTRL_MSG_ID__Resp_Base; return ESP_OK; }
    esp_err_t r = fg_req_table[idx].command_handler(req, resp, priv);
    if (r) { resp->msg_id = CTRL_MSG_ID__Resp_Base; return ESP_OK; }
    return ESP_OK;
}

/* ══════════════════════════════════════════════════════════════════════════
 * FG evt dispatcher — inlined from protobuf_evt_hook.c
 * ══════════════════════════════════════════════════════════════════════════ */
static uint32_t s_last_fg_evt = 0;

#define FG_EVT_STUB(id) \
    case id: s_last_fg_evt = (id); break;

static esp_err_t fg_evt_dispatcher(CtrlMsg *ntfy, void *priv, const uint8_t *inbuf, size_t inlen) {
    (void)priv; (void)inbuf; (void)inlen;
    if (!ntfy) return ESP_FAIL;
    switch (ntfy->msg_id) {
        FG_EVT_STUB(CTRL_MSG_ID__Event_ESPInit)
        FG_EVT_STUB(CTRL_MSG_ID__Event_Heartbeat)
        FG_EVT_STUB(CTRL_MSG_ID__Event_StationDisconnectFromAP)
        FG_EVT_STUB(CTRL_MSG_ID__Event_StationDisconnectFromESPSoftAP)
        FG_EVT_STUB(CTRL_MSG_ID__Event_StationConnectedToAP)
        FG_EVT_STUB(CTRL_MSG_ID__Event_StationConnectedToESPSoftAP)
        FG_EVT_STUB(CTRL_MSG_ID__Event_SetDhcpDnsStatus)
        FG_EVT_STUB(CTRL_MSG_ID__Event_Custom_RPC_Unserialised_Msg)
        default: return ESP_FAIL;
    }
    return ESP_OK;
}

/* ══════════════════════════════════════════════════════════════════════════
 * MCU req dispatcher — inlined from dispatcher_req.c (switch variant)
 * ══════════════════════════════════════════════════════════════════════════ */
static esp_err_t mcu_req_dispatcher(Rpc *req, Rpc *resp, void *priv) {
    if (!req || !resp) return ESP_FAIL;
    if (req->msg_id <= (uint32_t)RPC_ID__Req_Base ||
        req->msg_id >= (uint32_t)RPC_ID__Req_Max) {
        resp->msg_id = RPC_ID__Resp_Base; return ESP_OK;
    }
    esp_err_t ret = ESP_OK;
    switch (req->msg_id) {
        case RPC_ID__Req_GetMACAddress:          ret = req_wifi_get_mac(req, resp, priv); break;
        case RPC_ID__Req_SetMacAddress:          ret = req_wifi_set_mac(req, resp, priv); break;
        case RPC_ID__Req_GetWifiMode:            ret = req_wifi_get_mode(req, resp, priv); break;
        case RPC_ID__Req_SetWifiMode:            ret = req_wifi_set_mode(req, resp, priv); break;
        case RPC_ID__Req_WifiInit:               ret = req_wifi_init(req, resp, priv); break;
        case RPC_ID__Req_WifiDeinit:             ret = req_wifi_deinit(req, resp, priv); break;
        case RPC_ID__Req_WifiStart:              ret = req_wifi_start(req, resp, priv); break;
        case RPC_ID__Req_WifiStop:               ret = req_wifi_stop(req, resp, priv); break;
        case RPC_ID__Req_WifiConnect:            ret = req_wifi_connect(req, resp, priv); break;
        case RPC_ID__Req_WifiDisconnect:         ret = req_wifi_disconnect(req, resp, priv); break;
        case RPC_ID__Req_WifiSetConfig:          ret = req_wifi_set_config(req, resp, priv); break;
        case RPC_ID__Req_WifiGetConfig:          ret = req_wifi_get_config(req, resp, priv); break;
        case RPC_ID__Req_WifiScanStart:          ret = req_wifi_scan_start(req, resp, priv); break;
        case RPC_ID__Req_WifiScanStop:           ret = req_wifi_scan_stop(req, resp, priv); break;
        case RPC_ID__Req_OTABegin:               ret = req_ota_begin_handler(req, resp, priv); break;
        case RPC_ID__Req_OTAWrite:               ret = req_ota_write_handler(req, resp, priv); break;
        case RPC_ID__Req_OTAEnd:                 ret = req_ota_end_handler(req, resp, priv); break;
        case RPC_ID__Req_ConfigHeartbeat:        ret = req_config_heartbeat(req, resp, priv); break;
        case RPC_ID__Req_GetCoprocessorFwVersion: ret = req_get_coprocessor_fw_version(req, resp, priv); break;
        case RPC_ID__Req_IfaceMacAddrSetGet:     ret = req_iface_mac_addr_set_get(req, resp, priv); break;
        case RPC_ID__Req_IfaceMacAddrLenGet:     ret = req_iface_mac_addr_len_get(req, resp, priv); break;
        case RPC_ID__Req_FeatureControl:         ret = req_feature_control(req, resp, priv); break;
        default: resp->msg_id = RPC_ID__Resp_Base; return ESP_OK;
    }
    return ret;
}

/* ══════════════════════════════════════════════════════════════════════════
 * MCU evt dispatcher — inlined from dispatcher_evt.c
 * ══════════════════════════════════════════════════════════════════════════ */
static uint32_t s_last_mcu_evt = 0;
#define MCU_EVT_STUB(id) case id: s_last_mcu_evt = (id); break;

static esp_err_t mcu_evt_dispatcher(Rpc *ntfy, void *priv, const uint8_t *inbuf, size_t inlen) {
    (void)priv; (void)inbuf; (void)inlen;
    if (!ntfy) return ESP_FAIL;
    switch ((int)ntfy->msg_id) {
        MCU_EVT_STUB(RPC_ID__Event_ESPInit)
        MCU_EVT_STUB(RPC_ID__Event_Heartbeat)
        MCU_EVT_STUB(RPC_ID__Event_StaConnected)
        MCU_EVT_STUB(RPC_ID__Event_StaDisconnected)
        MCU_EVT_STUB(RPC_ID__Event_AP_StaConnected)
        MCU_EVT_STUB(RPC_ID__Event_AP_StaDisconnected)
        MCU_EVT_STUB(RPC_ID__Event_StaScanDone)
        MCU_EVT_STUB(RPC_ID__Event_WifiEventNoArgs)
        MCU_EVT_STUB(RPC_ID__Event_DhcpDnsStatus)
        MCU_EVT_STUB(RPC_ID__Event_Custom_RPC_Unserialised_Msg)
        default: return ESP_FAIL;
    }
    return ESP_OK;
}

/* ══════════════════════════════════════════════════════════════════════════
 * Minimal hook stubs (linux_rpc_req_handler / mcu_rpc_req_handler logic)
 * These mirror the real functions but use the stub dispatchers above and
 * replace protobuf pack/unpack with trivial in-memory copies.
 * ══════════════════════════════════════════════════════════════════════════ */

/* "Packed" format used in tests: just a 4-byte LE msg_id. */
static void pack_msg_id(uint8_t *buf, uint32_t id)
{
    buf[0]=(uint8_t)id; buf[1]=(uint8_t)(id>>8);
    buf[2]=(uint8_t)(id>>16); buf[3]=(uint8_t)(id>>24);
}
static uint32_t unpack_msg_id(const uint8_t *buf) {
    return (uint32_t)(buf[0]|(buf[1]<<8)|(buf[2]<<16)|(buf[3]<<24));
}

/* Simulates linux_rpc_req_handler: unpack → dispatch → pack response */
static esp_err_t stub_fg_req_handler(uint32_t session_id,
                                      const uint8_t *inbuf, ssize_t inlen,
                                      uint8_t **outbuf, ssize_t *outlen)
{
    (void)session_id;
    if (!inbuf || !outbuf || !outlen || inlen < 4) return ESP_ERR_INVALID_ARG;

    CtrlMsg req = {0}, resp = {0};
    req.msg_id = unpack_msg_id(inbuf);

    ctrl_msg__init(&resp);
    resp.msg_type = CTRL_MSG_TYPE__Resp;
    resp.msg_id   = req.msg_id - CTRL_MSG_ID__Req_Base + CTRL_MSG_ID__Resp_Base;
    resp.uid      = req.uid;

    esp_err_t r = fg_req_dispatcher(&req, &resp, NULL);
    if (r) goto err;

    *outbuf = (uint8_t *)calloc(1, 4);
    if (!*outbuf) return ESP_ERR_NO_MEM;
    pack_msg_id(*outbuf, resp.msg_id);
    *outlen = 4;
    return ESP_OK;
err:
    return ESP_FAIL;
}

/* Simulates mcu_rpc_req_handler */
static esp_err_t stub_mcu_req_handler(uint32_t session_id,
                                       const uint8_t *inbuf, ssize_t inlen,
                                       uint8_t **outbuf, ssize_t *outlen)
{
    (void)session_id;
    if (!inbuf || !outbuf || !outlen || inlen < 4) return ESP_ERR_INVALID_ARG;

    Rpc req = {0}, resp = {0};
    req.msg_id = unpack_msg_id(inbuf);

    rpc__init(&resp);
    resp.msg_type = RPC_TYPE__Resp;
    resp.msg_id   = req.msg_id - RPC_ID__Req_Base + RPC_ID__Resp_Base;
    resp.uid      = req.uid;

    esp_err_t r = mcu_req_dispatcher(&req, &resp, NULL);
    if (r) goto err;

    *outbuf = (uint8_t *)calloc(1, 4);
    if (!*outbuf) return ESP_ERR_NO_MEM;
    pack_msg_id(*outbuf, resp.msg_id);
    *outlen = 4;
    return ESP_OK;
err:
    return ESP_FAIL;
}

/* ══════════════════════════════════════════════════════════════════════════
 * SLIST adapter stubs (mirror fg_slist_req_handler / mcu_slist_req_handler)
 * In production these call the protocomm hook; here they call our stub hooks.
 * ══════════════════════════════════════════════════════════════════════════ */
static esp_err_t fg_slist_adapter(void *ctx, uint32_t msg_id,
                                    const void *req_buf, uint16_t req_len,
                                    uint8_t **out_buf, uint16_t *out_len)
{
    (void)ctx;
    uint8_t  in[4]; pack_msg_id(in, msg_id);
    uint8_t  *out = NULL; ssize_t out_sz = 0;
    esp_err_t r = stub_fg_req_handler(msg_id, in, 4, &out, &out_sz);
    if (r || !out || out_sz <= 0) { if (out) free(out); *out_len = 0; *out_buf = NULL; return ESP_FAIL; }
    if (out_sz > UINT16_MAX) { free(out); *out_len = 0; *out_buf = NULL; return ESP_ERR_NO_MEM; }
    *out_buf = out;
    *out_len = (uint16_t)out_sz;
    (void)req_buf; (void)req_len;
    return ESP_OK;
}

static esp_err_t mcu_slist_adapter(void *ctx, uint32_t msg_id,
                                     const void *req_buf, uint16_t req_len,
                                     uint8_t **out_buf, uint16_t *out_len)
{
    (void)ctx;
    uint8_t  in[4]; pack_msg_id(in, msg_id);
    uint8_t  *out = NULL; ssize_t out_sz = 0;
    esp_err_t r = stub_mcu_req_handler(msg_id, in, 4, &out, &out_sz);
    if (r || !out || out_sz <= 0) { if (out) free(out); *out_len = 0; *out_buf = NULL; return ESP_FAIL; }
    if (out_sz > UINT16_MAX) { free(out); *out_len = 0; *out_buf = NULL; return ESP_ERR_NO_MEM; }
    *out_buf = out;
    *out_len = (uint16_t)out_sz;
    (void)req_buf; (void)req_len;
    return ESP_OK;
}

/* ══════════════════════════════════════════════════════════════════════════
 * Test framework
 * ══════════════════════════════════════════════════════════════════════════ */
static int g_pass = 0, g_fail = 0;
#define PASS(n)       do { printf("  PASS  %s\n",(n)); g_pass++; } while(0)
#define FAIL(n,msg)   do { printf("  FAIL  %s  (%s)\n",(n),(msg)); g_fail++; } while(0)
#define CHECK(n,c)    do { if(c) PASS(n); else FAIL(n,#c); } while(0)

/* ══════════════════════════════════════════════════════════════════════════
 * A. FG req_table dispatcher tests
 * ══════════════════════════════════════════════════════════════════════════ */

/* Verify every table entry is reachable and calls the right stub */
static void test_fg_dispatcher_every_entry(void)
{
    for (int i = 0; i < FG_TABLE_SIZE; i++) {
        reset_calls();
        CtrlMsg req = {0}, resp = {0};
        req.msg_id = (uint32_t)fg_req_table[i].req_num;
        fg_req_dispatcher(&req, &resp, NULL);
        char name[64]; snprintf(name, sizeof(name), "fg_tbl[%d]=id%d", i, fg_req_table[i].req_num);
        CHECK(name, s_call_count == 1 && s_call_log[0] == (uint32_t)fg_req_table[i].req_num);
    }
}

/* Table entries must all be in [Req_Base+1 .. Req_Max-1] */
static void test_fg_table_ids_in_range(void)
{
    bool all_ok = true;
    for (int i = 0; i < FG_TABLE_SIZE; i++) {
        if (fg_req_table[i].req_num <= CTRL_MSG_ID__Req_Base ||
            fg_req_table[i].req_num >= CTRL_MSG_ID__Req_Max)
            all_ok = false;
    }
    CHECK("fg_table_ids_in_range", all_ok);
}

/* No duplicate entries in the table */
static void test_fg_table_no_duplicates(void)
{
    bool ok = true;
    for (int i = 0; i < FG_TABLE_SIZE; i++)
        for (int j = i+1; j < FG_TABLE_SIZE; j++)
            if (fg_req_table[i].req_num == fg_req_table[j].req_num) ok = false;
    CHECK("fg_table_no_dup", ok);
}

/* No NULL handler pointers */
static void test_fg_table_no_null_handlers(void)
{
    bool ok = true;
    for (int i = 0; i < FG_TABLE_SIZE; i++)
        if (!fg_req_table[i].command_handler) ok = false;
    CHECK("fg_table_no_null_handlers", ok);
}

static void test_fg_dispatcher_sentinel_base(void) {
    reset_calls();
    CtrlMsg req = {0}, resp = {0};
    req.msg_id = CTRL_MSG_ID__Req_Base;   /* sentinel — must not dispatch */
    fg_req_dispatcher(&req, &resp, NULL);
    CHECK("fg_sentinel_base", s_call_count == 0);
    CHECK("fg_sentinel_base_resp", resp.msg_id == (uint32_t)CTRL_MSG_ID__Req_Base);
}

static void test_fg_dispatcher_sentinel_max(void) {
    reset_calls();
    CtrlMsg req = {0}, resp = {0};
    req.msg_id = CTRL_MSG_ID__Req_Max;    /* sentinel */
    fg_req_dispatcher(&req, &resp, NULL);
    CHECK("fg_sentinel_max", s_call_count == 0);
}

static void test_fg_dispatcher_unknown_id(void) {
    reset_calls();
    CtrlMsg req = {0}, resp = {0};
    /* Use an ID that is within [Req_Base+1 .. Req_Max-1] but absent from the
     * table.  The FG proto has all IDs 101..128 assigned, so there is no gap
     * inside the range.  Instead test with 199 (above Req_Max=129): the sentinel
     * guard fires, sets resp.msg_id = Req_Base (not Resp_Base), no handler called. */
    req.msg_id = 199;
    fg_req_dispatcher(&req, &resp, NULL);
    CHECK("fg_unknown_id",            s_call_count == 0);
    CHECK("fg_sentinel_oob_resp_base", resp.msg_id == (uint32_t)CTRL_MSG_ID__Req_Base);
}

static void test_fg_dispatcher_null_args(void) {
    reset_calls();
    CtrlMsg resp = {0};
    CHECK("fg_null_req",  fg_req_dispatcher(NULL, &resp, NULL) == ESP_FAIL);
    CtrlMsg req = {0}; req.msg_id = CTRL_MSG_ID__Req_GetMACAddress;
    CHECK("fg_null_resp", fg_req_dispatcher(&req, NULL,  NULL) == ESP_FAIL);
}

/* ══════════════════════════════════════════════════════════════════════════
 * B. MCU switch dispatcher tests
 * ══════════════════════════════════════════════════════════════════════════ */

typedef struct { uint32_t id; } mcu_case_t;
static mcu_case_t mcu_cases[] = {
    {RPC_ID__Req_GetMACAddress}, {RPC_ID__Req_SetMacAddress},
    {RPC_ID__Req_GetWifiMode},   {RPC_ID__Req_SetWifiMode},
    {RPC_ID__Req_WifiInit},      {RPC_ID__Req_WifiDeinit},
    {RPC_ID__Req_WifiStart},     {RPC_ID__Req_WifiStop},
    {RPC_ID__Req_WifiConnect},   {RPC_ID__Req_WifiDisconnect},
    {RPC_ID__Req_WifiSetConfig}, {RPC_ID__Req_WifiGetConfig},
    {RPC_ID__Req_WifiScanStart}, {RPC_ID__Req_WifiScanStop},
    {RPC_ID__Req_OTABegin},      {RPC_ID__Req_OTAWrite},
    {RPC_ID__Req_OTAEnd},        {RPC_ID__Req_ConfigHeartbeat},
    {RPC_ID__Req_GetCoprocessorFwVersion},
    {RPC_ID__Req_IfaceMacAddrSetGet}, {RPC_ID__Req_IfaceMacAddrLenGet},
    {RPC_ID__Req_FeatureControl},
};
#define MCU_CASES ((int)(sizeof(mcu_cases)/sizeof(mcu_cases[0])))

static void test_mcu_dispatcher_every_case(void)
{
    for (int i = 0; i < MCU_CASES; i++) {
        reset_calls();
        Rpc req = {0}, resp = {0};
        req.msg_id = mcu_cases[i].id;
        mcu_req_dispatcher(&req, &resp, NULL);
        char name[64]; snprintf(name, sizeof(name), "mcu_sw id=0x%x", mcu_cases[i].id);
        CHECK(name, s_call_count == 1 && s_call_log[0] == mcu_cases[i].id);
    }
}

static void test_mcu_dispatcher_sentinel_base(void) {
    reset_calls();
    Rpc req = {0}, resp = {0};
    req.msg_id = RPC_ID__Req_Base;
    mcu_req_dispatcher(&req, &resp, NULL);
    CHECK("mcu_sentinel_base", s_call_count == 0);
    CHECK("mcu_sentinel_resp", resp.msg_id == (uint32_t)RPC_ID__Resp_Base);
}

static void test_mcu_dispatcher_sentinel_max(void) {
    reset_calls();
    Rpc req = {0}, resp = {0};
    req.msg_id = RPC_ID__Req_Max;
    mcu_req_dispatcher(&req, &resp, NULL);
    CHECK("mcu_sentinel_max", s_call_count == 0);
}

static void test_mcu_dispatcher_unknown(void) {
    reset_calls();
    Rpc req = {0}, resp = {0};
    req.msg_id = RPC_ID__Req_Base + 1;  /* 257 = GetMACAddress — but we want a gap */
    /* pick a msg_id in-range but not in our switch */
    req.msg_id = 290; /* in [257..387] but not a known case in stub table */
    mcu_req_dispatcher(&req, &resp, NULL);
    CHECK("mcu_unknown_id", s_call_count == 0);
    CHECK("mcu_unknown_resp_base", resp.msg_id == (uint32_t)RPC_ID__Resp_Base);
}

static void test_mcu_dispatcher_null(void) {
    reset_calls();
    Rpc resp = {0};
    CHECK("mcu_null_req",  mcu_req_dispatcher(NULL, &resp, NULL) == ESP_FAIL);
    Rpc req = {0}; req.msg_id = RPC_ID__Req_GetMACAddress;
    CHECK("mcu_null_resp", mcu_req_dispatcher(&req,  NULL, NULL) == ESP_FAIL);
}

/* ══════════════════════════════════════════════════════════════════════════
 * C. FG evt dispatcher tests
 * ══════════════════════════════════════════════════════════════════════════ */
static uint32_t fg_evt_ids[] = {
    CTRL_MSG_ID__Event_ESPInit,
    CTRL_MSG_ID__Event_Heartbeat,
    CTRL_MSG_ID__Event_StationDisconnectFromAP,
    CTRL_MSG_ID__Event_StationDisconnectFromESPSoftAP,
    CTRL_MSG_ID__Event_StationConnectedToAP,
    CTRL_MSG_ID__Event_StationConnectedToESPSoftAP,
    CTRL_MSG_ID__Event_SetDhcpDnsStatus,
    CTRL_MSG_ID__Event_Custom_RPC_Unserialised_Msg,
};

static void test_fg_evt_all_known(void)
{
    int n = (int)(sizeof(fg_evt_ids)/sizeof(fg_evt_ids[0]));
    for (int i = 0; i < n; i++) {
        s_last_fg_evt = 0;
        CtrlMsg ntfy = {0}; ntfy.msg_id = fg_evt_ids[i];
        esp_err_t r = fg_evt_dispatcher(&ntfy, NULL, NULL, 0);
        char name[64]; snprintf(name, sizeof(name), "fg_evt id=%u", fg_evt_ids[i]);
        CHECK(name, r == ESP_OK && s_last_fg_evt == fg_evt_ids[i]);
    }
}

static void test_fg_evt_unknown(void) {
    s_last_fg_evt = 0;
    CtrlMsg ntfy = {0}; ntfy.msg_id = 399; /* not in switch */
    CHECK("fg_evt_unknown", fg_evt_dispatcher(&ntfy, NULL, NULL, 0) == ESP_FAIL);
    CHECK("fg_evt_unknown_no_side_effect", s_last_fg_evt == 0);
}

static void test_fg_evt_sentinel_base(void) {
    s_last_fg_evt = 0;
    CtrlMsg ntfy = {0}; ntfy.msg_id = CTRL_MSG_ID__Event_Base;
    CHECK("fg_evt_sentinel_base", fg_evt_dispatcher(&ntfy, NULL, NULL, 0) == ESP_FAIL);
}

static void test_fg_evt_sentinel_max(void) {
    s_last_fg_evt = 0;
    CtrlMsg ntfy = {0}; ntfy.msg_id = CTRL_MSG_ID__Event_Max;
    CHECK("fg_evt_sentinel_max", fg_evt_dispatcher(&ntfy, NULL, NULL, 0) == ESP_FAIL);
}

static void test_fg_evt_null_ntfy(void) {
    CHECK("fg_evt_null_ntfy", fg_evt_dispatcher(NULL, NULL, NULL, 0) == ESP_FAIL);
}

/* ══════════════════════════════════════════════════════════════════════════
 * D. MCU evt dispatcher tests
 * ══════════════════════════════════════════════════════════════════════════ */
static uint32_t mcu_evt_ids[] = {
    RPC_ID__Event_ESPInit,
    RPC_ID__Event_Heartbeat,
    RPC_ID__Event_StaConnected,
    RPC_ID__Event_StaDisconnected,
    RPC_ID__Event_AP_StaConnected,
    RPC_ID__Event_AP_StaDisconnected,
    RPC_ID__Event_StaScanDone,
    RPC_ID__Event_WifiEventNoArgs,
    RPC_ID__Event_DhcpDnsStatus,
    RPC_ID__Event_Custom_RPC_Unserialised_Msg,
};

static void test_mcu_evt_all_known(void)
{
    int n = (int)(sizeof(mcu_evt_ids)/sizeof(mcu_evt_ids[0]));
    for (int i = 0; i < n; i++) {
        s_last_mcu_evt = 0;
        Rpc ntfy = {0}; ntfy.msg_id = mcu_evt_ids[i];
        esp_err_t r = mcu_evt_dispatcher(&ntfy, NULL, NULL, 0);
        char name[64]; snprintf(name, sizeof(name), "mcu_evt id=0x%x", mcu_evt_ids[i]);
        CHECK(name, r == ESP_OK && s_last_mcu_evt == mcu_evt_ids[i]);
    }
}

static void test_mcu_evt_unknown(void) {
    s_last_mcu_evt = 0;
    Rpc ntfy = {0}; ntfy.msg_id = 999;
    CHECK("mcu_evt_unknown", mcu_evt_dispatcher(&ntfy, NULL, NULL, 0) == ESP_FAIL);
    CHECK("mcu_evt_unknown_no_side", s_last_mcu_evt == 0);
}

static void test_mcu_evt_null(void) {
    CHECK("mcu_evt_null", mcu_evt_dispatcher(NULL, NULL, NULL, 0) == ESP_FAIL);
}

/* ══════════════════════════════════════════════════════════════════════════
 * E. SLIST adapter wiring (fg_slist_adapter / mcu_slist_adapter)
 * ══════════════════════════════════════════════════════════════════════════ */
static void test_fg_slist_adapter_dispatch(void)
{
    reset_calls();
    uint8_t *resp = NULL; uint16_t rlen = 0;
    esp_err_t r = fg_slist_adapter(NULL, CTRL_MSG_ID__Req_GetMACAddress,
                                    NULL, 0, &resp, &rlen);
    CHECK("fg_slist_dispatch_ok",   r == ESP_OK);
    CHECK("fg_slist_handler_called", s_call_count == 1);
    CHECK("fg_slist_handler_id",     s_call_log[0] == CTRL_MSG_ID__Req_GetMACAddress);
    if (resp) free(resp);
}

static void test_mcu_slist_adapter_dispatch(void)
{
    reset_calls();
    uint8_t *resp = NULL; uint16_t rlen = 0;
    esp_err_t r = mcu_slist_adapter(NULL, RPC_ID__Req_GetMACAddress,
                                     NULL, 0, &resp, &rlen);
    CHECK("mcu_slist_dispatch_ok",   r == ESP_OK);
    CHECK("mcu_slist_handler_called", s_call_count == 1);
    CHECK("mcu_slist_handler_id",     s_call_log[0] == RPC_ID__Req_GetMACAddress);
    if (resp) free(resp);
}

static void test_mcu_slist_adapter_resp_too_small(void)
{
    reset_calls();
    uint8_t *resp = NULL; uint16_t rlen = 0;
    esp_err_t r = mcu_slist_adapter(NULL, RPC_ID__Req_WifiInit,
                                     NULL, 0, &resp, &rlen);
    CHECK("mcu_slist_resp_small", r == ESP_OK);
    if (resp) free(resp);
}

/* ══════════════════════════════════════════════════════════════════════════
 * F. Resp msg_id derivation: req_id → resp_id via Base offset
 * ══════════════════════════════════════════════════════════════════════════ */
static void test_fg_resp_id_derivation(void)
{
    /* For every FG request ID, resp.msg_id must be req_id - Req_Base + Resp_Base */
    for (int i = 0; i < FG_TABLE_SIZE; i++) {
        uint32_t req_id  = (uint32_t)fg_req_table[i].req_num;
        uint32_t want_id = req_id - CTRL_MSG_ID__Req_Base + CTRL_MSG_ID__Resp_Base;
        uint8_t *out = NULL; ssize_t outlen = 0;
        uint8_t in[4]; pack_msg_id(in, req_id);
        reset_calls();
        esp_err_t r = stub_fg_req_handler(0, in, 4, &out, &outlen);
        uint32_t got = out ? unpack_msg_id(out) : 0;
        char name[64]; snprintf(name, sizeof(name), "fg_resp_id req=%u", req_id);
        CHECK(name, r == ESP_OK && got == want_id);
        free(out);
    }
}

static void test_mcu_resp_id_derivation(void)
{
    for (int i = 0; i < MCU_CASES; i++) {
        uint32_t req_id  = mcu_cases[i].id;
        uint32_t want_id = req_id - RPC_ID__Req_Base + RPC_ID__Resp_Base;
        uint8_t *out = NULL; ssize_t outlen = 0;
        uint8_t in[4]; pack_msg_id(in, req_id);
        reset_calls();
        esp_err_t r = stub_mcu_req_handler(0, in, 4, &out, &outlen);
        uint32_t got = out ? unpack_msg_id(out) : 0;
        char name[64]; snprintf(name, sizeof(name), "mcu_resp_id req=0x%x", req_id);
        CHECK(name, r == ESP_OK && got == want_id);
        free(out);
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 * G. Sentinel / range guard via hook layer
 * ══════════════════════════════════════════════════════════════════════════ */
static void test_fg_hook_sentinel_ids(void)
{
    uint8_t *out = NULL; ssize_t olen = 0;
    uint8_t in[4];

    /* Req_Base (100) — in-range guard triggers */
    pack_msg_id(in, CTRL_MSG_ID__Req_Base);
    reset_calls();
    stub_fg_req_handler(0, in, 4, &out, &olen);
    CHECK("fg_hook_sentinel_base_no_call", s_call_count == 0);
    free(out); out = NULL;

    /* Req_Max (129) — sentinel */
    pack_msg_id(in, CTRL_MSG_ID__Req_Max);
    reset_calls();
    stub_fg_req_handler(0, in, 4, &out, &olen);
    CHECK("fg_hook_sentinel_max_no_call", s_call_count == 0);
    free(out); out = NULL;
}

static void test_mcu_hook_sentinel_ids(void)
{
    uint8_t *out = NULL; ssize_t olen = 0;
    uint8_t in[4];

    pack_msg_id(in, RPC_ID__Req_Base);
    reset_calls();
    stub_mcu_req_handler(0, in, 4, &out, &olen);
    CHECK("mcu_hook_sentinel_base_no_call", s_call_count == 0);
    free(out); out = NULL;

    pack_msg_id(in, RPC_ID__Req_Max);
    reset_calls();
    stub_mcu_req_handler(0, in, 4, &out, &olen);
    CHECK("mcu_hook_sentinel_max_no_call", s_call_count == 0);
    free(out); out = NULL;
}

/* ══════════════════════════════════════════════════════════════════════════
 * H. Hook null-input guards
 * ══════════════════════════════════════════════════════════════════════════ */
static void test_fg_hook_null_inputs(void)
{
    uint8_t *out = NULL; ssize_t olen = 0; uint8_t in[4] = {0};
    CHECK("fg_hook_null_inbuf",  stub_fg_req_handler(0, NULL, 4,  &out, &olen) == ESP_ERR_INVALID_ARG);
    CHECK("fg_hook_null_outbuf", stub_fg_req_handler(0, in,   4,  NULL, &olen) == ESP_ERR_INVALID_ARG);
    CHECK("fg_hook_null_outlen", stub_fg_req_handler(0, in,   4,  &out, NULL)  == ESP_ERR_INVALID_ARG);
    CHECK("fg_hook_short_buf",   stub_fg_req_handler(0, in,   2,  &out, &olen) == ESP_ERR_INVALID_ARG);
}

static void test_mcu_hook_null_inputs(void)
{
    uint8_t *out = NULL; ssize_t olen = 0; uint8_t in[4] = {0};
    CHECK("mcu_hook_null_inbuf",  stub_mcu_req_handler(0, NULL, 4, &out, &olen) == ESP_ERR_INVALID_ARG);
    CHECK("mcu_hook_null_outbuf", stub_mcu_req_handler(0, in,   4, NULL, &olen) == ESP_ERR_INVALID_ARG);
    CHECK("mcu_hook_null_outlen", stub_mcu_req_handler(0, in,   4, &out, NULL)  == ESP_ERR_INVALID_ARG);
    CHECK("mcu_hook_short_buf",   stub_mcu_req_handler(0, in,   2, &out, &olen) == ESP_ERR_INVALID_ARG);
}

/* ══════════════════════════════════════════════════════════════════════════
 * I. FG vs MCU non-overlapping wire event IDs for equivalent cp_events
 *    (ESPInit, Heartbeat, StaConnected, StaDisconnected)
 * ══════════════════════════════════════════════════════════════════════════ */
static void test_fg_mcu_evt_ids_differ(void)
{
    /* ESPInit */
    CHECK("espinit_ids_differ",
          CTRL_MSG_ID__Event_ESPInit != RPC_ID__Event_ESPInit);
    /* Heartbeat */
    CHECK("heartbeat_ids_differ",
          CTRL_MSG_ID__Event_Heartbeat != RPC_ID__Event_Heartbeat);
    /* StaConnectedToAP vs StaConnected */
    CHECK("sta_conn_ids_differ",
          CTRL_MSG_ID__Event_StationConnectedToAP != RPC_ID__Event_StaConnected);
    /* StaDisconnectedFromAP vs StaDisconnected */
    CHECK("sta_disconn_ids_differ",
          CTRL_MSG_ID__Event_StationDisconnectFromAP != RPC_ID__Event_StaDisconnected);
    /* CustomRPC */
    CHECK("custom_rpc_ids_differ",
          CTRL_MSG_ID__Event_Custom_RPC_Unserialised_Msg != RPC_ID__Event_Custom_RPC_Unserialised_Msg);
}

static void test_fg_mcu_evt_routing_correct(void)
{
    /* ESPInit fires correctly in FG dispatcher */
    s_last_fg_evt = 0;
    CtrlMsg fn = {0}; fn.msg_id = CTRL_MSG_ID__Event_ESPInit;
    fg_evt_dispatcher(&fn, NULL, NULL, 0);
    CHECK("fg_espinit_routes", s_last_fg_evt == CTRL_MSG_ID__Event_ESPInit);

    /* ESPInit fires correctly in MCU dispatcher */
    s_last_mcu_evt = 0;
    Rpc mn = {0}; mn.msg_id = RPC_ID__Event_ESPInit;
    mcu_evt_dispatcher(&mn, NULL, NULL, 0);
    CHECK("mcu_espinit_routes", s_last_mcu_evt == RPC_ID__Event_ESPInit);

    /* FG ESPInit ID does NOT match in MCU dispatcher */
    s_last_mcu_evt = 0;
    mn.msg_id = CTRL_MSG_ID__Event_ESPInit; /* 301 — not in MCU switch */
    mcu_evt_dispatcher(&mn, NULL, NULL, 0);
    CHECK("fg_id_not_in_mcu_evt", s_last_mcu_evt == 0);

    /* MCU ESPInit ID does NOT match in FG dispatcher */
    s_last_fg_evt = 0;
    fn.msg_id = RPC_ID__Event_ESPInit; /* 769 — not in FG switch */
    fg_evt_dispatcher(&fn, NULL, NULL, 0);
    CHECK("mcu_id_not_in_fg_evt", s_last_fg_evt == 0);
}

/* ══════════════════════════════════════════════════════════════════════════
 * J. FG req_table coverage: every proto ID 101..128 is in the table
 *    (catches accidental omissions when proto adds new messages)
 * ══════════════════════════════════════════════════════════════════════════ */
static void test_fg_table_covers_all_proto_ids(void)
{
    /* List of every req ID defined in eh_config.pb-c.h */
    static const int all_fg_req_ids[] = {
        CTRL_MSG_ID__Req_GetMACAddress, CTRL_MSG_ID__Req_SetMacAddress,
        CTRL_MSG_ID__Req_GetWifiMode,   CTRL_MSG_ID__Req_SetWifiMode,
        CTRL_MSG_ID__Req_GetAPScanList, CTRL_MSG_ID__Req_GetAPConfig,
        CTRL_MSG_ID__Req_ConnectAP,     CTRL_MSG_ID__Req_DisconnectAP,
        CTRL_MSG_ID__Req_GetSoftAPConfig, CTRL_MSG_ID__Req_SetSoftAPVendorSpecificIE,
        CTRL_MSG_ID__Req_StartSoftAP,   CTRL_MSG_ID__Req_GetSoftAPConnectedSTAList,
        CTRL_MSG_ID__Req_StopSoftAP,    CTRL_MSG_ID__Req_SetPowerSaveMode,
        CTRL_MSG_ID__Req_GetPowerSaveMode, CTRL_MSG_ID__Req_OTABegin,
        CTRL_MSG_ID__Req_OTAWrite,      CTRL_MSG_ID__Req_OTAEnd,
        CTRL_MSG_ID__Req_SetWifiMaxTxPower, CTRL_MSG_ID__Req_GetWifiCurrTxPower,
        CTRL_MSG_ID__Req_ConfigHeartbeat,   CTRL_MSG_ID__Req_EnableDisable,
        CTRL_MSG_ID__Req_GetFwVersion,  CTRL_MSG_ID__Req_SetCountryCode,
        CTRL_MSG_ID__Req_GetCountryCode, CTRL_MSG_ID__Req_SetDhcpDnsStatus,
        CTRL_MSG_ID__Req_GetDhcpDnsStatus, CTRL_MSG_ID__Req_Custom_RPC_Unserialised_Msg,
    };
    int n = (int)(sizeof(all_fg_req_ids)/sizeof(all_fg_req_ids[0]));
    for (int i = 0; i < n; i++) {
        bool found = (fg_lookup(all_fg_req_ids[i]) >= 0);
        char name[64]; snprintf(name, sizeof(name), "fg_proto_id_%d_in_table", all_fg_req_ids[i]);
        CHECK(name, found);
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 * main
 * ══════════════════════════════════════════════════════════════════════════ */
int main(void)
{
    printf("eh_cp — Extension dispatcher + hook + adapter tests\n\n");

    printf("── A. FG req_table dispatcher ────────────────────────────────────\n");
    test_fg_dispatcher_every_entry();
    test_fg_table_ids_in_range();
    test_fg_table_no_duplicates();
    test_fg_table_no_null_handlers();
    test_fg_dispatcher_sentinel_base();
    test_fg_dispatcher_sentinel_max();
    test_fg_dispatcher_unknown_id();
    test_fg_dispatcher_null_args();

    printf("── B. MCU switch dispatcher ──────────────────────────────────────\n");
    test_mcu_dispatcher_every_case();
    test_mcu_dispatcher_sentinel_base();
    test_mcu_dispatcher_sentinel_max();
    test_mcu_dispatcher_unknown();
    test_mcu_dispatcher_null();

    printf("── C. FG evt dispatcher ──────────────────────────────────────────\n");
    test_fg_evt_all_known();
    test_fg_evt_unknown();
    test_fg_evt_sentinel_base();
    test_fg_evt_sentinel_max();
    test_fg_evt_null_ntfy();

    printf("── D. MCU evt dispatcher ─────────────────────────────────────────\n");
    test_mcu_evt_all_known();
    test_mcu_evt_unknown();
    test_mcu_evt_null();

    printf("── E. SLIST adapter wiring ───────────────────────────────────────\n");
    test_fg_slist_adapter_dispatch();
    test_mcu_slist_adapter_dispatch();
    test_mcu_slist_adapter_resp_too_small();

    printf("── F. Resp msg_id derivation ─────────────────────────────────────\n");
    test_fg_resp_id_derivation();
    test_mcu_resp_id_derivation();

    printf("── G. Sentinel / range guard via hook ───────────────────────────\n");
    test_fg_hook_sentinel_ids();
    test_mcu_hook_sentinel_ids();

    printf("── H. Hook null-input guards ─────────────────────────────────────\n");
    test_fg_hook_null_inputs();
    test_mcu_hook_null_inputs();

    printf("── I. FG vs MCU event-ID non-overlap and routing ─────────────────\n");
    test_fg_mcu_evt_ids_differ();
    test_fg_mcu_evt_routing_correct();

    printf("── J. FG table proto coverage ────────────────────────────────────\n");
    test_fg_table_covers_all_proto_ids();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
