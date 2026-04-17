/* system_test.c — System-level integration tests for the ESP-Hosted CP
 *
 * Philosophy: these tests operate on the FULL pipeline rather than a single
 * component.  Each test exercises multiple layers together:
 *
 *   proto wire encoder → scanner → SLIST dispatch → handler → resp decoder
 *
 * Test groups:
 *   A  Proto wire encode/decode round-trips (real varint layout)
 *   B  TLV framing (compose + parse) — used by protocomm serial layer
 *   C  SLIST registry: complex multi-node scenarios & sorting invariants
 *   D  Full dispatch pipeline: wire bytes in → response bytes out
 *   E  Overlap edge-case matrix (touching, nested, wrapping, reversed)
 *   F  Multi-extension coexistence: FG + MCU registered simultaneously
 *   G  Init/re-init/idempotency sequences (mirrors ext_rpc_mcu_init logic)
 *   H  Event routing: send_event → serialise → wire bytes
 *   I  Fault injection: corrupt proto, truncated varint, wrong wire types
 *   J  Resp msg_id derivation: every FG and MCU ID, both formulas
 *   K  Proto scanner stress: field ordering, multi-field skip, 5-byte varint
 *   L  Registry state integrity after a sequence of register→dispatch→re-register
 *
 * Compile & run (no IDF needed):
 *   make -C eh_fg/coprocessor/extensions/test run_system
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <assert.h>
#include <sys/queue.h>

/* ══════════════════════════════════════════════════════════════════════════
 * Platform / IDF stubs
 * ══════════════════════════════════════════════════════════════════════════ */
typedef int      esp_err_t;
typedef long     ssize_t;
#define ESP_OK               0
#define ESP_FAIL            -1
#define ESP_ERR_INVALID_ARG  0x102
#define ESP_ERR_NO_MEM       0x101
#define ESP_ERR_INVALID_STATE 0x103
#define ESP_ERR_NOT_FOUND    0x105
#define SUCCESS  0
#define FAILURE -1

/* Logging — silenced in test build */
#define ESP_LOGE(t,f,...) ((void)0)
#define ESP_LOGI(t,f,...) ((void)0)
#define ESP_LOGD(t,f,...) ((void)0)
#define ESP_LOGW(t,f,...) ((void)0)

/* FreeRTOS critical section stubs */
typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
#define taskENTER_CRITICAL(m) ((void)(m))
#define taskEXIT_CRITICAL(m)  ((void)(m))

/* Mutex stubs (registry uses these) */
typedef void* SemaphoreHandle_t;
static SemaphoreHandle_t g_stub_mutex = (void*)1;
#define xSemaphoreCreateMutex()           (g_stub_mutex)
#define xSemaphoreTake(m,t)               (1)
#define xSemaphoreGive(m)                 ((void)0)
#define portMAX_DELAY                     0xFFFFFFFF

#ifndef PRIx32
#define PRIx32 "x"
#define PRIu32 "u"
#endif

/* ══════════════════════════════════════════════════════════════════════════
 * Proto ID constants — FG (eh_config.proto)
 * ══════════════════════════════════════════════════════════════════════════ */
#define CTRL_MSG_ID__Req_Base                     100
#define CTRL_MSG_ID__Req_GetMACAddress            101
#define CTRL_MSG_ID__Req_SetMacAddress            102
#define CTRL_MSG_ID__Req_GetWifiMode              103
#define CTRL_MSG_ID__Req_SetWifiMode              104
#define CTRL_MSG_ID__Req_GetAPScanList            105
#define CTRL_MSG_ID__Req_GetAPConfig              106
#define CTRL_MSG_ID__Req_ConnectAP                107
#define CTRL_MSG_ID__Req_DisconnectAP             108
#define CTRL_MSG_ID__Req_GetSoftAPConfig          109
#define CTRL_MSG_ID__Req_SetSoftAPVendorSpecificIE 110
#define CTRL_MSG_ID__Req_StartSoftAP              111
#define CTRL_MSG_ID__Req_GetSoftAPConnectedSTAList 112
#define CTRL_MSG_ID__Req_StopSoftAP               113
#define CTRL_MSG_ID__Req_SetPowerSaveMode         114
#define CTRL_MSG_ID__Req_GetPowerSaveMode         115
#define CTRL_MSG_ID__Req_OTABegin                 116
#define CTRL_MSG_ID__Req_OTAWrite                 117
#define CTRL_MSG_ID__Req_OTAEnd                   118
#define CTRL_MSG_ID__Req_SetWifiMaxTxPower        119
#define CTRL_MSG_ID__Req_GetWifiCurrTxPower       120
#define CTRL_MSG_ID__Req_ConfigHeartbeat          121
#define CTRL_MSG_ID__Req_EnableDisable            122
#define CTRL_MSG_ID__Req_GetFwVersion             123
#define CTRL_MSG_ID__Req_SetCountryCode           124
#define CTRL_MSG_ID__Req_GetCountryCode           125
#define CTRL_MSG_ID__Req_Custom_RPC_Unserialised_Msg 126
#define CTRL_MSG_ID__Req_SetDhcpDnsStatus             127
#define CTRL_MSG_ID__Req_GetDhcpDnsStatus             128
#define CTRL_MSG_ID__Req_Max                      129
#define CTRL_MSG_ID__Resp_Base                    200
#define CTRL_MSG_ID__Resp_Max                     229
#define CTRL_MSG_ID__Event_Base                   300
#define CTRL_MSG_ID__Event_ESPInit                301
#define CTRL_MSG_ID__Event_Heartbeat              302
#define CTRL_MSG_ID__Event_StationDisconnectFromAP        303
#define CTRL_MSG_ID__Event_StationDisconnectFromESPSoftAP 304
#define CTRL_MSG_ID__Event_StationConnectedToAP           305
#define CTRL_MSG_ID__Event_StationConnectedToESPSoftAP    306
#define CTRL_MSG_ID__Event_Custom_RPC_Unserialised_Msg 307
#define CTRL_MSG_ID__Event_SetDhcpDnsStatus           308
#define CTRL_MSG_ID__Event_Max                    309

/* ══════════════════════════════════════════════════════════════════════════
 * Proto ID constants — MCU (eh_rpc.proto)
 * ══════════════════════════════════════════════════════════════════════════ */
#define RPC_ID__Req_Base                  256   /* 0x100 */
#define RPC_ID__Req_GetMACAddress         257   /* 0x101 */
#define RPC_ID__Req_SetMacAddress         258
#define RPC_ID__Req_GetWifiMode           259
#define RPC_ID__Req_SetWifiMode           260
#define RPC_ID__Req_WifiSetPs             270
#define RPC_ID__Req_WifiGetPs             271
#define RPC_ID__Req_OTABegin              272
#define RPC_ID__Req_OTAWrite              273
#define RPC_ID__Req_OTAEnd                274
#define RPC_ID__Req_ConfigHeartbeat       277
#define RPC_ID__Req_WifiInit              278
#define RPC_ID__Req_WifiDeinit            279
#define RPC_ID__Req_WifiStart             280
#define RPC_ID__Req_WifiStop              281
#define RPC_ID__Req_WifiConnect           282
#define RPC_ID__Req_WifiDisconnect        283
#define RPC_ID__Req_WifiSetConfig         284
#define RPC_ID__Req_WifiGetConfig         285
#define RPC_ID__Req_WifiScanStart         286
#define RPC_ID__Req_WifiScanStop          287
#define RPC_ID__Req_WifiScanGetApNum      288
#define RPC_ID__Req_WifiScanGetApRecords  289
#define RPC_ID__Req_WifiClearApList       290
#define RPC_ID__Req_WifiRestore           291
#define RPC_ID__Req_WifiClearFastConnect  292
#define RPC_ID__Req_WifiDeauthSta         293
#define RPC_ID__Req_WifiStaGetApInfo      294
#define RPC_ID__Req_WifiSetProtocol       297
#define RPC_ID__Req_WifiGetProtocol       298
#define RPC_ID__Req_WifiSetBandwidth      299
#define RPC_ID__Req_WifiGetBandwidth      300
#define RPC_ID__Req_WifiSetChannel        301
#define RPC_ID__Req_WifiGetChannel        302
#define RPC_ID__Req_GetCoprocessorFwVersion 350
#define RPC_ID__Req_WifiSetCountryCode    334
#define RPC_ID__Req_WifiGetCountryCode    335
#define RPC_ID__Req_WifiSetInactiveTime   325
#define RPC_ID__Req_WifiGetInactiveTime   326
#define RPC_ID__Req_SetDhcpDnsStatus      352
#define RPC_ID__Req_GetDhcpDnsStatus      353
#define RPC_ID__Req_WifiStaItwtSetup      355
#define RPC_ID__Req_WifiStaItwtTeardown   356
#define RPC_ID__Req_WifiStaItwtSuspend    357
#define RPC_ID__Req_WifiStaItwtGetFlowIdStatus 358
#define RPC_ID__Req_WifiStaItwtSendProbeReq 359
#define RPC_ID__Req_EapUseDefaultCertBundle 380
#define RPC_ID__Req_WifiSetOkcSupport     381
#define RPC_ID__Req_EapSetDomainName      382
#define RPC_ID__Req_EapSetDisableTimeCheck 383
#define RPC_ID__Req_EapSetEapMethods      384
#define RPC_ID__Req_IfaceMacAddrSetGet    385
#define RPC_ID__Req_IfaceMacAddrLenGet    386
#define RPC_ID__Req_FeatureControl        387
#define RPC_ID__Req_Max                   388   /* 0x184 */
#define RPC_ID__Resp_Base                 512   /* 0x200 */
#define RPC_ID__Event_Base                768   /* 0x300 */
#define RPC_ID__Event_ESPInit             769
#define RPC_ID__Event_Heartbeat           770
#define RPC_ID__Event_StaConnected        772
#define RPC_ID__Event_StaDisconnected     773
#define RPC_ID__Event_AP_StaConnected     774
#define RPC_ID__Event_AP_StaDisconnected  775
#define RPC_ID__Event_StaScanDone         776
#define RPC_ID__Event_WifiEventNoArgs     777
#define RPC_ID__Event_DhcpDnsStatus       778
#define RPC_ID__Event_CustomRpc                   788
#define RPC_ID__Event_Max                 789   /* 0x315 */

/* ══════════════════════════════════════════════════════════════════════════
 * ESP caps stubs
 * ══════════════════════════════════════════════════════════════════════════ */
#define ESP_IF_TYPE_STA   0
#define ESP_IF_TYPE_AP    1
#define ESP_IF_TYPE_HCI   2
#define ESP_IF_TYPE_SERIAL 3
#define ESP_IF_TYPE_PRIV  4
#define ESP_STA_IF        ESP_IF_TYPE_STA
#define ESP_AP_IF         ESP_IF_TYPE_AP
#define ESP_IF_TYPE_MAX   8
typedef int eh_if_type_t;
#define EH_FEAT_CAPS_COUNT  8
#define ESP_WLAN_SUPPORT            0x01
#define EH_FEAT_IDX_WIFI    0
#define EH_FEAT_IDX_BT      1
#define EH_FEAT_IDX_OTA     2

/* sdkconfig stubs */
#define CONFIG_ESP_HOSTED_CP_RPC_NODE_CONTAINS_NAME 0

/* ══════════════════════════════════════════════════════════════════════════
 * SLIST registry — inlined from eh_cp_registries.c
 * (stripped of FreeRTOS calls, replaced with stub mutex)
 * ══════════════════════════════════════════════════════════════════════════ */

typedef esp_err_t (*hosted_rx_cb_t)(void *ctx, void *buf, uint16_t len, void *eb);
typedef int       (*hosted_tx_cb_t)(void *ctx, void *frame, uint16_t len);

typedef struct {
    hosted_rx_cb_t rx;
    hosted_tx_cb_t tx;
    void          *ctx;
} eh_iface_entry_t;

typedef struct eh_rpc_req_node {
    uint16_t   msg_id_min;
    uint16_t   msg_id_max;
    esp_err_t (*handler)(void *ctx, uint32_t msg_id,
                         const void *req_buf, uint16_t req_len,
                         uint8_t **out_buf, uint16_t *out_len);
    void      *ctx;
    SLIST_ENTRY(eh_rpc_req_node) next;
} eh_rpc_req_node_t;

typedef struct eh_rpc_evt_node {
    uint16_t   event_id_min;
    uint16_t   event_id_max;
    esp_err_t (*serialise)(void *ctx, uint32_t event_id,
                           const void *data, uint16_t data_len,
                           uint8_t **out_buf, uint16_t *out_len);
    void      *ctx;
    SLIST_ENTRY(eh_rpc_evt_node) next;
} eh_rpc_evt_node_t;

SLIST_HEAD(rpc_req_list_t, eh_rpc_req_node);
SLIST_HEAD(rpc_evt_list_t, eh_rpc_evt_node);

static struct rpc_req_list_t g_rpc_req_list;
static struct rpc_evt_list_t g_rpc_evt_list;

/* iface table */
static eh_iface_entry_t g_iface_table[ESP_IF_TYPE_MAX];

/* cap accumulators */
static uint8_t  s_caps     = 0;
static uint32_t s_ext_caps = 0;
static uint32_t s_feat_caps[EH_FEAT_CAPS_COUNT] = {0};

static void registry_reset(void) {
    SLIST_INIT(&g_rpc_req_list);
    SLIST_INIT(&g_rpc_evt_list);
    memset(g_iface_table, 0, sizeof(g_iface_table));
    s_caps = 0; s_ext_caps = 0;
    memset(s_feat_caps, 0, sizeof(s_feat_caps));
}

static bool ranges_overlap(uint16_t min_a, uint16_t max_a,
                            uint16_t min_b, uint16_t max_b) {
    return (min_a <= max_b) && (min_b <= max_a);
}

static esp_err_t reg_req_register(eh_rpc_req_node_t *node) {
    if (!node || !node->handler) return ESP_ERR_INVALID_ARG;
    if (node->msg_id_min > node->msg_id_max) return ESP_ERR_INVALID_ARG;
    eh_rpc_req_node_t *n;
    SLIST_FOREACH(n, &g_rpc_req_list, next)
        if (ranges_overlap(node->msg_id_min, node->msg_id_max, n->msg_id_min, n->msg_id_max))
            return ESP_ERR_INVALID_STATE;
    /* sorted insert ascending by msg_id_min */
    eh_rpc_req_node_t *prev = NULL;
    SLIST_FOREACH(n, &g_rpc_req_list, next) {
        if (n->msg_id_min > node->msg_id_min) break;
        prev = n;
    }
    if (!prev) SLIST_INSERT_HEAD(&g_rpc_req_list, node, next);
    else        SLIST_INSERT_AFTER(prev, node, next);
    return ESP_OK;
}

static esp_err_t reg_evt_register(eh_rpc_evt_node_t *node) {
    if (!node || !node->serialise) return ESP_ERR_INVALID_ARG;
    if (node->event_id_min > node->event_id_max) return ESP_ERR_INVALID_ARG;
    eh_rpc_evt_node_t *n;
    SLIST_FOREACH(n, &g_rpc_evt_list, next)
        if (ranges_overlap(node->event_id_min, node->event_id_max, n->event_id_min, n->event_id_max))
            return ESP_ERR_INVALID_STATE;
    SLIST_INSERT_HEAD(&g_rpc_evt_list, node, next);
    return ESP_OK;
}

static esp_err_t reg_dispatch_req(uint32_t msg_id,
                                   const void *req_buf, uint16_t req_len,
                                   uint8_t **out_buf, uint16_t *out_len) {
    eh_rpc_req_node_t *n;
    SLIST_FOREACH(n, &g_rpc_req_list, next) {
        if (msg_id > (uint32_t)n->msg_id_max) continue;
        if (msg_id < (uint32_t)n->msg_id_min) break;
        return n->handler(n->ctx, msg_id, req_buf, req_len, out_buf, out_len);
    }
    return ESP_ERR_NOT_FOUND;
}

static esp_err_t reg_dispatch_req_noresp(uint32_t msg_id,
                                         const void *req_buf, uint16_t req_len) {
    uint8_t *out = NULL;
    uint16_t out_len = 0;
    esp_err_t r = reg_dispatch_req(msg_id, req_buf, req_len, &out, &out_len);
    if (out) free(out);
    return r;
}

static esp_err_t reg_send_event(uint32_t event_id, const void *data, uint16_t len) {
    eh_rpc_evt_node_t *n;
    SLIST_FOREACH(n, &g_rpc_evt_list, next) {
        if (event_id > (uint32_t)n->event_id_max) continue;
        if (event_id < (uint32_t)n->event_id_min) continue;
        uint8_t *out = NULL;
        uint16_t out_len = 0;
        esp_err_t r = n->serialise(n->ctx, event_id, data, len, &out, &out_len);
        if (out) free(out);
        return r;
    }
    return ESP_ERR_NOT_FOUND;
}

static void reg_add_cap_bits(uint8_t caps, uint32_t ext) { s_caps |= caps; s_ext_caps |= ext; }
static void reg_add_feat_caps(uint8_t idx, uint32_t bits) {
    if (idx < EH_FEAT_CAPS_COUNT) s_feat_caps[idx] |= bits;
}
static uint8_t  reg_get_caps(void)     { return s_caps; }
static uint32_t reg_get_ext_caps(void) { return s_ext_caps; }

/* ══════════════════════════════════════════════════════════════════════════
 * Proto field-2 scanner — inlined from eh_cp_registries.c
 * ══════════════════════════════════════════════════════════════════════════ */

static uint32_t proto_extract_msg_id(const uint8_t *buf, uint16_t len, uint32_t *out) {
    if (!buf || !out || len == 0) return 0;
    uint16_t i = 0;
    while (i < len) {
        uint8_t tag       = buf[i++];
        uint8_t field_num = (uint8_t)(tag >> 3);
        uint8_t wire_type = (uint8_t)(tag & 0x07u);
        if (field_num == 2 && wire_type == 0) {
            uint32_t v = 0; uint8_t shift = 0;
            while (i < len && shift < 35) {
                uint8_t b = buf[i++];
                v |= (uint32_t)(b & 0x7Fu) << shift;
                if (!(b & 0x80u)) { *out = v; return 2u; }
                shift = (uint8_t)(shift + 7u);
            }
            return 0;
        }
        switch (wire_type) {
        case 0: while (i < len && (buf[i] & 0x80u)) i++; if (i < len) i++; break;
        case 1: i = (uint16_t)(i + 8u); break;
        case 2: {
            uint32_t slen = 0; uint8_t sh = 0;
            while (i < len && sh < 35) {
                uint8_t b = buf[i++]; slen |= (uint32_t)(b & 0x7Fu) << sh;
                if (!(b & 0x80u)) break; sh = (uint8_t)(sh + 7u);
            }
            i = (uint16_t)(i + slen); break;
        }
        case 5: i = (uint16_t)(i + 4u); break;
        default: return 0;
        }
    }
    return 0;
}

/* ══════════════════════════════════════════════════════════════════════════
 * Proto varint encoder — builds minimal wire bytes for field N, wire type 0
 * ══════════════════════════════════════════════════════════════════════════ */

/* Encode a single varint into buf; return bytes written */
static int encode_varint(uint8_t *buf, uint32_t value) {
    int n = 0;
    while (value > 0x7Fu) {
        buf[n++] = (uint8_t)((value & 0x7Fu) | 0x80u);
        value >>= 7;
    }
    buf[n++] = (uint8_t)(value & 0x7Fu);
    return n;
}

/* Encode a proto message with field 1 (msg_type) and field 2 (msg_id).
 * Layout: tag(1,0) varint(msg_type) tag(2,0) varint(msg_id)
 * Returns total bytes written.
 */
static int encode_ctrl_msg(uint8_t *buf, uint32_t msg_type, uint32_t msg_id) {
    int pos = 0;
    /* field 1, wire type 0 */
    buf[pos++] = (1u << 3) | 0u;
    pos += encode_varint(buf + pos, msg_type);
    /* field 2, wire type 0 */
    buf[pos++] = (2u << 3) | 0u;
    pos += encode_varint(buf + pos, msg_id);
    return pos;
}

/* Same but also encodes field 3 (uid) — used to test field-order independence */
static int encode_ctrl_msg_with_uid(uint8_t *buf, uint32_t msg_type,
                                    uint32_t msg_id, uint32_t uid) {
    int pos = encode_ctrl_msg(buf, msg_type, msg_id);
    buf[pos++] = (3u << 3) | 0u;
    pos += encode_varint(buf + pos, uid);
    return pos;
}

/* Encode only field 2 (msg_id) — field 1 absent */
static int encode_msg_id_only(uint8_t *buf, uint32_t msg_id) {
    int pos = 0;
    buf[pos++] = (2u << 3) | 0u;
    pos += encode_varint(buf + pos, msg_id);
    return pos;
}

/* ══════════════════════════════════════════════════════════════════════════
 * TLV framing stubs (mirrors compose_tlv / parse_tlv in rpc_ll.c)
 * ══════════════════════════════════════════════════════════════════════════ */
#define TLV_T_EPNAME  1
#define TLV_T_DATA    2

static int tlv_compose(const char *epname, const uint8_t *data, uint16_t dlen,
                       uint8_t *out, size_t out_max) {
    size_t ep_len = strlen(epname);
    size_t total  = 1+2+ep_len + 1+2+dlen;
    if (total > out_max) return -1;
    size_t pos = 0;
    out[pos++] = TLV_T_EPNAME;
    out[pos++] = (uint8_t)(ep_len & 0xFF);
    out[pos++] = (uint8_t)((ep_len >> 8) & 0xFF);
    memcpy(out + pos, epname, ep_len); pos += ep_len;
    out[pos++] = TLV_T_DATA;
    out[pos++] = (uint8_t)(dlen & 0xFF);
    out[pos++] = (uint8_t)((dlen >> 8) & 0xFF);
    memcpy(out + pos, data, dlen); pos += dlen;
    return (int)pos;
}

static int tlv_parse_epname(const uint8_t *buf, size_t len,
                             char *ep_out, size_t ep_max,
                             const uint8_t **data_out, uint16_t *data_len_out) {
    size_t i = 0;
    int got_ep = 0, got_data = 0;
    while (i + 3 <= len) {
        uint8_t  type   = buf[i];
        uint16_t vlen   = (uint16_t)(buf[i+1] | (buf[i+2] << 8));
        i += 3;
        if (i + vlen > len) return -1;
        if (type == TLV_T_EPNAME) {
            if (vlen >= ep_max) return -1;
            memcpy(ep_out, buf + i, vlen);
            ep_out[vlen] = '\0';
            got_ep = 1;
        } else if (type == TLV_T_DATA) {
            *data_out     = buf + i;
            *data_len_out = vlen;
            got_data = 1;
        }
        i += vlen;
    }
    return (got_ep && got_data) ? 0 : -1;
}

/* ══════════════════════════════════════════════════════════════════════════
 * Test framework
 * ══════════════════════════════════════════════════════════════════════════ */
static int g_pass = 0, g_fail = 0;
#define PASS(n)     do { printf("  PASS  %s\n",(n)); g_pass++; } while(0)
#define FAIL(n,msg) do { printf("  FAIL  %s  (%s)\n",(n),(msg)); g_fail++; } while(0)
#define CHECK(n,c)  do { if(c) PASS(n); else FAIL(n,#c); } while(0)

/* ══════════════════════════════════════════════════════════════════════════
 * Shared handler infrastructure
 * ══════════════════════════════════════════════════════════════════════════ */

/* Per-call record so tests can verify exactly which handler fired */
typedef struct {
    uint32_t msg_id;
    uint8_t  req_copy[64];
    uint16_t req_len;
    esp_err_t ret_override;   /* inject error if != ESP_OK */
    uint32_t resp_id;         /* written into response wire bytes */
} call_record_t;

#define MAX_CALL_LOG 32
static call_record_t s_log[MAX_CALL_LOG];
static int           s_log_n = 0;
static void          reset_log(void) { s_log_n = 0; }

/* Generic request handler: records call, encodes resp with resp_id = req_id + offset */
static esp_err_t generic_req_handler(void *ctx, uint32_t msg_id,
                                     const void *req_buf, uint16_t req_len,
                                     uint8_t **out_buf, uint16_t *out_len) {
    if (s_log_n < MAX_CALL_LOG) {
        s_log[s_log_n].msg_id  = msg_id;
        s_log[s_log_n].req_len = req_len < 64 ? req_len : 64;
        memcpy(s_log[s_log_n].req_copy, req_buf, s_log[s_log_n].req_len);
        s_log[s_log_n].ret_override = ESP_OK;
    }
    /* use ctx as a uint32_t* pointing to the resp_id offset from Req_Base */
    uint32_t *offsets = (uint32_t *)ctx;  /* [resp_base, req_base] */
    uint32_t resp_id = msg_id - offsets[1] + offsets[0];
    uint8_t *buf = malloc(8);
    if (!buf) return ESP_ERR_NO_MEM;
    int len = encode_msg_id_only(buf, resp_id);
    *out_buf = buf;
    *out_len = (uint16_t)len;
    if (s_log_n < MAX_CALL_LOG) {
        s_log[s_log_n].resp_id = resp_id;
        s_log_n++;
    }
    return ESP_OK;
}

/* Error-injecting handler */
static esp_err_t failing_req_handler(void *ctx, uint32_t msg_id,
                                     const void *req_buf, uint16_t req_len,
                                     uint8_t **out_buf, uint16_t *out_len) {
    (void)ctx;(void)msg_id;(void)req_buf;(void)req_len;
    *out_buf = NULL;
    *out_len = 0;
    if (s_log_n < MAX_CALL_LOG) { s_log[s_log_n++].msg_id = msg_id; }
    return ESP_FAIL;
}

/* Event serialise: copies event_id into first 4 bytes of out */
static esp_err_t generic_evt_serialise(void *ctx, uint32_t event_id,
                                       const void *data, uint16_t data_len,
                                       uint8_t **out_buf, uint16_t *out_len) {
    (void)ctx;(void)data;(void)data_len;
    uint8_t *buf = malloc(8);
    if (!buf) return ESP_ERR_NO_MEM;
    int len = encode_msg_id_only(buf, event_id);
    *out_buf = buf;
    *out_len = (uint16_t)len;
    if (s_log_n < MAX_CALL_LOG) { s_log[s_log_n++].msg_id = event_id; }
    return ESP_OK;
}

/* Failing event serialise */
static esp_err_t failing_evt_serialise(void *ctx, uint32_t event_id,
                                       const void *data, uint16_t data_len,
                                       uint8_t **out_buf, uint16_t *out_len) {
    (void)ctx;(void)event_id;(void)data;(void)data_len;
    *out_len = 0;
    *out_buf = NULL;
    if (s_log_n < MAX_CALL_LOG) { s_log[s_log_n++].msg_id = event_id; }
    return ESP_FAIL;
}

/* Offsets used by FG and MCU handlers */
static uint32_t fg_offsets[2]  = { CTRL_MSG_ID__Resp_Base, CTRL_MSG_ID__Req_Base };
static uint32_t mcu_offsets[2] = { RPC_ID__Resp_Base,      RPC_ID__Req_Base      };

/* ══════════════════════════════════════════════════════════════════════════
 * Helper: build wire bytes and dispatch, return decoded resp_id
 * ══════════════════════════════════════════════════════════════════════════ */
static esp_err_t dispatch_proto_req(uint32_t msg_id, uint32_t *resp_id_out) {
    uint8_t  wire[32];
    int      wlen  = encode_msg_id_only(wire, msg_id);
    uint8_t *resp = NULL;
    uint16_t rlen = 0;
    esp_err_t r = reg_dispatch_req(msg_id, wire, (uint16_t)wlen, &resp, &rlen);
    if (r != ESP_OK) return r;
    uint32_t extracted = 0;
    proto_extract_msg_id(resp, rlen, &extracted);
    free(resp);
    if (resp_id_out) *resp_id_out = extracted;
    return ESP_OK;
}

/* ══════════════════════════════════════════════════════════════════════════
 * A. Proto wire encode/decode round-trips
 * ══════════════════════════════════════════════════════════════════════════ */
static void test_A_proto_roundtrip(void) {
    /* Single-byte varint: FG IDs 101..128 all fit in one byte after tag */
    uint32_t fg_ids[] = {
        CTRL_MSG_ID__Req_GetMACAddress, CTRL_MSG_ID__Req_OTABegin,
        CTRL_MSG_ID__Req_ConfigHeartbeat, CTRL_MSG_ID__Req_Custom_RPC_Unserialised_Msg,
    };
    for (int i = 0; i < (int)(sizeof(fg_ids)/sizeof(fg_ids[0])); i++) {
        uint8_t buf[16]; uint32_t got = 0;
        int len = encode_msg_id_only(buf, fg_ids[i]);
        uint32_t f = proto_extract_msg_id(buf, (uint16_t)len, &got);
        char name[48]; snprintf(name, sizeof(name), "fg_roundtrip_%u", fg_ids[i]);
        CHECK(name, f == 2 && got == fg_ids[i]);
    }

    /* Multi-byte varint: MCU IDs > 127 require 2 bytes */
    uint32_t mcu_ids[] = {
        RPC_ID__Req_GetMACAddress,    /* 257 = 0x101 → needs 2 bytes */
        RPC_ID__Req_WifiInit,         /* 278 */
        RPC_ID__Req_GetCoprocessorFwVersion, /* 350 */
        RPC_ID__Req_FeatureControl,   /* 387 */
        RPC_ID__Req_Max - 1,          /* 387 again but via formula */
        RPC_ID__Event_ESPInit,        /* 769 = 0x301 → 2 bytes */
        RPC_ID__Event_CustomRpc, /* 788 */
    };
    for (int i = 0; i < (int)(sizeof(mcu_ids)/sizeof(mcu_ids[0])); i++) {
        uint8_t buf[16]; uint32_t got = 0;
        int len = encode_msg_id_only(buf, mcu_ids[i]);
        uint32_t f = proto_extract_msg_id(buf, (uint16_t)len, &got);
        char name[48]; snprintf(name, sizeof(name), "mcu_roundtrip_0x%x", mcu_ids[i]);
        CHECK(name, f == 2 && got == mcu_ids[i] && len == 3); /* tag + 2 varint bytes */
    }

    /* Full message with field 1 + field 2 + field 3 */
    {
        uint8_t buf[32]; uint32_t got = 0;
        int len = encode_ctrl_msg_with_uid(buf, 1 /*Req*/, CTRL_MSG_ID__Req_ConnectAP, 42);
        uint32_t f = proto_extract_msg_id(buf, (uint16_t)len, &got);
        CHECK("fg_full_msg_extract", f == 2 && got == CTRL_MSG_ID__Req_ConnectAP);
    }

    /* field 2 comes BEFORE field 1 — scanner must still find it */
    {
        uint8_t buf[16]; int pos = 0; uint32_t got = 0;
        buf[pos++] = (2u << 3) | 0u;           /* field 2, varint */
        pos += encode_varint(buf + pos, (uint32_t)CTRL_MSG_ID__Req_SetWifiMode);
        buf[pos++] = (1u << 3) | 0u;           /* field 1, varint */
        pos += encode_varint(buf + pos, 1u);
        uint32_t f = proto_extract_msg_id(buf, (uint16_t)pos, &got);
        CHECK("field2_before_field1", f == 2 && got == CTRL_MSG_ID__Req_SetWifiMode);
    }

    /* field 3 before field 2 — scanner must skip field 3 then find field 2 */
    {
        uint8_t buf[16]; int pos = 0; uint32_t got = 0;
        buf[pos++] = (3u << 3) | 0u;  pos += encode_varint(buf + pos, 99u);   /* f3 */
        buf[pos++] = (2u << 3) | 0u;  pos += encode_varint(buf + pos, 333u);  /* f2 */
        uint32_t f = proto_extract_msg_id(buf, (uint16_t)pos, &got);
        CHECK("field3_then_field2", f == 2 && got == 333u);
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 * B. TLV framing round-trips
 * ══════════════════════════════════════════════════════════════════════════ */
static void test_B_tlv_framing(void) {
    /* Compose and parse with endpoint "RPCRsp" */
    {
        const char *epname = "RPCRsp";
        uint8_t proto[32]; int plen = encode_msg_id_only(proto, CTRL_MSG_ID__Req_OTABegin);
        uint8_t frame[256];
        int flen = tlv_compose(epname, proto, (uint16_t)plen, frame, sizeof(frame));
        CHECK("tlv_compose_ok", flen > 0);

        char ep_out[32]; const uint8_t *data_ptr = NULL; uint16_t data_len = 0;
        int r = tlv_parse_epname(frame, (size_t)flen, ep_out, sizeof(ep_out), &data_ptr, &data_len);
        CHECK("tlv_parse_ok",      r == 0);
        CHECK("tlv_epname_match",  strcmp(ep_out, epname) == 0);
        CHECK("tlv_data_len",      data_len == (uint16_t)plen);

        /* Extract msg_id from the data payload */
        uint32_t got = 0;
        proto_extract_msg_id(data_ptr, data_len, &got);
        CHECK("tlv_msg_id_intact", got == CTRL_MSG_ID__Req_OTABegin);
    }

    /* Compose with a 2-byte varint MCU ID */
    {
        const char *epname = "RPCRsp";
        uint8_t proto[32]; int plen = encode_msg_id_only(proto, RPC_ID__Req_WifiConnect);
        uint8_t frame[256];
        int flen = tlv_compose(epname, proto, (uint16_t)plen, frame, sizeof(frame));
        char ep_out[32]; const uint8_t *dp = NULL; uint16_t dl = 0;
        tlv_parse_epname(frame, (size_t)flen, ep_out, sizeof(ep_out), &dp, &dl);
        uint32_t got = 0; proto_extract_msg_id(dp, dl, &got);
        CHECK("tlv_mcu_id_intact", got == RPC_ID__Req_WifiConnect);
    }

    /* Legacy endpoint name "ctrlResp" */
    {
        const char *epname = "ctrlResp";
        uint8_t proto[8]; int plen = encode_msg_id_only(proto, CTRL_MSG_ID__Req_GetWifiMode);
        uint8_t frame[128];
        int flen = tlv_compose(epname, proto, (uint16_t)plen, frame, sizeof(frame));
        char ep_out[32]; const uint8_t *dp = NULL; uint16_t dl = 0;
        int r = tlv_parse_epname(frame, (size_t)flen, ep_out, sizeof(ep_out), &dp, &dl);
        CHECK("tlv_legacy_ep_ok",   r == 0);
        CHECK("tlv_legacy_epname",  strcmp(ep_out, "ctrlResp") == 0);
    }

    /* Buffer too small: compose should return -1 */
    {
        uint8_t proto[4] = {0x10, 0x6B};  /* field2 varint 107 */
        uint8_t small[5];
        int r = tlv_compose("RPCRsp", proto, 2, small, sizeof(small));
        CHECK("tlv_compose_overflow", r < 0);
    }

    /* Truncated TLV: length field claims more bytes than available */
    {
        uint8_t bad[8];
        bad[0] = TLV_T_EPNAME; bad[1] = 100; bad[2] = 0; /* claims 100 bytes */
        memset(bad + 3, 'X', 5);
        char ep_out[32]; const uint8_t *dp = NULL; uint16_t dl = 0;
        int r = tlv_parse_epname(bad, 8, ep_out, sizeof(ep_out), &dp, &dl);
        CHECK("tlv_parse_truncated", r < 0);
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 * C. SLIST registry: complex multi-node scenarios & sorting invariants
 * ══════════════════════════════════════════════════════════════════════════ */
static void test_C_slist_complex(void) {
    registry_reset();

    /* Register 5 non-overlapping ranges in REVERSE order, verify sorted insert */
    eh_rpc_req_node_t nodes[5];
    static uint32_t dummy_ctx[5][2];
    /* Ranges: [200,250], [100,150], [400,450], [50,80], [300,350] */
    uint16_t mins[] = {200,100,400, 50,300};
    uint16_t maxs[] = {250,150,450, 80,350};
    for (int i = 0; i < 5; i++) {
        memset(&nodes[i], 0, sizeof(nodes[i]));
        nodes[i].msg_id_min = mins[i]; nodes[i].msg_id_max = maxs[i];
        nodes[i].handler    = generic_req_handler;
        dummy_ctx[i][0] = 600; dummy_ctx[i][1] = 0;
        nodes[i].ctx        = dummy_ctx[i];
        CHECK("multi_register_ok", reg_req_register(&nodes[i]) == ESP_OK);
    }

    /* Verify sorted order: walk list and confirm ascending msg_id_min */
    eh_rpc_req_node_t *n;
    uint16_t prev_min = 0; bool sorted = true;
    SLIST_FOREACH(n, &g_rpc_req_list, next) {
        if (n->msg_id_min < prev_min) { sorted = false; break; }
        prev_min = n->msg_id_min;
    }
    CHECK("slist_sorted_after_5_inserts", sorted);

    /* First node in list must have the lowest min (50) */
    eh_rpc_req_node_t *head = SLIST_FIRST(&g_rpc_req_list);
    CHECK("slist_head_is_lowest", head && head->msg_id_min == 50);

    /* Dispatch to each range — exactly the right node fires */
    reset_log();
    uint32_t test_ids[] = {60, 120, 220, 320, 420};
    for (int i = 0; i < 5; i++) {
        uint8_t wire[8]; int wl = encode_msg_id_only(wire, test_ids[i]);
        esp_err_t r = reg_dispatch_req_noresp(test_ids[i], wire, (uint16_t)wl);
        char name[48]; snprintf(name, sizeof(name), "multi_dispatch_id_%u", test_ids[i]);
        CHECK(name, r == ESP_OK);
    }
    CHECK("multi_dispatch_5_calls", s_log_n == 5);

    /* Gaps between nodes return NOT_FOUND */
    esp_err_t gap = reg_dispatch_req_noresp(151, NULL, 0);
    CHECK("gap_not_found", gap == ESP_ERR_NOT_FOUND);
    gap = reg_dispatch_req_noresp(81, NULL, 0);
    CHECK("gap2_not_found", gap == ESP_ERR_NOT_FOUND);

    /* Adding an overlapping range must fail */
    eh_rpc_req_node_t bad;
    memset(&bad, 0, sizeof(bad));
    bad.msg_id_min = 140; bad.msg_id_max = 160; /* overlaps [100,150] at 140-150 */
    bad.handler = generic_req_handler; bad.ctx = dummy_ctx[0];
    CHECK("overlap_rejected", reg_req_register(&bad) == ESP_ERR_INVALID_STATE);

    /* Adding a touching-but-not-overlapping range is also rejected (ranges_overlap
     * is inclusive: [50,80] and [80,90] share 80 → overlap) */
    eh_rpc_req_node_t touching;
    memset(&touching, 0, sizeof(touching));
    touching.msg_id_min = 80; touching.msg_id_max = 90;
    touching.handler = generic_req_handler; touching.ctx = dummy_ctx[0];
    CHECK("touching_rejected", reg_req_register(&touching) == ESP_ERR_INVALID_STATE);

    /* Range [81,90] does NOT overlap [50,80] → must succeed */
    eh_rpc_req_node_t adjacent;
    memset(&adjacent, 0, sizeof(adjacent));
    adjacent.msg_id_min = 81; adjacent.msg_id_max = 90;
    adjacent.handler = generic_req_handler; adjacent.ctx = dummy_ctx[0];
    CHECK("adjacent_accepted", reg_req_register(&adjacent) == ESP_OK);
}

/* ══════════════════════════════════════════════════════════════════════════
 * D. Full pipeline: proto wire bytes → scanner → SLIST dispatch → resp bytes
 * ══════════════════════════════════════════════════════════════════════════ */
static void test_D_full_pipeline(void) {
    registry_reset();

    /* Register FG range [101,128] */
    static eh_rpc_req_node_t fg_req;
    memset(&fg_req, 0, sizeof(fg_req));
    fg_req.msg_id_min = CTRL_MSG_ID__Req_Base + 1;
    fg_req.msg_id_max = CTRL_MSG_ID__Req_Max  - 1;
    fg_req.handler    = generic_req_handler;
    fg_req.ctx        = fg_offsets;
    CHECK("fg_reg_ok", reg_req_register(&fg_req) == ESP_OK);

    /* Register MCU range [257,387] */
    static eh_rpc_req_node_t mcu_req;
    memset(&mcu_req, 0, sizeof(mcu_req));
    mcu_req.msg_id_min = RPC_ID__Req_Base + 1;
    mcu_req.msg_id_max = RPC_ID__Req_Max  - 1;
    mcu_req.handler    = generic_req_handler;
    mcu_req.ctx        = mcu_offsets;
    CHECK("mcu_reg_ok", reg_req_register(&mcu_req) == ESP_OK);

    /* Pipeline: encode full CtrlMsg proto → extract msg_id → dispatch → decode resp */
    uint32_t all_fg[] = {
        CTRL_MSG_ID__Req_GetMACAddress, CTRL_MSG_ID__Req_ConnectAP,
        CTRL_MSG_ID__Req_OTABegin, CTRL_MSG_ID__Req_GetFwVersion,
        CTRL_MSG_ID__Req_Custom_RPC_Unserialised_Msg,
    };
    for (int i = 0; i < (int)(sizeof(all_fg)/sizeof(all_fg[0])); i++) {
        uint8_t wire[32]; int wl = encode_ctrl_msg(wire, 1, all_fg[i]);
        uint32_t scanner_id = 0;
        uint32_t f = proto_extract_msg_id(wire, (uint16_t)wl, &scanner_id);
        CHECK("fg_scanner_ok", f == 2 && scanner_id == (uint32_t)all_fg[i]);

        uint32_t resp_id = 0;
        esp_err_t r = dispatch_proto_req(scanner_id, &resp_id);
        uint32_t want = (uint32_t)(all_fg[i] - CTRL_MSG_ID__Req_Base + CTRL_MSG_ID__Resp_Base);
        char name[64]; snprintf(name, sizeof(name), "fg_pipeline_id%d", all_fg[i]);
        CHECK(name, r == ESP_OK && resp_id == want);
    }

    /* Same for MCU IDs — 2-byte varint path */
    uint32_t all_mcu[] = {
        RPC_ID__Req_GetMACAddress, RPC_ID__Req_WifiInit, RPC_ID__Req_OTABegin,
        RPC_ID__Req_GetCoprocessorFwVersion, RPC_ID__Req_FeatureControl,
    };
    for (int i = 0; i < (int)(sizeof(all_mcu)/sizeof(all_mcu[0])); i++) {
        uint8_t wire[32]; int wl = encode_ctrl_msg(wire, 1, all_mcu[i]);
        uint32_t scanner_id = 0;
        proto_extract_msg_id(wire, (uint16_t)wl, &scanner_id);
        uint32_t resp_id = 0;
        esp_err_t r = dispatch_proto_req(scanner_id, &resp_id);
        uint32_t want = (uint32_t)(all_mcu[i] - RPC_ID__Req_Base + RPC_ID__Resp_Base);
        char name[64]; snprintf(name, sizeof(name), "mcu_pipeline_0x%x", all_mcu[i]);
        CHECK(name, r == ESP_OK && resp_id == want);
    }

    /* IDs in the gap [129,256] between FG and MCU → NOT_FOUND */
    uint32_t gaps[] = {130, 200, 255};
    for (int i = 0; i < 3; i++) {
        esp_err_t r = dispatch_proto_req(gaps[i], NULL);
        char name[32]; snprintf(name, sizeof(name), "gap_id_%u", gaps[i]);
        CHECK(name, r == ESP_ERR_NOT_FOUND);
    }

    /* Sentinel IDs exactly at Base and Max are outside registered range → NOT_FOUND */
    CHECK("fg_base_sentinel", dispatch_proto_req(CTRL_MSG_ID__Req_Base, NULL) == ESP_ERR_NOT_FOUND);
    CHECK("fg_max_sentinel",  dispatch_proto_req(CTRL_MSG_ID__Req_Max,  NULL) == ESP_ERR_NOT_FOUND);
    CHECK("mcu_base_sentinel",dispatch_proto_req(RPC_ID__Req_Base,      NULL) == ESP_ERR_NOT_FOUND);
    CHECK("mcu_max_sentinel", dispatch_proto_req(RPC_ID__Req_Max,       NULL) == ESP_ERR_NOT_FOUND);
}

/* ══════════════════════════════════════════════════════════════════════════
 * E. Overlap edge-case matrix
 * ══════════════════════════════════════════════════════════════════════════ */
static void test_E_overlap_matrix(void) {
    /* For each sub-case: register anchor [100,200], then test candidate */
    typedef struct { uint16_t lo; uint16_t hi; bool should_fail; const char *name; } ov_case_t;
    static ov_case_t cases[] = {
        /* fully inside */    {120, 150, true,  "inside"},
        /* fully outside */   {210, 300, false, "outside_hi"},
        {  0,  99, false, "outside_lo"},
        /* touching hi */     {200, 210, true,  "touch_hi"},
        /* touching lo */     { 90, 100, true,  "touch_lo"},
        /* wraps around */    { 50, 250, true,  "wrap"},
        /* exact same */      {100, 200, true,  "exact_dup"},
        /* one-wide inside */ {150, 150, true,  "one_wide_inside"},
        /* adjacent hi ok */  {201, 210, false, "adjacent_hi_ok"},
        /* adjacent lo ok */  { 90,  99, false, "adjacent_lo_ok"},
    };
    for (int i = 0; i < (int)(sizeof(cases)/sizeof(cases[0])); i++) {
        registry_reset();
        static eh_rpc_req_node_t anchor, cand;
        memset(&anchor, 0, sizeof(anchor));
        anchor.msg_id_min = 100; anchor.msg_id_max = 200;
        anchor.handler = generic_req_handler; anchor.ctx = fg_offsets;
        reg_req_register(&anchor);

        memset(&cand, 0, sizeof(cand));
        cand.msg_id_min = cases[i].lo; cand.msg_id_max = cases[i].hi;
        cand.handler = generic_req_handler; cand.ctx = fg_offsets;
        esp_err_t r = reg_req_register(&cand);
        bool failed = (r == ESP_ERR_INVALID_STATE);
        char name[48]; snprintf(name, sizeof(name), "overlap_%s", cases[i].name);
        CHECK(name, failed == cases[i].should_fail);
    }

    /* Same matrix for event nodes */
    typedef struct { uint16_t lo; uint16_t hi; bool should_fail; const char *name; } ev_case_t;
    static ev_case_t ev_cases[] = {
        {305, 307, true,  "evt_inside"},
        {310, 320, false, "evt_outside"},
        {308, 308, true,  "evt_touch_hi"},
        {309, 320, false, "evt_adjacent_hi"},
    };
    for (int i = 0; i < (int)(sizeof(ev_cases)/sizeof(ev_cases[0])); i++) {
        registry_reset();
        static eh_rpc_evt_node_t anchor_e, cand_e;
        memset(&anchor_e, 0, sizeof(anchor_e));
        anchor_e.event_id_min = 300; anchor_e.event_id_max = 308;
        anchor_e.serialise = generic_evt_serialise;
        reg_evt_register(&anchor_e);

        memset(&cand_e, 0, sizeof(cand_e));
        cand_e.event_id_min = ev_cases[i].lo; cand_e.event_id_max = ev_cases[i].hi;
        cand_e.serialise = generic_evt_serialise;
        esp_err_t r = reg_evt_register(&cand_e);
        bool failed = (r == ESP_ERR_INVALID_STATE);
        char name[48]; snprintf(name, sizeof(name), "%s", ev_cases[i].name);
        CHECK(name, failed == ev_cases[i].should_fail);
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 * F. Multi-extension coexistence: FG + MCU registered simultaneously
 * ══════════════════════════════════════════════════════════════════════════ */
static void test_F_fg_mcu_coexistence(void) {
    registry_reset();
    static eh_rpc_req_node_t fg, mcu;
    static eh_rpc_evt_node_t fg_e, mcu_e;

    /* Requests */
    memset(&fg,  0, sizeof(fg));
    memset(&mcu, 0, sizeof(mcu));
    fg.msg_id_min  = CTRL_MSG_ID__Req_Base + 1; fg.msg_id_max  = CTRL_MSG_ID__Req_Max  - 1;
    mcu.msg_id_min = RPC_ID__Req_Base + 1;      mcu.msg_id_max = RPC_ID__Req_Max       - 1;
    fg.handler  = generic_req_handler; fg.ctx  = fg_offsets;
    mcu.handler = generic_req_handler; mcu.ctx = mcu_offsets;
    CHECK("coex_fg_reg",  reg_req_register(&fg)  == ESP_OK);
    CHECK("coex_mcu_reg", reg_req_register(&mcu) == ESP_OK);

    /* Events */
    memset(&fg_e,  0, sizeof(fg_e));
    memset(&mcu_e, 0, sizeof(mcu_e));
    fg_e.event_id_min  = CTRL_MSG_ID__Event_Base + 1; fg_e.event_id_max  = CTRL_MSG_ID__Event_Max - 1;
    mcu_e.event_id_min = RPC_ID__Event_Base + 1;       mcu_e.event_id_max = RPC_ID__Event_Max - 1;
    fg_e.serialise  = generic_evt_serialise;
    mcu_e.serialise = generic_evt_serialise;
    CHECK("coex_fg_evt_reg",  reg_evt_register(&fg_e)  == ESP_OK);
    CHECK("coex_mcu_evt_reg", reg_evt_register(&mcu_e) == ESP_OK);

    /* Every FG request ID routes to FG handler (verified via resp_id formula) */
    for (int id = CTRL_MSG_ID__Req_Base+1; id < CTRL_MSG_ID__Req_Max; id++) {
        uint32_t resp_id = 0; dispatch_proto_req((uint32_t)id, &resp_id);
        uint32_t want = (uint32_t)(id - CTRL_MSG_ID__Req_Base + CTRL_MSG_ID__Resp_Base);
        char name[48]; snprintf(name, sizeof(name), "coex_fg_all_id%d", id);
        CHECK(name, resp_id == want);
    }

    /* A representative set of MCU IDs routes to MCU handler */
    uint32_t mcu_sample[] = {257, 278, 350, 385, 387};
    for (int i = 0; i < 5; i++) {
        uint32_t resp_id = 0; dispatch_proto_req(mcu_sample[i], &resp_id);
        uint32_t want = mcu_sample[i] - RPC_ID__Req_Base + RPC_ID__Resp_Base;
        char name[48]; snprintf(name, sizeof(name), "coex_mcu_0x%x", mcu_sample[i]);
        CHECK(name, resp_id == want);
    }

    /* Re-registering either range is rejected */
    static eh_rpc_req_node_t dup_fg;
    memset(&dup_fg, 0, sizeof(dup_fg));
    dup_fg.msg_id_min = 110; dup_fg.msg_id_max = 120;
    dup_fg.handler = generic_req_handler; dup_fg.ctx = fg_offsets;
    CHECK("coex_fg_dup_rejected", reg_req_register(&dup_fg) == ESP_ERR_INVALID_STATE);

    /* Events: FG and MCU event IDs fire different serialise calls */
    reset_log();
    reg_send_event(CTRL_MSG_ID__Event_Heartbeat, NULL, 0);
    reg_send_event(RPC_ID__Event_ESPInit, NULL, 0);
    CHECK("coex_evt_2_calls", s_log_n == 2);
    /* First call should be whichever event fires first — check both IDs present */
    bool saw_fg_hb = false, saw_mcu_init = false;
    for (int i = 0; i < s_log_n; i++) {
        if (s_log[i].msg_id == CTRL_MSG_ID__Event_Heartbeat) saw_fg_hb   = true;
        if (s_log[i].msg_id == RPC_ID__Event_ESPInit)        saw_mcu_init = true;
    }
    CHECK("coex_fg_hb_event",   saw_fg_hb);
    CHECK("coex_mcu_init_event", saw_mcu_init);
}

/* ══════════════════════════════════════════════════════════════════════════
 * G. Init/re-init/idempotency sequences
 * ══════════════════════════════════════════════════════════════════════════ */

/* Simulates eh_cp_feat_rpc_ext_mcu_init() */
static bool  s_mcu_init_done = false;
static eh_rpc_req_node_t s_sim_mcu_req;
static eh_rpc_evt_node_t s_sim_mcu_evt;

static esp_err_t sim_mcu_init(void) {
    if (s_mcu_init_done) return ESP_OK;  /* idempotent */
    memset(&s_sim_mcu_req, 0, sizeof(s_sim_mcu_req));
    s_sim_mcu_req.msg_id_min = RPC_ID__Req_Base + 1;
    s_sim_mcu_req.msg_id_max = RPC_ID__Req_Max  - 1;
    s_sim_mcu_req.handler    = generic_req_handler;
    s_sim_mcu_req.ctx        = mcu_offsets;
    esp_err_t r = reg_req_register(&s_sim_mcu_req);
    if (r != ESP_OK) return r;

    memset(&s_sim_mcu_evt, 0, sizeof(s_sim_mcu_evt));
    s_sim_mcu_evt.event_id_min = RPC_ID__Event_Base + 1;
    s_sim_mcu_evt.event_id_max = RPC_ID__Event_Max  - 1;
    s_sim_mcu_evt.serialise    = generic_evt_serialise;
    r = reg_evt_register(&s_sim_mcu_evt);
    if (r != ESP_OK) return r;

    reg_add_cap_bits(0, 0x01);
    reg_add_feat_caps(EH_FEAT_IDX_WIFI, 0x1);
    reg_add_feat_caps(EH_FEAT_IDX_OTA,  0x1);
    s_mcu_init_done = true;
    return ESP_OK;
}

static void test_G_init_sequences(void) {
    registry_reset(); s_mcu_init_done = false;

    /* First init succeeds */
    CHECK("sim_init_first",  sim_mcu_init() == ESP_OK);
    /* Second call is idempotent (returns OK without re-registering) */
    CHECK("sim_init_second", sim_mcu_init() == ESP_OK);

    /* Registry has exactly one req and one evt node */
    int req_count = 0, evt_count = 0;
    eh_rpc_req_node_t *rn; SLIST_FOREACH(rn, &g_rpc_req_list, next) req_count++;
    eh_rpc_evt_node_t *en; SLIST_FOREACH(en, &g_rpc_evt_list, next) evt_count++;
    CHECK("init_one_req_node", req_count == 1);
    CHECK("init_one_evt_node", evt_count == 1);

    /* Caps were accumulated exactly once (idempotent second call must not double-add) */
    CHECK("init_caps_once", reg_get_ext_caps() == 0x01);
    CHECK("init_feat_wifi", s_feat_caps[EH_FEAT_IDX_WIFI] == 0x1);

    /* Attempt to register a second conflicting req node AFTER init → rejected */
    static eh_rpc_req_node_t conflict;
    memset(&conflict, 0, sizeof(conflict));
    conflict.msg_id_min = 300; conflict.msg_id_max = 386;
    conflict.handler = generic_req_handler; conflict.ctx = mcu_offsets;
    CHECK("init_conflict_rejected", reg_req_register(&conflict) == ESP_ERR_INVALID_STATE);

    /* A completely new range (e.g. a hypothetical third extension) CAN register */
    static eh_rpc_req_node_t third;
    memset(&third, 0, sizeof(third));
    third.msg_id_min = 1000; third.msg_id_max = 1099;
    third.handler = generic_req_handler; third.ctx = mcu_offsets;
    CHECK("init_third_ext_ok", reg_req_register(&third) == ESP_OK);

    /* Dispatch to the third extension works */
    uint32_t resp_id = 0;
    esp_err_t r = dispatch_proto_req(1050, &resp_id);
    CHECK("third_ext_dispatch", r == ESP_OK);
}

/* ══════════════════════════════════════════════════════════════════════════
 * H. Event routing: send_event → serialise callback → wire bytes
 * ══════════════════════════════════════════════════════════════════════════ */
static void test_H_event_routing(void) {
    registry_reset();
    static eh_rpc_evt_node_t fg_e, mcu_e;

    memset(&fg_e,  0, sizeof(fg_e));
    fg_e.event_id_min  = CTRL_MSG_ID__Event_Base + 1;
    fg_e.event_id_max  = CTRL_MSG_ID__Event_Max  - 1;
    fg_e.serialise     = generic_evt_serialise;
    reg_evt_register(&fg_e);

    memset(&mcu_e, 0, sizeof(mcu_e));
    mcu_e.event_id_min = RPC_ID__Event_Base + 1;
    mcu_e.event_id_max = RPC_ID__Event_Max  - 1;
    mcu_e.serialise    = generic_evt_serialise;
    reg_evt_register(&mcu_e);

    /* All FG event IDs in [301,308] must fire */
    uint32_t fg_evts[] = {301,302,303,304,305,306,307,308};
    for (int i = 0; i < 8; i++) {
        reset_log();
        esp_err_t r = reg_send_event(fg_evts[i], NULL, 0);
        char name[48]; snprintf(name, sizeof(name), "evt_fg_%u", fg_evts[i]);
        CHECK(name, r == ESP_OK && s_log_n == 1 && s_log[0].msg_id == fg_evts[i]);
    }

    /* FG sentinels 300, 309 → NOT_FOUND */
    CHECK("evt_fg_base_sentinel", reg_send_event(300, NULL, 0) == ESP_ERR_NOT_FOUND);
    CHECK("evt_fg_max_sentinel",  reg_send_event(309, NULL, 0) == ESP_ERR_NOT_FOUND);

    /* MCU events */
    uint32_t mcu_evts[] = {769,770,772,773,774,775,776,777,778,788};
    for (int i = 0; i < 10; i++) {
        reset_log();
        esp_err_t r = reg_send_event(mcu_evts[i], NULL, 0);
        char name[48]; snprintf(name, sizeof(name), "evt_mcu_0x%x", mcu_evts[i]);
        CHECK(name, r == ESP_OK && s_log_n == 1 && s_log[0].msg_id == mcu_evts[i]);
    }

    /* MCU sentinels */
    CHECK("evt_mcu_base_sentinel", reg_send_event(768, NULL, 0) == ESP_ERR_NOT_FOUND);
    CHECK("evt_mcu_max_sentinel",  reg_send_event(789, NULL, 0) == ESP_ERR_NOT_FOUND);

    /* Gap between FG and MCU events (e.g. 500) → NOT_FOUND */
    CHECK("evt_gap_500", reg_send_event(500, NULL, 0) == ESP_ERR_NOT_FOUND);

    /* Failing serialise propagates ESP_FAIL */
    static eh_rpc_evt_node_t fail_e;
    memset(&fail_e, 0, sizeof(fail_e));
    fail_e.event_id_min = 2000; fail_e.event_id_max = 2010;
    fail_e.serialise = failing_evt_serialise;
    reg_evt_register(&fail_e);
    CHECK("evt_fail_propagated", reg_send_event(2005, NULL, 0) == ESP_FAIL);
}

/* ══════════════════════════════════════════════════════════════════════════
 * I. Fault injection: corrupt proto, truncated varint, wrong wire types
 * ══════════════════════════════════════════════════════════════════════════ */
static void test_I_fault_injection(void) {
    /* Tests 1-8 use a catch-all range [1, 0xFFFF] so any valid id dispatches */
    registry_reset();
    static eh_rpc_req_node_t node;
    memset(&node, 0, sizeof(node));
    node.msg_id_min = 1; node.msg_id_max = 0xFFFF;
    node.handler = generic_req_handler; node.ctx = mcu_offsets;
    reg_req_register(&node);

    /* 1. Completely empty buffer → scanner returns 0 */
    {uint32_t v=0; CHECK("fault_empty_buf", proto_extract_msg_id(NULL,0,&v) == 0); }

    /* 2. Zero-length buffer → scanner returns 0 */
    {uint8_t b[4]={0}; uint32_t v=0; CHECK("fault_zero_len", proto_extract_msg_id(b,0,&v) == 0); }

    /* 3. Buffer with only field 1 (no field 2) */
    {
        uint8_t b[8]; int len = 0;
        b[len++] = (1u<<3)|0u; len += encode_varint(b+len, 1u); /* field 1 only */
        uint32_t v=0; CHECK("fault_no_field2", proto_extract_msg_id(b,(uint16_t)len,&v) == 0);
    }

    /* 4. Truncated varint for field 2 (MSB set on last byte, stream ends) */
    {
        uint8_t b[] = { (2u<<3)|0u, 0x80, 0x80 }; /* tag + 2 MSB-set bytes, no terminator */
        uint32_t v=0; CHECK("fault_truncated_varint", proto_extract_msg_id(b,3,&v) == 0);
    }

    /* 5. Unknown wire type in field before field 2 → scanner aborts, returns 0 */
    {
        uint8_t b[16]; int pos = 0; uint32_t v = 0;
        b[pos++] = (1u<<3)|6u;    /* field 1, wire type 6 = invalid */
        b[pos++] = (2u<<3)|0u;
        pos += encode_varint(b+pos, 200u);
        CHECK("fault_bad_wire_type", proto_extract_msg_id(b,(uint16_t)pos,&v) == 0);
    }

    /* 6. Field 2 present but followed by garbage — scanner stops cleanly after extracting */
    {
        uint8_t b[32]; int pos = 0; uint32_t v = 0;
        b[pos++] = (2u<<3)|0u; pos += encode_varint(b+pos, 300u);
        b[pos++] = 0xFF; b[pos++] = 0xFF; b[pos++] = 0xFF; /* garbage */
        uint32_t f = proto_extract_msg_id(b,(uint16_t)pos,&v);
        CHECK("fault_garbage_after_f2", f == 2 && v == 300u);
    }

    /* 7. Wire type 2 (length-delimited) with claimed length that would overflow */
    {
        uint8_t b[16]; int pos = 0; uint32_t v = 0;
        b[pos++] = (1u<<3)|2u;   /* field 1, len-delimited */
        pos += encode_varint(b+pos, 200u);  /* claims 200 bytes follow */
        /* only ~12 bytes remain in buffer → scanner should skip gracefully */
        b[pos++] = (2u<<3)|0u; pos += encode_varint(b+pos, 400u);
        /* The scan will jump i += 200 past end of buffer so field 2 is never found */
        uint32_t f = proto_extract_msg_id(b,(uint16_t)pos,&v);
        /* field 2 follows the overflowed skip, so it won't be reached */
        (void)f; /* result is implementation-defined for this malformed case */
        CHECK("fault_len_delimited_overflow_no_crash", 1); /* test just that it doesn't crash */
    }

    /* 8. dispatch_req with NULL req_buf (zero length) — handler is still called,
     *    req_len=0 so handler never dereferences req_buf */
    {
        reset_log();
        /* range [1,0xFFFF] is registered; msg_id=500 is in range */
        esp_err_t r = reg_dispatch_req_noresp(500, NULL, 0);
        CHECK("fault_null_req_buf_no_crash", r == ESP_OK);
    }

    /* 9. Dispatch to a handler that returns ESP_FAIL — error propagates */
    {
        registry_reset();
        static eh_rpc_req_node_t fn;
        memset(&fn, 0, sizeof(fn)); fn.msg_id_min = 50; fn.msg_id_max = 60;
        fn.handler = failing_req_handler; fn.ctx = NULL;
        reg_req_register(&fn);
        uint8_t wire[8]; int wl = encode_msg_id_only(wire, 55);
        esp_err_t r = reg_dispatch_req_noresp(55, wire, (uint16_t)wl);
        CHECK("fault_handler_fail_propagated", r == ESP_FAIL);
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 * J. Resp msg_id derivation: every FG and MCU req ID
 * ══════════════════════════════════════════════════════════════════════════ */
static void test_J_resp_id_derivation(void) {
    registry_reset();
    static eh_rpc_req_node_t fg, mcu;
    memset(&fg,  0, sizeof(fg));  fg.msg_id_min  = 101; fg.msg_id_max  = 128; fg.handler  = generic_req_handler; fg.ctx  = fg_offsets;
    memset(&mcu, 0, sizeof(mcu)); mcu.msg_id_min = 257; mcu.msg_id_max = 387; mcu.handler = generic_req_handler; mcu.ctx = mcu_offsets;
    reg_req_register(&fg); reg_req_register(&mcu);

    /* Every single FG request ID */
    for (int id = CTRL_MSG_ID__Req_Base+1; id < CTRL_MSG_ID__Req_Max; id++) {
        uint32_t resp_id = 0; dispatch_proto_req((uint32_t)id, &resp_id);
        uint32_t want = (uint32_t)(id - CTRL_MSG_ID__Req_Base + CTRL_MSG_ID__Resp_Base);
        char name[48]; snprintf(name, sizeof(name), "resp_fg_%d", id);
        CHECK(name, resp_id == want);
    }

    /* Spot-check MCU IDs across the range */
    uint32_t mcu_ids[] = {257,258,259,260,270,271,272,277,278,279,280,281,282,283,
                          284,285,286,287,288,289,290,291,292,293,294,297,298,299,
                          300,301,302,325,326,334,335,350,352,353,355,356,357,358,
                          359,380,381,382,383,384,385,386,387};
    for (int i = 0; i < (int)(sizeof(mcu_ids)/sizeof(mcu_ids[0])); i++) {
        uint32_t resp_id = 0; dispatch_proto_req(mcu_ids[i], &resp_id);
        uint32_t want = mcu_ids[i] - RPC_ID__Req_Base + RPC_ID__Resp_Base;
        char name[48]; snprintf(name, sizeof(name), "resp_mcu_0x%x", mcu_ids[i]);
        CHECK(name, resp_id == want);
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 * K. Proto scanner stress: field ordering, large varints, 5-byte varints
 * ══════════════════════════════════════════════════════════════════════════ */
static void test_K_scanner_stress(void) {
    /* 5-byte varint (value 0x10000000 = 268435456) */
    {
        uint8_t b[16]; int pos = 0; uint32_t v = 0;
        b[pos++] = (2u<<3)|0u;
        /* encode 0x10000000 as 5-byte varint: 80 80 80 80 01 */
        b[pos++] = 0x80; b[pos++] = 0x80; b[pos++] = 0x80; b[pos++] = 0x80; b[pos++] = 0x01;
        uint32_t f = proto_extract_msg_id(b,(uint16_t)pos,&v);
        CHECK("scanner_5byte_varint", f == 2 && v == 0x10000000u);
    }

    /* Max meaningful value: 0xFFFF (65535) = 3 varint bytes: FF FF 03 */
    {
        uint8_t b[16]; int pos = 0; uint32_t v = 0;
        b[pos++] = (2u<<3)|0u;
        b[pos++] = 0xFF; b[pos++] = 0xFF; b[pos++] = 0x03;
        uint32_t f = proto_extract_msg_id(b,(uint16_t)pos,&v);
        CHECK("scanner_0xffff", f == 2 && v == 0xFFFFu);
    }

    /* Field 2 buried after two 64-bit fixed fields (wire type 1) */
    {
        uint8_t b[64]; int pos = 0; uint32_t v = 0;
        b[pos++] = (1u<<3)|1u; memset(b+pos,0xAA,8); pos += 8; /* f1: 64-bit */
        b[pos++] = (3u<<3)|1u; memset(b+pos,0xBB,8); pos += 8; /* f3: 64-bit */
        b[pos++] = (2u<<3)|0u; pos += encode_varint(b+pos, 769u); /* f2 */
        uint32_t f = proto_extract_msg_id(b,(uint16_t)pos,&v);
        CHECK("scanner_skip_two_64bit", f == 2 && v == 769u);
    }

    /* Field 2 after a 32-bit fixed field (wire type 5) */
    {
        uint8_t b[32]; int pos = 0; uint32_t v = 0;
        b[pos++] = (1u<<3)|5u; memset(b+pos,0xCC,4); pos += 4;
        b[pos++] = (2u<<3)|0u; pos += encode_varint(b+pos, 387u);
        uint32_t f = proto_extract_msg_id(b,(uint16_t)pos,&v);
        CHECK("scanner_skip_32bit", f == 2 && v == 387u);
    }

    /* Field 2 after a length-delimited field (wire type 2) with real embedded bytes */
    {
        uint8_t b[64]; int pos = 0; uint32_t v = 0;
        b[pos++] = (1u<<3)|2u;  /* f1: len-delimited */
        b[pos++] = 5;           /* 5 bytes */
        memcpy(b+pos, "hello", 5); pos += 5;
        b[pos++] = (2u<<3)|0u; pos += encode_varint(b+pos, 282u);
        uint32_t f = proto_extract_msg_id(b,(uint16_t)pos,&v);
        CHECK("scanner_skip_len_delim", f == 2 && v == 282u);
    }

    /* Multiple back-to-back messages: scanner only processes first message */
    {
        uint8_t b[64]; int pos = 0; uint32_t v = 0;
        pos += encode_msg_id_only(b+pos, 101u);   /* msg 1: id=101 */
        pos += encode_msg_id_only(b+pos, 200u);   /* msg 2: id=200 (field 2 again!) */
        /* proto_extract_msg_id returns the FIRST field-2 found = 101 */
        proto_extract_msg_id(b,(uint16_t)pos,&v);
        CHECK("scanner_first_field2_wins", v == 101u);
    }

    /* NULL output pointer — must return 0, no crash */
    {
        uint8_t b[8]; int len = encode_msg_id_only(b, 111u);
        CHECK("scanner_null_out", proto_extract_msg_id(b,(uint16_t)len,NULL) == 0);
    }

    /* Single-byte buffer (tag only, no value) */
    {
        uint8_t b[1] = {(2u<<3)|0u}; uint32_t v = 0;
        CHECK("scanner_one_byte", proto_extract_msg_id(b,1,&v) == 0);
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 * L. Registry state integrity: register→dispatch→re-register sequence
 * ══════════════════════════════════════════════════════════════════════════ */
static void test_L_state_integrity(void) {
    registry_reset();

    /* Step 1: register three non-overlapping nodes in scrambled order */
    static eh_rpc_req_node_t n1, n2, n3;
    static uint32_t ctx1[2] = {600,0}, ctx2[2] = {700,300}, ctx3[2] = {800,500};
    memset(&n1,0,sizeof(n1)); n1.msg_id_min=200; n1.msg_id_max=299; n1.handler=generic_req_handler; n1.ctx=ctx1;
    memset(&n2,0,sizeof(n2)); n2.msg_id_min=100; n2.msg_id_max=199; n2.handler=generic_req_handler; n2.ctx=ctx2;
    memset(&n3,0,sizeof(n3)); n3.msg_id_min=300; n3.msg_id_max=399; n3.handler=generic_req_handler; n3.ctx=ctx3;
    /* Insert in order n1(200-299), n2(100-199), n3(300-399) */
    CHECK("sl_reg_n1", reg_req_register(&n1) == ESP_OK);
    CHECK("sl_reg_n2", reg_req_register(&n2) == ESP_OK);
    CHECK("sl_reg_n3", reg_req_register(&n3) == ESP_OK);

    /* Step 2: verify sorted order n2 → n1 → n3 */
    eh_rpc_req_node_t *cur = SLIST_FIRST(&g_rpc_req_list);
    CHECK("sl_order_first",  cur && cur->msg_id_min == 100);
    cur = SLIST_NEXT(cur, next);
    CHECK("sl_order_second", cur && cur->msg_id_min == 200);
    cur = SLIST_NEXT(cur, next);
    CHECK("sl_order_third",  cur && cur->msg_id_min == 300);

    /* Step 3: dispatch boundary values */
    uint8_t wire[8]; int wl;

    wl = encode_msg_id_only(wire, 100);
    CHECK("sl_boundary_100", reg_dispatch_req_noresp(100, wire, (uint16_t)wl) == ESP_OK);
    wl = encode_msg_id_only(wire, 199);
    CHECK("sl_boundary_199", reg_dispatch_req_noresp(199, wire, (uint16_t)wl) == ESP_OK);
    wl = encode_msg_id_only(wire, 200);
    CHECK("sl_boundary_200", reg_dispatch_req_noresp(200, wire, (uint16_t)wl) == ESP_OK);
    wl = encode_msg_id_only(wire, 399);
    CHECK("sl_boundary_399", reg_dispatch_req_noresp(399, wire, (uint16_t)wl) == ESP_OK);
    wl = encode_msg_id_only(wire, 99);
    CHECK("sl_below_99",  reg_dispatch_req_noresp(99,  wire, (uint16_t)wl) == ESP_ERR_NOT_FOUND);
    wl = encode_msg_id_only(wire, 400);
    CHECK("sl_above_400", reg_dispatch_req_noresp(400, wire, (uint16_t)wl) == ESP_ERR_NOT_FOUND);

    /* Step 4: insert a 4th node between existing ones — [205,210] overlaps n1[200,299] */
    static eh_rpc_req_node_t n4;
    memset(&n4,0,sizeof(n4)); n4.msg_id_min=205; n4.msg_id_max=210;
    n4.handler=generic_req_handler; n4.ctx=ctx1;
    CHECK("sl_inside_overlap_rejected", reg_req_register(&n4) == ESP_ERR_INVALID_STATE);

    /* Step 5: insert a valid 4th node [400, 450] */
    static eh_rpc_req_node_t n5;
    memset(&n5,0,sizeof(n5)); n5.msg_id_min=400; n5.msg_id_max=450;
    n5.handler=generic_req_handler; n5.ctx=mcu_offsets;
    CHECK("sl_new_node_ok", reg_req_register(&n5) == ESP_OK);

    /* Step 6: capability accumulator accumulates additively */
    reg_add_cap_bits(0x01, 0x00);
    reg_add_cap_bits(0x02, 0x10);
    reg_add_cap_bits(0x04, 0x20);
    CHECK("sl_caps_or",     reg_get_caps()     == 0x07);
    CHECK("sl_ext_caps_or", reg_get_ext_caps() == 0x30);

    /* feat_caps per index */
    reg_add_feat_caps(0, 0x0F);
    reg_add_feat_caps(1, 0x11);
    reg_add_feat_caps(0, 0xF0);  /* OR into index 0 again */
    CHECK("sl_feat0", s_feat_caps[0] == 0xFF);
    CHECK("sl_feat1", s_feat_caps[1] == 0x11);
    CHECK("sl_feat2", s_feat_caps[2] == 0x00);

    /* OOB feat_cap index silently dropped */
    reg_add_feat_caps(EH_FEAT_CAPS_COUNT, 0xFF);
    reg_add_feat_caps(255, 0xFF);
    bool all_zero = true;
    for (int i = EH_FEAT_CAPS_COUNT-1; i >= 0; i--)
        if (s_feat_caps[i] != 0 && i != 0 && i != 1) { all_zero = false; break; }
    CHECK("sl_feat_oob_safe", all_zero);
}

/* ══════════════════════════════════════════════════════════════════════════
 * main
 * ══════════════════════════════════════════════════════════════════════════ */
int main(void) {
    printf("eh_cp — System integration tests\n\n");

    printf("── A. Proto wire encode/decode round-trips ────────────────────────\n");
    test_A_proto_roundtrip();

    printf("── B. TLV framing (compose + parse) ──────────────────────────────\n");
    test_B_tlv_framing();

    printf("── C. SLIST registry: multi-node, sorting, overlap matrix ─────────\n");
    test_C_slist_complex();

    printf("── D. Full pipeline: wire bytes → scanner → dispatch → resp ───────\n");
    test_D_full_pipeline();

    printf("── E. Overlap edge-case matrix ────────────────────────────────────\n");
    test_E_overlap_matrix();

    printf("── F. Multi-extension coexistence (FG + MCU simultaneous) ─────────\n");
    test_F_fg_mcu_coexistence();

    printf("── G. Init/re-init/idempotency sequences ──────────────────────────\n");
    test_G_init_sequences();

    printf("── H. Event routing: send_event → serialise → wire bytes ──────────\n");
    test_H_event_routing();

    printf("── I. Fault injection: corrupt/truncated/wrong-type proto ──────────\n");
    test_I_fault_injection();

    printf("── J. Resp msg_id derivation: every FG and MCU req ID ─────────────\n");
    test_J_resp_id_derivation();

    printf("── K. Proto scanner stress: field ordering, varints, edge cases ────\n");
    test_K_scanner_stress();

    printf("── L. Registry state integrity after register→dispatch→re-register ─\n");
    test_L_state_integrity();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
