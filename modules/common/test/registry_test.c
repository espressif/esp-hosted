/* registry_test.c — Three-Registry host-compiled unit tests
 *
 * Tests Registry 1 (iface table), Registry 2 (cap accumulator),
 * Registry 3 (RPC SLIST: req sorted-insert, overlap, dispatch, evt, proto
 * scanner) and the critical FG / MCU coexistence scenarios.
 *
 * Compile:
 *   make -C components/common/eh_cp/test run
 * or manually:
 *   gcc -std=c11 -Wall -Wextra -Wpedantic -Wshadow \
 *       -o /tmp/registry_test_bin \
 *       components/common/eh_cp/test/registry_test.c
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <sys/queue.h>  /* BSD SLIST — available on macOS and Linux */

/* ══════════════════════════════════════════════════════════════════════════
 * Minimal stubs replacing IDF types / macros
 * ══════════════════════════════════════════════════════════════════════════ */
typedef int esp_err_t;
#define ESP_OK                   0
#define ESP_FAIL                -1
#define ESP_ERR_INVALID_ARG     0x102
#define ESP_ERR_INVALID_STATE   0x103
#define ESP_ERR_NOT_FOUND       0x105
#define ESP_ERR_NO_MEM          0x101

static inline const char *esp_err_to_name(esp_err_t e) __attribute__((unused));
static inline const char *esp_err_to_name(esp_err_t e) {
    switch (e) {
    case ESP_OK:                return "ESP_OK";
    case ESP_FAIL:              return "ESP_FAIL";
    case ESP_ERR_INVALID_ARG:  return "ESP_ERR_INVALID_ARG";
    case ESP_ERR_INVALID_STATE: return "ESP_ERR_INVALID_STATE";
    case ESP_ERR_NOT_FOUND:    return "ESP_ERR_NOT_FOUND";
    case ESP_ERR_NO_MEM:       return "ESP_ERR_NO_MEM";
    default:                   return "UNKNOWN";
    }
}

#define ESP_LOGE(tag, fmt, ...) ((void)0)
#define ESP_LOGW(tag, fmt, ...) ((void)0)
#define ESP_LOGI(tag, fmt, ...) ((void)0)
#define ESP_LOGD(tag, fmt, ...) ((void)0)

#ifndef PRIx32
#define PRIx32 "x"
#define PRIu32 "u"
#endif

/* ══════════════════════════════════════════════════════════════════════════
 * Inline registry implementation (stripped of FreeRTOS / IDF deps)
 * Mirrors eh_cp_registries.c exactly — same logic, no OS calls.
 * ══════════════════════════════════════════════════════════════════════════ */
#define ESP_IF_TYPE_MAX          8
#define EH_FEAT_CAPS_COUNT 8

typedef int (*hosted_rx_cb_t)(void *ctx, void *buf, uint16_t len, void *eb);
typedef int (*hosted_tx_cb_t)(void *ctx, void *frame, uint16_t len);

typedef struct {
    hosted_rx_cb_t rx;
    hosted_tx_cb_t tx;
    void          *ctx;
} eh_iface_entry_t;

/* ── Registry 1 ─────────────────────────────────────────────────────────── */
static eh_iface_entry_t g_iface_table[ESP_IF_TYPE_MAX];
static void r1_reset(void) { memset(g_iface_table, 0, sizeof(g_iface_table)); }

static esp_err_t r1_register_rx(uint8_t iface, hosted_rx_cb_t cb, void *ctx)
{
    if (iface >= ESP_IF_TYPE_MAX || !cb)  return ESP_ERR_INVALID_ARG;
    if (g_iface_table[iface].rx && iface != 0 && iface != 1)
        return ESP_ERR_INVALID_STATE;
    g_iface_table[iface].rx  = cb;
    g_iface_table[iface].ctx = ctx;
    return ESP_OK;
}
static esp_err_t r1_register_tx(uint8_t iface, hosted_tx_cb_t cb, void *ctx)
{
    if (iface >= ESP_IF_TYPE_MAX) return ESP_ERR_INVALID_ARG;
    g_iface_table[iface].tx  = cb;
    g_iface_table[iface].ctx = ctx;
    return ESP_OK;
}
static esp_err_t r1_dispatch_rx(uint8_t iface, void *buf, uint16_t len, void *eb)
{
    if (iface >= ESP_IF_TYPE_MAX) return ESP_ERR_INVALID_ARG;
    if (!g_iface_table[iface].rx) return ESP_OK;
    return (esp_err_t)g_iface_table[iface].rx(g_iface_table[iface].ctx, buf, len, eb);
}

/* ── Registry 2 ─────────────────────────────────────────────────────────── */
static uint8_t  s_caps     = 0;
static uint32_t s_ext_caps = 0;
static uint32_t s_feat_caps[EH_FEAT_CAPS_COUNT] = {0};

static void r2_reset(void) {
    s_caps = 0; s_ext_caps = 0;
    memset(s_feat_caps, 0, sizeof(s_feat_caps));
}
static void     r2_add_caps(uint8_t c, uint32_t e) { s_caps |= c; s_ext_caps |= e; }
static void     r2_add_feat(uint8_t idx, uint32_t bits)
    { if (idx < EH_FEAT_CAPS_COUNT) s_feat_caps[idx] |= bits; }
static uint8_t  r2_get_caps(void)     { return s_caps; }
static uint32_t r2_get_ext_caps(void) { return s_ext_caps; }
static void     r2_get_feat(uint32_t out[EH_FEAT_CAPS_COUNT])
    { memcpy(out, s_feat_caps, sizeof(s_feat_caps)); }

/* ── Registry 3 types ───────────────────────────────────────────────────── */
typedef struct eh_rpc_req_node {
    uint16_t   msg_id_min, msg_id_max;
    esp_err_t (*handler)(void *ctx, uint32_t msg_id,
                         const void *req_buf, uint16_t req_len,
                         void *resp_buf, uint16_t *resp_len, uint16_t resp_max);
    void      *ctx;
    SLIST_ENTRY(eh_rpc_req_node) next;
} eh_rpc_req_node_t;

typedef struct eh_rpc_evt_node {
    uint16_t   event_id_min, event_id_max;
    esp_err_t (*serialise)(void *ctx, uint32_t event_id,
                           const void *data, uint16_t data_len,
                           void *out_buf, uint16_t *out_len, uint16_t out_max);
    void      *ctx;
    SLIST_ENTRY(eh_rpc_evt_node) next;
} eh_rpc_evt_node_t;

SLIST_HEAD(rpc_req_list, eh_rpc_req_node);
SLIST_HEAD(rpc_evt_list, eh_rpc_evt_node);
static struct rpc_req_list g_rpc_req_list;
static struct rpc_evt_list g_rpc_evt_list;

static bool r3_ranges_overlap(uint16_t mina, uint16_t maxa, uint16_t minb, uint16_t maxb)
    { return (mina <= maxb) && (minb <= maxa); }

static void r3_reset(void) {
    SLIST_INIT(&g_rpc_req_list);
    SLIST_INIT(&g_rpc_evt_list);
}

static esp_err_t r3_req_register(eh_rpc_req_node_t *node)
{
    if (!node || !node->handler)             return ESP_ERR_INVALID_ARG;
    if (node->msg_id_min > node->msg_id_max) return ESP_ERR_INVALID_ARG;
    eh_rpc_req_node_t *n;
    SLIST_FOREACH(n, &g_rpc_req_list, next) {
        if (r3_ranges_overlap(node->msg_id_min, node->msg_id_max, n->msg_id_min, n->msg_id_max))
            return ESP_ERR_INVALID_STATE;
    }
    eh_rpc_req_node_t *prev = NULL;
    SLIST_FOREACH(n, &g_rpc_req_list, next) {
        if (n->msg_id_min > node->msg_id_min) break;
        prev = n;
    }
    if (!prev) SLIST_INSERT_HEAD(&g_rpc_req_list, node, next);
    else       SLIST_INSERT_AFTER(prev, node, next);
    return ESP_OK;
}

static esp_err_t r3_evt_register(eh_rpc_evt_node_t *node)
{
    if (!node || !node->serialise)               return ESP_ERR_INVALID_ARG;
    if (node->event_id_min > node->event_id_max) return ESP_ERR_INVALID_ARG;
    eh_rpc_evt_node_t *n;
    SLIST_FOREACH(n, &g_rpc_evt_list, next) {
        if (r3_ranges_overlap(node->event_id_min, node->event_id_max, n->event_id_min, n->event_id_max))
            return ESP_ERR_INVALID_STATE;
    }
    SLIST_INSERT_HEAD(&g_rpc_evt_list, node, next);
    return ESP_OK;
}

static esp_err_t r3_dispatch_req(uint32_t msg_id,
                                  const void *req, uint16_t req_len,
                                  void *resp, uint16_t *resp_len, uint16_t resp_max)
{
    eh_rpc_req_node_t *n;
    SLIST_FOREACH(n, &g_rpc_req_list, next) {
        if (msg_id > (uint32_t)n->msg_id_max) continue;
        if (msg_id < (uint32_t)n->msg_id_min) break;
        return n->handler(n->ctx, msg_id, req, req_len, resp, resp_len, resp_max);
    }
    return ESP_ERR_NOT_FOUND;
}

static esp_err_t r3_send_event(uint32_t event_id, const void *data, uint16_t data_len)
{
    eh_rpc_evt_node_t *n;
    SLIST_FOREACH(n, &g_rpc_evt_list, next) {
        if (event_id > (uint32_t)n->event_id_max) continue;
        if (event_id < (uint32_t)n->event_id_min) continue;
        uint8_t out[4096]; uint16_t out_len = 0;
        return n->serialise(n->ctx, event_id, data, data_len, out, &out_len, (uint16_t)sizeof(out));
    }
    return ESP_ERR_NOT_FOUND;
}

/* ── Proto field-2 scanner (copy of production code) ───────────────────── */
static uint32_t proto_extract_msg_id(const uint8_t *buf, uint16_t len, uint32_t *msg_id_out)
{
    if (!buf || !msg_id_out || len == 0) return 0;
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
                if (!(b & 0x80u)) { *msg_id_out = v; return 2u; }
                shift = (uint8_t)(shift + 7u);
            }
            return 0;
        }
        switch (wire_type) {
        case 0: while (i < len && (buf[i] & 0x80u)) i++; if (i < len) i++; break;
        case 1: i = (uint16_t)(i + 8u); break;
        case 2: {
            uint32_t slen = 0; uint8_t shift = 0;
            while (i < len && shift < 35) {
                uint8_t b = buf[i++];
                slen |= (uint32_t)(b & 0x7Fu) << shift;
                if (!(b & 0x80u)) break;
                shift = (uint8_t)(shift + 7u);
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
 * Test framework
 * ══════════════════════════════════════════════════════════════════════════ */
static int g_pass = 0, g_fail = 0;
#define PASS(n)       do { printf("  PASS  %s\n", (n)); g_pass++; } while(0)
#define FAIL(n, msg)  do { printf("  FAIL  %s  (%s)\n", (n), (msg)); g_fail++; } while(0)
#define CHECK(n, c)   do { if (c) PASS(n); else FAIL(n, #c); } while(0)

/* ══════════════════════════════════════════════════════════════════════════
 * Range constants — match extension source
 * ══════════════════════════════════════════════════════════════════════════ */
/* Linux FG (ctrl_lib proto): Req [101..128], Evt [301..308] */
#define FG_REQ_MIN  101u
#define FG_REQ_MAX  128u
#define FG_EVT_MIN  301u
#define FG_EVT_MAX  308u

/* MCU (eh_rpc.proto): Req [0x101..0x183], Evt [0x301..0x314] */
#define MCU_REQ_MIN  0x101u
#define MCU_REQ_MAX  0x183u
#define MCU_EVT_MIN  0x301u
#define MCU_EVT_MAX  0x314u

/* ══════════════════════════════════════════════════════════════════════════
 * Dummy callbacks
 * ══════════════════════════════════════════════════════════════════════════ */
static int s_last_rx_iface = -1, s_rx_call_count = 0;
static int dummy_rx_cb(void *ctx, void *buf, uint16_t len, void *eb)
    { (void)buf;(void)len;(void)eb; s_last_rx_iface=(int)(uintptr_t)ctx; s_rx_call_count++; return ESP_OK; }
static int dummy_rx_fail(void *ctx, void *buf, uint16_t len, void *eb)
    { (void)ctx;(void)buf;(void)len;(void)eb; return ESP_FAIL; }
static int dummy_tx_cb(void *ctx, void *frame, uint16_t len)
    { (void)ctx;(void)frame;(void)len; return ESP_OK; }

/* Echo req handler — writes msg_id as 4-byte LE into resp */
static esp_err_t echo_req_handler(void *ctx, uint32_t msg_id,
                                   const void *req_buf, uint16_t req_len,
                                   void *resp_buf, uint16_t *resp_len, uint16_t resp_max)
{
    (void)ctx;(void)req_buf;(void)req_len;
    if (resp_max < 4) return ESP_ERR_NO_MEM;
    uint8_t *r = (uint8_t *)resp_buf;
    r[0]=(uint8_t)msg_id; r[1]=(uint8_t)(msg_id>>8);
    r[2]=(uint8_t)(msg_id>>16); r[3]=(uint8_t)(msg_id>>24);
    *resp_len = 4; return ESP_OK;
}
static esp_err_t fail_req_handler(void *ctx, uint32_t msg_id,
                                   const void *rq, uint16_t rl,
                                   void *rs, uint16_t *rsl, uint16_t rm)
    { (void)ctx;(void)msg_id;(void)rq;(void)rl;(void)rs;(void)rm; *rsl=0; return ESP_FAIL; }

static uint32_t s_last_event = 0;
static esp_err_t echo_evt_serialise(void *ctx, uint32_t event_id,
                                     const void *data, uint16_t data_len,
                                     void *out_buf, uint16_t *out_len, uint16_t out_max)
{
    (void)ctx;(void)data;(void)data_len;(void)out_max;
    s_last_event = event_id;
    uint8_t *o = (uint8_t *)out_buf;
    o[0]=(uint8_t)event_id; o[1]=(uint8_t)(event_id>>8);
    o[2]=(uint8_t)(event_id>>16); o[3]=(uint8_t)(event_id>>24);
    *out_len = 4; return ESP_OK;
}

/* ══════════════════════════════════════════════════════════════════════════
 * Registry 1 tests
 * ══════════════════════════════════════════════════════════════════════════ */
static void test_r1_null_cb(void) {
    r1_reset();
    CHECK("r1_null_cb", r1_register_rx(0, NULL, NULL) == ESP_ERR_INVALID_ARG);
}
static void test_r1_oob(void) {
    r1_reset();
    CHECK("r1_oob_rx",       r1_register_rx(ESP_IF_TYPE_MAX, dummy_rx_cb, NULL) == ESP_ERR_INVALID_ARG);
    CHECK("r1_oob_tx",       r1_register_tx(ESP_IF_TYPE_MAX, dummy_tx_cb, NULL) == ESP_ERR_INVALID_ARG);
    CHECK("r1_oob_dispatch", r1_dispatch_rx(ESP_IF_TYPE_MAX, NULL, 0, NULL) == ESP_ERR_INVALID_ARG);
}
static void test_r1_dispatch(void) {
    r1_reset(); s_rx_call_count = 0;
    r1_register_rx(2, dummy_rx_cb, (void *)(uintptr_t)2);
    CHECK("r1_dispatch_ok",    r1_dispatch_rx(2, NULL, 0, NULL) == ESP_OK);
    CHECK("r1_dispatch_count", s_rx_call_count == 1);
    CHECK("r1_dispatch_ctx",   s_last_rx_iface == 2);
}
static void test_r1_no_handler(void) {
    r1_reset();
    CHECK("r1_no_handler", r1_dispatch_rx(3, NULL, 0, NULL) == ESP_OK); /* silent drop */
}
static void test_r1_error_prop(void) {
    r1_reset();
    r1_register_rx(4, dummy_rx_fail, NULL);
    CHECK("r1_err_prop", r1_dispatch_rx(4, NULL, 0, NULL) == ESP_FAIL);
}
static void test_r1_exclusive(void) {
    r1_reset();
    CHECK("r1_exc_first",  r1_register_rx(2, dummy_rx_cb, NULL) == ESP_OK);
    CHECK("r1_exc_second", r1_register_rx(2, dummy_rx_cb, NULL) == ESP_ERR_INVALID_STATE);
}
static void test_r1_sta_rereg(void) {
    r1_reset();
    r1_register_rx(0, dummy_rx_cb, (void *)(uintptr_t)10);
    r1_register_rx(0, dummy_rx_cb, (void *)(uintptr_t)20);
    s_last_rx_iface = -1;
    r1_dispatch_rx(0, NULL, 0, NULL);
    CHECK("r1_sta_override", s_last_rx_iface == 20);
}
static void test_r1_tx_clear(void) {
    r1_reset();
    r1_register_tx(1, dummy_tx_cb, NULL);
    CHECK("r1_tx_set",  g_iface_table[1].tx == dummy_tx_cb);
    r1_register_tx(1, NULL, NULL);
    CHECK("r1_tx_gone", g_iface_table[1].tx == NULL);
}

/* ══════════════════════════════════════════════════════════════════════════
 * Registry 2 tests
 * ══════════════════════════════════════════════════════════════════════════ */
static void test_r2_accumulate(void) {
    r2_reset();
    r2_add_caps(0x01, 0x00000002u);
    r2_add_caps(0x04, 0x00000008u);
    CHECK("r2_caps",     r2_get_caps()     == 0x05);
    CHECK("r2_ext_caps", r2_get_ext_caps() == 0x0000000Au);
}
static void test_r2_feat(void) {
    r2_reset();
    r2_add_feat(0, 0x01u); r2_add_feat(0, 0x02u); r2_add_feat(1, 0xFFu);
    uint32_t out[EH_FEAT_CAPS_COUNT]; r2_get_feat(out);
    CHECK("r2_feat0", out[0] == 0x03u);
    CHECK("r2_feat1", out[1] == 0xFFu);
    CHECK("r2_feat2", out[2] == 0x00u);
}
static void test_r2_oob(void) {
    r2_reset();
    r2_add_feat(EH_FEAT_CAPS_COUNT, 0xFFu); /* out-of-bounds, must not crash */
    uint32_t out[EH_FEAT_CAPS_COUNT]; r2_get_feat(out);
    bool all_zero = true;
    for (int i = 0; i < EH_FEAT_CAPS_COUNT; i++) if (out[i]) all_zero = false;
    CHECK("r2_oob_safe", all_zero);
}
static void test_r2_idempotent(void) {
    r2_reset();
    r2_add_caps(0xAA, 0xDEADBEEFu); r2_add_caps(0xAA, 0xDEADBEEFu);
    CHECK("r2_idem_caps",     r2_get_caps()     == 0xAA);
    CHECK("r2_idem_ext_caps", r2_get_ext_caps() == 0xDEADBEEFu);
}
static void test_r2_fg_mcu_coexist(void) {
    r2_reset();
    r2_add_caps(0x01, 0); r2_add_caps(0, 0x00000001u);
    r2_add_feat(0, 0x01u); r2_add_feat(2, 0x01u);
    CHECK("r2_coexist_caps",  r2_get_caps()     == 0x01);
    CHECK("r2_coexist_ext",   r2_get_ext_caps() == 0x01u);
    uint32_t out[EH_FEAT_CAPS_COUNT]; r2_get_feat(out);
    CHECK("r2_coexist_wifi",  out[0] == 0x01u);
    CHECK("r2_coexist_ota",   out[2] == 0x01u);
}

/* ══════════════════════════════════════════════════════════════════════════
 * Registry 3 tests
 * ══════════════════════════════════════════════════════════════════════════ */
static void test_r3_null(void) {
    r3_reset();
    CHECK("r3_req_null_node",    r3_req_register(NULL) == ESP_ERR_INVALID_ARG);
    CHECK("r3_evt_null_node",    r3_evt_register(NULL) == ESP_ERR_INVALID_ARG);
}
static void test_r3_null_handler(void) {
    r3_reset();
    static eh_rpc_req_node_t n; memset(&n,0,sizeof(n));
    n.msg_id_min=10; n.msg_id_max=20;
    CHECK("r3_null_req_handler", r3_req_register(&n) == ESP_ERR_INVALID_ARG);
    static eh_rpc_evt_node_t e; memset(&e,0,sizeof(e));
    e.event_id_min=10; e.event_id_max=20;
    CHECK("r3_null_evt_handler", r3_evt_register(&e) == ESP_ERR_INVALID_ARG);
}
static void test_r3_inverted(void) {
    r3_reset();
    static eh_rpc_req_node_t n; memset(&n,0,sizeof(n));
    n.handler=echo_req_handler; n.msg_id_min=50; n.msg_id_max=10;
    CHECK("r3_inv_req", r3_req_register(&n) == ESP_ERR_INVALID_ARG);
    static eh_rpc_evt_node_t e; memset(&e,0,sizeof(e));
    e.serialise=echo_evt_serialise; e.event_id_min=50; e.event_id_max=10;
    CHECK("r3_inv_evt", r3_evt_register(&e) == ESP_ERR_INVALID_ARG);
}
static void test_r3_single_dispatch(void) {
    r3_reset();
    static eh_rpc_req_node_t n; memset(&n,0,sizeof(n));
    n.msg_id_min=100; n.msg_id_max=200; n.handler=echo_req_handler;
    r3_req_register(&n);
    uint8_t resp[16]; uint16_t rlen=0;
    CHECK("r3_min",   r3_dispatch_req(100,NULL,0,resp,&rlen,16) == ESP_OK);
    CHECK("r3_mid",   r3_dispatch_req(150,NULL,0,resp,&rlen,16) == ESP_OK);
    CHECK("r3_max",   r3_dispatch_req(200,NULL,0,resp,&rlen,16) == ESP_OK);
    CHECK("r3_below", r3_dispatch_req(99, NULL,0,resp,&rlen,16) == ESP_ERR_NOT_FOUND);
    CHECK("r3_above", r3_dispatch_req(201,NULL,0,resp,&rlen,16) == ESP_ERR_NOT_FOUND);
}
static void test_r3_overlap(void) {
    r3_reset();
    static eh_rpc_req_node_t n1,n2,n3;
    memset(&n1,0,sizeof(n1)); n1.msg_id_min=100; n1.msg_id_max=200; n1.handler=echo_req_handler;
    memset(&n2,0,sizeof(n2)); n2.msg_id_min=150; n2.msg_id_max=250; n2.handler=echo_req_handler;
    memset(&n3,0,sizeof(n3)); n3.msg_id_min=200; n3.msg_id_max=300; n3.handler=echo_req_handler;
    CHECK("r3_ovlp_first", r3_req_register(&n1) == ESP_OK);
    CHECK("r3_ovlp_mid",   r3_req_register(&n2) == ESP_ERR_INVALID_STATE);
    CHECK("r3_ovlp_touch", r3_req_register(&n3) == ESP_ERR_INVALID_STATE);
}
static void test_r3_sorted_insert(void) {
    r3_reset();
    static eh_rpc_req_node_t nA,nB,nC;
    memset(&nA,0,sizeof(nA)); nA.msg_id_min=300; nA.msg_id_max=399; nA.handler=echo_req_handler;
    memset(&nB,0,sizeof(nB)); nB.msg_id_min=200; nB.msg_id_max=299; nB.handler=echo_req_handler;
    memset(&nC,0,sizeof(nC)); nC.msg_id_min=100; nC.msg_id_max=199; nC.handler=echo_req_handler;
    r3_req_register(&nA); r3_req_register(&nB); r3_req_register(&nC);
    eh_rpc_req_node_t *n; uint16_t prev=0; bool sorted=true;
    SLIST_FOREACH(n, &g_rpc_req_list, next) {
        if (n->msg_id_min < prev) { sorted=false; break; }
        prev = n->msg_id_min;
    }
    CHECK("r3_sorted", sorted);
}
static void test_r3_echo_resp(void) {
    r3_reset();
    static eh_rpc_req_node_t n; memset(&n,0,sizeof(n));
    n.msg_id_min=1; n.msg_id_max=0xFFFF; n.handler=echo_req_handler;
    r3_req_register(&n);
    uint8_t resp[16]; uint16_t rlen=0;
    r3_dispatch_req(0x1A2B, NULL, 0, resp, &rlen, 16);
    uint32_t got = (uint32_t)(resp[0]|(resp[1]<<8)|(resp[2]<<16)|(resp[3]<<24));
    CHECK("r3_echo_len", rlen == 4);
    CHECK("r3_echo_val", got == 0x1A2Bu);
}
static void test_r3_handler_fail(void) {
    r3_reset();
    static eh_rpc_req_node_t n; memset(&n,0,sizeof(n));
    n.msg_id_min=10; n.msg_id_max=20; n.handler=fail_req_handler;
    r3_req_register(&n);
    uint8_t resp[16]; uint16_t rlen=0;
    CHECK("r3_fail_prop", r3_dispatch_req(15,NULL,0,resp,&rlen,16) == ESP_FAIL);
}
static void test_r3_empty(void) {
    r3_reset();
    uint8_t resp[16]; uint16_t rlen=0;
    CHECK("r3_req_empty", r3_dispatch_req(100,NULL,0,resp,&rlen,16) == ESP_ERR_NOT_FOUND);
    CHECK("r3_evt_empty", r3_send_event(300,NULL,0) == ESP_ERR_NOT_FOUND);
}
static void test_r3_evt(void) {
    r3_reset();
    static eh_rpc_evt_node_t n; memset(&n,0,sizeof(n));
    n.event_id_min=300; n.event_id_max=400; n.serialise=echo_evt_serialise;
    r3_evt_register(&n);
    s_last_event=0; r3_send_event(300,NULL,0); CHECK("r3_evt_min_id", s_last_event==300);
    s_last_event=0; r3_send_event(400,NULL,0); CHECK("r3_evt_max_id", s_last_event==400);
    CHECK("r3_evt_miss",  r3_send_event(299,NULL,0) == ESP_ERR_NOT_FOUND);
    CHECK("r3_evt_miss2", r3_send_event(401,NULL,0) == ESP_ERR_NOT_FOUND);
}
static void test_r3_evt_overlap(void) {
    r3_reset();
    static eh_rpc_evt_node_t n1,n2;
    memset(&n1,0,sizeof(n1)); n1.event_id_min=300; n1.event_id_max=400; n1.serialise=echo_evt_serialise;
    memset(&n2,0,sizeof(n2)); n2.event_id_min=350; n2.event_id_max=450; n2.serialise=echo_evt_serialise;
    CHECK("r3_evt_ovlp_first",  r3_evt_register(&n1) == ESP_OK);
    CHECK("r3_evt_ovlp_second", r3_evt_register(&n2) == ESP_ERR_INVALID_STATE);
}

/* ══════════════════════════════════════════════════════════════════════════
 * FG / MCU coexistence tests
 * ══════════════════════════════════════════════════════════════════════════ */
static eh_rpc_req_node_t s_fg_req, s_mcu_req;
static eh_rpc_evt_node_t s_fg_evt, s_mcu_evt;

static void setup_fg_mcu(void)
{
    r3_reset();
    memset(&s_fg_req,  0, sizeof(s_fg_req));
    s_fg_req.msg_id_min = FG_REQ_MIN;  s_fg_req.msg_id_max = FG_REQ_MAX;
    s_fg_req.handler    = echo_req_handler;

    memset(&s_mcu_req, 0, sizeof(s_mcu_req));
    s_mcu_req.msg_id_min = MCU_REQ_MIN; s_mcu_req.msg_id_max = MCU_REQ_MAX;
    s_mcu_req.handler    = echo_req_handler;

    memset(&s_fg_evt,  0, sizeof(s_fg_evt));
    s_fg_evt.event_id_min = FG_EVT_MIN;  s_fg_evt.event_id_max = FG_EVT_MAX;
    s_fg_evt.serialise    = echo_evt_serialise;

    memset(&s_mcu_evt, 0, sizeof(s_mcu_evt));
    s_mcu_evt.event_id_min = MCU_EVT_MIN; s_mcu_evt.event_id_max = MCU_EVT_MAX;
    s_mcu_evt.serialise    = echo_evt_serialise;
}

static void test_fg_mcu_no_overlap(void) {
    CHECK("coexist_req_clear", !r3_ranges_overlap(FG_REQ_MIN,FG_REQ_MAX,MCU_REQ_MIN,MCU_REQ_MAX));
    CHECK("coexist_evt_clear", !r3_ranges_overlap(FG_EVT_MIN,FG_EVT_MAX,MCU_EVT_MIN,MCU_EVT_MAX));
}

static void test_fg_mcu_both_register(void) {
    setup_fg_mcu();
    CHECK("coexist_fg_req_reg",  r3_req_register(&s_fg_req)  == ESP_OK);
    CHECK("coexist_mcu_req_reg", r3_req_register(&s_mcu_req) == ESP_OK);
    CHECK("coexist_fg_evt_reg",  r3_evt_register(&s_fg_evt)  == ESP_OK);
    CHECK("coexist_mcu_evt_reg", r3_evt_register(&s_mcu_evt) == ESP_OK);
}

static void test_fg_mcu_req_boundaries(void) {
    setup_fg_mcu();
    r3_req_register(&s_fg_req); r3_req_register(&s_mcu_req);
    uint8_t resp[16]; uint16_t rlen=0;

    CHECK("fg_first",     r3_dispatch_req(FG_REQ_MIN,   NULL,0,resp,&rlen,16) == ESP_OK);
    CHECK("fg_last",      r3_dispatch_req(FG_REQ_MAX,   NULL,0,resp,&rlen,16) == ESP_OK);
    CHECK("fg_before",    r3_dispatch_req(FG_REQ_MIN-1, NULL,0,resp,&rlen,16) == ESP_ERR_NOT_FOUND);
    CHECK("fg_after",     r3_dispatch_req(FG_REQ_MAX+1, NULL,0,resp,&rlen,16) == ESP_ERR_NOT_FOUND);
    CHECK("gap_between",  r3_dispatch_req(200,          NULL,0,resp,&rlen,16) == ESP_ERR_NOT_FOUND);
    CHECK("mcu_first",    r3_dispatch_req(MCU_REQ_MIN,  NULL,0,resp,&rlen,16) == ESP_OK);
    CHECK("mcu_last",     r3_dispatch_req(MCU_REQ_MAX,  NULL,0,resp,&rlen,16) == ESP_OK);
    CHECK("mcu_before",   r3_dispatch_req(MCU_REQ_MIN-1,NULL,0,resp,&rlen,16) == ESP_ERR_NOT_FOUND);
    CHECK("mcu_after",    r3_dispatch_req(MCU_REQ_MAX+1,NULL,0,resp,&rlen,16) == ESP_ERR_NOT_FOUND);
}

static void test_fg_mcu_evt_boundaries(void) {
    setup_fg_mcu();
    r3_evt_register(&s_fg_evt); r3_evt_register(&s_mcu_evt);

    CHECK("fg_evt_first",    r3_send_event(FG_EVT_MIN,   NULL,0) == ESP_OK);
    CHECK("fg_evt_last",     r3_send_event(FG_EVT_MAX,   NULL,0) == ESP_OK);
    CHECK("fg_evt_before",   r3_send_event(FG_EVT_MIN-1, NULL,0) == ESP_ERR_NOT_FOUND);
    CHECK("fg_evt_after",    r3_send_event(FG_EVT_MAX+1, NULL,0) == ESP_ERR_NOT_FOUND);
    CHECK("mcu_evt_first",   r3_send_event(MCU_EVT_MIN,  NULL,0) == ESP_OK);
    CHECK("mcu_evt_last",    r3_send_event(MCU_EVT_MAX,  NULL,0) == ESP_OK);
    CHECK("mcu_evt_before",  r3_send_event(MCU_EVT_MIN-1,NULL,0) == ESP_ERR_NOT_FOUND);
    CHECK("mcu_evt_after",   r3_send_event(MCU_EVT_MAX+1,NULL,0) == ESP_ERR_NOT_FOUND);
}

static void test_fg_mcu_echo_correct(void) {
    setup_fg_mcu();
    r3_req_register(&s_fg_req); r3_req_register(&s_mcu_req);
    uint8_t resp[16]; uint16_t rlen=0;
    uint32_t fg_id=115, mcu_id=0x150;

    r3_dispatch_req(fg_id, NULL, 0, resp, &rlen, 16);
    uint32_t got_fg = (uint32_t)(resp[0]|(resp[1]<<8)|(resp[2]<<16)|(resp[3]<<24));
    CHECK("fg_echo_id", got_fg == fg_id);

    r3_dispatch_req(mcu_id, NULL, 0, resp, &rlen, 16);
    uint32_t got_mcu = (uint32_t)(resp[0]|(resp[1]<<8)|(resp[2]<<16)|(resp[3]<<24));
    CHECK("mcu_echo_id", got_mcu == mcu_id);
}

static void test_fg_mcu_dup_rejected(void) {
    setup_fg_mcu();
    r3_req_register(&s_fg_req); r3_req_register(&s_mcu_req);
    static eh_rpc_req_node_t d1,d2;
    memset(&d1,0,sizeof(d1)); d1.msg_id_min=FG_REQ_MIN;  d1.msg_id_max=FG_REQ_MAX;  d1.handler=echo_req_handler;
    memset(&d2,0,sizeof(d2)); d2.msg_id_min=MCU_REQ_MIN; d2.msg_id_max=MCU_REQ_MAX; d2.handler=echo_req_handler;
    CHECK("fg_dup_rejected",  r3_req_register(&d1) == ESP_ERR_INVALID_STATE);
    CHECK("mcu_dup_rejected", r3_req_register(&d2) == ESP_ERR_INVALID_STATE);
}

static void test_fg_mcu_sentinels(void) {
    setup_fg_mcu();
    r3_req_register(&s_fg_req); r3_req_register(&s_mcu_req);
    r3_evt_register(&s_fg_evt); r3_evt_register(&s_mcu_evt);
    uint8_t resp[16]; uint16_t rlen=0;
    /* Base/Max sentinels are outside [min,max] by design (+1/-1 in init) */
    CHECK("fg_base_sentinel",      r3_dispatch_req(100,    NULL,0,resp,&rlen,16) == ESP_ERR_NOT_FOUND);
    CHECK("fg_max_sentinel",       r3_dispatch_req(129,    NULL,0,resp,&rlen,16) == ESP_ERR_NOT_FOUND);
    CHECK("mcu_base_sentinel",     r3_dispatch_req(0x100u, NULL,0,resp,&rlen,16) == ESP_ERR_NOT_FOUND);
    CHECK("mcu_max_sentinel",      r3_dispatch_req(0x184u, NULL,0,resp,&rlen,16) == ESP_ERR_NOT_FOUND);
    CHECK("fg_evt_base_sentinel",  r3_send_event(300,    NULL,0) == ESP_ERR_NOT_FOUND);
    CHECK("fg_evt_max_sentinel",   r3_send_event(309,    NULL,0) == ESP_ERR_NOT_FOUND);
    CHECK("mcu_evt_base_sentinel", r3_send_event(0x300u, NULL,0) == ESP_ERR_NOT_FOUND);
    CHECK("mcu_evt_max_sentinel",  r3_send_event(0x315u, NULL,0) == ESP_ERR_NOT_FOUND);
}

static void test_fg_mcu_sorted_regardless_of_insert_order(void) {
    setup_fg_mcu();
    /* Insert MCU first (larger range), FG second — list must still be ascending */
    r3_req_register(&s_mcu_req);
    r3_req_register(&s_fg_req);
    eh_rpc_req_node_t *n; uint16_t prev=0; bool sorted=true;
    SLIST_FOREACH(n, &g_rpc_req_list, next) {
        if (n->msg_id_min < prev) { sorted=false; break; }
        prev = n->msg_id_min;
    }
    CHECK("fg_mcu_sorted", sorted);
}

static void test_fg_mcu_event_correct_id(void) {
    setup_fg_mcu();
    r3_evt_register(&s_fg_evt); r3_evt_register(&s_mcu_evt);
    s_last_event=0; r3_send_event(305, NULL, 0); CHECK("fg_evt_echo",  s_last_event==305);
    s_last_event=0; r3_send_event(0x310u, NULL, 0); CHECK("mcu_evt_echo", s_last_event==0x310u);
}

/* ══════════════════════════════════════════════════════════════════════════
 * Proto scanner tests
 * ══════════════════════════════════════════════════════════════════════════ */
static uint8_t make_tag(uint8_t fn) { return (uint8_t)((fn << 3) | 0); }
static uint8_t encode_varint(uint8_t *buf, uint32_t v) {
    uint8_t n=0;
    do { buf[n]=(uint8_t)(v&0x7Fu); v>>=7; if(v) buf[n]|=0x80u; n++; } while(v);
    return n;
}
static uint8_t build_f1_f2(uint8_t *buf, uint32_t f1, uint32_t f2) {
    uint8_t pos=0;
    buf[pos++]=make_tag(1); pos+=encode_varint(buf+pos,f1);
    buf[pos++]=make_tag(2); pos+=encode_varint(buf+pos,f2);
    return pos;
}

static void test_proto_basic(void) {
    uint8_t buf[32]; uint32_t mid=0;
    uint8_t len=build_f1_f2(buf,42,123);
    CHECK("proto_found", proto_extract_msg_id(buf,len,&mid)==2);
    CHECK("proto_value", mid==123);
}
static void test_proto_f2_first(void) {
    uint8_t buf[32]; uint32_t mid=0; uint8_t pos=0;
    buf[pos++]=make_tag(2); pos+=encode_varint(buf+pos,999u);
    buf[pos++]=make_tag(1); pos+=encode_varint(buf+pos,1u);
    CHECK("proto_f2_first", proto_extract_msg_id(buf,pos,&mid)==2);
    CHECK("proto_f2_val",   mid==999);
}
static void test_proto_mcu_id(void) {
    uint8_t buf[32]; uint32_t mid=0;
    uint8_t len=build_f1_f2(buf,1,MCU_REQ_MIN);
    CHECK("proto_mcu_found", proto_extract_msg_id(buf,len,&mid)==2);
    CHECK("proto_mcu_val",   mid==MCU_REQ_MIN);
}
static void test_proto_fg_id(void) {
    uint8_t buf[32]; uint32_t mid=0;
    uint8_t len=build_f1_f2(buf,1,FG_REQ_MIN);
    CHECK("proto_fg_found", proto_extract_msg_id(buf,len,&mid)==2);
    CHECK("proto_fg_val",   mid==FG_REQ_MIN);
}
static void test_proto_no_f2(void) {
    uint8_t buf[16]; uint32_t mid=0; uint8_t pos=0;
    buf[pos++]=make_tag(1); pos+=encode_varint(buf+pos,77u);
    CHECK("proto_no_f2", proto_extract_msg_id(buf,pos,&mid)==0);
}
static void test_proto_null(void) {
    uint8_t buf[8]; uint32_t mid=0;
    CHECK("proto_null_buf", proto_extract_msg_id(NULL,8,&mid)==0);
    CHECK("proto_null_out", proto_extract_msg_id(buf,8,NULL)==0);
    CHECK("proto_zero_len", proto_extract_msg_id(buf,0,&mid)==0);
}
static void test_proto_skip_len_delim(void) {
    uint8_t buf[64]; uint32_t mid=0; uint8_t pos=0;
    buf[pos++]=(uint8_t)((1<<3)|2);
    const char *pl="hello"; uint8_t plen=(uint8_t)strlen(pl);
    pos+=encode_varint(buf+pos,plen); memcpy(buf+pos,pl,plen); pos+=plen;
    buf[pos++]=make_tag(2); pos+=encode_varint(buf+pos,0xABCDu);
    CHECK("proto_skip_ld_found", proto_extract_msg_id(buf,pos,&mid)==2);
    CHECK("proto_skip_ld_val",   mid==0xABCDu);
}
static void test_proto_truncated(void) {
    uint8_t buf[4]={make_tag(2),0x80,0x80,0x80}; uint32_t mid=0;
    CHECK("proto_truncated", proto_extract_msg_id(buf,4,&mid)==0);
}

/* ══════════════════════════════════════════════════════════════════════════
 * End-to-end: proto scanner → SLIST dispatch
 * ══════════════════════════════════════════════════════════════════════════ */
static void test_e2e_fg(void) {
    setup_fg_mcu();
    r3_req_register(&s_fg_req); r3_req_register(&s_mcu_req);
    uint8_t wire[32]; uint8_t resp[16]; uint16_t rlen=0;
    uint32_t target=115;
    uint8_t wlen=build_f1_f2(wire,0,target);
    uint32_t mid=0; uint32_t field=proto_extract_msg_id(wire,wlen,&mid);
    CHECK("e2e_fg_field",   field==2);
    CHECK("e2e_fg_mid",     mid==target);
    CHECK("e2e_fg_dispatch",r3_dispatch_req(mid,wire,wlen,resp,&rlen,16)==ESP_OK);
    uint32_t echo=(uint32_t)(resp[0]|(resp[1]<<8)|(resp[2]<<16)|(resp[3]<<24));
    CHECK("e2e_fg_echo",    echo==target);
}
static void test_e2e_mcu(void) {
    setup_fg_mcu();
    r3_req_register(&s_fg_req); r3_req_register(&s_mcu_req);
    uint8_t wire[32]; uint8_t resp[16]; uint16_t rlen=0;
    uint32_t target=0x150;
    uint8_t wlen=build_f1_f2(wire,0,target);
    uint32_t mid=0; uint32_t field=proto_extract_msg_id(wire,wlen,&mid);
    CHECK("e2e_mcu_field",   field==2);
    CHECK("e2e_mcu_mid",     mid==target);
    CHECK("e2e_mcu_dispatch",r3_dispatch_req(mid,wire,wlen,resp,&rlen,16)==ESP_OK);
    uint32_t echo=(uint32_t)(resp[0]|(resp[1]<<8)|(resp[2]<<16)|(resp[3]<<24));
    CHECK("e2e_mcu_echo",    echo==target);
}
static void test_e2e_gap(void) {
    setup_fg_mcu();
    r3_req_register(&s_fg_req); r3_req_register(&s_mcu_req);
    uint8_t wire[32]; uint8_t resp[16]; uint16_t rlen=0;
    uint32_t target=500; /* in gap */
    uint8_t wlen=build_f1_f2(wire,0,target);
    uint32_t mid=0; proto_extract_msg_id(wire,wlen,&mid);
    CHECK("e2e_gap_nf", r3_dispatch_req(mid,wire,wlen,resp,&rlen,16)==ESP_ERR_NOT_FOUND);
}

/* ══════════════════════════════════════════════════════════════════════════
 * main
 * ══════════════════════════════════════════════════════════════════════════ */
int main(void)
{
    printf("eh_cp — Three-Registry + FG/MCU coexistence + Proto scanner\n\n");

    printf("── Registry 1: Interface table ──────────────────────────────────\n");
    test_r1_null_cb();
    test_r1_oob();
    test_r1_dispatch();
    test_r1_no_handler();
    test_r1_error_prop();
    test_r1_exclusive();
    test_r1_sta_rereg();
    test_r1_tx_clear();

    printf("── Registry 2: Capability accumulator ───────────────────────────\n");
    test_r2_accumulate();
    test_r2_feat();
    test_r2_oob();
    test_r2_idempotent();
    test_r2_fg_mcu_coexist();

    printf("── Registry 3: SLIST RPC dispatch ───────────────────────────────\n");
    test_r3_null();
    test_r3_null_handler();
    test_r3_inverted();
    test_r3_single_dispatch();
    test_r3_overlap();
    test_r3_sorted_insert();
    test_r3_echo_resp();
    test_r3_handler_fail();
    test_r3_empty();
    test_r3_evt();
    test_r3_evt_overlap();

    printf("── FG / MCU coexistence ─────────────────────────────────────────\n");
    test_fg_mcu_no_overlap();
    test_fg_mcu_both_register();
    test_fg_mcu_req_boundaries();
    test_fg_mcu_evt_boundaries();
    test_fg_mcu_echo_correct();
    test_fg_mcu_dup_rejected();
    test_fg_mcu_sentinels();
    test_fg_mcu_sorted_regardless_of_insert_order();
    test_fg_mcu_event_correct_id();

    printf("── Proto field-2 scanner ────────────────────────────────────────\n");
    test_proto_basic();
    test_proto_f2_first();
    test_proto_mcu_id();
    test_proto_fg_id();
    test_proto_no_f2();
    test_proto_null();
    test_proto_skip_len_delim();
    test_proto_truncated();

    printf("── End-to-end: wire → scanner → dispatch ────────────────────────\n");
    test_e2e_fg();
    test_e2e_mcu();
    test_e2e_gap();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
