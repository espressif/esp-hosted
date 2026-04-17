/*
 * registry_test.c — Host-compiled unit tests for Registry 3 (realloc table)
 *
 * Tests the flat realloc-table RPC dispatch introduced in Decision 9 (2026-03-12).
 * No IDF, no FreeRTOS — all stubs provided inline.
 *
 * Compile & run (from repo root):
 *   gcc -std=c11 -Wall -Wextra -Wshadow -g \
 *       -I components/coprocessor/eh_cp_core/include \
 *       -o /tmp/registry_test \
 *       components/coprocessor/extensions/test/registry_test.c
 *   /tmp/registry_test
 *
 * Coverage:
 *   A. Register req range — normal, overlap, NULL handler, bad range
 *   B. Unregister req range — exact match, not found
 *   C. Register evt range — normal, overlap, NULL serialise
 *   D. Unregister evt range — exact match, not found
 *   E. Dispatch req — exact hit, binary search (multiple ranges), miss
 *   F. Dispatch req — boundary: first and last id in range
 *   G. Send event — hit, miss
 *   H. Table growth — register > GROW_STEP ranges, verify all dispatch
 *   I. Sort invariant — register out of order, verify sorted dispatch
 *   J. Re-register after unregister (hot-reload pattern)
 *   K. Proto field-2 scanner — varint, multi-field, field absent, truncated
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <inttypes.h>

/* ── Minimal stubs so registry code compiles standalone ─────────────────── */

typedef int esp_err_t;
#define ESP_OK               0
#define ESP_FAIL            -1
#define ESP_ERR_INVALID_ARG  0x102
#define ESP_ERR_INVALID_STATE 0x103
#define ESP_ERR_NO_MEM       0x101
#define ESP_ERR_NOT_FOUND    0x105

#define ESP_LOGE(t,f,...) ((void)0)
#define ESP_LOGI(t,f,...) ((void)0)
#define ESP_LOGD(t,f,...) ((void)0)
#define ESP_LOGW(t,f,...) ((void)0)
#define esp_err_to_name(e) "err"
#ifndef PRIx32
#define PRIx32 "x"
#endif

/* FreeRTOS stubs */
typedef struct { int dummy; } SemaphoreHandle_t;
typedef struct { int dummy; } portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED {0}
#define portMAX_DELAY 0xFFFFFFFFu
static SemaphoreHandle_t xSemaphoreCreateMutex(void) { SemaphoreHandle_t h = {1}; return h; }
static void xSemaphoreTake(SemaphoreHandle_t h, unsigned d) { (void)h; (void)d; }
static void xSemaphoreGive(SemaphoreHandle_t h) { (void)h; }
#define taskENTER_CRITICAL(x) ((void)(x))
#define taskEXIT_CRITICAL(x)  ((void)(x))

/* eh_caps.h stubs */
typedef enum { ESP_STA_IF=0, ESP_AP_IF=1, ESP_IF_TYPE_MAX=8 } eh_if_type_t;
#define EH_FEAT_CAPS_COUNT 8

/* eh_cp_rpc_ll.h stub */
#define MAX_SERIAL_DATA_SIZE 4096
static int s_protocomm_event_id = -1;
static int eh_cp_protocomm_process_rpc_evt(const char *ep,
                                                    int event_id,
                                                    const void *buf, int len)
{
    (void)ep; (void)buf; (void)len;
    s_protocomm_event_id = event_id;
    return ESP_OK;
}
#define RPC_EP_NAME_EVT "RPCEvt"

/* eh_log.h stub */
/* (nothing needed) */

/* ── Pull in the implementation directly ─────────────────────────────────── */
/* We define only what the .c needs that isn't provided above. */

/* Fake the iface table (Registry 1) and caps (Registry 2) so registries.c
 * compiles — we only exercise Registry 3 in these tests. */
typedef esp_err_t (*hosted_rx_cb_t)(void *, void *, uint16_t, void *);
typedef int       (*hosted_tx_cb_t)(void *, void *, uint16_t);
typedef struct { hosted_rx_cb_t rx; hosted_tx_cb_t tx; void *ctx; } eh_iface_entry_t;
eh_iface_entry_t g_iface_table[8];

/* Callback typedefs (mirrors eh_cp_core.h) */
typedef struct {
    uint32_t        msg_id;
    const uint8_t  *req_buf;
    uint16_t        req_len;
    uint8_t       **out_buf;
    uint16_t       *out_len;
} eh_rpc_req_params_t;
typedef esp_err_t (*eh_rpc_req_handler_t)(void *, const eh_rpc_req_params_t *);
typedef esp_err_t (*eh_rpc_evt_serialise_t)(void *, uint32_t,
        const void *, uint16_t, uint8_t **, uint16_t *);

/* ── Registry 3 implementation (copy-compiled inline) ───────────────────── */

#define EH_CP_RPC_TABLE_GROW_STEP 4u

typedef struct {
    uint16_t             id_min;
    uint16_t             id_max;
    eh_rpc_req_handler_t handler;
    void                *ctx;
} rpc_req_entry_t;

typedef struct {
    uint16_t              id_min;
    uint16_t              id_max;
    eh_rpc_evt_serialise_t serialise;
    void                 *ctx;
} rpc_evt_entry_t;

static rpc_req_entry_t *s_req_table = NULL;
static size_t           s_req_count = 0;
static size_t           s_req_cap   = 0;
static rpc_evt_entry_t *s_evt_table = NULL;
static size_t           s_evt_count = 0;
static size_t           s_evt_cap   = 0;

static SemaphoreHandle_t g_rpc_registry_mutex;

static void reg_lock(void)   { xSemaphoreTake(g_rpc_registry_mutex, portMAX_DELAY); }
static void reg_unlock(void) { xSemaphoreGive(g_rpc_registry_mutex); }

static bool ranges_overlap(uint16_t a0, uint16_t a1, uint16_t b0, uint16_t b1)
{ return (a0 <= b1) && (b0 <= a1); }

static esp_err_t table_grow(void **tp, size_t *cap, size_t count, size_t esz)
{
    if (count < *cap) return ESP_OK;
    size_t nc = *cap + EH_CP_RPC_TABLE_GROW_STEP;
    void *p = realloc(*tp, nc * esz);
    if (!p) return ESP_ERR_NO_MEM;
    *tp = p; *cap = nc; return ESP_OK;
}

static esp_err_t rpc_req_register(uint16_t mn, uint16_t mx,
                                   eh_rpc_req_handler_t h, void *ctx)
{
    if (!h)      return ESP_ERR_INVALID_ARG;
    if (mn > mx) return ESP_ERR_INVALID_ARG;
    reg_lock();
    for (size_t i = 0; i < s_req_count; i++)
        if (ranges_overlap(mn, mx, s_req_table[i].id_min, s_req_table[i].id_max))
            { reg_unlock(); return ESP_ERR_INVALID_STATE; }
    esp_err_t r = table_grow((void **)&s_req_table, &s_req_cap,
                               s_req_count, sizeof(rpc_req_entry_t));
    if (r) { reg_unlock(); return r; }
    size_t ins = s_req_count;
    while (ins > 0 && s_req_table[ins-1].id_min > mn) {
        s_req_table[ins] = s_req_table[ins-1]; ins--;
    }
    s_req_table[ins] = (rpc_req_entry_t){mn, mx, h, ctx};
    s_req_count++;
    reg_unlock();
    return ESP_OK;
}

static esp_err_t rpc_req_unregister(uint16_t mn, uint16_t mx)
{
    reg_lock();
    for (size_t i = 0; i < s_req_count; i++) {
        if (s_req_table[i].id_min == mn && s_req_table[i].id_max == mx) {
            memmove(&s_req_table[i], &s_req_table[i+1],
                    (s_req_count - i - 1) * sizeof(rpc_req_entry_t));
            s_req_count--;
            reg_unlock();
            return ESP_OK;
        }
    }
    reg_unlock();
    return ESP_ERR_NOT_FOUND;
}

static esp_err_t rpc_evt_register(uint16_t mn, uint16_t mx,
                                   eh_rpc_evt_serialise_t s, void *ctx)
{
    if (!s)      return ESP_ERR_INVALID_ARG;
    if (mn > mx) return ESP_ERR_INVALID_ARG;
    reg_lock();
    for (size_t i = 0; i < s_evt_count; i++)
        if (ranges_overlap(mn, mx, s_evt_table[i].id_min, s_evt_table[i].id_max))
            { reg_unlock(); return ESP_ERR_INVALID_STATE; }
    esp_err_t r = table_grow((void **)&s_evt_table, &s_evt_cap,
                               s_evt_count, sizeof(rpc_evt_entry_t));
    if (r) { reg_unlock(); return r; }
    s_evt_table[s_evt_count++] = (rpc_evt_entry_t){mn, mx, s, ctx};
    reg_unlock();
    return ESP_OK;
}

static esp_err_t rpc_evt_unregister(uint16_t mn, uint16_t mx)
{
    reg_lock();
    for (size_t i = 0; i < s_evt_count; i++) {
        if (s_evt_table[i].id_min == mn && s_evt_table[i].id_max == mx) {
            memmove(&s_evt_table[i], &s_evt_table[i+1],
                    (s_evt_count - i - 1) * sizeof(rpc_evt_entry_t));
            s_evt_count--;
            reg_unlock();
            return ESP_OK;
        }
    }
    reg_unlock();
    return ESP_ERR_NOT_FOUND;
}

static esp_err_t rpc_dispatch_req(uint32_t msg_id,
                                   const void *req_buf, uint16_t req_len,
                                   uint8_t **out_buf, uint16_t *out_len)
{
    reg_lock();
    size_t lo = 0, hi = s_req_count;
    while (lo < hi) {
        size_t mid = lo + (hi - lo) / 2;
        if ((uint32_t)s_req_table[mid].id_max < msg_id)       lo = mid + 1;
        else if ((uint32_t)s_req_table[mid].id_min > msg_id)  hi = mid;
        else {
            eh_rpc_req_handler_t fn = s_req_table[mid].handler;
            void *ctx = s_req_table[mid].ctx;
            reg_unlock();
            eh_rpc_req_params_t p = {
                .msg_id  = msg_id,
                .req_buf = (const uint8_t *)req_buf,
                .req_len = req_len,
                .out_buf = out_buf,
                .out_len = out_len,
            };
            return fn(ctx, &p);
        }
    }
    reg_unlock();
    return ESP_ERR_NOT_FOUND;
}

static esp_err_t rpc_dispatch_req_noresp(uint32_t msg_id)
{
    uint8_t *out = NULL;
    uint16_t out_len = 0;
    esp_err_t r = rpc_dispatch_req(msg_id, NULL, 0, &out, &out_len);
    if (out) {
        free(out);
    }
    return r;
}

static esp_err_t rpc_send_event(uint32_t event_id, const void *data, uint16_t len)
{
    reg_lock();
    for (size_t i = 0; i < s_evt_count; i++) {
        if (event_id < (uint32_t)s_evt_table[i].id_min) continue;
        if (event_id > (uint32_t)s_evt_table[i].id_max) continue;
        eh_rpc_evt_serialise_t fn = s_evt_table[i].serialise;
        void *ctx = s_evt_table[i].ctx;
        reg_unlock();
        uint8_t *out = NULL;
        uint16_t out_len = 0;
        esp_err_t r = fn(ctx, event_id, data, len, &out, &out_len);
        if (r == ESP_OK)
            r = eh_cp_protocomm_process_rpc_evt(RPC_EP_NAME_EVT,
                                                        (int)event_id, out, (int)out_len);
        if (out) free(out);
        return r;
    }
    reg_unlock();
    return ESP_ERR_NOT_FOUND;
}

/* proto field-2 scanner (verbatim from registries.c) */
static uint32_t proto_extract_msg_id(const uint8_t *buf, uint16_t len, uint32_t *out)
{
    if (!buf || !out || !len) return 0;
    uint16_t i = 0;
    while (i < len) {
        uint8_t tag = buf[i++];
        uint8_t fn  = (uint8_t)(tag >> 3);
        uint8_t wt  = (uint8_t)(tag & 7);
        if (fn == 2 && wt == 0) {
            uint32_t v = 0; uint8_t sh = 0;
            while (i < len && sh < 35) {
                uint8_t b = buf[i++];
                v |= (uint32_t)(b & 0x7f) << sh;
                if (!(b & 0x80)) { *out = v; return 2; }
                sh = (uint8_t)(sh + 7);
            }
            return 0;
        }
        switch (wt) {
        case 0: while (i < len && (buf[i]&0x80)) i++; if (i<len) i++; break;
        case 1: i=(uint16_t)(i+8); break;
        case 2: { uint32_t sl=0; uint8_t sh=0;
                  while(i<len&&sh<35){uint8_t b=buf[i++];sl|=(uint32_t)(b&0x7f)<<sh;if(!(b&0x80))break;sh=(uint8_t)(sh+7);}
                  i=(uint16_t)(i+sl); break; }
        case 5: i=(uint16_t)(i+4); break;
        default: return 0;
        }
    }
    return 0;
}

/* ── Test framework ──────────────────────────────────────────────────────── */

static int g_pass = 0, g_fail = 0;
#define PASS(n)       do { printf("  PASS  %s\n",(n)); g_pass++; } while(0)
#define FAIL(n,msg)   do { printf("  FAIL  %s  (%s)\n",(n),(msg)); g_fail++; } while(0)
#define CHECK(n,c)    do { if(c) PASS(n); else FAIL(n,#c); } while(0)

/* Reset all state between test groups */
static void reset_tables(void)
{
    free(s_req_table); s_req_table = NULL; s_req_count = 0; s_req_cap = 0;
    free(s_evt_table); s_evt_table = NULL; s_evt_count = 0; s_evt_cap = 0;
    s_protocomm_event_id = -1;
    g_rpc_registry_mutex = xSemaphoreCreateMutex();
}

/* ── Handler stubs ───────────────────────────────────────────────────────── */

static uint32_t s_last_req_id = 0;
static uint32_t s_last_evt_id = 0;
static esp_err_t s_handler_ret = ESP_OK;

static esp_err_t req_handler_A(void *ctx, const eh_rpc_req_params_t *p)
{
    (void)ctx;
    s_last_req_id = p->msg_id;
    *p->out_len = 0;
    *p->out_buf = NULL;
    return s_handler_ret;
}

static esp_err_t req_handler_B(void *ctx, const eh_rpc_req_params_t *p)
{
    (void)ctx;
    s_last_req_id = p->msg_id | 0x10000u;
    *p->out_len = 0;
    *p->out_buf = NULL;
    return s_handler_ret;
}

static esp_err_t evt_serialise_A(void *ctx, uint32_t event_id,
        const void *d, uint16_t dl, uint8_t **ob, uint16_t *ol)
{
    (void)ctx;(void)d;(void)dl;
    s_last_evt_id = event_id; *ol = 0;
    *ob = NULL;
    return s_handler_ret;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * A. Register req range
 * ═══════════════════════════════════════════════════════════════════════════ */
static void test_A(void)
{
    reset_tables();

    CHECK("A1_reg_normal",      rpc_req_register(100, 199, req_handler_A, NULL) == ESP_OK);
    CHECK("A2_count_1",         s_req_count == 1);
    CHECK("A3_overlap_reject",  rpc_req_register(150, 250, req_handler_A, NULL) == ESP_ERR_INVALID_STATE);
    CHECK("A4_count_still_1",   s_req_count == 1);
    CHECK("A5_adjacent_ok",     rpc_req_register(200, 299, req_handler_B, NULL) == ESP_OK);
    CHECK("A6_count_2",         s_req_count == 2);
    CHECK("A7_null_handler",    rpc_req_register(400, 499, NULL, NULL) == ESP_ERR_INVALID_ARG);
    CHECK("A8_bad_range",       rpc_req_register(500, 400, req_handler_A, NULL) == ESP_ERR_INVALID_ARG);
    CHECK("A9_single_id_ok",    rpc_req_register(600, 600, req_handler_A, NULL) == ESP_OK);
    CHECK("A10_same_range_dup", rpc_req_register(100, 199, req_handler_A, NULL) == ESP_ERR_INVALID_STATE);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * B. Unregister req range
 * ═══════════════════════════════════════════════════════════════════════════ */
static void test_B(void)
{
    reset_tables();
    rpc_req_register(100, 199, req_handler_A, NULL);
    rpc_req_register(200, 299, req_handler_B, NULL);

    CHECK("B1_unreg_first",     rpc_req_unregister(100, 199) == ESP_OK);
    CHECK("B2_count_after",     s_req_count == 1);
    /* Remaining entry should still dispatch */
    s_last_req_id = 0;
    rpc_dispatch_req_noresp(250);
    CHECK("B3_remaining_works", (s_last_req_id & 0xFFFFu) == 250);
    CHECK("B4_not_found",       rpc_req_unregister(100, 199) == ESP_ERR_NOT_FOUND);
    CHECK("B5_unreg_second",    rpc_req_unregister(200, 299) == ESP_OK);
    CHECK("B6_empty",           s_req_count == 0);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * C. Register evt range
 * ═══════════════════════════════════════════════════════════════════════════ */
static void test_C(void)
{
    reset_tables();
    CHECK("C1_reg_normal",     rpc_evt_register(300, 399, evt_serialise_A, NULL) == ESP_OK);
    CHECK("C2_count_1",        s_evt_count == 1);
    CHECK("C3_overlap_reject", rpc_evt_register(350, 450, evt_serialise_A, NULL) == ESP_ERR_INVALID_STATE);
    CHECK("C4_null_serialise", rpc_evt_register(500, 599, NULL, NULL) == ESP_ERR_INVALID_ARG);
    CHECK("C5_bad_range",      rpc_evt_register(700, 600, evt_serialise_A, NULL) == ESP_ERR_INVALID_ARG);
    CHECK("C6_adjacent_ok",    rpc_evt_register(400, 499, evt_serialise_A, NULL) == ESP_OK);
    CHECK("C7_count_2",        s_evt_count == 2);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * D. Unregister evt range
 * ═══════════════════════════════════════════════════════════════════════════ */
static void test_D(void)
{
    reset_tables();
    rpc_evt_register(300, 399, evt_serialise_A, NULL);
    rpc_evt_register(400, 499, evt_serialise_A, NULL);

    CHECK("D1_unreg_ok",    rpc_evt_unregister(300, 399) == ESP_OK);
    CHECK("D2_count",       s_evt_count == 1);
    CHECK("D3_not_found",   rpc_evt_unregister(300, 399) == ESP_ERR_NOT_FOUND);
    CHECK("D4_unreg_last",  rpc_evt_unregister(400, 499) == ESP_OK);
    CHECK("D5_empty",       s_evt_count == 0);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * E. Dispatch req — hit, miss
 * ═══════════════════════════════════════════════════════════════════════════ */
static void test_E(void)
{
    reset_tables();
    rpc_req_register(100, 199, req_handler_A, NULL);
    rpc_req_register(300, 399, req_handler_B, NULL);

    s_last_req_id = 0;
    CHECK("E1_hit_A",        rpc_dispatch_req_noresp(150) == ESP_OK);
    CHECK("E2_called_A",     s_last_req_id == 150);

    s_last_req_id = 0;
    CHECK("E3_hit_B",        rpc_dispatch_req_noresp(350) == ESP_OK);
    CHECK("E4_called_B",     (s_last_req_id & 0xFFFFu) == 350);

    CHECK("E5_miss_between", rpc_dispatch_req_noresp(250) == ESP_ERR_NOT_FOUND);
    CHECK("E6_miss_before",  rpc_dispatch_req_noresp(50) == ESP_ERR_NOT_FOUND);
    CHECK("E7_miss_after",   rpc_dispatch_req_noresp(500) == ESP_ERR_NOT_FOUND);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * F. Dispatch req — boundary IDs (first and last in range)
 * ═══════════════════════════════════════════════════════════════════════════ */
static void test_F(void)
{
    reset_tables();
    rpc_req_register(200, 299, req_handler_A, NULL);

    s_last_req_id = 0;
    CHECK("F1_boundary_min",      rpc_dispatch_req_noresp(200) == ESP_OK);
    CHECK("F2_min_called",        s_last_req_id == 200);

    s_last_req_id = 0;
    CHECK("F3_boundary_max",      rpc_dispatch_req_noresp(299) == ESP_OK);
    CHECK("F4_max_called",        s_last_req_id == 299);

    CHECK("F5_just_before_range", rpc_dispatch_req_noresp(199) == ESP_ERR_NOT_FOUND);
    CHECK("F6_just_after_range",  rpc_dispatch_req_noresp(300) == ESP_ERR_NOT_FOUND);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * G. Send event — hit, miss
 * ═══════════════════════════════════════════════════════════════════════════ */
static void test_G(void)
{
    reset_tables();
    rpc_evt_register(500, 599, evt_serialise_A, NULL);

    s_last_evt_id = 0; s_protocomm_event_id = -1;
    CHECK("G1_hit",              rpc_send_event(550, NULL, 0) == ESP_OK);
    CHECK("G2_serialise_called", s_last_evt_id == 550);
    CHECK("G3_protocomm_called", s_protocomm_event_id == 550);

    CHECK("G4_miss",             rpc_send_event(400, NULL, 0) == ESP_ERR_NOT_FOUND);
    CHECK("G5_miss_after",       rpc_send_event(600, NULL, 0) == ESP_ERR_NOT_FOUND);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * H. Table growth: register > GROW_STEP (4) ranges, verify all dispatch
 * ═══════════════════════════════════════════════════════════════════════════ */
static void test_H(void)
{
    reset_tables();
    /* Register 9 non-overlapping single-ID ranges */
    for (int i = 0; i < 9; i++) {
        uint16_t id = (uint16_t)(1000 + i * 10);
        CHECK("H_reg", rpc_req_register(id, id, req_handler_A, NULL) == ESP_OK);
    }
    CHECK("H_count", s_req_count == 9);
    CHECK("H_cap_grew", s_req_cap >= 9);  /* should have grown at least twice */

    /* Verify every range dispatches */
    bool all_ok = true;
    for (int i = 0; i < 9; i++) {
        uint16_t id = (uint16_t)(1000 + i * 10);
        s_last_req_id = 0;
        esp_err_t r = rpc_dispatch_req_noresp(id);
        if (r != ESP_OK || s_last_req_id != id) all_ok = false;
    }
    CHECK("H_all_dispatch", all_ok);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * I. Sort invariant: register out-of-order, verify sorted dispatch works
 * ═══════════════════════════════════════════════════════════════════════════ */
static void test_I(void)
{
    reset_tables();
    /* Insert in reverse order */
    rpc_req_register(500, 599, req_handler_B, NULL);
    rpc_req_register(100, 199, req_handler_A, NULL);
    rpc_req_register(300, 399, req_handler_A, NULL);

    /* Table must be sorted: 100-199, 300-399, 500-599 */
    CHECK("I1_sorted_0", s_req_table[0].id_min == 100);
    CHECK("I2_sorted_1", s_req_table[1].id_min == 300);
    CHECK("I3_sorted_2", s_req_table[2].id_min == 500);

    /* Binary search must find all three */
    s_last_req_id = 0;
    rpc_dispatch_req_noresp(150);
    CHECK("I4_dispatch_first",  s_last_req_id == 150);

    s_last_req_id = 0;
    rpc_dispatch_req_noresp(350);
    CHECK("I5_dispatch_middle", s_last_req_id == 350);

    s_last_req_id = 0;
    rpc_dispatch_req_noresp(550);
    CHECK("I6_dispatch_last",   (s_last_req_id & 0xFFFFu) == 550);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * J. Re-register after unregister (hot-reload pattern)
 * ═══════════════════════════════════════════════════════════════════════════ */
static void test_J(void)
{
    reset_tables();
    rpc_req_register(100, 199, req_handler_A, NULL);

    CHECK("J1_unreg",    rpc_req_unregister(100, 199) == ESP_OK);
    CHECK("J2_miss",     rpc_dispatch_req_noresp(150) == ESP_ERR_NOT_FOUND);
    CHECK("J3_rereg",    rpc_req_register(100, 199, req_handler_B, NULL) == ESP_OK);

    s_last_req_id = 0;
    rpc_dispatch_req_noresp(150);
    CHECK("J4_hits_new", (s_last_req_id & 0xFFFFu) == 150);
    /* handler_B marks bit 16 — verify it was the new handler called */
    CHECK("J5_new_handler", (s_last_req_id >> 16) == 1);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * K. Proto field-2 scanner
 * ═══════════════════════════════════════════════════════════════════════════ */
static void test_K(void)
{
    uint32_t v = 0;

    /* Encode: field 1 (varint, value 1), field 2 (varint, value 0x42) */
    /* field tag for field_num=1, wire_type=0: (1<<3)|0 = 0x08          */
    /* field tag for field_num=2, wire_type=0: (2<<3)|0 = 0x10          */
    uint8_t buf1[] = { 0x08, 0x01,   /* field 1 = 1 */
                       0x10, 0x42 }; /* field 2 = 0x42 */
    v = 0;
    CHECK("K1_find_field2",  proto_extract_msg_id(buf1, sizeof(buf1), &v) == 2);
    CHECK("K2_value_correct", v == 0x42);

    /* Only field 1 — field 2 absent */
    uint8_t buf2[] = { 0x08, 0x05 };
    v = 99;
    CHECK("K3_field2_absent", proto_extract_msg_id(buf2, sizeof(buf2), &v) == 0);

    /* Field 2 only */
    uint8_t buf3[] = { 0x10, 0x7F };
    v = 0;
    CHECK("K4_field2_only",   proto_extract_msg_id(buf3, sizeof(buf3), &v) == 2);
    CHECK("K5_value_7f",      v == 0x7F);

    /* Multi-byte varint for field 2: value = 0x80 = 0x80 0x01 in varint */
    /* tag: 0x10, then 0x80 0x01 */
    uint8_t buf4[] = { 0x10, 0x80, 0x01 };
    v = 0;
    CHECK("K6_multibyte",     proto_extract_msg_id(buf4, sizeof(buf4), &v) == 2);
    CHECK("K7_value_0x80",    v == 0x80);

    /* Truncated varint — MSB set but no continuation byte */
    uint8_t buf5[] = { 0x10, 0x80 };
    v = 99;
    CHECK("K8_truncated",     proto_extract_msg_id(buf5, sizeof(buf5), &v) == 0);

    /* NULL / zero-length guards */
    CHECK("K9_null_buf",  proto_extract_msg_id(NULL, 4, &v) == 0);
    CHECK("K10_zero_len", proto_extract_msg_id(buf1, 0,  &v) == 0);
    CHECK("K11_null_out", proto_extract_msg_id(buf1, sizeof(buf1), NULL) == 0);

    /* Field 2 is a LEN type (wire_type=2), not varint — scanner must skip */
    /* tag for field 1 varint=0x08, then field 2 LEN tag=(2<<3)|2=0x12, len=1, data=0xAA */
    /* field 3 varint tag=(3<<3)|0=0x18, value=0x55                                      */
    uint8_t buf6[] = { 0x08, 0x01,          /* field 1 varint = 1 */
                       0x12, 0x01, 0xAA,    /* field 2 LEN, length=1 */
                       0x18, 0x55 };        /* field 3 varint = 0x55 */
    v = 0;
    /* field 2 exists but is wire_type=2 (LEN), not varint — scanner won't match */
    CHECK("K12_field2_len_type", proto_extract_msg_id(buf6, sizeof(buf6), &v) == 0);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * main
 * ═══════════════════════════════════════════════════════════════════════════ */
int main(void)
{
    printf("eh_cp — Registry 3 realloc-table unit tests (2026-03-12)\n\n");

    printf("── A. Register req range ─────────────────────────────────────────\n");
    test_A();
    printf("── B. Unregister req range ───────────────────────────────────────\n");
    test_B();
    printf("── C. Register evt range ─────────────────────────────────────────\n");
    test_C();
    printf("── D. Unregister evt range ───────────────────────────────────────\n");
    test_D();
    printf("── E. Dispatch req – hit and miss ────────────────────────────────\n");
    test_E();
    printf("── F. Dispatch req – boundary IDs ───────────────────────────────\n");
    test_F();
    printf("── G. Send event ─────────────────────────────────────────────────\n");
    test_G();
    printf("── H. Table growth (>4 ranges) ───────────────────────────────────\n");
    test_H();
    printf("── I. Sort invariant – out-of-order insert ───────────────────────\n");
    test_I();
    printf("── J. Hot-reload: unregister then re-register ────────────────────\n");
    test_J();
    printf("── K. Proto field-2 scanner ──────────────────────────────────────\n");
    test_K();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
