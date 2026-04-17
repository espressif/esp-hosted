/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 */

#include "eh_cp_master_config.h"
#if EH_CP_FEAT_RPC_READY
/*
 * eh_cp_feat_rpc_registries.c
 *
 * Implements Registry 3 — RPC request/event realloc-table dispatch.
 * Moved out of eh_cp_core per design spec 19.
 */

#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <inttypes.h>

#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "eh_cp_feat_rpc.h"
#include "eh_log.h"

static const char TAG[] = "ehcp_rpc_reg";

/* ─────────────────────────────────────────────────────────────────────────────
 * Registry 3 — RPC realloc-table dispatch
 *
 * Two flat arrays of inline entry structs.  No SLIST, no node pointers.
 * Both arrays start NULL and grow via realloc(+4 entries) on demand.
 * req table is kept sorted ascending by id_min (sorted insert + memmove shift).
 * evt table is unsorted (append only).
 * ─────────────────────────────────────────────────────────────────────────────*/

/* How many slots to add per realloc call. */
#define EH_CP_RPC_TABLE_GROW_STEP  4u

typedef struct {
    uint16_t             id_min;
    uint16_t             id_max;
    eh_rpc_req_handler_t handler;
    void                *ctx;
} rpc_req_entry_t;

typedef struct {
    uint16_t               id_min;
    uint16_t               id_max;
    eh_rpc_evt_serialise_t serialise;
    void                  *ctx;
} rpc_evt_entry_t;

static rpc_req_entry_t *s_req_table = NULL;
static size_t           s_req_count = 0;
static size_t           s_req_cap   = 0;

static rpc_evt_entry_t *s_evt_table = NULL;
static size_t           s_evt_count = 0;
static size_t           s_evt_cap   = 0;

static SemaphoreHandle_t g_rpc_registry_mutex = NULL;

esp_err_t eh_cp_rpc_registries_init(void)
{
    if (!g_rpc_registry_mutex) {
        g_rpc_registry_mutex = xSemaphoreCreateMutex();
        if (!g_rpc_registry_mutex) {
            ESP_LOGE(TAG, "Failed to create RPC registry mutex");
            return ESP_ERR_NO_MEM;
        }
    }
    return ESP_OK;
}

void eh_cp_rpc_registry_lock(void)
{
    if (g_rpc_registry_mutex) {
        xSemaphoreTake(g_rpc_registry_mutex, portMAX_DELAY);
    }
}

void eh_cp_rpc_registry_unlock(void)
{
    if (g_rpc_registry_mutex) {
        xSemaphoreGive(g_rpc_registry_mutex);
    }
}

/* ── Shared helpers ──────────────────────────────────────────────────────── */

static bool ranges_overlap(uint16_t a_min, uint16_t a_max,
                            uint16_t b_min, uint16_t b_max)
{
    return (a_min <= b_max) && (b_min <= a_max);
}

static esp_err_t table_grow(void **table_ptr, size_t *cap,
                             size_t count, size_t element_size)
{
    if (count < *cap) return ESP_OK;

    size_t new_cap = *cap + EH_CP_RPC_TABLE_GROW_STEP;
    void *p = realloc(*table_ptr, new_cap * element_size);
    if (!p) {
        ESP_LOGE(TAG, "table_grow: realloc(%zu slots × %zu B) failed",
                 new_cap, element_size);
        return ESP_ERR_NO_MEM;
    }
    *table_ptr = p;
    *cap       = new_cap;
    return ESP_OK;
}

/* ── Request table — register ────────────────────────────────────────────── */

esp_err_t eh_cp_rpc_req_register(uint16_t id_min, uint16_t id_max,
                                          eh_rpc_req_handler_t handler, void *ctx)
{
    if (!handler) {
        ESP_LOGE(TAG, "rpc_req_register: NULL handler");
        return ESP_ERR_INVALID_ARG;
    }
    if (id_min > id_max) {
        ESP_LOGE(TAG, "rpc_req_register: id_min 0x%04x > id_max 0x%04x", id_min, id_max);
        return ESP_ERR_INVALID_ARG;
    }

    eh_cp_rpc_registry_lock();

    for (size_t i = 0; i < s_req_count; i++) {
        if (ranges_overlap(id_min, id_max, s_req_table[i].id_min, s_req_table[i].id_max)) {
            ESP_LOGE(TAG, "rpc_req_register: [0x%04x,0x%04x] overlaps [0x%04x,0x%04x]",
                     id_min, id_max, s_req_table[i].id_min, s_req_table[i].id_max);
            eh_cp_rpc_registry_unlock();
            return ESP_ERR_INVALID_STATE;
        }
    }

    esp_err_t gr = table_grow((void **)&s_req_table, &s_req_cap,
                               s_req_count, sizeof(rpc_req_entry_t));
    if (gr != ESP_OK) { eh_cp_rpc_registry_unlock(); return gr; }

    size_t ins = s_req_count;
    while (ins > 0 && s_req_table[ins - 1].id_min > id_min) {
        s_req_table[ins] = s_req_table[ins - 1];
        ins--;
    }
    s_req_table[ins] = (rpc_req_entry_t){ id_min, id_max, handler, ctx };
    s_req_count++;

    ESP_LOGI(TAG, "++ RPC Req [%u]: [0x%04x->0x%04x] => %p (Total %zu)",
             (unsigned int)ins, id_min, id_max, handler, s_req_count);

    eh_cp_rpc_registry_unlock();
    return ESP_OK;
}

esp_err_t eh_cp_rpc_req_unregister(uint16_t id_min, uint16_t id_max)
{
    eh_cp_rpc_registry_lock();

    for (size_t i = 0; i < s_req_count; i++) {
        if (s_req_table[i].id_min == id_min && s_req_table[i].id_max == id_max) {
            memmove(&s_req_table[i], &s_req_table[i + 1],
                    (s_req_count - i - 1) * sizeof(rpc_req_entry_t));
            s_req_count--;
            eh_cp_rpc_registry_unlock();
            ESP_LOGD(TAG, "-- RPC Req [%u]: [0x%04x,0x%04x]", (unsigned int)i, id_min, id_max);
            return ESP_OK;
        }
    }

    eh_cp_rpc_registry_unlock();
    ESP_LOGW(TAG, "rpc_req_unregister: [0x%04x,0x%04x] not found", id_min, id_max);
    return ESP_ERR_NOT_FOUND;
}

/* ── Event table — register ──────────────────────────────────────────────── */

esp_err_t eh_cp_rpc_evt_register(uint16_t id_min, uint16_t id_max,
                                          eh_rpc_evt_serialise_t serialise, void *ctx)
{
    if (!serialise) {
        ESP_LOGE(TAG, "rpc_evt_register: NULL serialise");
        return ESP_ERR_INVALID_ARG;
    }
    if (id_min > id_max) {
        ESP_LOGE(TAG, "rpc_evt_register: id_min 0x%04x > id_max 0x%04x", id_min, id_max);
        return ESP_ERR_INVALID_ARG;
    }

    eh_cp_rpc_registry_lock();

    for (size_t i = 0; i < s_evt_count; i++) {
        if (ranges_overlap(id_min, id_max, s_evt_table[i].id_min, s_evt_table[i].id_max)) {
            ESP_LOGE(TAG, "rpc_evt_register: [0x%04x,0x%04x] overlaps existing",
                     id_min, id_max);
            eh_cp_rpc_registry_unlock();
            return ESP_ERR_INVALID_STATE;
        }
    }

    esp_err_t gr = table_grow((void **)&s_evt_table, &s_evt_cap,
                               s_evt_count, sizeof(rpc_evt_entry_t));
    if (gr != ESP_OK) { eh_cp_rpc_registry_unlock(); return gr; }

    s_evt_table[s_evt_count++] = (rpc_evt_entry_t){ id_min, id_max, serialise, ctx };

    ESP_LOGI(TAG, "++ RPC Evt [%u]: [0x%04x->0x%04x] => %p (Total %zu)",
             (unsigned int)s_evt_count, id_min, id_max, serialise, s_evt_count);

    eh_cp_rpc_registry_unlock();
    return ESP_OK;
}

esp_err_t eh_cp_rpc_evt_unregister(uint16_t id_min, uint16_t id_max)
{
    eh_cp_rpc_registry_lock();

    for (size_t i = 0; i < s_evt_count; i++) {
        if (s_evt_table[i].id_min == id_min && s_evt_table[i].id_max == id_max) {
            memmove(&s_evt_table[i], &s_evt_table[i + 1],
                    (s_evt_count - i - 1) * sizeof(rpc_evt_entry_t));
            s_evt_count--;
            eh_cp_rpc_registry_unlock();
            ESP_LOGD(TAG, "-- RPC Evt [%u]: [0x%04x,0x%04x]", (unsigned int)i, id_min, id_max);
            return ESP_OK;
        }
    }

    eh_cp_rpc_registry_unlock();
    ESP_LOGW(TAG, "rpc_evt_unregister: [0x%04x,0x%04x] not found", id_min, id_max);
    return ESP_ERR_NOT_FOUND;
}

/* ── Request dispatch — binary search on sorted table ───────────────────── */

esp_err_t eh_cp_rpc_dispatch_req(uint32_t msg_id,
                                         const void *req_buf, uint16_t req_len,
                                         uint8_t **out_buf, uint16_t *out_len)
{
    ESP_LOGD(TAG, "dispatch_req: enter msg_id=0x%04"PRIx32""
             " req_len=%u req_table_count=%zu",
             msg_id, req_len, s_req_count);

    eh_cp_rpc_registry_lock();

    size_t lo = 0, hi = s_req_count;
    while (lo < hi) {
        size_t mid = lo + (hi - lo) / 2;
        if ((uint32_t)s_req_table[mid].id_max < msg_id) {
            lo = mid + 1;
        } else if ((uint32_t)s_req_table[mid].id_min > msg_id) {
            hi = mid;
        } else {
            eh_rpc_req_handler_t handler = s_req_table[mid].handler;
            void *ctx = s_req_table[mid].ctx;
            eh_cp_rpc_registry_unlock();
            const eh_rpc_req_params_t params = {
                .msg_id  = msg_id,
                .req_buf = (const uint8_t *)req_buf,
                .req_len = req_len,
                .out_buf = out_buf,
                .out_len = out_len,
            };
            return handler(ctx, &params);
        }
    }

    eh_cp_rpc_registry_unlock();
    ESP_LOGW(TAG, "dispatch_req: no handler for msg_id 0x%04"PRIx32, msg_id);
    return ESP_ERR_NOT_FOUND;
}

/* ── Event send — linear scan ──────────────────────────────────────────── */

esp_err_t eh_cp_rpc_send_event(uint32_t event_id,
                                       const void *data, uint16_t len)
{
    ESP_LOGD(TAG, "send_event: enter event_id=0x%04"PRIx32""
             " data=%p len=%u evt_table_count=%zu",
             event_id, data, len, s_evt_count);

    eh_cp_rpc_registry_lock();

    for (size_t i = 0; i < s_evt_count; i++) {
        if (event_id < (uint32_t)s_evt_table[i].id_min) continue;
        if (event_id > (uint32_t)s_evt_table[i].id_max) continue;

        eh_rpc_evt_serialise_t serialise = s_evt_table[i].serialise;
        void *ctx = s_evt_table[i].ctx;
        eh_cp_rpc_registry_unlock();

        ESP_LOGD(TAG, "send_event: found serialiser=%p for 0x%04"PRIx32""
                 " data=%p len=%u",
                 serialise, event_id, data, len);

        uint8_t *out = NULL;
        uint16_t out_len = 0;
        const eh_rpc_evt_params_t params = {
            .event_id = event_id,
            .data     = data,
            .data_len = len,
            .out_buf  = &out,
            .out_len  = &out_len,
        };
        esp_err_t r = serialise(ctx, &params);

        ESP_LOGD(TAG, "send_event: serialise returned r=%d out=%p out_len=%u"
                 " for 0x%04"PRIx32, r, out, out_len, event_id);

        if (r == ESP_OK && out && out_len > 0) {
            ESP_LOGD(TAG, "send_event: forwarding %u bytes to protocomm ep=%s"
                     " event_id=0x%04"PRIx32,
                     out_len, eh_cp_feat_rpc_get_evt_ep(), event_id);
            r = eh_cp_feat_rpc_send_evt(eh_cp_feat_rpc_get_evt_ep(),
                                               (int)event_id,
                                               out, (int)out_len);
            ESP_LOGD(TAG, "send_event: protocomm returned r=%d for 0x%04"PRIx32,
                     r, event_id);
        } else {
            if (r == ESP_OK) r = ESP_ERR_INVALID_SIZE;
            ESP_LOGE(TAG, "send_event: serialise failed event_id=0x%04"PRIx32": %s"
                     " (out=%p out_len=%u)",
                     event_id, esp_err_to_name(r), out, out_len);
        }
        if (out) free(out);
        return r;
    }

    eh_cp_rpc_registry_unlock();
    ESP_LOGW(TAG, "send_event: no serialiser for event_id 0x%04"PRIx32, event_id);
    return ESP_ERR_NOT_FOUND;
}
#endif /* EH_CP_FEAT_RPC_READY */
