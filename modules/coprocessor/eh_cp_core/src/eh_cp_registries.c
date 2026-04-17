/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * eh_cp_registries.c
 *
 * Implements the two registries defined in eh_cp_core.h:
 *   Registry 1 — Interface RX/TX table
 *   Registry 2 — Capability accumulator
 *
 * Thread-safety:
 *   Capability accumulator uses a FreeRTOS spinlock (no mutex — called from
 *   boot-time single context).
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

#include "eh_cp_core.h"
#include "eh_caps.h"
#include "eh_log.h"

static const char TAG[] = "ehcp_reg";

/* ─────────────────────────────────────────────────────────────────────────────
 * Registry 1 — Interface RX/TX table
 * ─────────────────────────────────────────────────────────────────────────────*/

eh_iface_entry_t g_iface_table[ESP_IF_TYPE_MAX]; /* zero-initialised in BSS */

esp_err_t eh_cp_register_rx_cb(eh_if_type_t iface_type,
                                       hosted_rx_cb_t rx_cb, void *ctx)
{
    if (iface_type >= ESP_IF_TYPE_MAX) {
        ESP_LOGE(TAG, "register_rx_cb: invalid iface_type %d", iface_type);
        return ESP_ERR_INVALID_ARG;
    }
    if (!rx_cb) {
        ESP_LOGE(TAG, "register_rx_cb: rx_cb is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    /* Allow re-registration only for STA/AP (network split may override) */
    if (g_iface_table[iface_type].rx != NULL &&
        iface_type != ESP_STA_IF && iface_type != ESP_AP_IF) {
        ESP_LOGE(TAG, "register_rx_cb: iface %d already claimed", iface_type);
        return ESP_ERR_INVALID_STATE;
    }
    g_iface_table[iface_type].rx  = rx_cb;
    g_iface_table[iface_type].ctx = ctx;
    ESP_LOGD(TAG, "RX cb registered for iface %d", iface_type);
    return ESP_OK;
}

esp_err_t eh_cp_register_tx_cb(eh_if_type_t iface_type,
                                       hosted_tx_cb_t tx_cb, void *ctx)
{
    if (iface_type >= ESP_IF_TYPE_MAX) {
        ESP_LOGE(TAG, "register_tx_cb: invalid iface_type %d", iface_type);
        return ESP_ERR_INVALID_ARG;
    }
    /* NULL explicitly allowed — clears any existing TX override */
    g_iface_table[iface_type].tx  = tx_cb;
    g_iface_table[iface_type].ctx = ctx;
    ESP_LOGD(TAG, "TX cb %s for iface %d", tx_cb ? "registered" : "cleared", iface_type);
    return ESP_OK;
}

esp_err_t eh_cp_dispatch_rx(eh_if_type_t iface_type,
                                    void *buf, uint16_t len, void *eb)
{
    if (iface_type >= ESP_IF_TYPE_MAX) {
        ESP_LOGE(TAG, "dispatch_rx: invalid iface_type %d", iface_type);
        return ESP_ERR_INVALID_ARG;
    }
    const eh_iface_entry_t *e = &g_iface_table[iface_type];
    if (!e->rx) {
        /* Return NOT_FOUND so callers with fallback paths (e.g. STA->esp_wifi_internal_tx)
         * know no handler consumed the frame and can apply their own fallback. */
        return ESP_ERR_NOT_FOUND;
    }
    return e->rx(e->ctx, buf, len, eb);
}


/* ─────────────────────────────────────────────────────────────────────────────
 * Registry 2 — Capability accumulator
 * ─────────────────────────────────────────────────────────────────────────────*/

static uint8_t  s_caps     = 0;
static uint32_t s_ext_caps = 0;
static uint32_t s_feat_caps[EH_FEAT_CAPS_COUNT] = {0};

static portMUX_TYPE s_cap_spinlock = portMUX_INITIALIZER_UNLOCKED;

void eh_cp_add_feat_cap_bits(uint8_t caps_bits, uint32_t ext_caps_bits)
{
    taskENTER_CRITICAL(&s_cap_spinlock);
    s_caps     |= caps_bits;
    s_ext_caps |= ext_caps_bits;
    taskEXIT_CRITICAL(&s_cap_spinlock);
}

void eh_cp_add_feat_cap_bits_idx(uint8_t index, uint32_t feature_bitmask)
{
    if (index >= EH_FEAT_CAPS_COUNT) {
        ESP_LOGE(TAG, "add_feature_cap_bits: index %u out of range", index);
        return;
    }
    taskENTER_CRITICAL(&s_cap_spinlock);
    s_feat_caps[index] |= feature_bitmask;
    taskEXIT_CRITICAL(&s_cap_spinlock);
}

void eh_cp_clear_feat_cap_bits(uint8_t caps_bits, uint32_t ext_caps_bits)
{
    taskENTER_CRITICAL(&s_cap_spinlock);
    s_caps     &= (uint8_t) ~caps_bits;
    s_ext_caps &= ~ext_caps_bits;
    taskEXIT_CRITICAL(&s_cap_spinlock);
}

void eh_cp_clear_feat_cap_bits_idx(uint8_t index, uint32_t feature_bitmask)
{
    if (index >= EH_FEAT_CAPS_COUNT) {
        ESP_LOGE(TAG, "clear_feature_cap_bits: index %u out of range", index);
        return;
    }
    taskENTER_CRITICAL(&s_cap_spinlock);
    s_feat_caps[index] &= ~feature_bitmask;
    taskEXIT_CRITICAL(&s_cap_spinlock);
}

uint8_t eh_cp_get_caps(void)
{
    uint8_t v;
    taskENTER_CRITICAL(&s_cap_spinlock);
    v = s_caps;
    taskEXIT_CRITICAL(&s_cap_spinlock);
    return v;
}

uint32_t eh_cp_get_ext_caps(void)
{
    uint32_t v;
    taskENTER_CRITICAL(&s_cap_spinlock);
    v = s_ext_caps;
    taskEXIT_CRITICAL(&s_cap_spinlock);
    return v;
}

void eh_cp_get_feat_caps(uint32_t out_feat_caps[EH_FEAT_CAPS_COUNT])
{
    taskENTER_CRITICAL(&s_cap_spinlock);
    memcpy(out_feat_caps, s_feat_caps, sizeof(s_feat_caps));
    taskEXIT_CRITICAL(&s_cap_spinlock);
}


/* ─────────────────────────────────────────────────────────────────────────────
 * Lightweight proto field-2 scanner
 * ─────────────────────────────────────────────────────────────────────────────*/

uint32_t eh_proto_extract_msg_id(const uint8_t *buf,
                                         uint16_t len,
                                         uint32_t *msg_id_out)
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
        case 0:
            while (i < len && (buf[i] & 0x80u)) i++;
            if (i < len) i++;
            break;
        case 1: i = (uint16_t)(i + 8u); break;
        case 2: {
            uint32_t slen = 0; uint8_t shift = 0;
            while (i < len && shift < 35) {
                uint8_t b = buf[i++];
                slen |= (uint32_t)(b & 0x7Fu) << shift;
                if (!(b & 0x80u)) break;
                shift = (uint8_t)(shift + 7u);
            }
            i = (uint16_t)(i + slen);
            break;
        }
        case 5: i = (uint16_t)(i + 4u); break;
        default: return 0;
        }
    }
    return 0;
}
