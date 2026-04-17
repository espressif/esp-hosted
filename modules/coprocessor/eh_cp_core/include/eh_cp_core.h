/* SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD */
/* SPDX-License-Identifier: Apache-2.0 */

#ifndef EH_CP_CORE_H
#define EH_CP_CORE_H

/*
 * eh_cp_core.h — Two-Registry Architecture
 *
 * This header is the ONLY cp_core header that extensions should include.
 * Core never includes any extension header; the dependency flows one way only:
 *   extensions → core
 *
 * Two independent registries:
 *   1. Interface RX/TX table   (g_iface_table[ESP_IF_TYPE_MAX])
 *   2. Capability accumulator  (caps, ext_caps, feat_caps[8])
 */

#include "esp_err.h"
#include "eh_common_interface.h" /* eh_if_type_t, ESP_IF_TYPE_MAX */
#include "eh_common_caps.h"      /* EH_FEAT_CAPS_COUNT */
#include "eh_cp_master_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 * Registry 1 — Interface RX/TX Table
 * ═══════════════════════════════════════════════════════════════════════════ */

typedef esp_err_t (*hosted_rx_cb_t)(void *ctx, void *buf, uint16_t len, void *eb);
typedef int       (*hosted_tx_cb_t)(void *ctx, void *frame, uint16_t len);

typedef struct {
    hosted_rx_cb_t rx;
    hosted_tx_cb_t tx;
    void          *ctx;
} eh_iface_entry_t;

extern eh_iface_entry_t g_iface_table[ESP_IF_TYPE_MAX];

extern volatile uint8_t hdr_ver_negotiated;
extern volatile uint8_t rpc_ver_negotiated;

esp_err_t eh_cp_register_rx_cb(eh_if_type_t iface_type,
                                       hosted_rx_cb_t rx_cb, void *ctx);
esp_err_t eh_cp_register_tx_cb(eh_if_type_t iface_type,
                                       hosted_tx_cb_t tx_cb, void *ctx);
esp_err_t eh_cp_dispatch_rx(eh_if_type_t iface_type,
                                    void *buf, uint16_t len, void *eb);


/* ═══════════════════════════════════════════════════════════════════════════
 * Registry 2 — Capability Accumulator
 * ═══════════════════════════════════════════════════════════════════════════ */

void     eh_cp_add_feat_cap_bits(uint8_t caps_bits, uint32_t ext_caps_bits);
void     eh_cp_add_feat_cap_bits_idx(uint8_t index, uint32_t feature_bitmask);
void     eh_cp_clear_feat_cap_bits(uint8_t caps_bits, uint32_t ext_caps_bits);
void     eh_cp_clear_feat_cap_bits_idx(uint8_t index, uint32_t feature_bitmask);
uint8_t  eh_cp_get_caps(void);
uint32_t eh_cp_get_ext_caps(void);
void     eh_cp_get_feat_caps(uint32_t out_feat_caps[EH_FEAT_CAPS_COUNT]);


/* ═══════════════════════════════════════════════════════════════════════════
 * Extension Auto-Init — EH_CP_FEAT_REGISTER
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * Each extension places one descriptor in the ".eh_cp_feat_descs" linker section
 * using EH_CP_FEAT_REGISTER().  Core's auto_feat_init_task walks the section at boot,
 * sorts by init_priority ascending (lower = runs first), and calls init().
 *
 * Suggested priorities (not enforced — just a convention):
 *   100 — _rpc_ tier (lightweight, only registers table entries)
 *   200 — _feat_ tier (may start tasks, allocate buffers)
 *   300 — late init (depends on feat_ being ready)
 *
 * Affinity values: tskNO_AFFINITY (-1), 0 (PRO_CPU), 1 (APP_CPU).
 * First implementation runs all init_fn()s in a single task regardless of
 * affinity; per-init pinning is a future optimisation.
 */
typedef esp_err_t (*eh_cp_feat_init_fn_t)(void);

typedef struct {
    eh_cp_feat_init_fn_t  init_fn;
    eh_cp_feat_init_fn_t  deinit_fn;
    const char               *name;
    int                       affinity;
    int                       priority;
} eh_cp_feat_desc_t;

/*
 * EH_CP_FEAT_REGISTER(init_fn, deinit_fn, name_str, affinity, priority)
 *
 * Placed at file scope in the extension's .c file.  Zero cost if the
 * translation unit is not compiled (disabled extension = no flash/RAM used).
 *
 * Example:
 *   EH_CP_FEAT_REGISTER(rpc_wifi_init, NULL, "rpc_wifi", tskNO_AFFINITY, 100);
 */
#define EH_CP_FEAT_REGISTER(_init, _deinit, _name, _affinity, _prio)          \
    static const eh_cp_feat_desc_t                                        \
    __eh_cp_feat_desc_##_init                                             \
    __attribute__((section(".eh_cp_feat_descs_dram"), used, aligned(4))) = {  \
        .init_fn   = (_init),                                                 \
        .deinit_fn = (_deinit),                                               \
        .name      = (_name),                                                 \
        .affinity  = (_affinity),                                             \
        .priority  = (_prio),                                                 \
    }

/* Linker section bounds for .eh_cp_feat_descs_dram (custom ld). */
extern const eh_cp_feat_desc_t _eh_cp_feat_descs_start;
extern const eh_cp_feat_desc_t _eh_cp_feat_descs_end;

/* Event group bit set by auto_feat_init_task when all extensions have finished
 * their init_fn().  transport_init_task waits on this before calling
 * generate_startup_event() so all cap bits are accumulated first. */
#define EH_CP_FEAT_INIT_DONE_BIT   (1u << 0)
extern EventGroupHandle_t g_auto_feat_init_done_eg;


/* ═══════════════════════════════════════════════════════════════════════════
 * Lightweight proto field-2 scanner
 * ═══════════════════════════════════════════════════════════════════════════ */

/*
 * Extract the msg_id (proto field 2 varint) from raw proto bytes.
 * All .proto files in this repo place msg_id at field 2 by convention.
 * Returns 2 on success (field number found), 0 if field 2 is absent.
 */
uint32_t eh_proto_extract_msg_id(const uint8_t *buf, uint16_t len,
                                         uint32_t *msg_id_out);

#ifdef __cplusplus
}
#endif

#endif /* EH_CP_CORE_H */
