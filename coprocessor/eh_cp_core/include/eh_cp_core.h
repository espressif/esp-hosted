/* SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD */
/* SPDX-License-Identifier: Apache-2.0 */

#ifndef EH_CP_CORE_H
#define EH_CP_CORE_H

/* Sole header extensions include; dependency flows extensions -> core only. */

#include "esp_err.h"
#include "eh_common_interface.h"
#include "eh_common_caps.h"
#include "eh_cp_master_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#ifdef __cplusplus
extern "C" {
#endif

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


void     eh_cp_add_feat_cap_bits(uint8_t caps_bits, uint32_t ext_caps_bits);
void     eh_cp_add_feat_cap_bits_idx(uint8_t index, uint32_t feature_bitmask);
void     eh_cp_clear_feat_cap_bits(uint8_t caps_bits, uint32_t ext_caps_bits);
void     eh_cp_clear_feat_cap_bits_idx(uint8_t index, uint32_t feature_bitmask);
uint8_t  eh_cp_get_caps(void);
uint32_t eh_cp_get_ext_caps(void);
void     eh_cp_get_feat_caps(uint32_t out_feat_caps[EH_FEAT_CAPS_COUNT]);


/* Extension auto-init: descriptors placed in .eh_cp_feat_descs_dram via
 * EH_CP_FEAT_REGISTER; core sorts ascending by priority and calls init_fn.
 * Convention: 100=rpc, 200=feat, 300=late. */
typedef esp_err_t (*eh_cp_feat_init_fn_t)(void);

typedef struct {
    eh_cp_feat_init_fn_t  init_fn;
    eh_cp_feat_init_fn_t  deinit_fn;
    const char               *name;
    int                       affinity;
    int                       priority;
} eh_cp_feat_desc_t;

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

extern const eh_cp_feat_desc_t _eh_cp_feat_descs_start;
extern const eh_cp_feat_desc_t _eh_cp_feat_descs_end;

/* Set by auto_feat_init_task; host_reset_task waits for it before advertising caps. */
#define EH_CP_FEAT_INIT_DONE_BIT   (1u << 0)
extern EventGroupHandle_t g_auto_feat_init_done_eg;


/* Rpc.msg_id proto field number. */
#define EH_PROTO_FIELD_MSG_ID   2u

/* Extract msg_id (proto field EH_PROTO_FIELD_MSG_ID varint).
 * Returns EH_PROTO_FIELD_MSG_ID on success, 0 if absent. */
uint32_t eh_proto_extract_msg_id(const uint8_t *buf, uint16_t len,
                                         uint32_t *msg_id_out);

#ifdef __cplusplus
}
#endif

#endif /* EH_CP_CORE_H */
