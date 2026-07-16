/* SPDX-License-Identifier: Apache-2.0 */
/* iTWT extension public API (6 RPCs). Field semantics match
 * esp_wifi_sta_itwt_*; ints are proto passthrough int32. */

#ifndef EH_HOST_FEAT_WIFI_EXT_ITWT_H_
#define EH_HOST_FEAT_WIFI_EXT_ITWT_H_

#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Lifecycle — registered into the auto-init walk. */
int eh_host_feat_wifi_ext_itwt_init(void);
int eh_host_feat_wifi_ext_itwt_deinit(void);

/* Setup individual TWT agreement. bitmask_1 packs trigger/flow_type/
 * flow_id/wake_invl_expn/wake_duration_unit. */
esp_err_t eh_host_wifi_sta_itwt_setup(uint32_t setup_cmd,
                                 uint32_t bitmask_1,
                                 uint32_t min_wake_dura,
                                 uint32_t wake_invl_mant,
                                 uint32_t twt_id,
                                 uint32_t timeout_time_ms);

esp_err_t eh_host_wifi_sta_itwt_teardown(int32_t flow_id);
esp_err_t eh_host_wifi_sta_itwt_suspend(int32_t flow_id, int32_t suspend_time_ms);
esp_err_t eh_host_wifi_sta_itwt_send_probe_req(int32_t timeout_ms);
esp_err_t eh_host_wifi_sta_itwt_set_target_wake_time_offset(int32_t offset_us);

/* On success, writes the active-flow bitmap (bit N == flow N active).
 * Returns 0 on CP-accept, -1 otherwise. */
esp_err_t eh_host_wifi_sta_itwt_get_flow_id_status(int32_t *out_bitmap);

/* Async event subscriptions: single-slot per event; NULL clears. */
typedef struct eh_host_itwt_setup_event {
    int32_t   status;
    uint32_t  reason;
    uint64_t  target_wake_time;
    uint32_t  setup_cmd;
    /* Discrete-field decode of the on-wire `bitmask_1` (mirrors upstream
     * rpc_evt.c:193-199 — same enum positions in eh_rpc_bitmasks.h).
     * `bitmask_1` is also exposed raw for forward-compat with future
     * bit additions. */
    uint8_t   trigger;
    uint8_t   flow_type;
    uint8_t   flow_id;             /* 3 bits */
    uint8_t   wake_invl_expn;      /* 5 bits */
    uint8_t   wake_duration_unit;
    uint32_t  reserved;            /* EH_HOST_WIFI_ITWT_CONFIG_1_GET_RESERVED_VAL */
    uint32_t  bitmask_1;
    uint32_t  min_wake_dura;
    uint32_t  wake_invl_mant;
    uint32_t  twt_id;
    uint32_t  timeout_time_ms;
} eh_host_itwt_setup_event_t;

typedef struct eh_host_itwt_teardown_event {
    uint32_t  flow_id;
    uint32_t  status;
} eh_host_itwt_teardown_event_t;

typedef struct eh_host_itwt_suspend_event {
    int32_t   status;
    uint32_t  flow_id_bitmap;
} eh_host_itwt_suspend_event_t;

typedef struct eh_host_itwt_probe_event {
    int32_t   status;
    uint32_t  reason;
} eh_host_itwt_probe_event_t;

typedef void (*eh_host_itwt_setup_cb_t)(const eh_host_itwt_setup_event_t *e, void *ctx);
typedef void (*eh_host_itwt_teardown_cb_t)(const eh_host_itwt_teardown_event_t *e, void *ctx);
typedef void (*eh_host_itwt_suspend_cb_t)(const eh_host_itwt_suspend_event_t *e, void *ctx);
typedef void (*eh_host_itwt_probe_cb_t)(const eh_host_itwt_probe_event_t *e, void *ctx);

esp_err_t eh_host_wifi_sta_itwt_on_setup(eh_host_itwt_setup_cb_t cb, void *ctx);
esp_err_t eh_host_wifi_sta_itwt_on_teardown(eh_host_itwt_teardown_cb_t cb, void *ctx);
esp_err_t eh_host_wifi_sta_itwt_on_suspend(eh_host_itwt_suspend_cb_t cb, void *ctx);
esp_err_t eh_host_wifi_sta_itwt_on_probe(eh_host_itwt_probe_cb_t cb, void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* EH_HOST_FEAT_WIFI_EXT_ITWT_H_ */
