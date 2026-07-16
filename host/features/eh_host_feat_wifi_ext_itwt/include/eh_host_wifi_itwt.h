/* SPDX-License-Identifier: Apache-2.0 */
/*
 * eh_host_wifi_itwt.h — RPC API wrappers for the iTWT feature.
 * 6 RPCs (setup/teardown/suspend/probe/twt-offset/get-flow-status) plus
 * single-slot subscribers for 4 events.
 */

#ifndef EH_HOST_WIFI_ITWT_H_
#define EH_HOST_WIFI_ITWT_H_

#include <stdint.h>

#include "esp_err.h"
#include "esp_wifi_he_types.h"     /* wifi_{i}twt_setup_config_t */

/* IDF-version split for the iTWT setup-config struct.
 * Pre-5.3 the type is wifi_twt_setup_config_t; from 5.3 onward IDF
 * renamed it to wifi_itwt_setup_config_t. Mirror the upstream MCU
 * (H_WIFI_HE_GREATER_THAN_ESP_IDF_5_3) so older targets still build. */
#include "eh_host_port_wifi.h"

/* Forward decls — full definitions live in
 * host/features/eh_host_feat_wifi_ext_itwt/include/eh_host_feat_wifi_ext_itwt.h.
 * Function pointers below only take `const T *`, so an incomplete type
 * is sufficient here. Consumers that dereference (i.e. callbacks reading
 * fields) include the feature header directly. */
typedef struct eh_host_itwt_setup_event    eh_host_itwt_setup_event_t;
typedef struct eh_host_itwt_teardown_event eh_host_itwt_teardown_event_t;
typedef struct eh_host_itwt_suspend_event  eh_host_itwt_suspend_event_t;
typedef struct eh_host_itwt_probe_event    eh_host_itwt_probe_event_t;

#ifdef __cplusplus
extern "C" {
#endif

/* Upstream signature: rpc_wifi_sta_itwt_setup(wifi_itwt_setup_config_t*)
 * on IDF ≥ 5.3, wifi_twt_setup_config_t* before that. Field layout is
 * identical across the rename, so the impl packs the bit-fields
 * (trigger / flow_type / flow_id / wake_invl_expn / wake_duration_unit)
 * into the wire's bitmask_1 the same way on both. */
#if EH_HOST_WIFI_HE_GREATER_THAN_ESP_IDF_5_3
esp_err_t eh_host_wifi_itwt_setup(const wifi_itwt_setup_config_t *setup_config);
#else
esp_err_t eh_host_wifi_itwt_setup(const wifi_twt_setup_config_t *setup_config);
#endif

esp_err_t eh_host_wifi_itwt_teardown(int flow_id);
esp_err_t eh_host_wifi_itwt_suspend(int flow_id, int suspend_time_ms);
esp_err_t eh_host_wifi_itwt_send_probe_req(int timeout_ms);
esp_err_t eh_host_wifi_itwt_set_target_wake_time_offset(int offset_us);
esp_err_t eh_host_wifi_itwt_get_flow_id_status(int *flow_id_bitmap);

/* ---------------- events: single-slot subscribers ----------------- */
/* Event-payload typedefs (eh_host_itwt_*_event_t) are owned by the
 * eh_host_feat_wifi_ext_itwt feature header pulled in above. */
typedef void (*eh_host_wifi_itwt_setup_cb_t)(
    const eh_host_itwt_setup_event_t *e, void *ctx);
typedef void (*eh_host_wifi_itwt_teardown_cb_t)(
    const eh_host_itwt_teardown_event_t *e, void *ctx);
typedef void (*eh_host_wifi_itwt_suspend_cb_t)(
    const eh_host_itwt_suspend_event_t *e, void *ctx);
typedef void (*eh_host_wifi_itwt_probe_cb_t)(
    const eh_host_itwt_probe_event_t *e, void *ctx);

esp_err_t eh_host_wifi_itwt_subscribe_setup(
    eh_host_wifi_itwt_setup_cb_t cb, void *ctx);
esp_err_t eh_host_wifi_itwt_unsubscribe_setup(
    eh_host_wifi_itwt_setup_cb_t cb);

esp_err_t eh_host_wifi_itwt_subscribe_teardown(
    eh_host_wifi_itwt_teardown_cb_t cb, void *ctx);
esp_err_t eh_host_wifi_itwt_unsubscribe_teardown(
    eh_host_wifi_itwt_teardown_cb_t cb);

esp_err_t eh_host_wifi_itwt_subscribe_suspend(
    eh_host_wifi_itwt_suspend_cb_t cb, void *ctx);
esp_err_t eh_host_wifi_itwt_unsubscribe_suspend(
    eh_host_wifi_itwt_suspend_cb_t cb);

esp_err_t eh_host_wifi_itwt_subscribe_probe(
    eh_host_wifi_itwt_probe_cb_t cb, void *ctx);
esp_err_t eh_host_wifi_itwt_unsubscribe_probe(
    eh_host_wifi_itwt_probe_cb_t cb);

#ifdef __cplusplus
}
#endif

#endif /* EH_HOST_WIFI_ITWT_H_ */
