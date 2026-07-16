/* SPDX-License-Identifier: Apache-2.0 */
/*
 * eh_host_wifi_dpp.h — RPC API wrappers for the WiFi DPP feature.
 *
 * Implementations live in
 * `host/features/eh_host_feat_rpc_ext_v2/src/eh_host_feat_rpc_ext_v2_api_wifi.c`.
 * Mirrors the request side of the 5 DPP RPCs and a single-slot
 * subscribe API for each of the 3 events.
 */

#ifndef EH_HOST_WIFI_DPP_H_
#define EH_HOST_WIFI_DPP_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

/* Forward decls — full definitions live in eh_host_feat_wifi_ext_dpp.h.
 * Function pointers below take `const T *`; an incomplete type suffices. */
typedef struct eh_host_dpp_uri_event  eh_host_dpp_uri_event_t;
typedef struct eh_host_dpp_cfg_event  eh_host_dpp_cfg_event_t;
typedef struct eh_host_dpp_fail_event eh_host_dpp_fail_event_t;

#ifdef __cplusplus
extern "C" {
#endif

/* IDF DPP supplicant types — duplicated locally so the host RPC API
 * stays usable without the IDF wpa_supplicant component on non-IDF
 * ports (host/compat/include/ does not yet redefine these). On real
 * IDF the duplicate enumerator/typedef values are bit-for-bit
 * identical to esp_dpp.h, so the linker sees no surprises. */
#ifndef EH_HOST_DPP_TYPES_DEFINED
#define EH_HOST_DPP_TYPES_DEFINED
typedef enum {
    DPP_BOOTSTRAP_QR_CODE,   /**< QR Code Method */
    DPP_BOOTSTRAP_PKEX,      /**< Proof of Knowledge Method */
    DPP_BOOTSTRAP_NFC_URI,   /**< NFC URI record Method */
} esp_supp_dpp_bootstrap_t;

typedef enum {
    ESP_SUPP_DPP_URI_READY,  /**< URI ready through Bootstrapping  */
    ESP_SUPP_DPP_CFG_RECVD,  /**< Config received via DPP Auth     */
    ESP_SUPP_DPP_PDR_RECVD,  /**< Peer Discovery Response received */
    ESP_SUPP_DPP_FAIL,       /**< DPP Authentication failure       */
} esp_supp_dpp_event_t;

typedef void (*esp_supp_dpp_event_cb_t)(esp_supp_dpp_event_t evt, void *data);
#endif /* EH_HOST_DPP_TYPES_DEFINED */

/* Upstream-aligned signatures (rpc_wrap.h):
 *   rpc_supp_dpp_init(esp_supp_dpp_event_cb_t evt_cb)
 *   rpc_supp_dpp_bootstrap_gen(const char *chan_list,
 *                              esp_supp_dpp_bootstrap_t type,
 *                              const char *key, const char *info)
 *
 * The host caches the event callback locally; on the wire the
 * "callback enable" bit is what the CP sees (cb!=NULL → enable). */

esp_err_t eh_host_wifi_dpp_init(esp_supp_dpp_event_cb_t evt_cb);
esp_err_t eh_host_wifi_dpp_deinit(void);
esp_err_t eh_host_wifi_dpp_start_listen(void);
esp_err_t eh_host_wifi_dpp_stop_listen(void);
esp_err_t eh_host_wifi_dpp_bootstrap_gen(const char *chan_list,
                                   esp_supp_dpp_bootstrap_t type,
                                   const char *key,
                                   const char *info);

/* ---------------- events: single-slot subscribers ----------------- */
typedef void (*eh_host_wifi_dpp_uri_cb_t)(
    const eh_host_dpp_uri_event_t *e, void *ctx);
typedef void (*eh_host_wifi_dpp_cfg_cb_t)(
    const eh_host_dpp_cfg_event_t *e, void *ctx);
typedef void (*eh_host_wifi_dpp_fail_cb_t)(
    const eh_host_dpp_fail_event_t *e, void *ctx);

esp_err_t eh_host_wifi_dpp_subscribe_uri_ready(
    eh_host_wifi_dpp_uri_cb_t cb, void *ctx);
esp_err_t eh_host_wifi_dpp_unsubscribe_uri_ready(
    eh_host_wifi_dpp_uri_cb_t cb);

esp_err_t eh_host_wifi_dpp_subscribe_cfg_recvd(
    eh_host_wifi_dpp_cfg_cb_t cb, void *ctx);
esp_err_t eh_host_wifi_dpp_unsubscribe_cfg_recvd(
    eh_host_wifi_dpp_cfg_cb_t cb);

esp_err_t eh_host_wifi_dpp_subscribe_fail(
    eh_host_wifi_dpp_fail_cb_t cb, void *ctx);
esp_err_t eh_host_wifi_dpp_unsubscribe_fail(
    eh_host_wifi_dpp_fail_cb_t cb);

#ifdef __cplusplus
}
#endif

#endif /* EH_HOST_WIFI_DPP_H_ */
