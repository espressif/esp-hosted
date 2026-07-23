/* SPDX-License-Identifier: Apache-2.0 */
/*
 * eh_host_wifi_dpp.h — RPC API wrappers for the WiFi DPP feature.
 *
 * Implementations live in
 * `host/features/eh_host_feat_wifi_ext_dpp/src/eh_host_wifi_dpp.c`.
 * Mirrors the request side of the 5 DPP RPCs
 */

#ifndef EH_HOST_WIFI_DPP_H_
#define EH_HOST_WIFI_DPP_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

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
 */

esp_err_t eh_host_wifi_dpp_init(esp_supp_dpp_event_cb_t evt_cb);
esp_err_t eh_host_wifi_dpp_deinit(void);
esp_err_t eh_host_wifi_dpp_start_listen(void);
esp_err_t eh_host_wifi_dpp_stop_listen(void);
esp_err_t eh_host_wifi_dpp_bootstrap_gen(const char *chan_list,
                                   esp_supp_dpp_bootstrap_t type,
                                   const char *key,
                                   const char *info);

#ifdef __cplusplus
}
#endif

#endif /* EH_HOST_WIFI_DPP_H_ */
