/* SPDX-License-Identifier: Apache-2.0 */
/* Wi-Fi DPP extension public API. Five RPCs drive Easy Connect on the CP. */

#ifndef EH_HOST_FEAT_WIFI_EXT_DPP_H_
#define EH_HOST_FEAT_WIFI_EXT_DPP_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

int eh_host_feat_wifi_ext_dpp_init(void);
int eh_host_feat_wifi_ext_dpp_deinit(void);

/* DPP request entry points live in eh_host_wifi_dpp.h. */

/* Async event subscriptions: single-slot, NULL clears. uri_ready
 * qrcode is valid only for the cb duration (copy to retain); cfg_recvd
 * ssid/password/bssid are inline copies. */

typedef struct eh_host_dpp_uri_event {
    const uint8_t *qrcode;
    size_t         qrcode_len;
} eh_host_dpp_uri_event_t;

typedef struct eh_host_dpp_cfg_event {
    uint8_t   ssid[33];          /* EH_RPC_SSID_LEN+1, NUL-terminated */
    uint32_t  ssid_len;
    uint8_t   password[65];      /* EH_RPC_PASSWORD_LEN+1, NUL-terminated */
    uint8_t   bssid[6];
    bool      bssid_set;
    int32_t   authmode;
} eh_host_dpp_cfg_event_t;

typedef struct eh_host_dpp_fail_event {
    int32_t   reason;
} eh_host_dpp_fail_event_t;

typedef void (*eh_host_dpp_uri_cb_t)(const eh_host_dpp_uri_event_t *e, void *ctx);
typedef void (*eh_host_dpp_cfg_cb_t)(const eh_host_dpp_cfg_event_t *e, void *ctx);
typedef void (*eh_host_dpp_fail_cb_t)(const eh_host_dpp_fail_event_t *e, void *ctx);

esp_err_t eh_host_wifi_dpp_on_uri_ready(eh_host_dpp_uri_cb_t cb, void *ctx);
esp_err_t eh_host_wifi_dpp_on_cfg_recvd(eh_host_dpp_cfg_cb_t cb, void *ctx);
esp_err_t eh_host_wifi_dpp_on_fail(eh_host_dpp_fail_cb_t cb, void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* EH_HOST_FEAT_WIFI_EXT_DPP_H_ */
