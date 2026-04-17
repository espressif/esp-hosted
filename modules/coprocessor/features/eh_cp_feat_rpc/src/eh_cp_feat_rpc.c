/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * eh_cp_feat_rpc.c — RPC transport extension entry point
 *
 * Registers at priority 5 via EH_CP_FEAT_REGISTER so that protocomm is
 * initialised before any priority-100 extension (rpc_fg, rpc_mcu) can
 * register WiFi handlers and attempt to emit RPC events.
 *
 * init_fn  → calls eh_cp_protocomm_init(ext_rpc_serial_write,
 *                                                ext_rpc_serial_read)
 *            which is defined in eh_cp_feat_rpc_serial.c (private).
 * deinit_fn→ calls eh_cp_protocomm_deinit().
 *
 * The 5-function public API (eh_cp_feat_rpc.h) is thin wrappers
 * over the private _ll layer so that no other component needs to include
 * eh_cp_feat_rpc_ll.h.
 */

#include <string.h>
#include "esp_log.h"
#include "esp_err.h"
#include "eh_cp_core.h"          /* EH_CP_FEAT_REGISTER, register_rx_cb */
#include "eh_cp_master_config.h"
#if EH_CP_FEAT_RPC_READY
#include "eh_cp_event.h"         /* EH_CP_EVT_PRIVATE_RPC_READY */
#include "eh_cp_feat_rpc.h"       /* own public API */
#include "eh_cp_feat_rpc_ll.h"    /* private ll layer */

static const char TAG[] = "ext_rpc";

static void ext_rpc_private_ready_handler(void *handler_arg,
                                          esp_event_base_t base,
                                          int32_t id,
                                          void *event_data)
{
    (void)handler_arg;
    if (base != EH_CP_EVENT || id != EH_CP_EVT_PRIVATE_RPC_READY) {
        return;
    }

    const eh_cp_rpc_ep_config_t *ep_cfg = (const eh_cp_rpc_ep_config_t *)event_data;
    const char *req_ep = (ep_cfg && ep_cfg->req_ep[0]) ? ep_cfg->req_ep : RPC_EP_NAME_REQ;
    const char *evt_ep = (ep_cfg && ep_cfg->evt_ep[0]) ? ep_cfg->evt_ep : RPC_EP_NAME_EVT;

    esp_err_t r = eh_cp_feat_rpc_set_endpoints(req_ep, evt_ep);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "private_ready: set_endpoints failed: %s", esp_err_to_name(r));
        return;
    }

    ESP_LOGI(TAG, "private_ready: endpoints set req=%s evt=%s", req_ep, evt_ep);
    esp_event_post(EH_CP_EVENT, EH_CP_EVT_ESP_INIT, NULL, 0, portMAX_DELAY);
}

/* ── Extension lifecycle ─────────────────────────────────────────────────── */

static esp_err_t eh_cp_feat_rpc_init(void)
{
    ESP_LOGI(TAG, "init: starting protocomm transport (priority 5)");

    esp_err_t ret = eh_cp_rpc_registries_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "registry init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = eh_cp_protocomm_init(eh_cp_feat_rpc_serial_write,
                                       eh_cp_feat_rpc_serial_read);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "protocomm init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = eh_cp_register_rx_cb(ESP_SERIAL_IF,
                                       eh_cp_feat_rpc_serial_rx,
                                       NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "register rx cb failed: %s", esp_err_to_name(ret));
        return ret;
    }

    esp_err_t eh = esp_event_handler_register(EH_CP_EVENT,
                                              EH_CP_EVT_PRIVATE_RPC_READY,
                                              ext_rpc_private_ready_handler,
                                              NULL);
    if (eh != ESP_OK) {
        ESP_LOGE(TAG, "event handler register failed: %s", esp_err_to_name(eh));
        return eh;
    }

    ESP_LOGI(TAG, "init: protocomm + pserial_task ready");
    return ESP_OK;
}

static esp_err_t eh_cp_feat_rpc_deinit(void)
{
    ESP_LOGI(TAG, "deinit: stopping protocomm transport");
    esp_event_handler_unregister(EH_CP_EVENT,
                                 EH_CP_EVT_PRIVATE_RPC_READY,
                                 ext_rpc_private_ready_handler);
    return eh_cp_protocomm_deinit();
}

#if EH_CP_FEAT_RPC_AUTO_INIT
EH_CP_FEAT_REGISTER(eh_cp_feat_rpc_init,
                   eh_cp_feat_rpc_deinit,
                   "ext_rpc",
                   tskNO_AFFINITY,
                   5);   /* must be < 100 (rpc_fg/mcu) */
#endif

/* ── Public API — thin wrappers over _ll ─────────────────────────────────── */

bool eh_cp_feat_rpc_is_ready(void)
{
    return (eh_cp_protocomm_get_instance() != NULL);
}

esp_err_t eh_cp_feat_rpc_set_endpoints(const char *req_ep,
                                               const char *evt_ep)
{
    if (!req_ep || !evt_ep) {
        return ESP_ERR_INVALID_ARG;
    }
    return eh_cp_rpc_set_endpoints(req_ep, (int)strlen(req_ep) + 1,
                                           evt_ep, (int)strlen(evt_ep) + 1);
}

const char *eh_cp_feat_rpc_get_evt_ep(void)
{
    return eh_cp_rpc_get_evt_ep();
}

esp_err_t eh_cp_feat_rpc_process_req(uint8_t *data, int len)
{
    return eh_cp_protocomm_process_rpc_req(data, len);
}

esp_err_t eh_cp_feat_rpc_send_evt(const char *epname,
                                          int event_id,
                                          void *data, int size)
{
    return eh_cp_protocomm_process_rpc_evt(epname, event_id, data, size);
}
#endif /* EH_CP_FEAT_RPC_READY */
