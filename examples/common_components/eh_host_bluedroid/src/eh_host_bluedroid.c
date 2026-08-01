/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/* SPDX-License-Identifier: Apache-2.0 */
/*
 * eh_host_bluedroid.c — Bluedroid <-> ESP-Hosted HCI port.
 *
 * Bluedroid speaks H4 (pkt-type | body) on both its send and recv hooks,
 * which is exactly the on-wire HCI form ESP-Hosted carries, so this port
 * is a thin forwarder. init/deinit are public (auto-init at priority after
 * feat_bt, or callable by the app when auto-init is disabled). The
 * hosted_hci_bluedroid_* ops keep their upstream names — the compat header
 * esp_hosted_bluedroid.h re-exports them for a bring-your-own-driver app.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"

#include "sdkconfig.h"

#include "esp_bluedroid_hci.h"

#include "eh_host_feat_bt.h"       /* eh_host_bt_controller_* */
#include "eh_host_feat_bt_mcu.h"   /* eh_host_bt_mcu_hci_register */
#if defined(CONFIG_ESP_HOSTED_HOST_BT_PORT_BLUEDROID_AUTO_INIT)
#include "eh_host_auto_init.h"
#endif

#include "eh_host_bluedroid.h"

static const char TAG[] = "eh_host_bluedroid";

/* Single subscriber — Bluedroid is the only HCI host stack per build. */
static esp_bluedroid_hci_driver_callbacks_t s_callback;

/* tx fn bound at register; never NULL (defaults to drop). */
static int hci_tx_unbound(const uint8_t *frame, uint16_t len)
{ (void)frame; (void)len; return -1; }
static eh_host_bt_mcu_hci_tx_fn_t s_hci_tx = hci_tx_unbound;

/* RX: ESP-Hosted → Bluedroid. frame is H4-prefixed, exactly what
 * notify_host_recv expects; forward as-is (valid for cb duration). */
static void bluedroid_rx(const uint8_t *frame, uint16_t len, void *user)
{
    (void)user;
    if (s_callback.notify_host_recv)
        s_callback.notify_host_recv((uint8_t *)frame, len);
}

void hosted_hci_bluedroid_open(void)  { }
void hosted_hci_bluedroid_close(void) { }

bool hosted_hci_bluedroid_check_send_available(void) { return true; }

esp_err_t hosted_hci_bluedroid_register_host_callback(
    const esp_bluedroid_hci_driver_callbacks_t *callback)
{
    if (!callback) {
        s_callback.notify_host_send_available = NULL;
        s_callback.notify_host_recv           = NULL;
        eh_host_bt_mcu_hci_unregister();
        s_hci_tx = hci_tx_unbound;
        return ESP_OK;
    }
    s_callback.notify_host_send_available = callback->notify_host_send_available;
    s_callback.notify_host_recv           = callback->notify_host_recv;

    eh_host_bt_mcu_hci_tx_fn_t tx = eh_host_bt_mcu_hci_register(bluedroid_rx, NULL);
    if (!tx) {
        ESP_LOGE(TAG, "HCI register failed");
        return ESP_FAIL;
    }
    s_hci_tx = tx;
    return ESP_OK;
}

/* TX: Bluedroid → ESP-Hosted. `data` is an H4-prefixed buffer owned by
 * Bluedroid; the transport copies it synchronously, so pass it straight
 * through — no port-side copy. */
void hosted_hci_bluedroid_send(uint8_t *data, uint16_t len)
{
    if (!data || len == 0) return;
    if (s_hci_tx(data, len) != 0)
        ESP_LOGE(TAG, "hci tx failed (len=%u)", len);
}

esp_err_t eh_host_bluedroid_init(void)
{
    /* Transport bring-up is not this port's concern — the boot constructor (or
     * the app) connects to the slave before any feature runs. This port only
     * attaches Bluedroid to feat_bt's HCI byte-pipe.
     *
     * feat_bt (lower priority) may already have brought the controller up;
     * ESP_ERR_INVALID_STATE means "already done" — treat as success. */
    esp_err_t err = eh_host_bt_controller_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "BT controller init failed: %s", esp_err_to_name(err));
        return err;
    }
    err = eh_host_bt_controller_enable();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "BT controller enable failed: %s", esp_err_to_name(err));
        return err;
    }

    static const esp_bluedroid_hci_driver_operations_t ops = {
        .send                   = hosted_hci_bluedroid_send,
        .check_send_available   = hosted_hci_bluedroid_check_send_available,
        .register_host_callback = hosted_hci_bluedroid_register_host_callback,
    };
    err = esp_bluedroid_attach_hci_driver(&ops);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "attach_hci_driver failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "ESP-Hosted Bluedroid port ready");
    return ESP_OK;
}

esp_err_t eh_host_bluedroid_deinit(void)
{
    (void)hosted_hci_bluedroid_register_host_callback(NULL);

    esp_err_t err = eh_host_bt_controller_disable();
    if (err != ESP_OK)
        ESP_LOGW(TAG, "BT controller disable failed: %s", esp_err_to_name(err));
    err = eh_host_bt_controller_deinit(false);
    if (err != ESP_OK)
        ESP_LOGW(TAG, "BT controller deinit failed: %s", esp_err_to_name(err));
    return ESP_OK;
}

/* Auto-init registration (priority 250 — after feat_bt's 150 so the HCI
 * byte-pipe is up first; reverse on deinit). Compiled only when auto-init is
 * chosen; with it off, init/deinit stay public for the app. */
#if defined(CONFIG_ESP_HOSTED_HOST_BT_PORT_BLUEDROID_AUTO_INIT)
EH_HOST_FEAT_REGISTER(eh_host_bluedroid_init, eh_host_bluedroid_deinit,
                      "bt_port_bluedroid", 250);
#endif
