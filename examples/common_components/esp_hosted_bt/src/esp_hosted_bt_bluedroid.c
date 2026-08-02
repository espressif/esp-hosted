/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/*
 * Bluedroid <-> ESP-Hosted HCI glue. Bluedroid speaks H4 on both its send and
 * recv hooks, so this is a thin forwarder. eh_bt_bind_bluedroid() attaches the
 * HCI driver; Bluedroid calls register_host_callback (→ our RX bind) during its
 * own enable, before the first HCI TX. Controller bring-up is the dispatcher's.
 */

#include "sdkconfig.h"
#if defined(CONFIG_BT_BLUEDROID_ENABLED)

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_bluedroid_hci.h"

#include "eh_host_feat_bt_mcu.h"        /* eh_host_bt_mcu_hci_register */
#include "esp_hosted_bt_priv.h"

static const char TAG[] = "esp_hosted_bt_bluedroid";

static esp_bluedroid_hci_driver_callbacks_t s_callback;

static int hci_tx_unbound(const uint8_t *frame, uint16_t len)
{ (void)frame; (void)len; return -1; }
static eh_host_bt_mcu_hci_tx_fn_t s_hci_tx = hci_tx_unbound;

/* RX: ESP-Hosted → Bluedroid. frame is H4-prefixed, exactly what
 * notify_host_recv expects; forward as-is (valid for cb duration). */
static void bluedroid_rx(const uint8_t *frame, uint16_t len, void *user)
{
    (void)user;
    if (s_callback.notify_host_recv)
        s_callback.notify_host_recv((uint8_t *)(uintptr_t)frame, len);
}

static void hosted_open(void)  { }

static bool hosted_check_send_available(void) { return true; }

static esp_err_t hosted_register_host_callback(
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

/* TX: Bluedroid → ESP-Hosted. `data` is H4-prefixed, copied synchronously by
 * the transport — pass straight through. */
static void hosted_send(uint8_t *data, uint16_t len)
{
    if (!data || len == 0) return;
    if (s_hci_tx(data, len) != 0)
        ESP_LOGE(TAG, "hci tx failed (len=%u)", len);
}

esp_err_t eh_bt_bind_bluedroid(void)
{
    static const esp_bluedroid_hci_driver_operations_t ops = {
        .send                   = hosted_send,
        .check_send_available   = hosted_check_send_available,
        .register_host_callback = hosted_register_host_callback,
    };
    hosted_open();
    esp_err_t err = esp_bluedroid_attach_hci_driver(&ops);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "attach_hci_driver failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "Bluedroid bound to hosted HCI");
    return ESP_OK;
}

void eh_bt_unbind_bluedroid(void)
{
    esp_bluedroid_detach_hci_driver();
    eh_host_bt_mcu_hci_unregister();
    s_hci_tx = hci_tx_unbound;
    s_callback.notify_host_send_available = NULL;
    s_callback.notify_host_recv           = NULL;
}

#endif /* CONFIG_BT_BLUEDROID_ENABLED */
