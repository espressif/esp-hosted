/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/* SPDX-License-Identifier: Apache-2.0 */
/*
 * eh_host_nimble.c — NimBLE <-> ESP-Hosted HCI port.
 *
 * NimBLE's transport layer declares weak `ble_transport_to_ll_*`
 * functions; our strong overrides forward H4 frames to ESP-Hosted.
 * RX is wired via eh_host_bt_mcu_hci_register, which returns the bound
 * tx fn. init/deinit are public (auto-init at priority after feat_bt,
 * or callable by the app when auto-init is disabled).
 */

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sdkconfig.h"

#include "host/ble_hs_mbuf.h"
#include "os/os_mbuf.h"
#include "nimble/transport.h"
#include "nimble/transport/hci_h4.h"   /* HCI_H4_{CMD,ACL,SCO,EVT} */
#include "nimble/hci_common.h"

#include "esp_hosted.h"
#include "eh_host_feat_bt_mcu.h"
#if defined(CONFIG_ESP_HOSTED_HOST_BT_PORT_NIMBLE_AUTO_INIT)
#include "eh_host_auto_init.h"
#endif

#include "eh_host_nimble.h"

static const char TAG[] = "eh_host_nimble";

#define BLE_HCI_EVT_HDR_LEN (2)

/* tx fn bound at register; never NULL (defaults to drop). */
static int hci_tx_unbound(const uint8_t *frame, uint16_t len)
{ (void)frame; (void)len; return -1; }
static eh_host_bt_mcu_hci_tx_fn_t s_hci_tx = hci_tx_unbound;

/* ─── RX: ESP-Hosted → NimBLE host. frame[0] = H4 type. ──────────── */
static void nimble_rx(const uint8_t *frame, uint16_t len, void *user)
{
    (void)user;
    if (len < 1) return;

    const uint8_t *body = &frame[1];
    uint16_t       blen = len - 1;

    if (frame[0] == HCI_H4_EVT) {
        if (blen < BLE_HCI_EVT_HDR_LEN) {
            ESP_LOGE(TAG, "RX evt too short: %u", blen);
            return;
        }
        int totlen = BLE_HCI_EVT_HDR_LEN + body[1];
        if ((uint16_t)totlen > blen) {
            ESP_LOGE(TAG, "RX evt truncated: totlen=%d blen=%u", totlen, blen);
            return;
        }
        bool is_adv_report =
            (body[0] == BLE_HCI_EVCODE_LE_META) &&
            (body[2] == BLE_HCI_LE_SUBEV_ADV_RPT ||
             body[2] == BLE_HCI_LE_SUBEV_EXT_ADV_RPT);

        uint8_t *evbuf = ble_transport_alloc_evt(is_adv_report ? 1 : 0);
        if (!evbuf) {
            if (!is_adv_report)
                ESP_LOGE(TAG, "transport_alloc_evt failed");
            return;
        }
        memcpy(evbuf, body, (size_t)totlen);
        if (ble_transport_to_hs_evt(evbuf))
            ESP_LOGE(TAG, "transport_to_hs_evt failed");
        return;
    }

    if (frame[0] == HCI_H4_ACL) {
        struct os_mbuf *m = ble_transport_alloc_acl_from_ll();
        if (!m) {
            ESP_LOGE(TAG, "alloc_acl_from_ll failed");
            return;
        }
        if (os_mbuf_append(m, body, blen)) {
            os_mbuf_free_chain(m);
            return;
        }
        ble_transport_to_hs_acl(m);
    }
}

/* ─── TX: NimBLE host → ESP-Hosted. Build one H4 frame, hand off. ── */
int ble_transport_to_ll_acl_impl(struct os_mbuf *om)
{
    int      body_len = OS_MBUF_PKTLEN(om);
    uint8_t *frame    = malloc((size_t)body_len + 1);
    int      ret      = ESP_FAIL;

    if (frame) {
        frame[0] = HCI_H4_ACL;
        if (ble_hs_mbuf_to_flat(om, frame + 1, (uint16_t)body_len, NULL) == 0)
            ret = s_hci_tx(frame, (uint16_t)(body_len + 1)) == 0 ? ESP_OK : ESP_FAIL;
        free(frame);
    }
    os_mbuf_free_chain(om);
    return ret;
}

int ble_transport_to_ll_cmd_impl(void *buf)
{
    int      body_len = 3 + ((uint8_t *)buf)[2];
    uint8_t *frame    = malloc((size_t)body_len + 1);
    int      ret      = ESP_FAIL;

    if (frame) {
        frame[0] = HCI_H4_CMD;
        memcpy(frame + 1, buf, (size_t)body_len);
        ret = s_hci_tx(frame, (uint16_t)(body_len + 1)) == 0 ? ESP_OK : ESP_FAIL;
        free(frame);
    }
    ble_transport_free(buf);
    return ret;
}

#if !defined(CONFIG_IDF_TARGET_ESP32) && !defined(CONFIG_IDF_TARGET_ESP32C3) \
    && !defined(CONFIG_IDF_TARGET_ESP32S3)
void ble_transport_ll_init(void)   { }
void ble_transport_ll_deinit(void) { }
#endif

esp_err_t eh_host_nimble_init(void)
{
    /* The CP may still be booting when the host reaches here — the minimal BT
     * app boots fast while the CP does full controller init, and slower-handshake
     * transports (UART/SPI-HD) widen the gap. connect_to_slave is idempotent, so
     * retry until the CP is ready instead of aborting on the first miss. */
    esp_err_t err = ESP_FAIL;
    for (int attempt = 0; attempt < 50; attempt++) {
        err = esp_hosted_connect_to_slave();
        if (err == ESP_OK) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "connect_to_slave failed: %s", esp_err_to_name(err));
        return err;
    }
    /* feat_bt (lower priority) may already have brought the controller up;
     * ESP_ERR_INVALID_STATE means "already done" — treat as success. */
    err = esp_hosted_bt_controller_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "BT controller init failed: %s", esp_err_to_name(err));
        return err;
    }
    err = esp_hosted_bt_controller_enable();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "BT controller enable failed: %s", esp_err_to_name(err));
        return err;
    }
    eh_host_bt_mcu_hci_tx_fn_t tx = eh_host_bt_mcu_hci_register(nimble_rx, NULL);
    if (!tx) {
        ESP_LOGE(TAG, "HCI register failed");
        return ESP_FAIL;
    }
    s_hci_tx = tx;
    ESP_LOGI(TAG, "ESP-Hosted NimBLE port ready");
    return ESP_OK;
}

esp_err_t eh_host_nimble_deinit(void)
{
    eh_host_bt_mcu_hci_unregister();
    s_hci_tx = hci_tx_unbound;

    esp_err_t err = esp_hosted_bt_controller_disable();
    if (err != ESP_OK)
        ESP_LOGW(TAG, "BT controller disable failed: %s", esp_err_to_name(err));
    err = esp_hosted_bt_controller_deinit(false);
    if (err != ESP_OK)
        ESP_LOGW(TAG, "BT controller deinit failed: %s", esp_err_to_name(err));
    return ESP_OK;
}

/* Auto-init registration (priority 250 — after feat_bt's 150 so the HCI
 * byte-pipe is up first; reverse on deinit). Compiled only when auto-init is
 * chosen; with it off, eh_host_nimble_init/deinit stay public for the app. */
#if defined(CONFIG_ESP_HOSTED_HOST_BT_PORT_NIMBLE_AUTO_INIT)
EH_HOST_FEAT_REGISTER(eh_host_nimble_init, eh_host_nimble_deinit,
                      "bt_port_nimble", 250);
#endif
