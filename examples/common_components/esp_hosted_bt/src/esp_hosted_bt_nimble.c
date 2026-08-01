/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/*
 * NimBLE <-> ESP-Hosted HCI glue. NimBLE's weak ble_transport_to_ll_* are given
 * strong overrides here (TX, link-time); RX is bound at eh_bt_bind_nimble().
 * Controller bring-up is the dispatcher's job (esp_hosted_bt.c).
 */

#include "sdkconfig.h"
#if defined(CONFIG_BT_NIMBLE_ENABLED)

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"

#include "host/ble_hs_mbuf.h"
#include "os/os_mbuf.h"
#include "nimble/transport.h"
#include "nimble/transport/hci_h4.h"   /* HCI_H4_{CMD,ACL,SCO,EVT} */
#include "nimble/hci_common.h"

#include "eh_host_feat_bt_mcu.h"        /* eh_host_bt_mcu_hci_register */
#include "esp_hosted_bt_priv.h"

static const char TAG[] = "esp_hosted_bt_nimble";

#define BLE_HCI_EVT_HDR_LEN (2)

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
            (blen > BLE_HCI_EVT_HDR_LEN) &&   /* a subevent byte (body[2]) is present */
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

esp_err_t eh_bt_bind_nimble(void)
{
    eh_host_bt_mcu_hci_tx_fn_t tx = eh_host_bt_mcu_hci_register(nimble_rx, NULL);
    if (!tx) {
        ESP_LOGE(TAG, "HCI register failed");
        return ESP_FAIL;
    }
    s_hci_tx = tx;
    ESP_LOGI(TAG, "NimBLE bound to hosted HCI");
    return ESP_OK;
}

void eh_bt_unbind_nimble(void)
{
    eh_host_bt_mcu_hci_unregister();
    s_hci_tx = hci_tx_unbound;
}

#endif /* CONFIG_BT_NIMBLE_ENABLED */
