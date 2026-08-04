/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/*
 * hci_demo — a "bring your own BT stack" template. No NimBLE, no Bluedroid: the
 * app binds its own raw-HCI handlers to the hosted HCI byte-pipe via the CUSTOM
 * path. To port YOUR stack, fill in the four custom_stack_* hooks below.
 *
 * The only ESP-Hosted-specific code is one call in app_main():
 *     esp_hosted_bt_host_stack_setup(&cfg)
 *   with ESP_HOSTED_BT_HOST_STACK_CONFIG_CUSTOM(rx, ctx).
 *
 * As a working demo this sends an HCI Reset and logs the controller's
 * Command-Complete, proving the path end to end: tx -> CP controller -> rx.
 */
#include "esp_log.h"
#include "nvs_flash.h"

#include "esp_hosted_bt_host_stack.h"

#define TAG "bt_custom_hci"

/* Hosted downlink: send one raw H4 HCI frame to the controller. Set by
 * esp_hosted_bt_host_stack_setup() (cfg.custom.tx). */
static esp_hosted_bt_hci_tx_fn_t s_hci_tx;

/* ==========================================================================
 *  Your BT host stack — fill these in
 * ========================================================================== */

/* Bring your host stack up (buffers, tasks, state). Runs before the controller. */
static void custom_stack_init(void)
{
    /* TODO: initialise your Bluetooth host stack here. */
}

/* Tear your host stack down; runs if bring-up fails (mirror of custom_stack_init). */
static void custom_stack_deinit(void)
{
    /* TODO: release whatever custom_stack_init() acquired. */
}

/* Uplink: the controller delivers one raw H4 HCI frame (byte 0 = H4 packet type).
 * Feed it into your stack's HCI parser here. */
static void custom_stack_rx(const uint8_t *frame, uint16_t len, void *ctx)
{
    (void)ctx;
    /* TODO: hand `frame`/`len` to your stack. Demo: recognise the
     * Command-Complete for our HCI Reset (event 0x0E, opcode 0x0C03). */
    if (len >= 7 && frame[0] == 0x04 && frame[1] == 0x0E
        && frame[4] == 0x03 && frame[5] == 0x0C) {
        ESP_LOGI(TAG, "custom rx: HCI Reset Command Complete, status=0x%02x", frame[6]);
    } else {
        ESP_LOGI(TAG, "custom rx: %u bytes, h4_type=0x%02x", len, frame[0]);
    }
}

/* Downlink helper: your stack calls this to send one raw H4 HCI frame. */
static int custom_stack_tx(const uint8_t *frame, uint16_t len)
{
    if (!s_hci_tx) {
        return -1;
    }
    return s_hci_tx(frame, len);
}

/* ========================================================================== */

/* HCI Reset, H4-framed: type 0x01 (command), opcode 0x0C03 (OGF 3 / OCF 1), 0 params. */
static const uint8_t HCI_RESET[] = { 0x01, 0x03, 0x0C, 0x00 };

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    custom_stack_init();

    /* Bring the CP controller up and bind our raw-HCI rx; setup fills cfg.custom.tx. */
    esp_hosted_bt_host_stack_cfg_t cfg =
        ESP_HOSTED_BT_HOST_STACK_CONFIG_CUSTOM(custom_stack_rx, NULL);
    ESP_ERROR_CHECK(esp_hosted_bt_host_stack_setup(&cfg));

    s_hci_tx = cfg.custom.tx;
    if (!s_hci_tx) {
        ESP_LOGE(TAG, "setup did not return a tx function");
        custom_stack_deinit();
        return;
    }

    ESP_LOGI(TAG, "custom stack bound; sending HCI Reset via the custom tx");
    if (custom_stack_tx(HCI_RESET, sizeof(HCI_RESET)) != 0) {
        ESP_LOGE(TAG, "custom tx failed");
    }
}
