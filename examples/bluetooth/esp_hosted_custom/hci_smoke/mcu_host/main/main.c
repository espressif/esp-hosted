/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/*
 * hci_smoke — a "bring-your-own BT stack" example. No NimBLE, no Bluedroid: the
 * app binds its own raw-HCI handler to the hosted HCI byte-pipe via the CUSTOM
 * path, sends an HCI Reset, and prints the controller's Command-Complete. This
 * proves the custom two-wire path end to end: tx -> CP controller -> rx.
 *
 * The only ESP-Hosted-specific code is one call: esp_hosted_bt_host_stack_setup()
 * with ESP_HOSTED_BT_HOST_STACK_CONFIG_CUSTOM(rx, ctx).
 */
#include "esp_log.h"
#include "nvs_flash.h"

#include "esp_hosted_bt_host_stack.h"

#define TAG "bt_custom_hci"

static esp_hosted_bt_hci_tx_fn_t s_tx;

/* HCI Reset, H4-framed: type 0x01 (command), opcode 0x0C03 (OGF 3 / OCF 1), 0 params. */
static const uint8_t HCI_RESET[] = { 0x01, 0x03, 0x0C, 0x00 };

/* Custom RX: raw H4 HCI frames from the controller. Recognise the
 * Command-Complete for Reset (event 0x0E carrying opcode 0x0C03). */
static void custom_rx(const uint8_t *frame, uint16_t len, void *ctx)
{
    (void)ctx;
    if (len >= 7 && frame[0] == 0x04 && frame[1] == 0x0E
        && frame[4] == 0x03 && frame[5] == 0x0C) {
        ESP_LOGI(TAG, "custom rx: HCI Reset Command Complete, status=0x%02x", frame[6]);
    } else {
        ESP_LOGI(TAG, "custom rx: %u bytes, h4_type=0x%02x", len, frame[0]);
    }
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Custom stack: bring the CP controller up and bind our raw-HCI rx/tx. No
     * IDF BT host stack is compiled in — this is the bring-your-own-stack path. */
    esp_hosted_bt_host_stack_cfg_t bt =
        ESP_HOSTED_BT_HOST_STACK_CONFIG_CUSTOM(custom_rx, NULL);
    ESP_ERROR_CHECK(esp_hosted_bt_host_stack_setup(&bt));

    s_tx = bt.custom.tx;
    if (!s_tx) {
        ESP_LOGE(TAG, "setup did not return a tx function");
        return;
    }

    ESP_LOGI(TAG, "custom stack bound; sending HCI Reset via the custom tx");
    if (s_tx(HCI_RESET, sizeof(HCI_RESET)) != 0) {
        ESP_LOGE(TAG, "custom tx failed");
    }
}
