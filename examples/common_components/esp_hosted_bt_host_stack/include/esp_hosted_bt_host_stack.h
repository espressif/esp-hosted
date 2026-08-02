/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/*
 * esp_hosted_bt_host_stack.h — one explicit entry point to bind a BT host stack to
 * ESP-Hosted's HCI byte-pipe. NimBLE and Bluedroid glue are built in; a custom
 * stack is a first-class option (you provide RX, you get TX). See
 * docs/design/bluetooth.md.
 *
 * Flow (identical for every stack):
 *   esp_hosted_init();  esp_hosted_connect_to_slave();
 *   esp_hosted_bt_host_stack_cfg_t cfg = ESP_HOSTED_BT_HOST_STACK_CONFIG_DEFAULT();
 *   esp_hosted_bt_host_stack_setup(&cfg);         // controller (+MAC) up + HCI bound
 *   // then the stack's own lifecycle (app-owned): esp_bluedroid_init/enable
 *   // or nimble_port_init().
 */
#ifndef ESP_HOSTED_BT_HOST_STACK_H_
#define ESP_HOSTED_BT_HOST_STACK_H_

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "sdkconfig.h"

#ifdef __cplusplus
extern "C" {
#endif

/* One raw H4 HCI frame (byte 0 = packet type). */
typedef void (*esp_hosted_bt_hci_rx_fn_t)(const uint8_t *frame, uint16_t len, void *ctx);
typedef int  (*esp_hosted_bt_hci_tx_fn_t)(const uint8_t *frame, uint16_t len);

typedef enum {
    ESP_HOSTED_BT_HOST_STACK_NIMBLE,
    ESP_HOSTED_BT_HOST_STACK_BLUEDROID,
    ESP_HOSTED_BT_HOST_STACK_CUSTOM,
} esp_hosted_bt_host_stack_t;

typedef struct {
    esp_hosted_bt_host_stack_t stack;                    /* which host stack to bind */
    const uint8_t        *bt_mac;                   /* NULL: use Kconfig default / leave as-is; else set before controller */
    bool                  bring_up_controller;      /* true: setup does controller init+enable */
    uint32_t              controller_ready_timeout_ms; /* 0: default; bounded wait/retry for CP BT */

    /* CUSTOM only: your RX handler + ctx in; setup fills tx (call it to send). */
    struct {
        esp_hosted_bt_hci_rx_fn_t rx;
        void                     *ctx;
        esp_hosted_bt_hci_tx_fn_t tx;               /* [out] */
    } custom;
} esp_hosted_bt_host_stack_cfg_t;

/* Stack picked from the IDF BT Kconfig you already set — no hosted BT-port knob. */
#if defined(CONFIG_BT_NIMBLE_ENABLED)
#  define ESP_HOSTED_BT_HOST_STACK_DEFAULT_TYPE ESP_HOSTED_BT_HOST_STACK_NIMBLE
#elif defined(CONFIG_BT_BLUEDROID_ENABLED)
#  define ESP_HOSTED_BT_HOST_STACK_DEFAULT_TYPE ESP_HOSTED_BT_HOST_STACK_BLUEDROID
#else
#  define ESP_HOSTED_BT_HOST_STACK_DEFAULT_TYPE ESP_HOSTED_BT_HOST_STACK_CUSTOM
#endif

#define ESP_HOSTED_BT_HOST_STACK_CONFIG_DEFAULT() (esp_hosted_bt_host_stack_cfg_t){ \
    .stack = ESP_HOSTED_BT_HOST_STACK_DEFAULT_TYPE, \
    .bt_mac = NULL, .bring_up_controller = true, .controller_ready_timeout_ms = 0, \
    .custom = { 0 } }

/* Explicit + compile-time-guarded selectors (transparency + fail-before-flash). */
#define ESP_HOSTED_BT_HOST_STACK_CONFIG_NIMBLE() (esp_hosted_bt_host_stack_cfg_t){ \
    .stack = ESP_HOSTED_BT_HOST_STACK_NIMBLE, .bring_up_controller = true }
#define ESP_HOSTED_BT_HOST_STACK_CONFIG_BLUEDROID() (esp_hosted_bt_host_stack_cfg_t){ \
    .stack = ESP_HOSTED_BT_HOST_STACK_BLUEDROID, .bring_up_controller = true }
#define ESP_HOSTED_BT_HOST_STACK_CONFIG_CUSTOM(rx_fn, rx_ctx) (esp_hosted_bt_host_stack_cfg_t){ \
    .stack = ESP_HOSTED_BT_HOST_STACK_CUSTOM, .bring_up_controller = true, \
    .custom = { .rx = (rx_fn), .ctx = (rx_ctx) } }

/**
 * @brief Bring the hosted BT binding up: (optional) set MAC, init+enable the CP
 *        controller, and wire the selected stack to the HCI byte-pipe.
 *        Call after esp_hosted_connect_to_slave(), before the stack's own init.
 *        The app owns the stack lifecycle (esp_bluedroid_init/enable, nimble_port_init).
 */
esp_err_t esp_hosted_bt_host_stack_setup(esp_hosted_bt_host_stack_cfg_t *cfg);

/** @brief Symmetric teardown: unbind HCI + disable/deinit the controller. */
esp_err_t esp_hosted_bt_host_stack_teardown(void);

#ifdef __cplusplus
}
#endif

#endif /* ESP_HOSTED_BT_HOST_STACK_H_ */
