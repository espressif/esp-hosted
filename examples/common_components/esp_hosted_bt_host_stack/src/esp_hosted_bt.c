/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/*
 * esp_hosted_bt.c — the stack-agnostic dispatcher. Applies MAC (if requested),
 * brings the CP controller up, and binds the selected host stack to feat_bt's
 * HCI byte-pipe. No auto-init: the app calls setup() explicitly.
 */

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "eh_host_feat_bt.h"       /* eh_host_bt_controller_* + eh_host_bt_apply_mac */
#include "eh_host_feat_bt_mcu.h"   /* eh_host_bt_mcu_hci_register (custom path) */

#include "esp_hosted_bt_host_stack.h"
#include "esp_hosted_bt_priv.h"

static const char TAG[] = "esp_hosted_bt";

#define EH_BT_CTRL_RETRY_STEP_MS   100u
#define EH_BT_CTRL_DEFAULT_TMO_MS  2000u   /* bounded wait for the CP BT to answer */

static esp_err_t bring_up_controller(uint32_t timeout_ms)
{
    if (timeout_ms == 0) timeout_ms = EH_BT_CTRL_DEFAULT_TMO_MS;

    esp_err_t r = ESP_FAIL;
    uint32_t waited = 0;
    for (;;) {
        r = eh_host_bt_controller_init();
        if (r == ESP_OK || r == ESP_ERR_INVALID_STATE) break;   /* INVALID_STATE = already up */
        if (waited >= timeout_ms) {
            ESP_LOGE(TAG, "controller init failed: 0x%x (CP not ready after %ums)", r, (unsigned)waited);
            return r;
        }
        vTaskDelay(pdMS_TO_TICKS(EH_BT_CTRL_RETRY_STEP_MS));
        waited += EH_BT_CTRL_RETRY_STEP_MS;
    }
    r = eh_host_bt_controller_enable();
    if (r != ESP_OK && r != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "controller enable failed: 0x%x", r);
        return r;
    }
    return ESP_OK;
}

esp_err_t esp_hosted_bt_host_stack_setup(esp_hosted_bt_host_stack_cfg_t *cfg)
{
    if (!cfg) return ESP_ERR_INVALID_ARG;

    /* Guard: the requested stack must match what was compiled in. */
    if (cfg->stack != ESP_HOSTED_BT_HOST_STACK_DEFAULT_TYPE) {
        ESP_LOGE(TAG, "cfg.stack (%d) != compiled stack (%d); check BT_NIMBLE/BLUEDROID Kconfig",
                 cfg->stack, ESP_HOSTED_BT_HOST_STACK_DEFAULT_TYPE);
        return ESP_ERR_INVALID_STATE;
    }

    eh_host_bt_apply_mac(cfg->bt_mac);

    if (cfg->bring_up_controller) {
        esp_err_t r = bring_up_controller(cfg->controller_ready_timeout_ms);
        if (r != ESP_OK) return r;
    }

    switch (cfg->stack) {
#if defined(CONFIG_BT_NIMBLE_ENABLED)
    case ESP_HOSTED_BT_HOST_STACK_NIMBLE:
        return eh_bt_bind_nimble();
#endif
#if defined(CONFIG_BT_BLUEDROID_ENABLED)
    case ESP_HOSTED_BT_HOST_STACK_BLUEDROID:
        return eh_bt_bind_bluedroid();
#endif
    case ESP_HOSTED_BT_HOST_STACK_CUSTOM:
        if (!cfg->custom.rx) {
            ESP_LOGE(TAG, "custom stack: cfg.custom.rx is NULL");
            return ESP_ERR_INVALID_ARG;
        }
        cfg->custom.tx = eh_host_bt_mcu_hci_register(cfg->custom.rx, cfg->custom.ctx);
        if (!cfg->custom.tx) {
            ESP_LOGE(TAG, "custom stack: HCI register failed");
            return ESP_FAIL;
        }
        ESP_LOGI(TAG, "custom BT stack bound");
        return ESP_OK;
    default:
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_ERR_INVALID_STATE;
}

esp_err_t esp_hosted_bt_host_stack_teardown(void)
{
#if defined(CONFIG_BT_NIMBLE_ENABLED)
    eh_bt_unbind_nimble();
#elif defined(CONFIG_BT_BLUEDROID_ENABLED)
    eh_bt_unbind_bluedroid();
#else
    eh_host_bt_mcu_hci_unregister();   /* custom */
#endif
    esp_err_t r = eh_host_bt_controller_disable();
    if (r != ESP_OK && r != ESP_ERR_INVALID_STATE)
        ESP_LOGW(TAG, "controller disable failed: 0x%x", r);
    r = eh_host_bt_controller_deinit(false);
    if (r != ESP_OK && r != ESP_ERR_INVALID_STATE)
        ESP_LOGW(TAG, "controller deinit failed: 0x%x", r);
    return ESP_OK;
}
