/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/* SPDX-License-Identifier: Apache-2.0 */
/*
 * bleprph_minimal.c — smallest NimBLE + ESP-Hosted example.
 *
 * Steps:
 *  1. NVS init.
 *  2. ESP-Hosted NimBLE bridge setup (one helper call).
 *  3. NimBLE host init + run.
 *  4. Configure GAP and advertise.
 *
 * Step 2 is the only ESP-Hosted-specific call.  Everything from
 * step 3 is stack-native NimBLE.
 */

#include <inttypes.h>
#include <string.h>

#include "esp_log.h"
#include "nvs_flash.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"

#include "esp_check.h"
#include "esp_hosted_bt_stack.h"

#include "sdkconfig.h"

#define TAG "bleprph_min"

#ifndef CONFIG_EXAMPLE_DEVICE_NAME
#  define CONFIG_EXAMPLE_DEVICE_NAME "esp-hosted-nim"
#endif

static uint8_t s_own_addr_type;

static int bleprph_gap_event(struct ble_gap_event *event, void *arg);

static void bleprph_advertise(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    const char *name;
    int rc;

    memset(&fields, 0, sizeof fields);
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_set_fields failed: %d", rc);
        return;
    }

    memset(&adv_params, 0, sizeof adv_params);
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, bleprph_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_start failed: %d", rc);
    } else {
        ESP_LOGI(TAG, "advertising as \"%s\"", name);
    }
}

static int bleprph_gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {
#if defined(BLE_GAP_EVENT_LINK_ESTAB)
    case BLE_GAP_EVENT_LINK_ESTAB:
#else
    case BLE_GAP_EVENT_CONNECT:
#endif
        ESP_LOGI(TAG, "connection %s (status=%d)",
                 event->connect.status == 0 ? "up" : "failed",
                 event->connect.status);
        if (event->connect.status != 0) {
            bleprph_advertise();
        }
        return 0;
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "disconnect (reason=%d) — re-advertising",
                 event->disconnect.reason);
        bleprph_advertise();
        return 0;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        bleprph_advertise();
        return 0;
    default:
        return 0;
    }
}

static void bleprph_on_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "ensure_addr failed: %d", rc);
        return;
    }
    rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "id_infer_auto failed: %d", rc);
        return;
    }
    bleprph_advertise();
}

static void bleprph_on_reset(int reason)
{
    ESP_LOGW(TAG, "NimBLE reset (reason=%d)", reason);
}

static void bleprph_host_task(void *param)
{
    (void)param;
    ESP_LOGI(TAG, "NimBLE host task started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void app_main(void)
{
    esp_err_t err;

    /* 1. NVS for BT PHY calibration. */
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    /* 2. Bring the hosted BT binding up: controller + HCI wired to NimBLE. */
    esp_hosted_bt_stack_cfg_t bt = ESP_HOSTED_BT_STACK_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_hosted_bt_stack_setup(&bt));

    /* 3. Standard NimBLE init. TX overrides (ble_transport_to_ll_*) are linked in. */
    err = nimble_port_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed: %d", err);
        return;
    }

    ble_hs_cfg.sync_cb  = bleprph_on_sync;
    ble_hs_cfg.reset_cb = bleprph_on_reset;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    ble_svc_gap_device_name_set(CONFIG_EXAMPLE_DEVICE_NAME);

    nimble_port_freertos_init(bleprph_host_task);

    ESP_LOGI(TAG, "init complete");
}
