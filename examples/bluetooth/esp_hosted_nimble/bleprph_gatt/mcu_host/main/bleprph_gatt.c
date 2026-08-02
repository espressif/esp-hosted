/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/*
 * bleprph_gatt.c — NimBLE peripheral with one GATT service.
 *
 * The app calls esp_hosted_bt_host_stack_setup() once (controller + HCI bind)
 * right before nimble_port_init(). Everything else is stack-native NimBLE.
 */

#include <inttypes.h>
#include <string.h>

#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "esp_check.h"
#include "esp_hosted_bt_host_stack.h"
#include "sdkconfig.h"

#define TAG "bleprph_gatt"

#ifndef CONFIG_EXAMPLE_DEVICE_NAME
#  define CONFIG_EXAMPLE_DEVICE_NAME "esp-hosted-nim-gatt"
#endif
#ifndef CONFIG_EXAMPLE_NOTIFY_INTERVAL_MS
#  define CONFIG_EXAMPLE_NOTIFY_INTERVAL_MS 1000
#endif

static uint8_t        s_own_addr_type;
static uint16_t       s_conn_handle = 0xFFFF;
static uint16_t       s_notify_attr_handle;
static bool           s_notify_subscribed = false;

/* Custom 128-bit service + char UUIDs */
static const ble_uuid128_t SVC_UUID = BLE_UUID128_INIT(
    0x00, 0x00, 0x00, 0xCC, 0x00, 0x00, 0x10, 0x00,
    0x80, 0x00, 0x00, 0x80, 0x5f, 0x9b, 0x34, 0xfb);
static const ble_uuid128_t RW_CHAR_UUID = BLE_UUID128_INIT(
    0x01, 0xFF, 0x00, 0xCC, 0x00, 0x00, 0x10, 0x00,
    0x80, 0x00, 0x00, 0x80, 0x5f, 0x9b, 0x34, 0xfb);
static const ble_uuid128_t NTF_CHAR_UUID = BLE_UUID128_INIT(
    0x02, 0xFF, 0x00, 0xCC, 0x00, 0x00, 0x10, 0x00,
    0x80, 0x00, 0x00, 0x80, 0x5f, 0x9b, 0x34, 0xfb);

static uint8_t s_rw_val[8] = {0xDE, 0xAD, 0xBE, 0xEF, 0, 0, 0, 0};

static int gatt_access_rw(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg);

static const struct ble_gatt_svc_def s_gatt_svcs[] = {
    {
        .type            = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid            = &SVC_UUID.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid       = &RW_CHAR_UUID.u,
                .access_cb  = gatt_access_rw,
                .flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
            },
            {
                .uuid       = &NTF_CHAR_UUID.u,
                .access_cb  = gatt_access_rw,    /* same cb, ignores reads here */
                .val_handle = &s_notify_attr_handle,
                .flags      = BLE_GATT_CHR_F_NOTIFY,
            },
            { 0 },
        },
    },
    { 0 },
};

static int gatt_access_rw(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle; (void)arg;
    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_READ_CHR:
        return os_mbuf_append(ctxt->om, s_rw_val, sizeof(s_rw_val));
    case BLE_GATT_ACCESS_OP_WRITE_CHR: {
        uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
        if (len > sizeof(s_rw_val)) len = sizeof(s_rw_val);
        int rc = ble_hs_mbuf_to_flat(ctxt->om, s_rw_val, len, NULL);
        ESP_LOGI(TAG, "RW char written, len=%u rc=%d", len, rc);
        return rc;
    }
    default:
        return BLE_ATT_ERR_UNLIKELY;
    }
}

static int bleprph_gap_event(struct ble_gap_event *event, void *arg);

static void bleprph_advertise(void)
{
    struct ble_gap_adv_params adv_params = {0};
    struct ble_hs_adv_fields fields = {0};
    const char *name = ble_svc_gap_device_name();
    int rc;

    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc) { ESP_LOGE(TAG, "adv_set_fields: %d", rc); return; }

    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, bleprph_gap_event, NULL);
    if (rc) { ESP_LOGE(TAG, "adv_start: %d", rc); return; }
    ESP_LOGI(TAG, "advertising as \"%s\"", name);
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
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
            ESP_LOGI(TAG, "connected, conn_handle=%u", s_conn_handle);
        } else {
            ESP_LOGE(TAG, "connect failed: %d", event->connect.status);
            bleprph_advertise();
        }
        return 0;
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "disconnected, reason=%d", event->disconnect.reason);
        s_conn_handle = 0xFFFF;
        s_notify_subscribed = false;
        bleprph_advertise();
        return 0;
    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == s_notify_attr_handle) {
            s_notify_subscribed = event->subscribe.cur_notify;
            ESP_LOGI(TAG, "notify subscribed=%d", s_notify_subscribed);
        }
        return 0;
    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU update, value=%d", event->mtu.value);
        return 0;
    default:
        return 0;
    }
}

static void notify_task(void *arg)
{
    (void)arg;
    uint32_t counter = 0;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(CONFIG_EXAMPLE_NOTIFY_INTERVAL_MS));
        if (s_notify_subscribed && s_conn_handle != 0xFFFF) {
            counter++;
            struct os_mbuf *om = ble_hs_mbuf_from_flat(&counter, sizeof(counter));
            if (om) {
                ble_gatts_notify_custom(s_conn_handle, s_notify_attr_handle, om);
            }
        }
    }
}

static void on_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    if (rc) { ESP_LOGE(TAG, "ensure_addr: %d", rc); return; }
    rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
    if (rc) { ESP_LOGE(TAG, "id_infer_auto: %d", rc); return; }
    bleprph_advertise();
}

static void on_reset(int reason) { ESP_LOGW(TAG, "NimBLE reset, reason=%d", reason); }

static void host_task(void *arg) { (void)arg; nimble_port_run(); nimble_port_freertos_deinit(); }

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    /* Bring the hosted BT binding up: controller + HCI wired to NimBLE. */
    esp_hosted_bt_host_stack_cfg_t bt = ESP_HOSTED_BT_HOST_STACK_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_hosted_bt_host_stack_setup(&bt));

    /* NimBLE init + register services. */
    ESP_ERROR_CHECK(nimble_port_init());

    ble_hs_cfg.sync_cb         = on_sync;
    ble_hs_cfg.reset_cb        = on_reset;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    int rc = ble_gatts_count_cfg(s_gatt_svcs);
    if (rc) { ESP_LOGE(TAG, "gatts_count_cfg: %d", rc); return; }
    rc = ble_gatts_add_svcs(s_gatt_svcs);
    if (rc) { ESP_LOGE(TAG, "gatts_add_svcs: %d", rc); return; }

    ble_svc_gap_device_name_set(CONFIG_EXAMPLE_DEVICE_NAME);

    nimble_port_freertos_init(host_task);
    xTaskCreate(notify_task, "notify", 3072, NULL, 5, NULL);

    ESP_LOGI(TAG, "init complete");
}
