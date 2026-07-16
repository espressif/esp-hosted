/* SPDX-License-Identifier: Apache-2.0 */
/* BT host feature lifecycle (all host types). Controller lifecycle RPC
 * lives in eh_host_bt.c; the HCI byte pipe is MCU-only, in
 * eh_host_feat_bt_mcu.c (the bridge owns its register/unregister). */

#include "sdkconfig.h"

#include "esp_log.h"

#include "eh_host_feat_bt.h"
#include "eh_host_auto_init.h"

#if defined(CONFIG_ESP_HOSTED_HOST_FEAT_BT_AUTO_SET_MAC) || \
    defined(CONFIG_ESP_HOSTED_HOST_FEAT_BT_AUTO_CONTROLLER_INIT)
#include "esp_err.h"
#include "eh_host_bt.h"
static const char *TAG = "eh_host_feat_bt";
#endif

#if CONFIG_ESP_HOSTED_HOST_FEAT_BT_AUTO_SET_MAC
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include "esp_mac.h"
#include "eh_host_sys.h"
#endif

#if CONFIG_ESP_HOSTED_HOST_FEAT_BT_AUTO_SET_MAC
static int parse_mac_str(const char *s, uint8_t out[6])
{
    if (!s) return -1;
    unsigned v[6];
    if (sscanf(s, "%x:%x:%x:%x:%x:%x",
               &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) != 6) return -1;
    for (int i = 0; i < 6; i++) {
        if (v[i] > 0xff) return -1;
        out[i] = (uint8_t)v[i];
    }
    return 0;
}
#endif

esp_err_t eh_host_feat_bt_init(void)
{
#if CONFIG_ESP_HOSTED_HOST_FEAT_BT_AUTO_SET_MAC
    uint8_t mac[6];
    if (parse_mac_str(CONFIG_ESP_HOSTED_HOST_FEAT_BT_MAC, mac) == 0) {
        esp_err_t e = eh_host_iface_mac_addr_set(mac, sizeof(mac), ESP_MAC_BT);
        if (e != ESP_OK) ESP_LOGW(TAG, "auto set BT MAC failed: 0x%x", e);
    } else {
        ESP_LOGW(TAG, "ESP_HOSTED_HOST_FEAT_BT_MAC malformed; expected xx:xx:xx:xx:xx:xx");
    }
#endif

#if CONFIG_ESP_HOSTED_HOST_FEAT_BT_AUTO_CONTROLLER_INIT
    esp_err_t r = eh_host_bt_controller_init();
    /* INVALID_STATE => already initialised; treat as success. */
    if (r != ESP_OK && r != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "auto controller init failed: 0x%x", r);
        return r;
    }
    r = eh_host_bt_controller_enable();
    if (r != ESP_OK && r != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "auto controller enable failed: 0x%x", r);
        return r;
    }
#endif
    return ESP_OK;
}

esp_err_t eh_host_feat_bt_deinit(void)
{
#if CONFIG_ESP_HOSTED_HOST_FEAT_BT_AUTO_CONTROLLER_INIT
    esp_err_t r = eh_host_bt_controller_disable();
    if (r != ESP_OK && r != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "controller disable failed: 0x%x", r);
    }
    r = eh_host_bt_controller_deinit(false);
    if (r != ESP_OK && r != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "controller deinit failed: 0x%x", r);
    }
#endif
    return ESP_OK;
}

EH_HOST_FEAT_REGISTER(eh_host_feat_bt_init, eh_host_feat_bt_deinit, "bt", 150);
